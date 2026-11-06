from __future__ import annotations

import argparse
import json
import logging
import math
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from .depth_obstacles import (
    CollisionCheckResult,
    ObstacleCloud,
    capture_realsense_obstacle_cloud,
    check_joint_path_collision,
    load_obstacle_cloud_json,
    save_obstacle_cloud_json,
)
from .dex3_hand import Dex3HandController
from .dex3_sdk2_backend import UnitreeSdk2Dex3Backend
from .move_arm_preset import SafeArmPoseController
from .task5_ik_reach import (
    DEFAULT_CAMERA_X,
    DEFAULT_CAMERA_Y,
    DEFAULT_CAMERA_Z,
    DEFAULT_CAMERA_PITCH_DOWN_DEG,
    RIGHT_SOFT_PREGRASP,
)
from .types import PoseXYZ
from .urdf_kinematics import active_joints, fk, load_urdf, path_to_link, position, solve_ik_position
from .vision_kinematics import CameraPoint, CameraToArmConfig, GraspOffset, yolo_camera_point_to_arm_target

JointPose = Tuple[float, float, float, float, float, float, float]

# Mirror of RIGHT_SOFT_PREGRASP using the same mirror rule as old move presets:
# pitch same, roll/yaw signs flipped, elbow same, wrist roll/yaw signs flipped.
LEFT_SOFT_PREGRASP: JointPose = (
    RIGHT_SOFT_PREGRASP[0],
    -RIGHT_SOFT_PREGRASP[1],
    -RIGHT_SOFT_PREGRASP[2],
    RIGHT_SOFT_PREGRASP[3],
    -RIGHT_SOFT_PREGRASP[4],
    RIGHT_SOFT_PREGRASP[5],
    -RIGHT_SOFT_PREGRASP[6],
)


@dataclass(frozen=True)
class BoxTargets:
    detected_center: PoseXYZ
    grasp_center: PoseXYZ
    left: PoseXYZ
    right: PoseXYZ


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plan two G1 hands to the left and right sides of a box.")
    parser.add_argument("--camera-x", type=float, help="Box center camera X from RealSense/YOLO, meters.")
    parser.add_argument("--camera-y", type=float, help="Box center camera Y from RealSense/YOLO, meters.")
    parser.add_argument("--camera-z", type=float, help="Box center camera Z/depth from RealSense/YOLO, meters.")
    parser.add_argument("--torso-x", type=float, help="Box center torso x, meters. Overrides camera point when all torso coords are supplied.")
    parser.add_argument("--torso-y", type=float, help="Box center torso y, meters.")
    parser.add_argument("--torso-z", type=float, help="Box center torso z, meters.")
    parser.add_argument("--box-width", type=float, default=0.28, help="Box width left/right in torso y, meters.")
    parser.add_argument("--box-height", type=float, default=0.16, help="Box height, meters. Used for default grasp height.")
    parser.add_argument("--box-depth", type=float, default=0.20, help="Box depth forward/back, meters. Only recorded for now.")
    parser.add_argument("--side-clearance", type=float, default=0.065, help="Extra distance outside each side of the box, meters.")
    parser.add_argument(
        "--grasp-strategy",
        choices=("finger-close", "flat-squeeze"),
        default="flat-squeeze",
        help="finger-close closes Dex3 fingers at the target; flat-squeeze uses flat palms and squeezes by moving arms inward.",
    )
    parser.add_argument("--squeeze-side-clearance", type=float, default=-0.015, help="Final side clearance for --grasp-strategy flat-squeeze, meters. Negative values squeeze into the box sides.")
    parser.add_argument("--squeeze-duration", type=float, default=2.0, help="Duration for final flat-palm squeeze motion, seconds.")
    parser.add_argument("--max-two-hand-center-y", type=float, default=0.10, help="Warn when box center is too far left/right for a two-hand grasp.")
    parser.add_argument(
        "--grasp-center-y-mode",
        choices=("detected", "zero", "clamp"),
        default="zero",
        help="How to choose the two-hand grasp center in torso y.",
    )
    parser.add_argument("--grasp-center-y-limit", type=float, default=0.08, help="Y clamp limit for --grasp-center-y-mode clamp.")
    parser.add_argument("--palm-forward-offset", type=float, default=-0.05, help="Offset from box center to palm targets in torso x, meters.")
    parser.add_argument("--palm-up-offset", type=float, default=0.075, help="Offset from box center to palm targets in torso z, meters.")
    parser.add_argument("--max-palm-target-x", type=float, default=0.44, help="Reject plans with palm targets farther forward than this torso x, meters.")
    parser.add_argument("--min-palm-target-z", type=float, default=0.10, help="Reject plans with palm targets lower than this torso z, meters.")
    parser.add_argument("--max-palm-target-z", type=float, default=0.18, help="Reject plans with palm targets higher than this torso z, meters.")
    parser.add_argument(
        "--hand-sequence",
        choices=("none", "open-only", "open-close"),
        default="open-only",
        help="Dex3 sequence during --execute. In flat-squeeze mode this sets flat palms at the intermediate waypoint.",
    )
    parser.add_argument("--grasp-method", choices=("pinch", "wrap", "side", "flat_palm"), default="flat_palm")
    parser.add_argument("--camera-target-x", type=float, default=DEFAULT_CAMERA_X)
    parser.add_argument("--camera-target-y", type=float, default=DEFAULT_CAMERA_Y)
    parser.add_argument("--camera-target-z", type=float, default=DEFAULT_CAMERA_Z)
    parser.add_argument("--camera-pitch-down-deg", type=float, default=DEFAULT_CAMERA_PITCH_DOWN_DEG)
    parser.add_argument("--urdf", default="config/g1_body29_hand14.urdf")
    parser.add_argument("--base-link", default="torso_link")
    parser.add_argument("--left-tip-link", default="left_hand_palm_link")
    parser.add_argument("--right-tip-link", default="right_hand_palm_link")
    parser.add_argument("--left-seed", default="", help="Comma-separated 7 left arm joint seed values.")
    parser.add_argument("--right-seed", default="", help="Comma-separated 7 right arm joint seed values.")
    parser.add_argument("--iterations", type=int, default=180)
    parser.add_argument("--max-error", type=float, default=0.045)
    parser.add_argument("--network-interface", default="eth0")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--pregrasp-duration", type=float, default=4.0)
    parser.add_argument("--execute", action="store_true", help="Actually move both arms. Without this, only calculate and print.")
    parser.add_argument("--obstacle-check", choices=("none", "json", "realsense"), default="none")
    parser.add_argument("--obstacle-json", default="")
    parser.add_argument("--obstacle-save-json", default="runs/box_obstacle_cloud_latest.json")
    parser.add_argument("--obstacle-clearance", type=float, default=0.07)
    parser.add_argument("--obstacle-extra-clearance", type=float, default=0.03)
    parser.add_argument("--obstacle-samples-per-leg", type=int, default=24)
    parser.add_argument("--obstacle-capture-s", type=float, default=0.3)
    parser.add_argument("--obstacle-stride-px", type=int, default=10)
    parser.add_argument("--obstacle-min-depth", type=float, default=0.15)
    parser.add_argument("--obstacle-max-depth", type=float, default=1.50)
    parser.add_argument("--obstacle-voxel-size", type=float, default=0.025)
    parser.add_argument("--obstacle-max-points", type=int, default=2500)
    parser.add_argument("--obstacle-exclude-target-radius", type=float, default=0.18)
    parser.add_argument("--obstacle-early-accept-distance", type=float, default=0.20)
    parser.add_argument(
        "--min-open-via-up",
        type=float,
        default=0.10,
        help="Minimum upward offset for the intermediate waypoint where flat palms may open.",
    )
    parser.add_argument(
        "--min-open-via-side-extra",
        type=float,
        default=0.12,
        help="Minimum extra side offset for the intermediate waypoint where flat palms may open.",
    )
    parser.add_argument("--hand-envelope-length", type=float, default=0.10, help="Conservative hand length for open-hand collision checks, meters.")
    parser.add_argument("--hand-envelope-width", type=float, default=0.09, help="Conservative hand width for open-hand collision checks, meters.")
    parser.add_argument("--hand-envelope-clearance", type=float, default=0.03, help="Extra clearance around the hand envelope, meters.")
    parser.add_argument("--avoidance-mode", choices=("none", "basic"), default="basic")
    parser.add_argument("--avoidance-up-values", default="0.10,0.18,0.28")
    parser.add_argument("--avoidance-side-extra-values", default="0.00,0.06,0.12")
    parser.add_argument("--avoidance-forward-values", default="-0.08,-0.16,0.0")
    parser.add_argument("--avoidance-max-error", type=float, default=0.045)
    parser.add_argument("--avoidance-iterations", type=int, default=60)
    parser.add_argument("--avoidance-max-candidates", type=int, default=24)
    parser.add_argument("--avoidance-replans", type=int, default=2)
    parser.add_argument("--avoidance-replan-scale", type=float, default=1.5)
    parser.add_argument("--snapshot-file", default="before_box_two_hand.json")
    parser.add_argument("--path-file", default="runs/box_two_hand_last_path.json")
    parser.add_argument("--save-json", default="runs/box_two_hand_latest.json")
    parser.add_argument("--log-file", default="box_two_hand.log")
    return parser.parse_args()


def configure_logging(log_file: str) -> None:
    handlers = [logging.StreamHandler()]
    if log_file:
        handlers.append(logging.FileHandler(log_file, encoding="utf-8"))
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s", handlers=handlers)


def run(args: argparse.Namespace) -> int:
    targets = build_box_targets(args)
    bounds_code = validate_palm_target_bounds(args, targets, "approach")
    if bounds_code != 0:
        save_report(args.save_json, build_bounds_blocked_report(args, targets, "approach"))
        return bounds_code
    model = load_urdf(args.urdf)
    left_chain = path_to_link(model, args.base_link, args.left_tip_link)
    right_chain = path_to_link(model, args.base_link, args.right_tip_link)

    controller: Optional[SafeArmPoseController] = None
    snapshot: Optional[Dict[str, List[float]]] = None
    if args.execute:
        controller = SafeArmPoseController(args.network_interface, duration_s=args.pregrasp_duration)
        snapshot = controller.snapshot()
        save_snapshot(args.snapshot_file, snapshot)

    left_seed = parse_joint_pose(args.left_seed) if args.left_seed else list(LEFT_SOFT_PREGRASP)
    right_seed = parse_joint_pose(args.right_seed) if args.right_seed else list(RIGHT_SOFT_PREGRASP)
    logging.info(
        "BOX_TWO_HAND planning seeds source=%s; execute snapshot is used only as path start",
        "explicit" if args.left_seed or args.right_seed else "soft_pregrasp",
    )
    left_q, left_error, left_pos = solve_hand_target(left_chain, targets.left, left_seed, args.iterations)
    right_q, right_error, right_pos = solve_hand_target(right_chain, targets.right, right_seed, args.iterations)
    if left_error > args.max_error or right_error > args.max_error:
        rebalance = find_rebalanced_grasp_y(
            args,
            targets,
            left_chain,
            right_chain,
            left_seed,
            right_seed,
        )
        if rebalance is not None:
            targets, left_q, left_error, left_pos, right_q, right_error, right_pos = rebalance

    report = {
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "box": {
            "width_m": args.box_width,
            "height_m": args.box_height,
            "depth_m": args.box_depth,
            "side_clearance_m": args.side_clearance,
            "grasp_strategy": args.grasp_strategy,
            "squeeze_side_clearance_m": args.squeeze_side_clearance,
            "squeeze_duration_s": args.squeeze_duration,
            "max_two_hand_center_y_m": args.max_two_hand_center_y,
            "grasp_center_y_mode": args.grasp_center_y_mode,
            "grasp_center_y_limit_m": args.grasp_center_y_limit,
            "palm_forward_offset_m": args.palm_forward_offset,
            "palm_up_offset_m": args.palm_up_offset,
            "max_palm_target_x_m": args.max_palm_target_x,
            "min_palm_target_z_m": args.min_palm_target_z,
            "max_palm_target_z_m": args.max_palm_target_z,
            "hand_sequence": args.hand_sequence,
            "grasp_method": args.grasp_method,
        },
        "targets": {
            "detected_center": pose_json(targets.detected_center),
            "grasp_center": pose_json(targets.grasp_center),
            "left": pose_json(targets.left),
            "right": pose_json(targets.right),
        },
        "left": {
            "tip_link": args.left_tip_link,
            "seed": left_seed,
            "ik_values": left_q,
            "ik_error_m": left_error,
            "ik_position": pose_tuple_json(left_pos),
        },
        "right": {
            "tip_link": args.right_tip_link,
            "seed": right_seed,
            "ik_values": right_q,
            "ik_error_m": right_error,
            "ik_position": pose_tuple_json(right_pos),
        },
    }
    save_report(args.save_json, report)
    print_report(report)

    if left_error > args.max_error or right_error > args.max_error:
        print_alignment_hint(args, targets, left_error, right_error)
        logging.error(
            "BOX_TWO_HAND IK error too high left=%.4f right=%.4f max=%.4f; not executing",
            left_error,
            right_error,
            args.max_error,
        )
        return 2

    left_path: List[List[float]] = [list(left_seed), list(left_seed), left_q]
    right_path: List[List[float]] = [list(right_seed), list(right_seed), right_q]
    if args.grasp_strategy == "flat-squeeze":
        squeeze_targets = build_box_targets(args, side_clearance=float(args.squeeze_side_clearance))
        bounds_code = validate_palm_target_bounds(args, squeeze_targets, "squeeze")
        if bounds_code != 0:
            return bounds_code
        left_squeeze_q, left_squeeze_error, _ = solve_hand_target(left_chain, squeeze_targets.left, left_q, args.iterations)
        right_squeeze_q, right_squeeze_error, _ = solve_hand_target(right_chain, squeeze_targets.right, right_q, args.iterations)
        logging.info(
            "BOX_TWO_HAND squeeze targets left=%s right=%s errors left=%.4f right=%.4f",
            squeeze_targets.left,
            squeeze_targets.right,
            left_squeeze_error,
            right_squeeze_error,
        )
        print("LEFT_SQUEEZE_TARGET_TORSO", json.dumps(pose_json(squeeze_targets.left), ensure_ascii=False))
        print("RIGHT_SQUEEZE_TARGET_TORSO", json.dumps(pose_json(squeeze_targets.right), ensure_ascii=False))
        print("LEFT_SQUEEZE_IK_ERROR_M", round(float(left_squeeze_error), 5))
        print("RIGHT_SQUEEZE_IK_ERROR_M", round(float(right_squeeze_error), 5))
        if left_squeeze_error > args.max_error or right_squeeze_error > args.max_error:
            logging.error(
                "BOX_TWO_HAND squeeze IK error too high left=%.4f right=%.4f max=%.4f; not executing",
                left_squeeze_error,
                right_squeeze_error,
                args.max_error,
            )
            return 2
        left_path.append(left_squeeze_q)
        right_path.append(right_squeeze_q)
    if args.obstacle_check != "none":
        obstacle_code, selected_left, selected_right = check_two_hand_obstacles(
            args,
            targets,
            left_chain,
            right_chain,
            left_path,
            right_path,
        )
        if obstacle_code != 0:
            return obstacle_code
        left_path = selected_left
        right_path = selected_right

    if not args.execute:
        logging.info("BOX_TWO_HAND print-only result=OK")
        return 0

    if controller is None or snapshot is None:
        raise RuntimeError("execute requested but arm controller was not initialized")
    save_motion_path(args.path_file, snapshot, left_path, right_path)
    dex3: Optional[Dex3HandController] = None
    hands_opened = False
    open_before_waypoint_index = max(1, len(left_path) - 2) if args.grasp_strategy == "flat-squeeze" else len(left_path) - 1
    for index, (left_waypoint, right_waypoint) in enumerate(zip(left_path[1:], right_path[1:]), start=1):
        if args.hand_sequence in ("open-only", "open-close") and not hands_opened and index == open_before_waypoint_index:
            dex3 = Dex3HandController(UnitreeSdk2Dex3Backend(args.network_interface))
            if args.grasp_strategy == "flat-squeeze":
                logging.info(
                    "BOX_TWO_HAND hand sequence 1/1: set flat palms before final approach waypoint %d/%d",
                    index,
                    len(left_path) - 1,
                )
                open_ack = dex3.flat_both()
            else:
                logging.info(
                    "BOX_TWO_HAND hand sequence 1/2: open both hands before final approach waypoint %d/%d",
                    index,
                    len(left_path) - 1,
                )
                open_ack = dex3.open_both()
            logging.info("BOX_TWO_HAND hand setup ok=%s message=%s", open_ack.ok, open_ack.message)
            if not open_ack.ok:
                return 4
            hands_opened = True
        if args.grasp_strategy == "flat-squeeze" and index == len(left_path) - 1:
            controller.duration_s = max(0.2, float(args.squeeze_duration))
        else:
            controller.duration_s = max(0.2, float(args.pregrasp_duration if index == 1 else args.duration))
        logging.info(
            "BOX_TWO_HAND execute waypoint %d/%d left=%s right=%s duration=%.2f",
            index,
            len(left_path) - 1,
            [round(value, 5) for value in left_waypoint],
            [round(value, 5) for value in right_waypoint],
            controller.duration_s,
        )
        ack = controller.move_poses(tuple(left_waypoint), tuple(right_waypoint))  # type: ignore[arg-type]
        logging.info("BOX_TWO_HAND waypoint %d/%d ok=%s message=%s", index, len(left_path) - 1, ack.ok, ack.message)
        if not ack.ok:
            return 3
    if args.grasp_strategy == "finger-close" and args.hand_sequence == "open-close" and dex3 is not None and hands_opened:
        logging.info("BOX_TWO_HAND hand sequence 2/2: close both hands method=%s", args.grasp_method)
        close_ack = dex3.close_both(args.grasp_method)
        logging.info("BOX_TWO_HAND close both ok=%s message=%s", close_ack.ok, close_ack.message)
        if not close_ack.ok:
            return 5
    if args.grasp_strategy == "flat-squeeze" and dex3 is not None and hands_opened:
        logging.info("BOX_TWO_HAND hand sequence final: squeeze thumbs after arm squeeze")
        thumb_ack = dex3.flat_thumb_squeeze_both()
        logging.info("BOX_TWO_HAND thumb squeeze ok=%s message=%s", thumb_ack.ok, thumb_ack.message)
        if not thumb_ack.ok:
            return 5
    return 0


def build_box_targets(args: argparse.Namespace, side_clearance: Optional[float] = None) -> BoxTargets:
    detected_center = read_box_center(args)
    grasp_center = choose_grasp_center(args, detected_center)
    return build_box_targets_from_center(args, detected_center, grasp_center, side_clearance)


def build_box_targets_from_center(
    args: argparse.Namespace,
    detected_center: PoseXYZ,
    grasp_center: PoseXYZ,
    side_clearance: Optional[float] = None,
) -> BoxTargets:
    half_width = max(0.0, float(args.box_width) / 2.0)
    side_offset = max(0.0, half_width + float(args.side_clearance if side_clearance is None else side_clearance))
    left = PoseXYZ(
        x_m=grasp_center.x_m + float(args.palm_forward_offset),
        y_m=grasp_center.y_m + side_offset,
        z_m=grasp_center.z_m + float(args.palm_up_offset),
    )
    right = PoseXYZ(
        x_m=grasp_center.x_m + float(args.palm_forward_offset),
        y_m=grasp_center.y_m - side_offset,
        z_m=grasp_center.z_m + float(args.palm_up_offset),
    )
    logging.info(
        "BOX targets detected_center=%s grasp_center=%s width=%.3f side_offset=%.3f left=%s right=%s",
        detected_center,
        grasp_center,
        args.box_width,
        side_offset,
        left,
        right,
    )
    if abs(detected_center.y_m) > float(args.max_two_hand_center_y):
        logging.warning(
            "BOX detected center y=%.3f is outside recommended two-hand corridor +/-%.3f; grasp_center_y_mode=%s grasp_y=%.3f",
            detected_center.y_m,
            args.max_two_hand_center_y,
            args.grasp_center_y_mode,
            grasp_center.y_m,
        )
    return BoxTargets(detected_center=detected_center, grasp_center=grasp_center, left=left, right=right)


def find_rebalanced_grasp_y(
    args: argparse.Namespace,
    targets: BoxTargets,
    left_chain,
    right_chain,
    left_seed: List[float],
    right_seed: List[float],
) -> Optional[Tuple[BoxTargets, List[float], float, Tuple[float, float, float], List[float], float, Tuple[float, float, float]]]:
    if args.grasp_center_y_mode != "clamp":
        return None
    start_y = float(targets.grasp_center.y_m)
    if abs(start_y) < 1e-6:
        return None

    best: Optional[Tuple[float, BoxTargets, List[float], float, Tuple[float, float, float], List[float], float, Tuple[float, float, float]]] = None
    for step in range(1, 9):
        y_m = start_y * (1.0 - step / 8.0)
        candidate_targets = build_box_targets_from_center(
            args,
            targets.detected_center,
            PoseXYZ(targets.detected_center.x_m, y_m, targets.detected_center.z_m),
        )
        if validate_palm_target_bounds(args, candidate_targets, "approach_rebalance") != 0:
            continue
        left_q, left_error, left_pos = solve_hand_target(left_chain, candidate_targets.left, left_seed, args.iterations)
        right_q, right_error, right_pos = solve_hand_target(right_chain, candidate_targets.right, right_seed, args.iterations)
        max_error = max(left_error, right_error)
        logging.info(
            "BOX_TWO_HAND rebalance grasp_y candidate old=%.3f new=%.3f left_error=%.4f right_error=%.4f",
            start_y,
            y_m,
            left_error,
            right_error,
        )
        if best is None or max_error < best[0]:
            best = (max_error, candidate_targets, left_q, left_error, left_pos, right_q, right_error, right_pos)
        if left_error <= args.max_error and right_error <= args.max_error:
            print("BOX_IK_REBALANCED_GRASP_Y", f"old={start_y:.3f}", f"new={y_m:.3f}", f"left_error={left_error:.4f}", f"right_error={right_error:.4f}")
            logging.info("BOX_TWO_HAND accepted rebalanced grasp_y old=%.3f new=%.3f", start_y, y_m)
            return candidate_targets, left_q, left_error, left_pos, right_q, right_error, right_pos

    if best is not None:
        max_error, candidate_targets, _, left_error, _, _, right_error, _ = best
        logging.info(
            "BOX_TWO_HAND no acceptable rebalanced grasp_y best=%.3f left_error=%.4f right_error=%.4f max_error=%.4f",
            candidate_targets.grasp_center.y_m,
            left_error,
            right_error,
            max_error,
        )
    return None


def choose_grasp_center(args: argparse.Namespace, detected_center: PoseXYZ) -> PoseXYZ:
    mode = args.grasp_center_y_mode
    if mode == "detected":
        y_m = detected_center.y_m
    elif mode == "clamp":
        limit = max(0.0, float(args.grasp_center_y_limit))
        y_m = max(-limit, min(limit, detected_center.y_m))
    else:
        y_m = 0.0
    return PoseXYZ(detected_center.x_m, y_m, detected_center.z_m)


def print_alignment_hint(args: argparse.Namespace, targets: BoxTargets, left_error: float, right_error: float) -> None:
    center_y = targets.detected_center.y_m
    corridor = float(args.max_two_hand_center_y)
    side_offset = float(args.box_width) * 0.5 + float(args.side_clearance)
    print(
        "BOX_IK_BLOCKED",
        json.dumps(
            {
                "reason": "two_hand_target_out_of_reach_or_bad_alignment",
                "left_ik_error_m": left_error,
                "right_ik_error_m": right_error,
                "max_error_m": args.max_error,
                "center_y_m": center_y,
                "grasp_center_y_m": targets.grasp_center.y_m,
                "grasp_center_y_mode": args.grasp_center_y_mode,
                "recommended_center_y_abs_max_m": corridor,
                "left_target_y_m": targets.left.y_m,
                "right_target_y_m": targets.right.y_m,
                "side_offset_m": side_offset,
                "what_to_do": "check x/z height and hand spacing; bbox/depth worked, but IK cannot place both palms at these targets",
            },
            ensure_ascii=False,
        ),
    )
    if args.camera_x is not None:
        desired_camera_x = float(args.camera_target_y) - 0.0
        current_camera_x = float(args.camera_x)
        print(
            "BOX_CAMERA_ALIGNMENT_HINT",
            json.dumps(
                {
                    "current_camera_x_m": current_camera_x,
                    "desired_camera_x_for_center_y_0_m": desired_camera_x,
                    "delta_camera_x_m": desired_camera_x - current_camera_x,
                    "note": "diagnostic only; with grasp_center_y_mode=zero the two-hand grasp uses torso center even if detected bbox center is shifted",
                },
                ensure_ascii=False,
            ),
        )


def read_box_center(args: argparse.Namespace) -> PoseXYZ:
    if args.torso_x is not None and args.torso_y is not None and args.torso_z is not None:
        return PoseXYZ(float(args.torso_x), float(args.torso_y), float(args.torso_z))
    if args.camera_x is None or args.camera_y is None or args.camera_z is None:
        raise ValueError("Provide either --torso-x/--torso-y/--torso-z or --camera-x/--camera-y/--camera-z")
    transform = CameraToArmConfig(
        camera_target_x_m=args.camera_target_x,
        camera_target_y_m=args.camera_target_y,
        camera_target_z_m=args.camera_target_z,
        camera_pitch_down_deg=args.camera_pitch_down_deg,
    )
    return yolo_camera_point_to_arm_target(
        CameraPoint(float(args.camera_x), float(args.camera_y), float(args.camera_z)),
        transform,
        GraspOffset(),
    )


def solve_hand_target(chain, target: PoseXYZ, seed: List[float], iterations: int) -> Tuple[List[float], float, Tuple[float, float, float]]:
    movable = active_joints(chain)
    if len(movable) != 7:
        raise ValueError(f"Expected 7 active arm joints, got {len(movable)}")
    q, error = solve_ik_position(
        chain,
        (target.x_m, target.y_m, target.z_m),
        seed_values=seed,
        iterations=iterations,
    )
    pos = position(fk(chain, {joint.name: q[index] for index, joint in enumerate(movable)}))
    return q, error, pos


def check_two_hand_obstacles(
    args: argparse.Namespace,
    targets: BoxTargets,
    left_chain,
    right_chain,
    left_direct: List[List[float]],
    right_direct: List[List[float]],
) -> Tuple[int, List[List[float]], List[List[float]]]:
    if args.obstacle_check == "json":
        if not args.obstacle_json:
            raise ValueError("--obstacle-json is required when --obstacle-check json")
        cloud = load_obstacle_cloud_json(args.obstacle_json)
    else:
        cloud = capture_realsense_obstacle_cloud(
            CameraToArmConfig(
                camera_target_x_m=args.camera_target_x,
                camera_target_y_m=args.camera_target_y,
                camera_target_z_m=args.camera_target_z,
                camera_pitch_down_deg=args.camera_pitch_down_deg,
            ),
            targets.detected_center,
            capture_s=args.obstacle_capture_s,
            stride_px=args.obstacle_stride_px,
            min_depth_m=args.obstacle_min_depth,
            max_depth_m=args.obstacle_max_depth,
            voxel_size_m=args.obstacle_voxel_size,
            max_points=args.obstacle_max_points,
            exclude_target_radius_m=args.obstacle_exclude_target_radius,
        )
    cloud = remove_box_body_points(cloud, args, targets)

    if args.obstacle_save_json:
        save_obstacle_cloud_json(args.obstacle_save_json, cloud)

    required_distance = float(args.obstacle_clearance) + float(args.obstacle_extra_clearance)
    distance_accept_eps = 0.005
    best_clear: Optional[Tuple[Tuple[float, float, float, float, float, float], str, List[List[float]], List[List[float]], float]] = None
    best_weak: Optional[Tuple[str, float]] = None
    best_blocked: Optional[Tuple[str, float, str]] = None
    best_ineligible: Optional[Tuple[str, float, str]] = None

    for attempt in range(max(1, int(args.avoidance_replans))):
        scale = float(args.avoidance_replan_scale) ** attempt
        candidates = build_two_hand_path_candidates(
            args,
            targets,
            left_chain,
            right_chain,
            left_direct,
            right_direct,
            scale=scale,
            attempt=attempt,
        )
        logging.info(
            "BOX_TWO_HAND obstacle replan attempt=%d candidates=%d required_distance=%.3f",
            attempt + 1,
            len(candidates),
            required_distance,
        )
        for index, (name, left_path, right_path) in enumerate(candidates, start=1):
            left_result = check_joint_path_collision(
                left_chain,
                left_path,
                cloud.points,
                clearance_m=args.obstacle_clearance,
                samples_per_leg=args.obstacle_samples_per_leg,
            )
            right_result = check_joint_path_collision(
                right_chain,
                right_path,
                cloud.points,
                clearance_m=args.obstacle_clearance,
                samples_per_leg=args.obstacle_samples_per_leg,
            )
            left_hand_result = check_open_hand_envelope_path(args, left_chain, left_path, cloud.points)
            right_hand_result = check_open_hand_envelope_path(args, right_chain, right_path, cloud.points)
            min_distance = min(
                left_result.min_distance_m,
                right_result.min_distance_m,
                left_hand_result.min_distance_m,
                right_hand_result.min_distance_m,
            )
            ok = left_result.ok and right_result.ok and left_hand_result.ok and right_hand_result.ok
            logging.info(
                "BOX_TWO_HAND obstacle candidate attempt=%d index=%d/%d name=%s ok=%s min_distance=%.3f left='%s' right='%s' left_hand='%s' right_hand='%s' left_obstacle=%s right_obstacle=%s left_hand_obstacle=%s right_hand_obstacle=%s",
                attempt + 1,
                index,
                len(candidates),
                name,
                ok,
                min_distance,
                left_result.message,
                right_result.message,
                left_hand_result.message,
                right_hand_result.message,
                format_obstacle_point(left_result.obstacle),
                format_obstacle_point(right_result.obstacle),
                format_obstacle_point(left_hand_result.obstacle),
                format_obstacle_point(right_hand_result.obstacle),
            )
            if ok and min_distance + distance_accept_eps >= required_distance:
                eligible, ineligible_reason = obstacle_candidate_open_via_eligible(args, name)
                if not eligible:
                    if best_ineligible is None or min_distance > best_ineligible[1]:
                        best_ineligible = (name, min_distance, ineligible_reason)
                    logging.info(
                        "BOX_TWO_HAND obstacle candidate not eligible for hand-open via name=%s min_distance=%.3f reason=%s",
                        name,
                        min_distance,
                        ineligible_reason,
                    )
                    continue
                preference_key = obstacle_candidate_preference_key(name, min_distance)
                if best_clear is None or preference_key < best_clear[0]:
                    best_clear = (preference_key, name, left_path, right_path, min_distance)
                if (
                    args.obstacle_early_accept_distance > 0
                    and name != "direct"
                    and min_distance + distance_accept_eps >= required_distance
                ):
                    print("BOX_OBSTACLE_CLEAR", f"path={name}", f"min_distance={min_distance:.3f}m", f"required={required_distance:.3f}m")
                    logging.info("BOX_TWO_HAND early selected via obstacle-safe path=%s min_distance=%.3f", name, min_distance)
                    return 0, left_path, right_path
                continue
            if ok:
                if best_weak is None or min_distance > best_weak[1]:
                    best_weak = (name, min_distance)
            else:
                reason = first_collision_reason(left_result, right_result, left_hand_result, right_hand_result)
                if best_blocked is None or min_distance > best_blocked[1]:
                    best_blocked = (name, min_distance, reason)

    if best_clear is not None:
        _, name, left_path, right_path, min_distance = best_clear
        print("BOX_OBSTACLE_CLEAR", f"path={name}", f"min_distance={min_distance:.3f}m", f"required={required_distance:.3f}m")
        logging.info("BOX_TWO_HAND selected best obstacle-safe path=%s min_distance=%.3f", name, min_distance)
        return 0, left_path, right_path
    if best_weak is not None:
        name, min_distance = best_weak
        print("BOX_NO_SAFE_PATH", f"best weak path={name} min_distance={min_distance:.3f}m required={required_distance:.3f}m")
        return 6, left_direct, right_direct
    if best_ineligible is not None:
        name, min_distance, reason = best_ineligible
        print("BOX_NO_SAFE_PATH", f"best ineligible path={name} min_distance={min_distance:.3f}m {reason}")
        return 6, left_direct, right_direct
    if best_blocked is not None:
        name, min_distance, reason = best_blocked
        print("BOX_NO_SAFE_PATH", f"best blocked path={name} min_distance={min_distance:.3f}m {reason}")
        return 6, left_direct, right_direct
    print("BOX_NO_SAFE_PATH no candidates")
    return 6, left_direct, right_direct


def remove_box_body_points(cloud: ObstacleCloud, args: argparse.Namespace, targets: BoxTargets) -> ObstacleCloud:
    half_x = max(0.0, float(args.box_depth) * 0.5 + 0.04)
    # The obstacle cloud often contains the visible side face of the box exactly
    # where the palms must make contact. Keep the vertical box-height gate so
    # table points below the box are still treated as obstacles.
    half_y = max(0.0, float(args.box_width) * 0.5 + float(args.side_clearance) + 0.04)
    z_min = targets.detected_center.z_m - max(0.0, float(args.box_height) * 0.5 + 0.02)
    z_max = targets.detected_center.z_m + max(0.0, float(args.box_height) * 0.5 + 0.04)
    center_x = targets.grasp_center.x_m
    center_y = targets.grasp_center.y_m

    kept = []
    removed = 0
    for point in cloud.points:
        inside_box_body = (
            abs(point[0] - center_x) <= half_x
            and abs(point[1] - center_y) <= half_y
            and z_min <= point[2] <= z_max
        )
        if inside_box_body:
            removed += 1
        else:
            kept.append(point)
    logging.info(
        "OBSTACLE box body filter removed=%d kept=%d center=(%.3f,%.3f) x_half=%.3f y_half=%.3f z=[%.3f,%.3f]",
        removed,
        len(kept),
        center_x,
        center_y,
        half_x,
        half_y,
        z_min,
        z_max,
    )
    return ObstacleCloud(points=kept, source=f"{cloud.source}+box_body_filtered")


def format_obstacle_point(point: Optional[Tuple[float, float, float]]) -> str:
    if point is None:
        return "none"
    return f"x={point[0]:.3f},y={point[1]:.3f},z={point[2]:.3f}"


def obstacle_candidate_preference_key(name: str, min_distance: float) -> Tuple[float, float, float, float, float, float]:
    if name == "direct":
        return (1.0, -float(min_distance), 0.0, 1.0, 0.0, 0.0)
    up_m = abs(_candidate_value(name, "up"))
    side_m = abs(_candidate_value(name, "side"))
    forward_m = _candidate_value(name, "forward")
    if abs(forward_m + 0.08) < 1e-6:
        forward_rank = 0.0
    elif abs(forward_m) < 1e-6:
        forward_rank = 1.0
    elif forward_m < -0.08:
        forward_rank = 2.0
    else:
        forward_rank = 3.0
    return (0.0, -float(min_distance), up_m, side_m, forward_rank, abs(forward_m + 0.08))


def obstacle_candidate_open_via_eligible(args: argparse.Namespace, name: str) -> Tuple[bool, str]:
    if args.grasp_strategy != "flat-squeeze" or args.hand_sequence not in ("open-only", "open-close"):
        return True, ""
    min_up = max(0.0, float(args.min_open_via_up))
    if name == "direct":
        return False, f"flat palms need an upper intermediate waypoint before opening; direct has no via, required via_up>={min_up:.3f}m"
    up_m = _candidate_value(name, "up")
    if up_m + 1e-9 < min_up:
        return False, f"flat palms open only at upper via; path via_up={up_m:.3f}m required>={min_up:.3f}m"
    min_side = max(0.0, float(args.min_open_via_side_extra))
    side_m = abs(_candidate_value(name, "side"))
    if side_m + 1e-9 < min_side:
        return False, f"flat palms open only at side via; path via_side={side_m:.3f}m required>={min_side:.3f}m"
    return True, ""


def first_collision_reason(*results: CollisionCheckResult) -> str:
    for result in results:
        if not result.ok:
            return result.message
    return results[0].message if results else "unknown collision reason"


def check_open_hand_envelope_path(
    args: argparse.Namespace,
    chain,
    q_waypoints: List[List[float]],
    obstacle_points: List[Tuple[float, float, float]],
) -> CollisionCheckResult:
    if args.grasp_strategy != "flat-squeeze" or args.hand_sequence not in ("open-only", "open-close"):
        return CollisionCheckResult(True, "open hand envelope disabled", float("inf"))
    if not obstacle_points:
        return CollisionCheckResult(True, "open hand envelope: no obstacle points", float("inf"))
    if len(q_waypoints) < 3:
        return CollisionCheckResult(False, "open hand envelope needs an intermediate waypoint before opening", 0.0)

    radius_m = hand_envelope_radius_m(args)
    check_from_waypoint = 0
    min_distance = float("inf")
    min_sample = -1
    min_obstacle: Optional[Tuple[float, float, float]] = None
    sample_index = 0

    for leg_index in range(check_from_waypoint, len(q_waypoints) - 1):
        start_q = q_waypoints[leg_index]
        end_q = q_waypoints[leg_index + 1]
        for q in sample_joint_leg(start_q, end_q, max(2, int(args.obstacle_samples_per_leg))):
            palm = palm_position_for_q(chain, q)
            for obstacle in obstacle_points:
                distance_m = point_distance(palm, obstacle)
                if distance_m < min_distance:
                    min_distance = distance_m
                    min_sample = sample_index
                    min_obstacle = obstacle
                if distance_m < radius_m - 0.005:
                    return CollisionCheckResult(
                        ok=False,
                        message=(
                            f"hand envelope collision risk: sample={sample_index} "
                            f"distance={distance_m:.3f}m radius={radius_m:.3f}m "
                            f"hand={args.hand_envelope_length:.3f}x{args.hand_envelope_width:.3f}m"
                        ),
                        min_distance_m=distance_m,
                        sample_index=sample_index,
                        segment_name="open_hand_envelope",
                        obstacle=obstacle,
                    )
            sample_index += 1

    return CollisionCheckResult(
        ok=True,
        message=f"hand envelope clear: min_distance={min_distance:.3f}m radius={radius_m:.3f}m",
        min_distance_m=min_distance,
        sample_index=min_sample,
        segment_name="open_hand_envelope",
        obstacle=min_obstacle,
    )


def hand_envelope_radius_m(args: argparse.Namespace) -> float:
    hand_reach = max(max(0.0, float(args.hand_envelope_length)), max(0.0, float(args.hand_envelope_width)))
    return hand_reach + max(0.0, float(args.hand_envelope_clearance))


def palm_position_for_q(chain, q: List[float]) -> Tuple[float, float, float]:
    movable = active_joints(chain)
    values_by_name = {joint.name: float(q[index]) for index, joint in enumerate(movable)}
    return position(fk(chain, values_by_name))


def sample_joint_leg(start_q: List[float], end_q: List[float], count: int) -> List[List[float]]:
    if len(start_q) != len(end_q):
        raise ValueError("joint waypoints must have the same length")
    return [
        [float(a) * (1.0 - ratio) + float(b) * ratio for a, b in zip(start_q, end_q)]
        for ratio in (index / (count - 1) for index in range(count))
    ]


def point_distance(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def _candidate_value(name: str, field: str) -> float:
    match = re.search(rf"_{field}([+-][0-9]+(?:\.[0-9]+)?)", name)
    return float(match.group(1)) if match else 0.0


def build_two_hand_path_candidates(
    args: argparse.Namespace,
    targets: BoxTargets,
    left_chain,
    right_chain,
    left_direct: List[List[float]],
    right_direct: List[List[float]],
    *,
    scale: float,
    attempt: int,
) -> List[Tuple[str, List[List[float]], List[List[float]]]]:
    candidates: List[Tuple[str, List[List[float]], List[List[float]]]] = []
    if attempt == 0:
        candidates.append(("direct", left_direct, right_direct))
    if args.avoidance_mode == "none":
        return candidates

    up_values = scaled_positive_values(parse_float_list(args.avoidance_up_values), scale)
    side_values = scaled_positive_values(parse_float_list(args.avoidance_side_extra_values), scale)
    forward_values = scaled_values(parse_float_list(args.avoidance_forward_values), scale)
    side_offset = float(args.box_width) * 0.5 + float(args.side_clearance)

    for up_m in up_values:
        for side_extra_m in side_values:
            for forward_m in forward_values:
                left_waypoint = PoseXYZ(
                    targets.grasp_center.x_m + float(args.palm_forward_offset) + forward_m,
                    targets.grasp_center.y_m + side_offset + side_extra_m,
                    targets.grasp_center.z_m + float(args.palm_up_offset) + up_m,
                )
                right_waypoint = PoseXYZ(
                    targets.grasp_center.x_m + float(args.palm_forward_offset) + forward_m,
                    targets.grasp_center.y_m - side_offset - side_extra_m,
                    targets.grasp_center.z_m + float(args.palm_up_offset) + up_m,
                )
                name = f"attempt{attempt + 1}_via_up{up_m:+.2f}_side{side_extra_m:+.2f}_forward{forward_m:+.2f}"
                left_mid = solve_waypoint_or_none(args, left_chain, left_waypoint, left_direct[0], name, "left")
                right_mid = solve_waypoint_or_none(args, right_chain, right_waypoint, right_direct[0], name, "right")
                if left_mid is None or right_mid is None:
                    continue
                candidates.append((name, [left_direct[0], left_mid] + left_direct[2:], [right_direct[0], right_mid] + right_direct[2:]))
                if len(candidates) >= max(1, int(args.avoidance_max_candidates)):
                    logging.info("BOX_TWO_HAND avoidance candidate limit reached max_candidates=%d", args.avoidance_max_candidates)
                    return candidates
    return candidates


def solve_waypoint_or_none(
    args: argparse.Namespace,
    chain,
    target: PoseXYZ,
    seed: List[float],
    name: str,
    side: str,
) -> Optional[List[float]]:
    q, error, _ = solve_hand_target(chain, target, seed, args.avoidance_iterations)
    if error > args.avoidance_max_error:
        logging.info(
            "BOX_TWO_HAND avoidance skip side=%s name=%s target=%s error=%.4f max=%.4f",
            side,
            name,
            target,
            error,
            args.avoidance_max_error,
        )
        return None
    logging.info("BOX_TWO_HAND avoidance waypoint ok side=%s name=%s target=%s error=%.4f", side, name, target, error)
    return q


def validate_palm_target_bounds(args: argparse.Namespace, targets: BoxTargets, phase: str) -> int:
    violations: List[str] = []
    max_x = float(args.max_palm_target_x)
    min_z = float(args.min_palm_target_z)
    max_z = float(args.max_palm_target_z)
    for side, target in (("left", targets.left), ("right", targets.right)):
        if target.x_m > max_x:
            violations.append(f"{side}.x={target.x_m:.3f}>{max_x:.3f}")
        if target.z_m < min_z:
            violations.append(f"{side}.z={target.z_m:.3f}<{min_z:.3f}")
        if target.z_m > max_z:
            violations.append(f"{side}.z={target.z_m:.3f}>{max_z:.3f}")
    if not violations:
        return 0
    message = ",".join(violations)
    print("BOX_TARGET_BOUNDS_BLOCKED", f"phase={phase}", message)
    logging.error("BOX_TWO_HAND target bounds blocked phase=%s violations=%s", phase, message)
    return 7


def build_bounds_blocked_report(args: argparse.Namespace, targets: BoxTargets, phase: str) -> Dict[str, object]:
    return {
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "result": "target_bounds_blocked",
        "phase": phase,
        "bounds": {
            "max_palm_target_x_m": args.max_palm_target_x,
            "min_palm_target_z_m": args.min_palm_target_z,
            "max_palm_target_z_m": args.max_palm_target_z,
        },
        "targets": {
            "detected_center": pose_json(targets.detected_center),
            "grasp_center": pose_json(targets.grasp_center),
            "left": pose_json(targets.left),
            "right": pose_json(targets.right),
        },
    }


def parse_float_list(text: str) -> List[float]:
    values = [float(part.strip()) for part in text.split(",") if part.strip()]
    if not values:
        raise ValueError("avoidance value list must not be empty")
    return values


def scaled_values(values: List[float], scale: float) -> List[float]:
    result: List[float] = []
    for value in values:
        scaled = value * scale
        if not any(abs(scaled - existing) < 1e-9 for existing in result):
            result.append(scaled)
    return result


def scaled_positive_values(values: List[float], scale: float) -> List[float]:
    return [max(0.0, value * scale) for value in values]


def parse_joint_pose(text: str) -> List[float]:
    values = [float(part.strip()) for part in text.split(",") if part.strip()]
    if len(values) != 7:
        raise ValueError("joint seed must contain exactly 7 comma-separated values")
    return values


def pose_json(pose: PoseXYZ) -> Dict[str, float]:
    return {"x_m": pose.x_m, "y_m": pose.y_m, "z_m": pose.z_m}


def pose_tuple_json(pose: Tuple[float, float, float]) -> Dict[str, float]:
    return {"x_m": pose[0], "y_m": pose[1], "z_m": pose[2]}


def save_report(path: str, report: Dict[str, object]) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    logging.info("BOX_TWO_HAND saved report path=%s", output)


def print_report(report: Dict[str, object]) -> None:
    print("BOX_DETECTED_CENTER_TORSO", json.dumps(report["targets"]["detected_center"], ensure_ascii=False))
    print("BOX_GRASP_CENTER_TORSO", json.dumps(report["targets"]["grasp_center"], ensure_ascii=False))
    print("LEFT_TARGET_TORSO", json.dumps(report["targets"]["left"], ensure_ascii=False))
    print("RIGHT_TARGET_TORSO", json.dumps(report["targets"]["right"], ensure_ascii=False))
    print("LEFT_IK_VALUES", [round(value, 5) for value in report["left"]["ik_values"]])
    print("LEFT_IK_ERROR_M", round(float(report["left"]["ik_error_m"]), 5))
    print("RIGHT_IK_VALUES", [round(value, 5) for value in report["right"]["ik_values"]])
    print("RIGHT_IK_ERROR_M", round(float(report["right"]["ik_error_m"]), 5))


def save_snapshot(path: str, snapshot: Dict[str, List[float]]) -> None:
    payload = {"created_at": time.strftime("%Y-%m-%d %H:%M:%S"), "snapshot": snapshot}
    Path(path).write_text(json.dumps(payload, indent=2), encoding="utf-8")
    logging.info("BOX_TWO_HAND saved start snapshot path=%s", path)


def save_motion_path(path: str, snapshot: Dict[str, List[float]], left_path: List[List[float]], right_path: List[List[float]]) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "snapshot": snapshot,
        "left_path": left_path,
        "right_path": right_path,
    }
    output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    logging.info("BOX_TWO_HAND saved motion path path=%s waypoints=%d", output, len(left_path))


def main() -> int:
    args = parse_args()
    configure_logging(args.log_file)
    try:
        return run(args)
    except Exception as exc:
        logging.exception("BOX_TWO_HAND failed: %s", exc)
        return 1


if __name__ == "__main__":
    sys.exit(main())
