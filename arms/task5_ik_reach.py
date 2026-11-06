from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from .cup_pipeline import parse_yolo_log
from .dex3_hand import Dex3HandController
from .dex3_sdk2_backend import UnitreeSdk2Dex3Backend
from .depth_obstacles import (
    CollisionCheckResult,
    capture_realsense_obstacle_cloud,
    check_joint_path_collision,
    load_obstacle_cloud_json,
    save_obstacle_cloud_json,
)
from .g1_arm_sdk2 import LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS, WAIST_JOINTS
from .move_arm_preset import SafeArmPoseController
from .record_arm_calibration import wait_low_state
from .safety import validate_hand_side
from .types import HandSide, PoseXYZ
from .urdf_kinematics import active_joints, fk, load_urdf, path_to_link, position, solve_ik_position
from .vision_kinematics import CameraPoint, CameraToArmConfig, GraspOffset, yolo_camera_point_to_arm_target

ArmSnapshot = Dict[str, List[float]]
JointPose = Tuple[float, float, float, float, float, float, float]

# Measured on 2026-08-17:
# torso -> right_shoulder from URDF origin of right_shoulder_pitch_joint:
#   (0.0039563, -0.10021, 0.24778)
# right_shoulder -> camera measured by tape:
#   (+0.115, +0.180, +0.080)
DEFAULT_CAMERA_X = 0.0039563 + 0.115
DEFAULT_CAMERA_Y = -0.10021 + 0.180
DEFAULT_CAMERA_Z = 0.24778 + 0.080
DEFAULT_CAMERA_PITCH_DOWN_DEG = 52.0

# Safer than the old right_cup_side_out_table76 pose. It moves the right arm
# away from the torso before any IK reach, but not as far sideways.
RIGHT_SOFT_PREGRASP: JointPose = (0.14, -0.46, 0.02, 0.98, -0.12, -0.17, 0.005)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="YOLO/RealSense point -> measured torso transform -> URDF IK -> optional SDK2 arm movement.")
    parser.add_argument("--hand", choices=("left", "right"), default="right")
    parser.add_argument("--camera-x", type=float, help="Camera X from YOLO/RealSense, meters.")
    parser.add_argument("--camera-y", type=float, help="Camera Y from YOLO/RealSense, meters.")
    parser.add_argument("--camera-z", type=float, help="Camera Z/depth from YOLO/RealSense, meters.")
    parser.add_argument("--yolo-log", default="")
    parser.add_argument("--min-confidence", type=float, default=0.20)
    parser.add_argument("--allow-low-confidence", action="store_true")
    parser.add_argument("--default-camera-y", type=float)
    parser.add_argument("--urdf", default="config/g1_body29_hand14.urdf")
    parser.add_argument("--base-link", default="torso_link")
    parser.add_argument("--tip-link", default="right_hand_palm_link")
    parser.add_argument("--camera-target-x", type=float, default=DEFAULT_CAMERA_X)
    parser.add_argument("--camera-target-y", type=float, default=DEFAULT_CAMERA_Y)
    parser.add_argument("--camera-target-z", type=float, default=DEFAULT_CAMERA_Z)
    parser.add_argument("--camera-pitch-down-deg", type=float, default=DEFAULT_CAMERA_PITCH_DOWN_DEG)
    parser.add_argument("--grasp-forward-offset", type=float, default=-0.20)
    parser.add_argument("--grasp-left-offset", type=float, default=-0.35)
    parser.add_argument("--grasp-up-offset", type=float, default=0.06)
    parser.add_argument("--seed", default="", help="Comma-separated 7 arm joint seed values. If omitted, the soft pregrasp pose is used.")
    parser.add_argument("--start-joints", default="", help="Comma-separated current 7 arm joints for no-execute planning without reading rt/lowstate.")
    parser.add_argument("--iterations", type=int, default=160)
    parser.add_argument("--max-error", type=float, default=0.025)
    parser.add_argument("--network-interface", default="eth0")
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--pregrasp-duration", type=float, default=4.0)
    parser.add_argument("--execute", action="store_true", help="Actually move the arm. Without this, only print IK.")
    parser.add_argument("--grip", choices=("none", "open", "close"), default="none")
    parser.add_argument("--obstacle-check", choices=("none", "json", "realsense"), default="none")
    parser.add_argument("--obstacle-json", default="", help="Obstacle cloud JSON path for --obstacle-check json.")
    parser.add_argument("--obstacle-save-json", default="runs/obstacle_cloud_latest.json")
    parser.add_argument("--obstacle-clearance", type=float, default=0.08, help="Minimum allowed distance from arm links to obstacle points.")
    parser.add_argument(
        "--obstacle-extra-clearance",
        type=float,
        default=0.07,
        help="Extra margin above --obstacle-clearance required before a path is accepted.",
    )
    parser.add_argument("--obstacle-samples-per-leg", type=int, default=24)
    parser.add_argument("--obstacle-capture-s", type=float, default=0.8)
    parser.add_argument("--obstacle-stride-px", type=int, default=12)
    parser.add_argument("--obstacle-min-depth", type=float, default=0.15)
    parser.add_argument("--obstacle-max-depth", type=float, default=1.50)
    parser.add_argument("--obstacle-voxel-size", type=float, default=0.025)
    parser.add_argument("--obstacle-max-points", type=int, default=2500)
    parser.add_argument("--obstacle-exclude-target-radius", type=float, default=0.13)
    parser.add_argument("--obstacle-early-accept-distance", type=float, default=0.0, help="Accept a clear path immediately when min distance is at least this value. 0 disables early accept.")
    parser.add_argument("--avoidance-mode", choices=("none", "basic"), default="basic")
    parser.add_argument("--avoidance-skip-direct", action="store_true", help="Do not choose the direct arm path; force planning through an avoidance waypoint.")
    parser.add_argument("--avoidance-up-values", default="0.12,0.22,0.32", help="Comma-separated z offsets for intermediate waypoints, meters.")
    parser.add_argument("--avoidance-left-values", default="0.0,0.10,-0.10,0.18,-0.18", help="Comma-separated y offsets for intermediate waypoints, meters.")
    parser.add_argument("--avoidance-forward-values", default="0.0,-0.08,0.08", help="Comma-separated x offsets for intermediate waypoints, meters.")
    parser.add_argument("--avoidance-approach-up-values", default="0.08,0.12,0.18", help="Comma-separated z offsets for the final above-target approach waypoint, meters.")
    parser.add_argument("--avoidance-max-error", type=float, default=0.035, help="Allowed IK error for avoidance waypoints, meters.")
    parser.add_argument("--avoidance-iterations", type=int, default=45, help="IK iterations for intermediate avoidance waypoints.")
    parser.add_argument("--avoidance-max-waypoint-tries", type=int, default=45, help="Maximum intermediate waypoint IK attempts per replan.")
    parser.add_argument("--avoidance-max-candidates", type=int, default=40, help="Maximum collision-check candidates per replan, including above-target variants.")
    parser.add_argument("--avoidance-replans", type=int, default=3, help="How many expanded avoidance searches to try before NO_SAFE_PATH.")
    parser.add_argument("--avoidance-replan-scale", type=float, default=1.6, help="Scale factor for left/forward/up offsets on each replan attempt.")
    parser.add_argument("--snapshot-file", default="before_ik_reach.json")
    parser.add_argument("--save-json", default="runs/ik_reach_latest.json")
    parser.add_argument("--log-file", default="ik_reach.log")
    return parser.parse_args()


def configure_logging(log_file: str) -> None:
    handlers = [logging.StreamHandler()]
    if log_file:
        handlers.append(logging.FileHandler(log_file, encoding="utf-8"))
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s", handlers=handlers)


def run(args: argparse.Namespace) -> int:
    hand = validate_hand_side(args.hand)
    if hand != "right":
        raise ValueError("task5_ik_reach is calibrated for the right arm only for now")

    detection = _read_camera_point(args)
    target = _target_from_camera(args, detection)
    model = load_urdf(args.urdf)
    chain = path_to_link(model, args.base_link, args.tip_link)
    movable = active_joints(chain)
    if len(movable) != 7:
        raise ValueError(f"Expected 7 active arm joints, got {len(movable)}")

    seed = _parse_seed(args.seed) if args.seed else list(RIGHT_SOFT_PREGRASP)
    q, error = solve_ik_position(
        chain,
        (target.x_m, target.y_m, target.z_m),
        seed_values=seed,
        iterations=args.iterations,
    )
    result_pos = position(fk(chain, {joint.name: q[index] for index, joint in enumerate(movable)}))
    report = {
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "hand": hand,
        "camera_point": {"x_m": detection.x_m, "y_m": detection.y_m, "z_m": detection.z_m},
        "camera_transform": {
            "camera_target_x": args.camera_target_x,
            "camera_target_y": args.camera_target_y,
            "camera_target_z": args.camera_target_z,
            "camera_pitch_down_deg": args.camera_pitch_down_deg,
        },
        "grasp_offset": {
            "forward_m": args.grasp_forward_offset,
            "left_m": args.grasp_left_offset,
            "up_m": args.grasp_up_offset,
        },
        "target_torso": {"x_m": target.x_m, "y_m": target.y_m, "z_m": target.z_m},
        "joint_names": [joint.name for joint in movable],
        "soft_pregrasp": list(RIGHT_SOFT_PREGRASP),
        "seed": seed,
        "ik_values": q,
        "ik_error_m": error,
        "ik_position": {"x_m": result_pos[0], "y_m": result_pos[1], "z_m": result_pos[2]},
    }
    _save_report(args.save_json, report)
    _print_report(report)

    if error > args.max_error:
        logging.error("IK_REACH error %.4f exceeds --max-error %.4f; not executing", error, args.max_error)
        return 2

    controller = None
    original_snapshot = None
    selected_q_path: List[List[float]] = []
    if args.execute or args.obstacle_check != "none":
        if args.execute:
            controller = SafeArmPoseController(args.network_interface, duration_s=args.pregrasp_duration)
            original_snapshot = controller.snapshot()
        else:
            original_snapshot = _planning_snapshot_without_dds(args)
        _save_snapshot(args.snapshot_file, hand, original_snapshot)
        selected_q_path = [original_snapshot[hand], list(RIGHT_SOFT_PREGRASP), q]

        obstacle_result, planned_q_path = _check_obstacles_if_needed(
            args,
            chain,
            target,
            selected_q_path,
        )
        if obstacle_result != 0:
            return obstacle_result
        selected_q_path = planned_q_path

    if not args.execute:
        logging.info("IK_REACH print-only result=OK")
        return 0

    assert controller is not None
    for index, waypoint_q in enumerate(selected_q_path[1:], start=1):
        controller.duration_s = max(0.2, float(args.pregrasp_duration if index == 1 else args.duration))
        logging.info(
            "IK_REACH execute waypoint %d/%d hand=%s joints=%s duration=%.2f",
            index,
            len(selected_q_path) - 1,
            hand,
            [round(v, 5) for v in waypoint_q],
            controller.duration_s,
        )
        ack = controller.move_pose(hand, tuple(waypoint_q))
        logging.info("IK_REACH waypoint %d/%d ok=%s message=%s", index, len(selected_q_path) - 1, ack.ok, ack.message)
        if not ack.ok:
            return 3 if index == 1 else 4

    if args.grip != "none":
        dex3 = Dex3HandController(UnitreeSdk2Dex3Backend(args.network_interface))
        hand_ack = dex3.open(hand) if args.grip == "open" else dex3.close(hand, "wrap")
        logging.info("IK_REACH grip=%s ok=%s message=%s", args.grip, hand_ack.ok, hand_ack.message)
        if not hand_ack.ok:
            return 5
    return 0


def _read_camera_point(args: argparse.Namespace) -> CameraPoint:
    if args.camera_x is not None and args.camera_y is not None and args.camera_z is not None:
        return CameraPoint(float(args.camera_x), float(args.camera_y), float(args.camera_z))
    if args.yolo_log:
        detection = parse_yolo_log(args.yolo_log, args.min_confidence, args.allow_low_confidence, args.default_camera_y)
        return CameraPoint(detection.x_m, detection.y_m, detection.z_m)
    raise ValueError("Provide --camera-x/--camera-y/--camera-z or --yolo-log")


def _target_from_camera(args: argparse.Namespace, point: CameraPoint) -> PoseXYZ:
    transform = CameraToArmConfig(
        camera_target_x_m=args.camera_target_x,
        camera_target_y_m=args.camera_target_y,
        camera_target_z_m=args.camera_target_z,
        camera_pitch_down_deg=args.camera_pitch_down_deg,
    )
    offset = GraspOffset(args.grasp_forward_offset, args.grasp_left_offset, args.grasp_up_offset)
    return yolo_camera_point_to_arm_target(point, transform, offset)


def _camera_transform_from_args(args: argparse.Namespace) -> CameraToArmConfig:
    return CameraToArmConfig(
        camera_target_x_m=args.camera_target_x,
        camera_target_y_m=args.camera_target_y,
        camera_target_z_m=args.camera_target_z,
        camera_pitch_down_deg=args.camera_pitch_down_deg,
    )


def _check_obstacles_if_needed(
    args: argparse.Namespace,
    chain,
    target: PoseXYZ,
    q_waypoints: List[List[float]],
) -> Tuple[int, List[List[float]]]:
    if args.obstacle_check == "none":
        logging.info("IK_REACH obstacle check disabled")
        return 0, q_waypoints

    if args.obstacle_check == "json":
        if not args.obstacle_json:
            raise ValueError("--obstacle-json is required when --obstacle-check json")
        cloud = load_obstacle_cloud_json(args.obstacle_json)
    else:
        cloud = capture_realsense_obstacle_cloud(
            _camera_transform_from_args(args),
            target,
            capture_s=args.obstacle_capture_s,
            stride_px=args.obstacle_stride_px,
            min_depth_m=args.obstacle_min_depth,
            max_depth_m=args.obstacle_max_depth,
            voxel_size_m=args.obstacle_voxel_size,
            max_points=args.obstacle_max_points,
            exclude_target_radius_m=args.obstacle_exclude_target_radius,
        )

    if args.obstacle_save_json:
        save_obstacle_cloud_json(args.obstacle_save_json, cloud)

    required_distance = float(args.obstacle_clearance) + float(args.obstacle_extra_clearance)
    max_attempts = max(1, int(args.avoidance_replans))
    best_weak: Optional[Tuple[str, CollisionCheckResult]] = None
    best_blocked: Optional[Tuple[str, CollisionCheckResult]] = None

    for attempt in range(max_attempts):
        scale = float(args.avoidance_replan_scale) ** attempt
        candidates = _build_q_path_candidates(args, chain, target, q_waypoints, scale=scale, attempt=attempt)
        best_clear: Optional[Tuple[str, List[List[float]], CollisionCheckResult]] = None
        logging.info(
            "IK_REACH replan attempt %d/%d scale=%.3f candidates=%d required_distance=%.3f",
            attempt + 1,
            max_attempts,
            scale,
            len(candidates),
            required_distance,
        )

        for index, (name, candidate_q_path) in enumerate(candidates, start=1):
            result = check_joint_path_collision(
                chain,
                candidate_q_path,
                cloud.points,
                clearance_m=args.obstacle_clearance,
                samples_per_leg=args.obstacle_samples_per_leg,
            )
            logging.info(
                "IK_REACH obstacle candidate attempt=%d index=%d/%d name=%s ok=%s message=%s min_distance=%.3f obstacle=%s",
                attempt + 1,
                index,
                len(candidates),
                name,
                result.ok,
                result.message,
                result.min_distance_m,
                result.obstacle,
            )
            if result.ok and result.min_distance_m >= required_distance:
                if best_clear is None or result.min_distance_m > best_clear[2].min_distance_m:
                    best_clear = (name, candidate_q_path, result)
                if args.obstacle_early_accept_distance > 0 and result.min_distance_m >= args.obstacle_early_accept_distance:
                    print("OBSTACLE_CLEAR", f"path={name}", result.message, f"required={required_distance:.3f}m")
                    print("OBSTACLE_SELECTED_WAYPOINTS", json.dumps(_path_summary(chain, candidate_q_path), ensure_ascii=False))
                    logging.info(
                        "IK_REACH early selected obstacle-safe path=%s waypoints=%d min_distance=%.3f required_distance=%.3f early_accept=%.3f attempt=%d",
                        name,
                        len(candidate_q_path),
                        result.min_distance_m,
                        required_distance,
                        args.obstacle_early_accept_distance,
                        attempt + 1,
                    )
                    return 0, candidate_q_path
                continue
            if result.ok:
                if best_weak is None or result.min_distance_m > best_weak[1].min_distance_m:
                    best_weak = (name, result)
                logging.warning(
                    "IK_REACH obstacle weak candidate name=%s min_distance=%.3f required=%.3f raw_clearance=%.3f",
                    name,
                    result.min_distance_m,
                    required_distance,
                    args.obstacle_clearance,
                )
                continue
            if best_blocked is None or result.min_distance_m > best_blocked[1].min_distance_m:
                best_blocked = (name, result)

        if best_clear is not None:
            name, candidate_q_path, result = best_clear
            print("OBSTACLE_CLEAR", f"path={name}", result.message, f"required={required_distance:.3f}m")
            print("OBSTACLE_SELECTED_WAYPOINTS", json.dumps(_path_summary(chain, candidate_q_path), ensure_ascii=False))
            logging.info(
                "IK_REACH selected best obstacle-safe path=%s waypoints=%d min_distance=%.3f required_distance=%.3f attempt=%d",
                name,
                len(candidate_q_path),
                result.min_distance_m,
                required_distance,
                attempt + 1,
            )
            return 0, candidate_q_path

        logging.warning("IK_REACH replan attempt %d/%d found no robust path", attempt + 1, max_attempts)

    if best_weak is not None:
        name, result = best_weak
        message = (
            f"best weak path={name} min_distance={result.min_distance_m:.3f}m "
            f"required={required_distance:.3f}m raw_clearance={args.obstacle_clearance:.3f}m"
        )
        print("NO_SAFE_PATH", message)
        logging.error("IK_REACH no robust obstacle path after replans: %s", message)
        return 6, q_waypoints

    if best_blocked is not None:
        name, result = best_blocked
        message = f"best blocked path={name}: {result.message}"
    else:
        message = "no obstacle path candidates"
    print("NO_SAFE_PATH", message)
    logging.error("IK_REACH all obstacle path candidates blocked after replans: %s", message)
    return 6, q_waypoints


def _build_q_path_candidates(
    args: argparse.Namespace,
    chain,
    target: PoseXYZ,
    direct_q_path: List[List[float]],
    *,
    scale: float = 1.0,
    attempt: int = 0,
) -> List[Tuple[str, List[List[float]]]]:
    skip_direct = bool(getattr(args, "avoidance_skip_direct", False))
    candidates: List[Tuple[str, List[List[float]]]] = [("direct", direct_q_path)] if attempt == 0 and not skip_direct else []
    if args.avoidance_mode == "none":
        return candidates

    start_q = direct_q_path[0]
    pregrasp_q = direct_q_path[1]
    final_q = direct_q_path[-1]
    up_values = _scaled_positive_values(_parse_float_list(args.avoidance_up_values), scale)
    left_values = _scaled_values(_parse_float_list(args.avoidance_left_values), scale)
    forward_values = _scaled_values(_parse_float_list(args.avoidance_forward_values), scale)
    approach_up_values = _scaled_positive_values(_parse_float_list(args.avoidance_approach_up_values), scale)
    max_tries = max(1, int(args.avoidance_max_waypoint_tries))
    max_candidates = max(1, int(args.avoidance_max_candidates))
    tries = 0

    for up_m in up_values:
        for left_m in left_values:
            for forward_m in forward_values:
                tries += 1
                if tries > max_tries:
                    logging.info(
                        "IK_REACH avoidance waypoint try limit reached attempt=%d max_tries=%d candidates=%d",
                        attempt + 1,
                        max_tries,
                        len(candidates),
                    )
                    return candidates
                waypoint = PoseXYZ(target.x_m + forward_m, target.y_m + left_m, target.z_m + up_m)
                name = f"attempt{attempt + 1}_via_up{up_m:+.2f}_left{left_m:+.2f}_forward{forward_m:+.2f}"
                q_mid = _solve_avoidance_waypoint(
                    args,
                    chain,
                    waypoint,
                    pregrasp_q,
                    name,
                )
                if q_mid is None:
                    continue
                candidates.append((name, [start_q, pregrasp_q, q_mid, final_q]))
                if len(candidates) >= max_candidates:
                    logging.info(
                        "IK_REACH avoidance candidate limit reached attempt=%d max_candidates=%d",
                        attempt + 1,
                        max_candidates,
                    )
                    return candidates
                for approach_up_m in approach_up_values:
                    approach = PoseXYZ(target.x_m, target.y_m, target.z_m + approach_up_m)
                    approach_name = f"{name}_above{approach_up_m:+.2f}"
                    q_above = _solve_avoidance_waypoint(
                        args,
                        chain,
                        approach,
                        q_mid,
                        approach_name,
                    )
                    if q_above is None:
                        continue
                    candidates.append((approach_name, [start_q, pregrasp_q, q_mid, q_above, final_q]))
                    if len(candidates) >= max_candidates:
                        logging.info(
                            "IK_REACH avoidance candidate limit reached attempt=%d max_candidates=%d",
                            attempt + 1,
                            max_candidates,
                        )
                        return candidates
    logging.info("IK_REACH avoidance candidates built count=%d", len(candidates))
    return candidates


def _solve_avoidance_waypoint(
    args: argparse.Namespace,
    chain,
    waypoint: PoseXYZ,
    seed_q: List[float],
    name: str,
) -> Optional[List[float]]:
    q, error = solve_ik_position(
        chain,
        (waypoint.x_m, waypoint.y_m, waypoint.z_m),
        seed_values=seed_q,
        iterations=args.avoidance_iterations,
    )
    if error > args.avoidance_max_error:
        logging.info(
            "IK_REACH avoidance skip name=%s target=%s error=%.4f max=%.4f",
            name,
            waypoint,
            error,
            args.avoidance_max_error,
        )
        return None
    logging.info(
        "IK_REACH avoidance waypoint ok name=%s target=%s error=%.4f",
        name,
        waypoint,
        error,
    )
    return q


def _scaled_values(values: List[float], scale: float) -> List[float]:
    result: List[float] = []
    for value in values:
        scaled = value * scale
        if not any(abs(scaled - existing) < 1e-9 for existing in result):
            result.append(scaled)
    return result


def _scaled_positive_values(values: List[float], scale: float) -> List[float]:
    return [max(0.0, value * scale) for value in values]


def _parse_float_list(text: str) -> List[float]:
    values = [float(part.strip()) for part in text.split(",") if part.strip()]
    if not values:
        raise ValueError("avoidance value list must not be empty")
    return values


def _parse_seed(seed: str) -> List[float]:
    values = [float(part.strip()) for part in seed.split(",") if part.strip()]
    if len(values) != 7:
        raise ValueError("--seed must contain exactly 7 comma-separated values")
    return values


def _planning_snapshot_without_dds(args: argparse.Namespace) -> ArmSnapshot:
    right = _parse_seed(args.start_joints) if args.start_joints else list(RIGHT_SOFT_PREGRASP)
    snapshot = {
        "waist": [0.0, 0.0, 0.0],
        "left": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "right": right,
    }
    logging.info(
        "IK_REACH planning snapshot without DDS right=%s source=%s",
        [round(v, 4) for v in right],
        "--start-joints" if args.start_joints else "RIGHT_SOFT_PREGRASP",
    )
    return snapshot


def _read_snapshot_without_arm_publisher(network_interface: str) -> ArmSnapshot:
    low_state = wait_low_state(network_interface)
    snapshot = {
        "waist": [float(low_state.motor_state[joint_id].q) for joint_id in WAIST_JOINTS],
        "left": [float(low_state.motor_state[joint_id].q) for joint_id in LEFT_ARM_JOINTS],
        "right": [float(low_state.motor_state[joint_id].q) for joint_id in RIGHT_ARM_JOINTS],
    }
    logging.info(
        "IK_REACH lowstate-only snapshot waist=%s left=%s right=%s",
        [round(v, 4) for v in snapshot["waist"]],
        [round(v, 4) for v in snapshot["left"]],
        [round(v, 4) for v in snapshot["right"]],
    )
    return snapshot


def _save_report(path: str, report: Dict[str, object]) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    logging.info("IK_REACH saved report path=%s", output)


def _print_report(report: Dict[str, object]) -> None:
    print("TARGET_TORSO", json.dumps(report["target_torso"], ensure_ascii=False))
    print("IK_VALUES", [round(value, 5) for value in report["ik_values"]])
    print("IK_ERROR_M", round(float(report["ik_error_m"]), 5))
    print("IK_POSITION", json.dumps(report["ik_position"], ensure_ascii=False))
    print("SOFT_PREGRASP", [round(value, 5) for value in report["soft_pregrasp"]])


def _path_summary(chain, q_path: List[List[float]]) -> List[Dict[str, object]]:
    movable = active_joints(chain)
    summary: List[Dict[str, object]] = []
    for index, q in enumerate(q_path):
        pos = position(fk(chain, {joint.name: q[joint_index] for joint_index, joint in enumerate(movable)}))
        summary.append(
            {
                "index": index,
                "q": [round(value, 5) for value in q],
                "palm": {"x_m": round(pos[0], 4), "y_m": round(pos[1], 4), "z_m": round(pos[2], 4)},
            }
        )
    return summary


def _save_snapshot(path: str, hand: HandSide, snapshot: ArmSnapshot) -> None:
    payload = {"hand": hand, "created_at": time.strftime("%Y-%m-%d %H:%M:%S"), "snapshot": snapshot}
    Path(path).write_text(json.dumps(payload, indent=2), encoding="utf-8")
    logging.info("IK_REACH saved original snapshot path=%s", path)


def main() -> int:
    args = parse_args()
    configure_logging(args.log_file)
    try:
        return run(args)
    except Exception as exc:
        logging.exception("IK_REACH failed: %s", exc)
        return 1


if __name__ == "__main__":
    sys.exit(main())
