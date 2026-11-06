from __future__ import annotations

import argparse
import logging
import subprocess
import sys
from pathlib import Path
from typing import List


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Detect a box and plan two-hand IK reach.")
    parser.add_argument("--detector", choices=("server", "local"), default="server")
    parser.add_argument("--server-url", default="http://192.168.10.179:8000", help="Agrohab app URL with /stream and /context.")
    parser.add_argument("--model", default="", help="Path to local trained YOLO .pt model. Used only with --detector local.")
    parser.add_argument("--target-class", default="box")
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--camera-mode", choices=("realsense", "opencv"), default="realsense")
    parser.add_argument("--color-fps", type=int, default=30)
    parser.add_argument("--depth-width", type=int, default=640)
    parser.add_argument("--depth-height", type=int, default=480)
    parser.add_argument("--depth-fps", type=int, default=30)
    parser.add_argument("--box-distance", type=float, default=0.60, help="Approximate camera-to-box distance in meters.")
    parser.add_argument("--depth-roi-ratio", type=float, default=0.45)
    parser.add_argument("--depth-stat", choices=("median", "nearest_percentile"), default="nearest_percentile")
    parser.add_argument("--depth-percentile", type=float, default=20.0)
    parser.add_argument("--min-depth", type=float, default=0.15)
    parser.add_argument("--max-depth", type=float, default=0.85)
    parser.add_argument("--bbox-edge-margin-px", type=float, default=2.0)
    parser.add_argument("--max-object-last-seen-s", type=float, default=0.75)
    parser.add_argument("--depth-fallback", choices=("none", "center_nearest"), default="center_nearest")
    parser.add_argument("--detect-max-time", type=float, default=8.0)
    parser.add_argument("--min-confidence", type=float, default=0.25)
    parser.add_argument("--context-frame-match", choices=("exact", "at_least"), default="exact")
    parser.add_argument("--camera-pitch-down-deg", type=float, default=42.0)
    parser.add_argument("--box-width", type=float, default=0.28)
    parser.add_argument("--box-height", type=float, default=0.16)
    parser.add_argument("--box-depth", type=float, default=0.20)
    parser.add_argument("--side-clearance", type=float, default=0.065)
    parser.add_argument("--grasp-strategy", choices=("finger-close", "flat-squeeze"), default="flat-squeeze")
    parser.add_argument("--squeeze-side-clearance", type=float, default=-0.015)
    parser.add_argument("--squeeze-duration", type=float, default=2.0)
    parser.add_argument("--max-two-hand-center-y", type=float, default=0.10)
    parser.add_argument("--grasp-center-y-mode", choices=("detected", "zero", "clamp"), default="zero")
    parser.add_argument("--grasp-center-y-limit", type=float, default=0.08)
    parser.add_argument("--palm-forward-offset", type=float, default=-0.05)
    parser.add_argument("--palm-up-offset", type=float, default=0.075)
    parser.add_argument("--max-palm-target-x", type=float, default=0.44)
    parser.add_argument("--min-palm-target-z", type=float, default=0.10)
    parser.add_argument("--max-palm-target-z", type=float, default=0.18)
    parser.add_argument("--hand-sequence", choices=("none", "open-only", "open-close"), default="open-only")
    parser.add_argument("--grasp-method", choices=("pinch", "wrap", "side", "flat_palm"), default="flat_palm")
    parser.add_argument("--network-interface", default="eth0")
    parser.add_argument("--execute", action="store_true", help="Actually move both arms after planning.")
    parser.add_argument("--pregrasp-duration", type=float, default=4.0)
    parser.add_argument("--max-error", type=float, default=0.045)
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
    parser.add_argument("--obstacle-early-accept-distance", type=float, default=0.10)
    parser.add_argument("--min-open-via-up", type=float, default=0.10)
    parser.add_argument("--min-open-via-side-extra", type=float, default=0.12)
    parser.add_argument("--hand-envelope-length", type=float, default=0.10)
    parser.add_argument("--hand-envelope-width", type=float, default=0.09)
    parser.add_argument("--hand-envelope-clearance", type=float, default=0.03)
    parser.add_argument("--avoidance-mode", choices=("none", "basic"), default="basic")
    parser.add_argument("--avoidance-up-values", default="0.10,0.18,0.28")
    parser.add_argument("--avoidance-side-extra-values", default="0.00,0.06,0.12")
    parser.add_argument("--avoidance-forward-values", default="-0.16,-0.08,0.0")
    parser.add_argument("--avoidance-max-error", type=float, default=0.045)
    parser.add_argument("--avoidance-iterations", type=int, default=60)
    parser.add_argument("--avoidance-max-candidates", type=int, default=24)
    parser.add_argument("--avoidance-replans", type=int, default=2)
    parser.add_argument("--avoidance-replan-scale", type=float, default=1.5)
    parser.add_argument("--snapshot-file", default="before_box_two_hand.json")
    parser.add_argument("--path-file", default="runs/box_two_hand_last_path.json")
    parser.add_argument("--detect-json", default="runs/server_box_latest.json")
    parser.add_argument("--detect-log", default="server_box_detector.log")
    parser.add_argument("--plan-json", default="runs/box_two_hand_latest.json")
    parser.add_argument("--log-file", default="box_detect_and_plan.log")
    return parser.parse_args()


def configure_logging(log_file: str) -> None:
    handlers = [logging.StreamHandler()]
    if log_file:
        handlers.append(logging.FileHandler(log_file, encoding="utf-8"))
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s", handlers=handlers)


def run(args: argparse.Namespace) -> int:
    if args.detector == "local" and not args.model:
        raise ValueError("--model is required when --detector local")

    detect_module = "arms.server_box_detector" if args.detector == "server" else "arms.rgb_box_detector"
    detect_cmd: List[str] = [
        sys.executable,
        "-B",
        "-m",
        detect_module,
        "--target-class",
        args.target_class,
        "--camera-index",
        str(args.camera_index),
        "--box-distance",
        str(args.box_distance),
        "--max-time",
        str(args.detect_max_time),
        "--min-confidence",
        str(args.min_confidence),
        "--save-debug",
        "--save-json",
        args.detect_json,
        "--log-file",
        args.detect_log,
    ]
    if args.detector == "server":
        detect_cmd.extend(
            [
                "--server-url",
                args.server_url,
                "--camera-mode",
                args.camera_mode,
                "--color-fps",
                str(args.color_fps),
                "--depth-width",
                str(args.depth_width),
                "--depth-height",
                str(args.depth_height),
                "--depth-fps",
                str(args.depth_fps),
                "--depth-roi-ratio",
                str(args.depth_roi_ratio),
                "--depth-stat",
                args.depth_stat,
                "--depth-percentile",
                str(args.depth_percentile),
                "--min-depth",
                str(args.min_depth),
                "--max-depth",
                str(args.max_depth),
                "--bbox-edge-margin-px",
                str(args.bbox_edge_margin_px),
                "--max-object-last-seen-s",
                str(args.max_object_last_seen_s),
                "--depth-fallback",
                args.depth_fallback,
                "--context-frame-match",
                args.context_frame_match,
            ]
        )
    else:
        detect_cmd.extend(["--model", args.model])
    logging.info("BOX_TASK8 step 1/2 detect start cmd=%s", " ".join(detect_cmd))
    detect_result = subprocess.run(detect_cmd, check=False)
    logging.info("BOX_TASK8 detect finished returncode=%d", detect_result.returncode)
    if detect_result.returncode != 0:
        return detect_result.returncode

    detection = _load_detection(args.detect_json)
    camera_point = detection["camera_point"]
    plan_cmd: List[str] = [
        sys.executable,
        "-B",
        "-m",
        "arms.task7_two_hand_box",
        "--camera-x",
        str(camera_point["x_m"]),
        "--camera-y",
        str(camera_point["y_m"]),
        "--camera-z",
        str(camera_point["z_m"]),
        "--camera-pitch-down-deg",
        str(args.camera_pitch_down_deg),
        "--box-width",
        str(args.box_width),
        "--box-height",
        str(args.box_height),
        "--box-depth",
        str(args.box_depth),
        "--side-clearance",
        str(args.side_clearance),
        "--grasp-strategy",
        args.grasp_strategy,
        "--squeeze-side-clearance",
        str(args.squeeze_side_clearance),
        "--squeeze-duration",
        str(args.squeeze_duration),
        "--max-two-hand-center-y",
        str(args.max_two_hand_center_y),
        "--grasp-center-y-mode",
        args.grasp_center_y_mode,
        "--grasp-center-y-limit",
        str(args.grasp_center_y_limit),
        "--palm-forward-offset",
        str(args.palm_forward_offset),
        "--palm-up-offset",
        str(args.palm_up_offset),
        "--max-palm-target-x",
        str(args.max_palm_target_x),
        "--min-palm-target-z",
        str(args.min_palm_target_z),
        "--max-palm-target-z",
        str(args.max_palm_target_z),
        "--hand-sequence",
        args.hand_sequence,
        "--grasp-method",
        args.grasp_method,
        "--network-interface",
        args.network_interface,
        "--pregrasp-duration",
        str(args.pregrasp_duration),
        "--max-error",
        str(args.max_error),
        "--obstacle-check",
        args.obstacle_check,
        "--obstacle-save-json",
        args.obstacle_save_json,
        "--obstacle-clearance",
        str(args.obstacle_clearance),
        "--obstacle-extra-clearance",
        str(args.obstacle_extra_clearance),
        "--obstacle-samples-per-leg",
        str(args.obstacle_samples_per_leg),
        "--obstacle-capture-s",
        str(args.obstacle_capture_s),
        "--obstacle-stride-px",
        str(args.obstacle_stride_px),
        "--obstacle-min-depth",
        str(args.obstacle_min_depth),
        "--obstacle-max-depth",
        str(args.obstacle_max_depth),
        "--obstacle-voxel-size",
        str(args.obstacle_voxel_size),
        "--obstacle-max-points",
        str(args.obstacle_max_points),
        "--obstacle-exclude-target-radius",
        str(args.obstacle_exclude_target_radius),
        "--obstacle-early-accept-distance",
        str(args.obstacle_early_accept_distance),
        "--min-open-via-up",
        str(args.min_open_via_up),
        "--min-open-via-side-extra",
        str(args.min_open_via_side_extra),
        "--hand-envelope-length",
        str(args.hand_envelope_length),
        "--hand-envelope-width",
        str(args.hand_envelope_width),
        "--hand-envelope-clearance",
        str(args.hand_envelope_clearance),
        "--avoidance-mode",
        args.avoidance_mode,
        f"--avoidance-up-values={args.avoidance_up_values}",
        f"--avoidance-side-extra-values={args.avoidance_side_extra_values}",
        f"--avoidance-forward-values={args.avoidance_forward_values}",
        "--avoidance-max-error",
        str(args.avoidance_max_error),
        "--avoidance-iterations",
        str(args.avoidance_iterations),
        "--avoidance-max-candidates",
        str(args.avoidance_max_candidates),
        "--avoidance-replans",
        str(args.avoidance_replans),
        "--avoidance-replan-scale",
        str(args.avoidance_replan_scale),
        "--snapshot-file",
        args.snapshot_file,
        "--path-file",
        args.path_file,
        "--save-json",
        args.plan_json,
        "--log-file",
        args.log_file.replace(".log", "_plan.log") if args.log_file else "box_detect_and_plan_plan.log",
    ]
    if args.obstacle_check == "json":
        plan_cmd.extend(["--obstacle-json", args.obstacle_json])
    if args.execute:
        plan_cmd.append("--execute")

    logging.info("BOX_TASK8 step 2/2 plan start cmd=%s", " ".join(plan_cmd))
    plan_result = subprocess.run(plan_cmd, check=False)
    logging.info("BOX_TASK8 plan finished returncode=%d", plan_result.returncode)
    return plan_result.returncode


def _load_detection(path: str):
    import json

    return json.loads(Path(path).read_text(encoding="utf-8"))


def main() -> int:
    args = parse_args()
    configure_logging(args.log_file)
    try:
        return run(args)
    except Exception as exc:
        logging.exception("BOX_TASK8 failed: %s", exc)
        return 1


if __name__ == "__main__":
    sys.exit(main())
