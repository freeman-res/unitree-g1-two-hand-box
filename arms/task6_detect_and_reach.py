from __future__ import annotations

import argparse
import logging
import subprocess
import sys
import time
from pathlib import Path
from typing import List

from .cup_pipeline import parse_yolo_log


DEFAULT_YOLO_DIR = "~/yolo_cup_project/V3"
DEFAULT_YOLO_SCRIPT = "itog_v3.py"
YOLO_CLASS_ALIASES = {
    "box": "suitcase",
    "cardboard_box": "suitcase",
    "package": "suitcase",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Detect object with old YOLO/RealSense code, then run URDF IK reach.")
    parser.add_argument("--hand", choices=("right",), default="right")
    parser.add_argument("--network-interface", default="eth0")
    parser.add_argument("--yolo-dir", default=DEFAULT_YOLO_DIR)
    parser.add_argument("--yolo-script", default=DEFAULT_YOLO_SCRIPT)
    parser.add_argument("--yolo-max-time", type=float, default=5.0)
    parser.add_argument("--yolo-min-confidence", type=float, default=0.07)
    parser.add_argument("--yolo-target-class", default="cup", help="Passed to old YOLO code as --target-class, for example cup or box.")
    parser.add_argument("--yolo-log", default="", help="Optional YOLO log path. Default creates one in the YOLO directory.")
    parser.add_argument("--stop-distance", type=float, default=0.60, help="Passed to old YOLO code to avoid forward approach when already near the cup.")
    parser.add_argument("--target-x", type=float, default=0.0, help="Passed to old YOLO code. Keep 0.0 when using IK offsets for lateral correction.")
    parser.add_argument("--profile-every", type=int, default=1)
    parser.add_argument("--approach-y-min", type=float, default=-1.0)
    parser.add_argument("--approach-y-max", type=float, default=1.0)
    parser.add_argument("--camera-pitch-down-deg", type=float, default=52.0)
    parser.add_argument("--grasp-forward-offset", type=float, default=-0.08)
    parser.add_argument("--grasp-left-offset", type=float, default=-0.155, help="Less negative moves the hand left; more negative moves it right.")
    parser.add_argument("--grasp-up-offset", type=float, default=0.16)
    parser.add_argument("--seed", default="", help="Optional IK seed. Empty means task5 uses soft pregrasp seed.")
    parser.add_argument("--start-joints", default="", help="Optional current 7 right-arm joints for debug planning without rt/lowstate.")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--pregrasp-duration", type=float, default=4.0)
    parser.add_argument("--max-error", type=float, default=0.025)
    parser.add_argument("--execute", action="store_true", help="Actually move the arm. Without this, detect and print IK only.")
    parser.add_argument("--grip", choices=("none", "open", "close"), default="none")
    parser.add_argument("--obstacle-check", choices=("none", "json", "realsense"), default="none")
    parser.add_argument("--obstacle-json", default="")
    parser.add_argument("--obstacle-save-json", default="runs/obstacle_cloud_latest.json")
    parser.add_argument("--obstacle-clearance", type=float, default=0.08)
    parser.add_argument("--obstacle-extra-clearance", type=float, default=0.07)
    parser.add_argument("--obstacle-samples-per-leg", type=int, default=24)
    parser.add_argument("--obstacle-capture-s", type=float, default=0.8)
    parser.add_argument("--obstacle-stride-px", type=int, default=12)
    parser.add_argument("--obstacle-min-depth", type=float, default=0.15)
    parser.add_argument("--obstacle-max-depth", type=float, default=1.50)
    parser.add_argument("--obstacle-voxel-size", type=float, default=0.025)
    parser.add_argument("--obstacle-max-points", type=int, default=2500)
    parser.add_argument("--obstacle-exclude-target-radius", type=float, default=0.13)
    parser.add_argument("--obstacle-early-accept-distance", type=float, default=0.0)
    parser.add_argument("--avoidance-mode", choices=("none", "basic"), default="basic")
    parser.add_argument("--avoidance-up-values", default="0.12,0.22,0.32")
    parser.add_argument("--avoidance-left-values", default="0.0,0.10,-0.10,0.18,-0.18")
    parser.add_argument("--avoidance-forward-values", default="0.0,-0.08,0.08")
    parser.add_argument("--avoidance-approach-up-values", default="0.08,0.12,0.18")
    parser.add_argument("--avoidance-max-error", type=float, default=0.035)
    parser.add_argument("--avoidance-iterations", type=int, default=45)
    parser.add_argument("--avoidance-max-waypoint-tries", type=int, default=45)
    parser.add_argument("--avoidance-max-candidates", type=int, default=40)
    parser.add_argument("--avoidance-replans", type=int, default=3)
    parser.add_argument("--avoidance-replan-scale", type=float, default=1.6)
    parser.add_argument("--snapshot-file", default="before_detect_and_reach.json")
    parser.add_argument("--save-json", default="runs/detect_and_reach_latest.json")
    parser.add_argument("--log-file", default="detect_and_reach.log")
    return parser.parse_args()


def configure_logging(log_file: str) -> None:
    handlers = [logging.StreamHandler()]
    if log_file:
        handlers.append(logging.FileHandler(log_file, encoding="utf-8"))
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s", handlers=handlers)


def run(args: argparse.Namespace) -> int:
    yolo_dir = Path(args.yolo_dir).expanduser().resolve()
    yolo_script = yolo_dir / args.yolo_script
    if not yolo_script.exists():
        raise FileNotFoundError(f"YOLO script not found: {yolo_script}")
    yolo_target_class = normalize_yolo_target_class(args.yolo_target_class)
    if yolo_target_class != args.yolo_target_class:
        logging.info("TASK6 YOLO target class alias %s -> %s", args.yolo_target_class, yolo_target_class)

    yolo_log = Path(args.yolo_log).expanduser() if args.yolo_log else yolo_dir / f"yolo_detect_reach_{int(time.time())}.log"
    if not yolo_log.is_absolute():
        yolo_log = (yolo_dir / yolo_log).resolve()
    yolo_log.parent.mkdir(parents=True, exist_ok=True)

    yolo_cmd = [
        sys.executable,
        str(yolo_script.name),
        args.network_interface,
        "--no-conversation",
        "--no-head-emotions",
        "--no-coffee-request",
        "--no-pregrasp-before-approach",
        "--no-arm-reach",
        "--save-debug",
        "--profile-every",
        str(args.profile_every),
        "--max-time",
        str(args.yolo_max_time),
        "--approach-y-min",
        str(args.approach_y_min),
        "--approach-y-max",
        str(args.approach_y_max),
        "--min-detection-confidence",
        str(args.yolo_min_confidence),
        "--target-class",
        yolo_target_class,
        "--stop-distance",
        str(args.stop_distance),
        "--target-x",
        str(args.target_x),
        "--log-file",
        str(yolo_log),
    ]
    logging.info("TASK6 step 1/2 YOLO start cwd=%s cmd=%s", yolo_dir, " ".join(yolo_cmd))
    yolo_result = subprocess.run(yolo_cmd, cwd=str(yolo_dir), check=False)
    logging.info("TASK6 YOLO finished returncode=%d log=%s", yolo_result.returncode, yolo_log)
    try:
        detection = parse_yolo_log(str(yolo_log), args.yolo_min_confidence, True, None)
        logging.info(
            "TASK6 YOLO usable detection frame=%s conf=%s x=%.3f y=%.3f z=%.3f",
            detection.frame,
            detection.confidence,
            detection.x_m,
            detection.y_m,
            detection.z_m,
        )
    except Exception as exc:
        logging.error("TASK6 no usable YOLO detection in %s after returncode=%d: %s", yolo_log, yolo_result.returncode, exc)
        return yolo_result.returncode if yolo_result.returncode != 0 else 2
    if yolo_result.returncode != 0:
        logging.warning("TASK6 continuing despite YOLO returncode=%d because detection was found", yolo_result.returncode)

    ik_cmd: List[str] = [
        sys.executable,
        "-B",
        "-m",
        "arms.task5_ik_reach",
        "--hand",
        args.hand,
        "--yolo-log",
        str(yolo_log),
        "--allow-low-confidence",
        "--min-confidence",
        str(args.yolo_min_confidence),
        "--camera-pitch-down-deg",
        str(args.camera_pitch_down_deg),
        "--grasp-forward-offset",
        str(args.grasp_forward_offset),
        "--grasp-left-offset",
        str(args.grasp_left_offset),
        "--grasp-up-offset",
        str(args.grasp_up_offset),
        "--network-interface",
        args.network_interface,
        "--duration",
        str(args.duration),
        "--pregrasp-duration",
        str(args.pregrasp_duration),
        "--max-error",
        str(args.max_error),
        "--snapshot-file",
        args.snapshot_file,
        "--save-json",
        args.save_json,
        "--log-file",
        args.log_file.replace(".log", "_ik.log") if args.log_file else "detect_and_reach_ik.log",
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
        "--avoidance-mode",
        args.avoidance_mode,
        f"--avoidance-up-values={args.avoidance_up_values}",
        f"--avoidance-left-values={args.avoidance_left_values}",
        f"--avoidance-forward-values={args.avoidance_forward_values}",
        f"--avoidance-approach-up-values={args.avoidance_approach_up_values}",
        "--avoidance-max-error",
        str(args.avoidance_max_error),
        "--avoidance-iterations",
        str(args.avoidance_iterations),
        "--avoidance-max-waypoint-tries",
        str(args.avoidance_max_waypoint_tries),
        "--avoidance-max-candidates",
        str(args.avoidance_max_candidates),
        "--avoidance-replans",
        str(args.avoidance_replans),
        "--avoidance-replan-scale",
        str(args.avoidance_replan_scale),
    ]
    if args.obstacle_json:
        ik_cmd.extend(["--obstacle-json", args.obstacle_json])
    if args.seed:
        ik_cmd.extend(["--seed", args.seed])
    if args.start_joints:
        ik_cmd.extend(["--start-joints", args.start_joints])
    if args.execute:
        ik_cmd.append("--execute")
    if args.grip != "none":
        ik_cmd.extend(["--grip", args.grip])

    logging.info("TASK6 step 2/2 IK start cmd=%s", " ".join(ik_cmd))
    ik_result = subprocess.run(ik_cmd, cwd=str(Path.cwd()), check=False)
    logging.info("TASK6 IK finished returncode=%d", ik_result.returncode)
    return ik_result.returncode


def normalize_yolo_target_class(name: str) -> str:
    normalized = name.strip().lower().replace("-", "_").replace(" ", "_")
    return YOLO_CLASS_ALIASES.get(normalized, name.strip())


def main() -> int:
    args = parse_args()
    configure_logging(args.log_file)
    try:
        return run(args)
    except Exception as exc:
        logging.exception("TASK6 failed: %s", exc)
        return 1


if __name__ == "__main__":
    sys.exit(main())


