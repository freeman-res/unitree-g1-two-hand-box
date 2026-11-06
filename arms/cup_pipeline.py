from __future__ import annotations

import argparse
import json
import logging
import re
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from .dex3_hand import Dex3HandController
from .dex3_sdk2_backend import UnitreeSdk2Dex3Backend
from .g1_arm_sdk2 import LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS, UnitreeSdk2ArmClient
from .move_arm_preset import PRESETS, SafeArmPoseController
from .record_arm_calibration import load_or_create, wait_low_state
from .safety import validate_hand_side, validate_pose
from .types import HandSide, PoseXYZ
from .vision_kinematics import CameraPoint, CameraToArmConfig, GraspOffset, yolo_camera_point_to_arm_target


@dataclass(frozen=True)
class YoloDetection:
    x_m: float
    y_m: float
    z_m: float
    confidence: Optional[float]
    frame: Optional[int]
    source: str
    line: str


_FLOAT_RE = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)"


def parse_yolo_log(path: str, min_confidence: float, allow_low_confidence: bool, default_y: Optional[float]) -> YoloDetection:
    log_path = Path(path)
    if not log_path.exists():
        raise FileNotFoundError(f"YOLO log not found: {path}")

    detections: List[YoloDetection] = []
    partial_without_y: List[Tuple[float, float, Optional[float], Optional[int], str]] = []
    for raw_line in log_path.read_text(encoding="utf-8", errors="replace").splitlines():
        if "X=" not in raw_line or "Z=" not in raw_line:
            continue
        x_value = _extract_float(raw_line, "X")
        y_value = _extract_float(raw_line, "Y")
        z_value = _extract_float(raw_line, "Z")
        conf_value = _extract_float(raw_line, "conf")
        frame_value = _extract_int(raw_line, "frame")
        if x_value is None or z_value is None:
            continue
        if y_value is None:
            partial_without_y.append((x_value, z_value, conf_value, frame_value, raw_line))
            continue
        detections.append(
            YoloDetection(
                x_m=x_value,
                y_m=y_value,
                z_m=z_value,
                confidence=conf_value,
                frame=frame_value,
                source=str(log_path),
                line=raw_line,
            )
        )

    candidates = [item for item in detections if item.confidence is None or item.confidence >= min_confidence]
    if not candidates and allow_low_confidence:
        candidates = detections
    if candidates:
        return candidates[-1]

    if default_y is not None and partial_without_y:
        filtered = [item for item in partial_without_y if item[2] is None or item[2] >= min_confidence]
        if not filtered and allow_low_confidence:
            filtered = partial_without_y
        if filtered:
            x_value, z_value, conf_value, frame_value, raw_line = filtered[-1]
            return YoloDetection(
                x_m=x_value,
                y_m=float(default_y),
                z_m=z_value,
                confidence=conf_value,
                frame=frame_value,
                source=str(log_path),
                line=raw_line + f" | default_y={default_y:.3f}",
            )

    raise ValueError(
        "No complete YOLO detection found. Need a log line with X=..., Y=..., Z=...; "
        "or pass --default-camera-y for logs that only print X/Z."
    )


def _extract_float(line: str, key: str) -> Optional[float]:
    match = re.search(rf"\b{re.escape(key)}=({_FLOAT_RE})", line)
    if match:
        return float(match.group(1))
    if key == "conf":
        loose_match = re.search(rf"\bconf\s+({_FLOAT_RE})", line)
        if loose_match:
            return float(loose_match.group(1))
    return None


def _extract_int(line: str, key: str) -> Optional[int]:
    match = re.search(rf"\b{re.escape(key)}=(\d+)", line)
    return int(match.group(1)) if match else None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Full cup test pipeline: YOLO coordinates, target conversion, calibration, arm reach, and grasp.")
    parser.add_argument("--mode", choices=("plan", "record-current", "reach-calibrated", "preset-pregrasp", "preset-grasp", "full-calibrated"), default="plan")
    parser.add_argument("--hand", choices=("left", "right"), default="right")
    parser.add_argument("--camera-x", type=float, help="Camera X from YOLO/RealSense, meters.")
    parser.add_argument("--camera-y", type=float, help="Camera Y from YOLO/RealSense, meters.")
    parser.add_argument("--camera-z", type=float, help="Camera Z/depth from YOLO/RealSense, meters.")
    parser.add_argument("--yolo-log", default="", help="Path to yolo_probe*.log. Used when camera-x/y/z are not supplied.")
    parser.add_argument("--min-confidence", type=float, default=0.20)
    parser.add_argument("--allow-low-confidence", action="store_true")
    parser.add_argument("--default-camera-y", type=float, help="Use this Y if the log prints only X/Z profile lines.")
    parser.add_argument("--network-interface", default="eth0")
    parser.add_argument("--arm-calibration", default="config/arm_calibration.json")
    parser.add_argument("--arm-duration", type=float, default=3.0)
    parser.add_argument("--camera-target-x", type=float, default=0.0)
    parser.add_argument("--camera-target-y", type=float, default=-0.15)
    parser.add_argument("--camera-target-z", type=float, default=0.50)
    parser.add_argument("--camera-x-to-target-y-sign", type=float, default=-1.0)
    parser.add_argument("--camera-y-to-target-z-sign", type=float, default=-1.0)
    parser.add_argument("--camera-pitch-down-deg", type=float, default=0.0, help="Camera downward pitch from horizontal, degrees.")
    parser.add_argument("--grasp-forward-offset", type=float, default=-0.10)
    parser.add_argument("--grasp-left-offset", type=float, default=0.0)
    parser.add_argument("--grasp-up-offset", type=float, default=0.0)
    parser.add_argument("--grip", choices=("none", "open", "close"), default="none")
    parser.add_argument("--calibration-name", default="cup_live")
    parser.add_argument("--preset-pregrasp", choices=sorted(PRESETS), default="right_cup_side_out_table76")
    parser.add_argument("--preset-final-a", choices=sorted(PRESETS), default="right_cup_forward_left_table76")
    parser.add_argument("--preset-final-b", choices=sorted(PRESETS), default="right_cup_forward_right_table76")
    parser.add_argument("--preset-duration", type=float, default=4.0)
    parser.add_argument("--snapshot-file", default="cup_pipeline_before_pose.json")
    parser.add_argument("--save-json", default="runs/cup_pipeline_latest.json")
    parser.add_argument("--log-file", default="cup_pipeline.log")
    return parser.parse_args()


def configure_logging(log_file: str) -> None:
    handlers = [logging.StreamHandler()]
    if log_file:
        handlers.append(logging.FileHandler(log_file, encoding="utf-8"))
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s", handlers=handlers)


def run(args: argparse.Namespace) -> int:
    hand = validate_hand_side(args.hand)
    detection = _read_detection(args)
    target = _target_from_detection(args, detection)
    validate_pose(target)
    report = _build_report(args, detection, target)
    _save_report(args.save_json, report)
    _print_report(report)

    if args.mode == "plan":
        logging.info("CUP_PIPELINE mode=plan saved=%s", args.save_json)
        return 0
    if args.mode == "record-current":
        return _record_current(args, hand, target)
    if args.mode == "reach-calibrated":
        return _reach_calibrated(args, hand, target)
    if args.mode == "preset-pregrasp":
        return _move_single_preset(args, hand, args.preset_pregrasp)
    if args.mode == "preset-grasp":
        return _preset_grasp_sequence(args, hand)
    if args.mode == "full-calibrated":
        return _full_calibrated(args, hand, target)
    raise ValueError(f"Unsupported mode: {args.mode}")


def _read_detection(args: argparse.Namespace) -> YoloDetection:
    if args.camera_x is not None and args.camera_y is not None and args.camera_z is not None:
        return YoloDetection(
            x_m=float(args.camera_x),
            y_m=float(args.camera_y),
            z_m=float(args.camera_z),
            confidence=None,
            frame=None,
            source="manual-args",
            line="manual --camera-x/y/z",
        )
    if args.yolo_log:
        detection = parse_yolo_log(args.yolo_log, args.min_confidence, args.allow_low_confidence, args.default_camera_y)
        logging.info(
            "CUP_PIPELINE yolo detection frame=%s conf=%s x=%.3f y=%.3f z=%.3f source=%s",
            detection.frame,
            detection.confidence,
            detection.x_m,
            detection.y_m,
            detection.z_m,
            detection.source,
        )
        return detection
    raise ValueError("Provide --camera-x/--camera-y/--camera-z or --yolo-log")


def _target_from_detection(args: argparse.Namespace, detection: YoloDetection) -> PoseXYZ:
    transform = CameraToArmConfig(
        camera_target_x_m=args.camera_target_x,
        camera_target_y_m=args.camera_target_y,
        camera_target_z_m=args.camera_target_z,
        camera_x_to_target_y_sign=args.camera_x_to_target_y_sign,
        camera_y_to_target_z_sign=args.camera_y_to_target_z_sign,
        camera_pitch_down_deg=args.camera_pitch_down_deg,
    )
    offset = GraspOffset(
        forward_m=args.grasp_forward_offset,
        left_m=args.grasp_left_offset,
        up_m=args.grasp_up_offset,
    )
    return yolo_camera_point_to_arm_target(CameraPoint(detection.x_m, detection.y_m, detection.z_m), transform, offset)


def _build_report(args: argparse.Namespace, detection: YoloDetection, target: PoseXYZ) -> Dict[str, object]:
    return {
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "mode": args.mode,
        "hand": args.hand,
        "detection": asdict(detection),
        "target": {"x_m": target.x_m, "y_m": target.y_m, "z_m": target.z_m},
        "transform": {
            "camera_target_x": args.camera_target_x,
            "camera_target_y": args.camera_target_y,
            "camera_target_z": args.camera_target_z,
            "camera_x_to_target_y_sign": args.camera_x_to_target_y_sign,
            "camera_y_to_target_z_sign": args.camera_y_to_target_z_sign,
            "camera_pitch_down_deg": args.camera_pitch_down_deg,
        },
        "grasp_offset": {
            "forward_m": args.grasp_forward_offset,
            "left_m": args.grasp_left_offset,
            "up_m": args.grasp_up_offset,
        },
    }


def _save_report(path: str, report: Dict[str, object]) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    logging.info("CUP_PIPELINE saved report path=%s", output)


def _print_report(report: Dict[str, object]) -> None:
    detection = report["detection"]
    target = report["target"]
    print("DETECTION", json.dumps(detection, ensure_ascii=False))
    print("TARGET", json.dumps(target, ensure_ascii=False))


def _record_current(args: argparse.Namespace, hand: HandSide, target: PoseXYZ) -> int:
    low_state = wait_low_state(args.network_interface)
    joint_ids = LEFT_ARM_JOINTS if hand == "left" else RIGHT_ARM_JOINTS
    joints = [float(low_state.motor_state[joint_id].q) for joint_id in joint_ids]
    output = Path(args.arm_calibration)
    output.parent.mkdir(parents=True, exist_ok=True)
    data = load_or_create(output)
    data.setdefault("left", [])
    data.setdefault("right", [])
    point = {
        "name": args.calibration_name,
        "x_m": target.x_m,
        "y_m": target.y_m,
        "z_m": target.z_m,
        "joints": joints,
    }
    data[hand].append(point)
    output.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    logging.info("CUP_PIPELINE calibration saved hand=%s point=%s output=%s", hand, point, output)
    return 0


def _reach_calibrated(args: argparse.Namespace, hand: HandSide, target: PoseXYZ) -> int:
    arm = UnitreeSdk2ArmClient(args.network_interface, args.arm_calibration, duration_s=args.arm_duration)
    ack = arm.move_to(hand, target)
    logging.info("CUP_PIPELINE reach_calibrated ok=%s message=%s", ack.ok, ack.message)
    if not ack.ok:
        return 2
    return _apply_grip_if_needed(args, hand)


def _full_calibrated(args: argparse.Namespace, hand: HandSide, target: PoseXYZ) -> int:
    dex3 = Dex3HandController(UnitreeSdk2Dex3Backend(args.network_interface))
    open_ack = dex3.open(hand)
    logging.info("CUP_PIPELINE full open ok=%s message=%s", open_ack.ok, open_ack.message)
    if not open_ack.ok:
        return 3
    args.grip = "close"
    return _reach_calibrated(args, hand, target)


def _move_single_preset(args: argparse.Namespace, hand: HandSide, preset: str) -> int:
    arm = SafeArmPoseController(args.network_interface, duration_s=args.preset_duration)
    snapshot = arm.snapshot()
    _save_snapshot(args.snapshot_file, hand, snapshot)
    ack = arm.move_pose(hand, PRESETS[preset])
    logging.info("CUP_PIPELINE preset=%s ok=%s message=%s", preset, ack.ok, ack.message)
    return 0 if ack.ok else 4


def _preset_grasp_sequence(args: argparse.Namespace, hand: HandSide) -> int:
    arm = SafeArmPoseController(args.network_interface, duration_s=args.preset_duration)
    snapshot = arm.snapshot()
    _save_snapshot(args.snapshot_file, hand, snapshot)
    for preset in (args.preset_pregrasp, args.preset_final_a, args.preset_final_b):
        ack = arm.move_pose(hand, PRESETS[preset])
        logging.info("CUP_PIPELINE preset_step=%s ok=%s message=%s", preset, ack.ok, ack.message)
        if not ack.ok:
            return 4
    return _apply_grip_if_needed(args, hand)


def _apply_grip_if_needed(args: argparse.Namespace, hand: HandSide) -> int:
    if args.grip == "none":
        return 0
    dex3 = Dex3HandController(UnitreeSdk2Dex3Backend(args.network_interface))
    ack = dex3.open(hand) if args.grip == "open" else dex3.close(hand, "wrap")
    logging.info("CUP_PIPELINE grip=%s ok=%s message=%s", args.grip, ack.ok, ack.message)
    return 0 if ack.ok else 5


def _save_snapshot(path: str, hand: HandSide, snapshot: Dict[str, List[float]]) -> None:
    payload = {"hand": hand, "created_at": time.strftime("%Y-%m-%d %H:%M:%S"), "snapshot": snapshot}
    Path(path).write_text(json.dumps(payload, indent=2), encoding="utf-8")
    logging.info("CUP_PIPELINE saved start snapshot path=%s", path)


def main() -> int:
    args = parse_args()
    configure_logging(args.log_file)
    try:
        return run(args)
    except Exception as exc:
        logging.exception("CUP_PIPELINE failed: %s", exc)
        return 1


if __name__ == "__main__":
    sys.exit(main())


