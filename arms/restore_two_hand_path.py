from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path
from typing import Dict, List, Tuple

from .dex3_hand import Dex3HandController
from .dex3_sdk2_backend import UnitreeSdk2Dex3Backend
from .move_arm_preset import ArmSnapshot, SafeArmPoseController


JointPath = List[List[float]]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Restore both G1 arms by replaying the last saved two-hand path backwards.")
    parser.add_argument("--network-interface", default="eth0")
    parser.add_argument("--path-file", default="runs/box_two_hand_last_path.json")
    parser.add_argument("--snapshot-file", default="before_box_two_hand.json")
    parser.add_argument("--home-snapshot-file", default="", help="Optional fixed baseline home snapshot for the final restore waypoint.")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--fallback-direct", action="store_true", help="If --path-file is missing, restore directly from --snapshot-file.")
    parser.add_argument("--home-side-out-delta", type=float, default=0.00, help="Move final home shoulder-roll slightly away from torso: left += delta, right -= delta.")
    parser.add_argument("--hand-sequence", choices=("none", "close"), default="close")
    parser.add_argument("--grasp-method", choices=("pinch", "wrap", "side"), default="wrap")
    parser.add_argument("--log-file", default="restore_both_arms_reverse_path.log")
    return parser.parse_args()


def configure_logging(log_file: str) -> None:
    handlers = [logging.StreamHandler()]
    if log_file:
        handlers.append(logging.FileHandler(log_file, encoding="utf-8"))
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s", handlers=handlers)


def run(args: argparse.Namespace) -> int:
    controller = SafeArmPoseController(args.network_interface, duration_s=args.duration)
    close_code = close_hands_if_needed(args)
    if close_code != 0:
        return close_code
    path_file = Path(args.path_file)
    home_snapshot = load_snapshot(args.home_snapshot_file) if args.home_snapshot_file and Path(args.home_snapshot_file).exists() else None
    if not path_file.exists():
        if not args.fallback_direct:
            logging.error("RESTORE_PATH missing path_file=%s", path_file)
            return 2
        snapshot = home_snapshot if home_snapshot is not None else load_snapshot(args.snapshot_file)
        apply_home_side_out_offset(snapshot, args.home_side_out_delta)
        logging.warning("RESTORE_PATH path missing, fallback direct snapshot=%s", args.snapshot_file)
        ack = controller.restore(snapshot)
        logging.info("RESTORE_PATH fallback result ok=%s message=%s", ack.ok, ack.message)
        return 0 if ack.ok else 3

    payload = json.loads(path_file.read_text(encoding="utf-8"))
    left_path = validate_path(payload.get("left_path"), "left_path")
    right_path = validate_path(payload.get("right_path"), "right_path")
    if len(left_path) != len(right_path):
        raise ValueError(f"left/right path length mismatch: {len(left_path)} != {len(right_path)}")
    if len(left_path) < 2:
        raise ValueError("restore path must contain at least 2 waypoints")

    reverse_left = list(reversed(left_path))
    reverse_right = list(reversed(right_path))
    if home_snapshot is not None:
        reverse_left[-1] = list(home_snapshot["left"])
        reverse_right[-1] = list(home_snapshot["right"])
        logging.info("RESTORE_PATH final waypoint replaced by fixed home snapshot=%s", args.home_snapshot_file)
    apply_home_side_out_offset_to_paths(reverse_left, reverse_right, args.home_side_out_delta)
    logging.info("RESTORE_PATH loaded path=%s waypoints=%d", path_file, len(left_path))

    for index, (left_waypoint, right_waypoint) in enumerate(zip(reverse_left[1:], reverse_right[1:]), start=1):
        controller.duration_s = max(0.2, float(args.duration))
        logging.info(
            "RESTORE_PATH execute reverse waypoint %d/%d left=%s right=%s duration=%.2f",
            index,
            len(reverse_left) - 1,
            round_list(left_waypoint),
            round_list(right_waypoint),
            controller.duration_s,
        )
        ack = controller.move_poses(tuple(left_waypoint), tuple(right_waypoint))  # type: ignore[arg-type]
        logging.info("RESTORE_PATH waypoint %d/%d ok=%s message=%s", index, len(reverse_left) - 1, ack.ok, ack.message)
        if not ack.ok:
            return 3
    logging.info("RESTORE_PATH result=OK")
    return 0


def close_hands_if_needed(args: argparse.Namespace) -> int:
    if args.hand_sequence == "none":
        logging.info("RESTORE_PATH hand sequence disabled")
        return 0
    dex3 = Dex3HandController(UnitreeSdk2Dex3Backend(args.network_interface))
    logging.info("RESTORE_PATH close both hands before home method=%s", args.grasp_method)
    ack = dex3.close_both(args.grasp_method)
    logging.info("RESTORE_PATH close both ok=%s message=%s", ack.ok, ack.message)
    return 0 if ack.ok else 4


def validate_path(value, label: str) -> JointPath:
    if not isinstance(value, list):
        raise ValueError(f"{label} must be a list")
    result: JointPath = []
    for index, waypoint in enumerate(value):
        if not isinstance(waypoint, list) or len(waypoint) != 7:
            raise ValueError(f"{label}[{index}] must contain 7 joint values")
        result.append([float(item) for item in waypoint])
    return result


def load_snapshot(path: str) -> ArmSnapshot:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    snapshot = payload["snapshot"]
    if len(snapshot.get("left", [])) != 7 or len(snapshot.get("right", [])) != 7:
        raise ValueError(f"Bad snapshot file: {path}")
    result: ArmSnapshot = {
        "left": [float(v) for v in snapshot["left"]],
        "right": [float(v) for v in snapshot["right"]],
    }
    if len(snapshot.get("waist", [])) == 3:
        result["waist"] = [float(v) for v in snapshot["waist"]]
    return result


def apply_home_side_out_offset(snapshot: ArmSnapshot, delta: float) -> None:
    delta = max(0.0, float(delta))
    if delta <= 0.0:
        return
    snapshot["left"][1] += delta
    snapshot["right"][1] -= delta
    logging.info(
        "RESTORE_PATH adjusted home side-out delta=%.3f left_shoulder_roll=%.4f right_shoulder_roll=%.4f",
        delta,
        snapshot["left"][1],
        snapshot["right"][1],
    )


def apply_home_side_out_offset_to_paths(left_path: JointPath, right_path: JointPath, delta: float) -> None:
    delta = max(0.0, float(delta))
    if delta <= 0.0 or not left_path or not right_path:
        return
    left_path[-1][1] += delta
    right_path[-1][1] -= delta
    logging.info(
        "RESTORE_PATH adjusted final waypoint side-out delta=%.3f left_shoulder_roll=%.4f right_shoulder_roll=%.4f",
        delta,
        left_path[-1][1],
        right_path[-1][1],
    )


def round_list(values: List[float]) -> List[float]:
    return [round(value, 4) for value in values]


def main() -> int:
    args = parse_args()
    configure_logging(args.log_file)
    try:
        return run(args)
    except Exception as exc:
        logging.exception("RESTORE_PATH failed: %s", exc)
        return 1


if __name__ == "__main__":
    sys.exit(main())
