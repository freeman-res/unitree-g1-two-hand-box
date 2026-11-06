from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

from .dex3_hand import Dex3HandController, Dex3Pose
from .dex3_sdk2_backend import UnitreeSdk2Dex3Backend, dex3_pose_to_sdk2_q
from .move_arm_preset import PRESETS, SafeArmPoseController
from .restore_two_hand_path import load_snapshot
from .task_right_thumb_probe import RIGHT_THUMB_TEST_POSE


FIXED_THUMB = (0.0, 1.0, 0.0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Test right Dex3 index/middle straightening while keeping the thumb fixed.")
    parser.add_argument("--network-interface", default="eth0")
    parser.add_argument("--execute", action="store_true", help="Actually send arm/hand commands. Without this, only prints q values.")
    parser.add_argument("--test-right-arm-pose", action="store_true", help="Move right arm to a low heat test pose.")
    parser.add_argument("--right-arm-preset", choices=sorted(PRESETS), default="right_cup_side_high_table80")
    parser.add_argument("--arm-duration", type=float, default=3.0)
    parser.add_argument("--auto", action="store_true", help="Automatically step through candidate front-finger poses.")
    parser.add_argument("--auto-hold-s", type=float, default=0.8)
    parser.add_argument("--snapshot-file", default="right_front_fingers_probe_start.json")
    parser.add_argument("--restore-arm-only", action="store_true")
    parser.add_argument("--log-file", default="right_front_fingers_probe.log")
    return parser.parse_args()


def configure_logging(log_file: str) -> None:
    handlers: list[logging.Handler] = [logging.StreamHandler()]
    if log_file:
        handlers.append(logging.FileHandler(log_file, encoding="utf-8"))
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s", handlers=handlers)


def run(args: argparse.Namespace) -> int:
    logging.info("FRONT_FINGERS_PROBE start args=%s", vars(args))
    arm: SafeArmPoseController | None = None

    if args.restore_arm_only:
        if args.execute:
            arm = SafeArmPoseController(args.network_interface, duration_s=args.arm_duration)
            snapshot = load_snapshot(args.snapshot_file)
            ack = arm.restore(snapshot)
            logging.info("FRONT_FINGERS_PROBE restore arm ok=%s message=%s", ack.ok, ack.message)
            return 0 if ack.ok else 2
        print("DRY_RUN restore arm from", args.snapshot_file)
        return 0

    if args.execute and args.test_right_arm_pose:
        arm = SafeArmPoseController(args.network_interface, duration_s=args.arm_duration)
        snapshot = arm.snapshot()
        _save_snapshot(args.snapshot_file, snapshot)

    if args.test_right_arm_pose:
        logging.info("FRONT_FINGERS_PROBE move right arm to test pose q=%s execute=%s", RIGHT_THUMB_TEST_POSE, args.execute)
        if args.execute:
            assert arm is not None
            ack = arm.move_pose("right", RIGHT_THUMB_TEST_POSE)
            logging.info("FRONT_FINGERS_PROBE test right arm pose ok=%s message=%s", ack.ok, ack.message)
            if not ack.ok:
                return 3
        else:
            print("DRY_RUN test right arm pose", [round(value, 4) for value in RIGHT_THUMB_TEST_POSE])

    hand = Dex3HandController(UnitreeSdk2Dex3Backend(args.network_interface)) if args.execute else None
    if args.auto:
        return run_auto_sequence(args, hand)

    print_help()
    while True:
        try:
            text = input("front_fingers> ").strip()
        except EOFError:
            print()
            return 0
        if not text:
            continue
        lowered = text.lower()
        if lowered in ("q", "quit", "exit"):
            return 0
        if lowered in ("h", "help", "?"):
            print_help()
            continue
        if lowered == "auto":
            code = run_auto_sequence(args, hand)
            if code != 0:
                return code
            continue
        if lowered == "restore":
            if args.execute:
                if arm is None:
                    arm = SafeArmPoseController(args.network_interface, duration_s=args.arm_duration)
                snapshot = load_snapshot(args.snapshot_file)
                ack = arm.restore(snapshot)
                logging.info("FRONT_FINGERS_PROBE restore arm ok=%s message=%s", ack.ok, ack.message)
                if not ack.ok:
                    return 4
            else:
                print("DRY_RUN restore arm from", args.snapshot_file)
            continue
        parts = lowered.replace(",", " ").split()
        if len(parts) != 4:
            print("Need four values: index0 index1 middle0 middle1, or command: auto/restore/quit")
            continue
        try:
            values = tuple(float(part) for part in parts)
        except ValueError:
            print("Bad number. Example: -0.3 -0.3 -0.3 -0.3")
            continue
        code = send_pose(args, hand, values, "custom")
        if code != 0:
            return code


def run_auto_sequence(args: argparse.Namespace, hand: Dex3HandController | None) -> int:
    print("AUTO right front-finger sequence. Press Enter for next pose, type q + Enter to stop.")
    for index, values in enumerate(AUTO_FRONT_FINGER_POSES, start=1):
        code = send_pose(args, hand, values, f"auto_{index:02d}", index=index, count=len(AUTO_FRONT_FINGER_POSES))
        if code != 0:
            return code
        text = input("next> ").strip().lower()
        if text in ("q", "quit", "exit"):
            return 0
    return 0


def send_pose(
    args: argparse.Namespace,
    hand: Dex3HandController | None,
    values: tuple[float, float, float, float],
    label: str,
    index: int | None = None,
    count: int | None = None,
) -> int:
    pose = make_front_finger_pose(values)
    q_values = dex3_pose_to_sdk2_q("right", pose)
    prefix = f"POSE {index}/{count}" if index is not None and count is not None else "RIGHT_FRONT_FINGERS_POSE"
    print(
        prefix,
        "normalized=",
        [round(value, 3) for value in values],
        "sdk_q=",
        [round(value, 4) for value in q_values],
    )
    if args.execute:
        assert hand is not None
        ack = hand.set_pose("right", pose, f"right_front_fingers_{label}")
        logging.info("FRONT_FINGERS_PROBE hand ok=%s message=%s", ack.ok, ack.message)
        if not ack.ok:
            return 5
    time.sleep(max(0.0, float(args.auto_hold_s)))
    return 0


def make_front_finger_pose(values: tuple[float, float, float, float]) -> Dex3Pose:
    index_flexion_0, index_flexion_1, middle_flexion_0, middle_flexion_1 = values
    thumb_abduction, thumb_flexion_0, thumb_flexion_1 = FIXED_THUMB
    return Dex3Pose(
        thumb_abduction,
        thumb_flexion_0,
        thumb_flexion_1,
        index_flexion_0,
        index_flexion_1,
        middle_flexion_0,
        middle_flexion_1,
    )


AUTO_FRONT_FINGER_POSES: tuple[tuple[float, float, float, float], ...] = (
    (0.0, 0.0, 0.0, 0.0),
    (-0.1, -0.1, -0.1, -0.1),
    (-0.2, -0.2, -0.2, -0.2),
    (-0.3, -0.3, -0.3, -0.3),
    (-0.4, -0.4, -0.4, -0.4),
    (-0.5, -0.5, -0.5, -0.5),
)


def print_help() -> None:
    print(
        "Commands:\n"
        "  auto                    -> run candidate sequence\n"
        "  I0 I1 M0 M1             -> custom index0 index1 middle0 middle1\n"
        "  restore                 -> restore right arm snapshot\n"
        "  quit                    -> exit\n"
        "Thumb is fixed at normalized=(0.0, 1.0, 0.0)."
    )


def _save_snapshot(path: str, snapshot) -> None:
    payload = {"snapshot": snapshot}
    Path(path).write_text(__import__("json").dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    logging.info("FRONT_FINGERS_PROBE saved arm snapshot path=%s", path)


def main() -> None:
    args = parse_args()
    configure_logging(args.log_file)
    try:
        code = run(args)
    except Exception:
        logging.exception("FRONT_FINGERS_PROBE failed")
        raise
    sys.exit(code)


if __name__ == "__main__":
    main()
