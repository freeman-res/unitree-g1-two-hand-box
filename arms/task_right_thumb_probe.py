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


RIGHT_THUMB_TEST_POSE = (0.14, -0.35, 0.02, 1.05, -0.12, -0.25, 0.005)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Interactively test right Dex3 thumb poses while keeping fingers straight.")
    parser.add_argument("--network-interface", default="eth0")
    parser.add_argument("--execute", action="store_true", help="Actually send arm/hand commands. Without this, only prints q values.")
    parser.add_argument("--raise-right-arm", action="store_true", help="Move right arm to a high preset before thumb testing.")
    parser.add_argument("--bend-right-elbow", action="store_true", help="Bend only the right elbow before thumb testing.")
    parser.add_argument("--test-right-arm-pose", action="store_true", help="Move right arm to a low heat thumb-test pose.")
    parser.add_argument("--right-elbow-rad", type=float, default=0.95, help="Absolute target for right local joint 4 / elbow.")
    parser.add_argument("--right-arm-preset", choices=sorted(PRESETS), default="right_cup_side_high_table80")
    parser.add_argument("--arm-duration", type=float, default=3.0)
    parser.add_argument("--auto", action="store_true", help="Automatically step through candidate thumb poses.")
    parser.add_argument("--auto-squeeze", action="store_true", help="Automatically step through thumb squeeze candidate poses.")
    parser.add_argument("--auto-hold-s", type=float, default=0.8)
    parser.add_argument("--snapshot-file", default="right_thumb_probe_start.json")
    parser.add_argument("--restore-arm-only", action="store_true")
    parser.add_argument("--log-file", default="right_thumb_probe.log")
    return parser.parse_args()


def configure_logging(log_file: str) -> None:
    handlers: list[logging.Handler] = [logging.StreamHandler()]
    if log_file:
        handlers.append(logging.FileHandler(log_file, encoding="utf-8"))
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s", handlers=handlers)


def run(args: argparse.Namespace) -> int:
    logging.info("THUMB_PROBE start args=%s", vars(args))
    arm: SafeArmPoseController | None = None

    if args.restore_arm_only:
        if args.execute:
            arm = SafeArmPoseController(args.network_interface, duration_s=args.arm_duration)
            snapshot = load_snapshot(args.snapshot_file)
            ack = arm.restore(snapshot)
            logging.info("THUMB_PROBE restore arm ok=%s message=%s", ack.ok, ack.message)
            return 0 if ack.ok else 2
        print("DRY_RUN restore arm from", args.snapshot_file)
        return 0

    arm_setup_requested = args.raise_right_arm or args.bend_right_elbow or args.test_right_arm_pose
    if args.execute and arm_setup_requested:
        arm = SafeArmPoseController(args.network_interface, duration_s=args.arm_duration)
        snapshot = arm.snapshot()
        _save_snapshot(args.snapshot_file, snapshot)

    if args.raise_right_arm:
        preset = PRESETS[args.right_arm_preset]
        logging.info("THUMB_PROBE raise right arm preset=%s q=%s execute=%s", args.right_arm_preset, preset, args.execute)
        if args.execute:
            assert arm is not None
            ack = arm.move_pose("right", preset)
            logging.info("THUMB_PROBE raise right arm ok=%s message=%s", ack.ok, ack.message)
            if not ack.ok:
                return 3
        else:
            print("DRY_RUN raise right arm", args.right_arm_preset, [round(value, 4) for value in preset])

    if args.test_right_arm_pose:
        logging.info("THUMB_PROBE move right arm to test pose q=%s execute=%s", RIGHT_THUMB_TEST_POSE, args.execute)
        if args.execute:
            assert arm is not None
            ack = arm.move_pose("right", RIGHT_THUMB_TEST_POSE)
            logging.info("THUMB_PROBE test right arm pose ok=%s message=%s", ack.ok, ack.message)
            if not ack.ok:
                return 7
        else:
            print("DRY_RUN test right arm pose", [round(value, 4) for value in RIGHT_THUMB_TEST_POSE])

    if args.bend_right_elbow:
        if args.execute:
            assert arm is not None
            snapshot = arm.snapshot()
            right = list(snapshot["right"])
            old_elbow = right[3]
            right[3] = float(args.right_elbow_rad)
            logging.info("THUMB_PROBE bend right elbow old=%.4f target=%.4f execute=%s", old_elbow, right[3], args.execute)
            ack = arm.move_pose("right", tuple(right))
            logging.info("THUMB_PROBE bend right elbow ok=%s message=%s", ack.ok, ack.message)
            if not ack.ok:
                return 6
        else:
            print("DRY_RUN bend right elbow joint4 ->", round(float(args.right_elbow_rad), 4))

    hand = Dex3HandController(UnitreeSdk2Dex3Backend(args.network_interface)) if args.execute else None
    if args.auto_squeeze:
        return run_auto_sequence(args, hand, AUTO_THUMB_SQUEEZE_POSES, "squeeze")
    if args.auto:
        return run_auto_sequence(args, hand, AUTO_THUMB_POSES, "auto")
    print_help()
    while True:
        try:
            text = input("thumb> ").strip()
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
            code = run_auto_sequence(args, hand, AUTO_THUMB_POSES, "auto")
            if code != 0:
                return code
            continue
        if lowered == "squeeze":
            code = run_auto_sequence(args, hand, AUTO_THUMB_SQUEEZE_POSES, "squeeze")
            if code != 0:
                return code
            continue
        if lowered == "restore":
            if args.execute:
                if arm is None:
                    arm = SafeArmPoseController(args.network_interface, duration_s=args.arm_duration)
                snapshot = load_snapshot(args.snapshot_file)
                ack = arm.restore(snapshot)
                logging.info("THUMB_PROBE restore arm ok=%s message=%s", ack.ok, ack.message)
                if not ack.ok:
                    return 4
            else:
                print("DRY_RUN restore arm from", args.snapshot_file)
            continue
        if lowered == "open":
            values = (0.0, 0.0, 0.0)
        elif lowered == "base":
            values = (1.0, 0.0, 0.0)
        elif lowered == "up":
            values = (1.0, 1.0, 0.0)
        elif lowered == "current":
            values = (0.0, -0.5, 0.0)
        else:
            parts = lowered.replace(",", " ").split()
            if len(parts) != 3:
                print("Need three values: thumb_abduction thumb_flexion_0 thumb_flexion_1, or command: open/base/up/restore/quit")
                continue
            try:
                values = tuple(float(part) for part in parts)
            except ValueError:
                print("Bad number. Example: 1.0 0.5 0.0")
                continue
        pose = make_thumb_pose(values)
        q_values = dex3_pose_to_sdk2_q("right", pose)
        print(
            "RIGHT_THUMB_POSE normalized=",
            [round(value, 3) for value in values],
            "sdk_q=",
            [round(value, 4) for value in q_values],
        )
        if args.execute:
            assert hand is not None
            ack = hand.set_pose("right", pose, f"right_thumb_probe_{values[0]:.2f}_{values[1]:.2f}_{values[2]:.2f}")
            logging.info("THUMB_PROBE hand ok=%s message=%s", ack.ok, ack.message)
            if not ack.ok:
                return 5
        time.sleep(max(0.0, float(args.auto_hold_s)))


def run_auto_sequence(
    args: argparse.Namespace,
    hand: Dex3HandController | None,
    poses: tuple[tuple[float, float, float], ...],
    label_prefix: str,
) -> int:
    print(f"AUTO thumb {label_prefix} poses. Press Enter for next, type q + Enter to stop.")
    for index, values in enumerate(poses, start=1):
        pose = make_thumb_pose(values)
        q_values = dex3_pose_to_sdk2_q("right", pose)
        print(
            f"POSE {index}/{len(poses)} normalized=",
            [round(value, 3) for value in values],
            "sdk_q=",
            [round(value, 4) for value in q_values],
        )
        if args.execute:
            assert hand is not None
            ack = hand.set_pose("right", pose, f"right_thumb_{label_prefix}_{index:02d}")
            logging.info("THUMB_PROBE auto pose=%d ok=%s message=%s", index, ack.ok, ack.message)
            if not ack.ok:
                return 5
        time.sleep(max(0.0, float(args.auto_hold_s)))
        text = input("next> ").strip().lower()
        if text in ("q", "quit", "exit"):
            return 0
    return 0


def make_thumb_pose(values: tuple[float, float, float]) -> Dex3Pose:
    thumb_abduction, thumb_flexion_0, thumb_flexion_1 = values
    return Dex3Pose(
        thumb_abduction,
        thumb_flexion_0,
        thumb_flexion_1,
        0.0,
        0.0,
        0.0,
        0.0,
    )


AUTO_THUMB_POSES: tuple[tuple[float, float, float], ...] = ((0.0, 1.0, 0.0),)
AUTO_THUMB_SQUEEZE_POSES: tuple[tuple[float, float, float], ...] = (
    (0.0, 1.0, 0.0),
    (0.0, 0.5, 0.0),
    (0.0, 0.0, 0.0),
    (0.0, -0.5, 0.0),
    (0.0, -0.75, 0.0),
    (0.0, -1.0, 0.0),
)


def print_help() -> None:
    print(
        "Commands:\n"
        "  open              -> 0.0 0.0 0.0\n"
        "  base              -> 1.0 0.0 0.0\n"
        "  up                -> 1.0 1.0 0.0\n"
        "  auto              -> rerun automatic candidate sequence\n"
        "  squeeze           -> rerun thumb squeeze candidate sequence\n"
        "  current           -> current main-script squeeze pose\n"
        "  A F0 F1           -> custom thumb_abduction thumb_flexion_0 thumb_flexion_1\n"
        "  restore           -> restore right arm snapshot\n"
        "  quit              -> exit\n"
        "Index and middle fingers always stay at 0.0."
    )


def _save_snapshot(path: str, snapshot) -> None:
    payload = {"snapshot": snapshot}
    Path(path).write_text(__import__("json").dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    logging.info("THUMB_PROBE saved arm snapshot path=%s", path)


def main() -> None:
    args = parse_args()
    configure_logging(args.log_file)
    try:
        code = run(args)
    except Exception:
        logging.exception("THUMB_PROBE failed")
        raise
    sys.exit(code)


if __name__ == "__main__":
    main()
