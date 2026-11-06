from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

from .dex3_hand import Dex3Pose
from .dex3_sdk2_backend import UnitreeSdk2Dex3Backend, dex3_pose_to_sdk2_q
from .move_arm_preset import SafeArmPoseController
from .restore_two_hand_path import load_snapshot
from .task_right_thumb_probe import RIGHT_THUMB_TEST_POSE


BASE_THUMB_POSE = Dex3Pose(0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Probe raw right Dex3 motor channels 3..6 one by one.")
    parser.add_argument("--network-interface", default="eth0")
    parser.add_argument("--execute", action="store_true", help="Actually send arm/hand commands. Without this, only prints q values.")
    parser.add_argument("--test-right-arm-pose", action="store_true")
    parser.add_argument("--arm-duration", type=float, default=3.0)
    parser.add_argument("--auto", action="store_true")
    parser.add_argument("--auto-hold-s", type=float, default=0.8)
    parser.add_argument("--probe-q", type=float, default=-0.6)
    parser.add_argument("--snapshot-file", default="right_dex3_motor_probe_start.json")
    parser.add_argument("--restore-arm-only", action="store_true")
    parser.add_argument("--log-file", default="right_dex3_motor_probe.log")
    return parser.parse_args()


def configure_logging(log_file: str) -> None:
    handlers: list[logging.Handler] = [logging.StreamHandler()]
    if log_file:
        handlers.append(logging.FileHandler(log_file, encoding="utf-8"))
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s", handlers=handlers)


def run(args: argparse.Namespace) -> int:
    logging.info("DEX3_MOTOR_PROBE start args=%s", vars(args))
    arm: SafeArmPoseController | None = None

    if args.restore_arm_only:
        if args.execute:
            arm = SafeArmPoseController(args.network_interface, duration_s=args.arm_duration)
            snapshot = load_snapshot(args.snapshot_file)
            ack = arm.restore(snapshot)
            logging.info("DEX3_MOTOR_PROBE restore arm ok=%s message=%s", ack.ok, ack.message)
            return 0 if ack.ok else 2
        print("DRY_RUN restore arm from", args.snapshot_file)
        return 0

    if args.execute and args.test_right_arm_pose:
        arm = SafeArmPoseController(args.network_interface, duration_s=args.arm_duration)
        snapshot = arm.snapshot()
        _save_snapshot(args.snapshot_file, snapshot)

    if args.test_right_arm_pose:
        logging.info("DEX3_MOTOR_PROBE move right arm to test pose q=%s execute=%s", RIGHT_THUMB_TEST_POSE, args.execute)
        if args.execute:
            assert arm is not None
            ack = arm.move_pose("right", RIGHT_THUMB_TEST_POSE)
            logging.info("DEX3_MOTOR_PROBE test right arm pose ok=%s message=%s", ack.ok, ack.message)
            if not ack.ok:
                return 3
        else:
            print("DRY_RUN test right arm pose", [round(value, 4) for value in RIGHT_THUMB_TEST_POSE])

    backend = UnitreeSdk2Dex3Backend(args.network_interface) if args.execute else None
    if args.auto:
        return run_auto_sequence(args, backend)

    print_help()
    while True:
        try:
            text = input("motor_probe> ").strip().lower()
        except EOFError:
            print()
            return 0
        if not text:
            continue
        if text in ("q", "quit", "exit"):
            return 0
        if text in ("h", "help", "?"):
            print_help()
            continue
        if text == "auto":
            code = run_auto_sequence(args, backend)
            if code != 0:
                return code
            continue
        parts = text.replace(",", " ").split()
        if len(parts) < 2:
            print("Need: motor_id q. Example: 5 -0.6")
            continue
        try:
            motor_id = int(parts[0])
            q_value = float(parts[1])
        except ValueError:
            print("Bad input. Example: 5 -0.6")
            continue
        code = send_raw_q(args, backend, f"manual_motor_{motor_id}", {motor_id: q_value})
        if code != 0:
            return code


def run_auto_sequence(args: argparse.Namespace, backend: UnitreeSdk2Dex3Backend | None) -> int:
    print("AUTO raw right Dex3 motor sequence. Press Enter for next pose, type q + Enter to stop.")
    probe_q = float(args.probe_q)
    sequence = (
        ("base_thumb_only", {}),
        ("motor_3_only", {3: probe_q}),
        ("motor_4_only", {4: probe_q}),
        ("motor_5_only", {5: probe_q}),
        ("motor_6_only", {6: probe_q}),
        ("motors_3_4_pair", {3: probe_q, 4: probe_q}),
        ("motors_5_6_pair", {5: probe_q, 6: probe_q}),
    )
    for index, (label, overrides) in enumerate(sequence, start=1):
        code = send_raw_q(args, backend, label, overrides, index=index, count=len(sequence))
        if code != 0:
            return code
        text = input("next> ").strip().lower()
        if text in ("q", "quit", "exit"):
            return 0
    return 0


def send_raw_q(
    args: argparse.Namespace,
    backend: UnitreeSdk2Dex3Backend | None,
    label: str,
    overrides: dict[int, float],
    index: int | None = None,
    count: int | None = None,
) -> int:
    q_values = dex3_pose_to_sdk2_q("right", BASE_THUMB_POSE)
    for motor_id, q_value in overrides.items():
        if not 0 <= motor_id < len(q_values):
            print(f"Bad motor_id={motor_id}; expected 0..6")
            return 4
        q_values[motor_id] = float(q_value)
    prefix = f"POSE {index}/{count}" if index is not None and count is not None else "RIGHT_DEX3_RAW_Q"
    print(prefix, label, "q=", [round(value, 4) for value in q_values])
    if args.execute:
        assert backend is not None
        try:
            msg = backend._make_hand_cmd(q_values)  # noqa: SLF001 - deliberate raw motor probe.
            for _ in range(backend.repeat):
                backend._publishers["right"].Write(msg)  # noqa: SLF001
                if backend.period_s:
                    time.sleep(backend.period_s)
        except Exception as exc:
            logging.error("DEX3_MOTOR_PROBE send failed label=%s error=%s", label, exc)
            return 5
    time.sleep(max(0.0, float(args.auto_hold_s)))
    return 0


def print_help() -> None:
    print(
        "Commands:\n"
        "  auto          -> run raw motor 3..6 sequence\n"
        "  MOTOR Q       -> set one raw motor, example: 5 -0.6\n"
        "  quit          -> exit\n"
        "Thumb is held at current good pose; only raw motor ids 3..6 are probed."
    )


def _save_snapshot(path: str, snapshot) -> None:
    payload = {"snapshot": snapshot}
    Path(path).write_text(__import__("json").dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    logging.info("DEX3_MOTOR_PROBE saved arm snapshot path=%s", path)


def main() -> None:
    args = parse_args()
    configure_logging(args.log_file)
    try:
        code = run(args)
    except Exception:
        logging.exception("DEX3_MOTOR_PROBE failed")
        raise
    sys.exit(code)


if __name__ == "__main__":
    main()
