from __future__ import annotations

import argparse
import logging
import sys
from typing import Any

from .arm_client import ArmClient
from .dex3_hand import Dex3HandController, DryRunDex3Backend, JsonUdpDex3Backend
from .safety import validate_hand_side
from .types import GripState, PoseXYZ


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Task 1: parameterized G1 arm movement and three-finger gripper control."
    )
    parser.add_argument("--hand", choices=("left", "right"), required=True)
    parser.add_argument("--x", type=float, required=True, help="Forward coordinate in base_link, meters.")
    parser.add_argument("--y", type=float, required=True, help="Left/right coordinate in base_link, meters.")
    parser.add_argument("--z", type=float, required=True, help="Height coordinate in base_link, meters.")
    parser.add_argument("--grip", choices=("open", "close"), required=True)
    parser.add_argument("--backend", choices=("dry-run", "udp", "sdk2"), default="udp")
    parser.add_argument("--arm-host", default="127.0.0.1")
    parser.add_argument("--arm-port", type=int, default=15001)
    parser.add_argument("--hand-host", default="127.0.0.1")
    parser.add_argument("--hand-port", type=int, default=15002)
    parser.add_argument("--network-interface", default="", help="Unitree SDK2 network interface, for example eth0.")
    parser.add_argument("--arm-calibration", default="config/arm_calibration.json")
    parser.add_argument("--arm-duration", type=float, default=2.0)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--dry-run", action="store_true", help="Alias for --backend dry-run.")
    parser.add_argument("--log-file", default="")
    return parser.parse_args()


def effective_backend(args: argparse.Namespace) -> str:
    return "dry-run" if args.dry_run else args.backend


def configure_logging(log_file: str) -> None:
    handlers: list[logging.Handler] = [logging.StreamHandler()]
    if log_file:
        handlers.append(logging.FileHandler(log_file, encoding="utf-8"))
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s", handlers=handlers)


def make_arm_client(args: argparse.Namespace) -> Any:
    backend = effective_backend(args)
    if backend == "sdk2":
        logging.info("TASK1 arm backend=sdk2 interface=%s calibration=%s", args.network_interface, args.arm_calibration)
        from .g1_arm_sdk2 import UnitreeSdk2ArmClient

        return UnitreeSdk2ArmClient(
            network_interface=args.network_interface,
            calibration_path=args.arm_calibration,
            duration_s=args.arm_duration,
        )
    logging.info("TASK1 arm backend=%s host=%s port=%d", backend, args.arm_host, args.arm_port)
    return ArmClient(args.arm_host, args.arm_port, args.timeout, dry_run=(backend == "dry-run"))


def make_hand_controller(args: argparse.Namespace) -> Dex3HandController:
    backend = effective_backend(args)
    if backend == "sdk2":
        logging.info("TASK1 hand backend=sdk2 interface=%s", args.network_interface)
        from .dex3_sdk2_backend import UnitreeSdk2Dex3Backend

        return Dex3HandController(UnitreeSdk2Dex3Backend(args.network_interface))
    if backend == "dry-run":
        logging.info("TASK1 hand backend=dry-run")
        return Dex3HandController(DryRunDex3Backend())
    logging.info("TASK1 hand backend=json-udp host=%s port=%d", args.hand_host, args.hand_port)
    return Dex3HandController(JsonUdpDex3Backend(args.hand_host, args.hand_port, args.timeout))


def run_task1(args: argparse.Namespace) -> int:
    backend = effective_backend(args)
    logging.info("TASK1 start backend=%s args=%s", backend, vars(args))
    hand = validate_hand_side(args.hand)
    grip: GripState = args.grip
    target = PoseXYZ(args.x, args.y, args.z)

    arm = make_arm_client(args)
    dex3 = make_hand_controller(args)

    logging.info("TASK1 target hand=%s pose=%s grip=%s backend=%s", hand, target, grip, backend)

    logging.info("TASK1 STEP 1/2 start: move arm to target")
    arm_ack = arm.move_to(hand, target)
    logging.info("TASK1 STEP 1/2 result ok=%s message=%s", arm_ack.ok, arm_ack.message)
    if not arm_ack.ok:
        logging.error("TASK1 failed: arm did not reach target")
        return 2

    logging.info("TASK1 STEP 2/2 start: set gripper=%s", grip)
    hand_ack = dex3.open(hand) if grip == "open" else dex3.close(hand, "wrap")
    logging.info("TASK1 STEP 2/2 result ok=%s message=%s", hand_ack.ok, hand_ack.message)
    if not hand_ack.ok:
        logging.error("TASK1 failed: gripper command failed")
        return 3

    logging.info("TASK1 result=OK")
    return 0


def main() -> int:
    args = parse_args()
    configure_logging(args.log_file)
    try:
        return run_task1(args)
    except ValueError as exc:
        logging.error("TASK1 invalid input: %s", exc)
        return 1
    except RuntimeError as exc:
        logging.error("TASK1 runtime error: %s", exc)
        return 4


if __name__ == "__main__":
    sys.exit(main())
