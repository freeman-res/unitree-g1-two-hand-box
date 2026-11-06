from __future__ import annotations

import argparse
import logging
import sys
import time
from typing import Any

from .arm_client import ArmClient
from .dex3_hand import Dex3HandController, DryRunDex3Backend, JsonUdpDex3Backend
from .safety import pose_with_offset, validate_hand_side, validate_pose
from .types import GraspMethod, PoseXYZ


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Task 2: move to a known object, grasp it with Dex3, lift, and hold."
    )
    parser.add_argument("--hand", choices=("left", "right"), required=True)
    parser.add_argument("--object-x", type=float, required=True, help="Object center x in base_link, meters.")
    parser.add_argument("--object-y", type=float, required=True, help="Object center y in base_link, meters.")
    parser.add_argument("--object-z", type=float, required=True, help="Object grasp height z in base_link, meters.")
    parser.add_argument("--grasp", choices=("pinch", "wrap", "side"), default="wrap")
    parser.add_argument("--approach-dz", type=float, default=0.12, help="Height above object before descent, meters.")
    parser.add_argument("--lift-dz", type=float, default=0.15, help="Lift distance after grasp, meters.")
    parser.add_argument("--hold-s", type=float, default=5.0, help="Hold duration for success check.")
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
        logging.info("TASK2 arm backend=sdk2 interface=%s calibration=%s", args.network_interface, args.arm_calibration)
        from .g1_arm_sdk2 import UnitreeSdk2ArmClient

        return UnitreeSdk2ArmClient(
            network_interface=args.network_interface,
            calibration_path=args.arm_calibration,
            duration_s=args.arm_duration,
        )
    logging.info("TASK2 arm backend=%s host=%s port=%d", backend, args.arm_host, args.arm_port)
    return ArmClient(args.arm_host, args.arm_port, args.timeout, dry_run=(backend == "dry-run"))


def make_hand_controller(args: argparse.Namespace) -> Dex3HandController:
    backend = effective_backend(args)
    if backend == "sdk2":
        logging.info("TASK2 hand backend=sdk2 interface=%s", args.network_interface)
        from .dex3_sdk2_backend import UnitreeSdk2Dex3Backend

        return Dex3HandController(UnitreeSdk2Dex3Backend(args.network_interface))
    if backend == "dry-run":
        logging.info("TASK2 hand backend=dry-run")
        return Dex3HandController(DryRunDex3Backend())
    logging.info("TASK2 hand backend=json-udp host=%s port=%d", args.hand_host, args.hand_port)
    return Dex3HandController(JsonUdpDex3Backend(args.hand_host, args.hand_port, args.timeout))


def run_task2(args: argparse.Namespace) -> int:
    backend = effective_backend(args)
    logging.info("TASK2 start backend=%s args=%s", backend, vars(args))
    hand = validate_hand_side(args.hand)
    grasp: GraspMethod = args.grasp
    object_pose = PoseXYZ(args.object_x, args.object_y, args.object_z)
    pregrasp_pose = pose_with_offset(object_pose, dz_m=args.approach_dz)
    lifted_pose = pose_with_offset(object_pose, dz_m=args.lift_dz)

    logging.info("TASK2 validate object pose")
    validate_pose(object_pose)
    logging.info("TASK2 validate pregrasp pose")
    validate_pose(pregrasp_pose)
    logging.info("TASK2 validate lifted pose")
    validate_pose(lifted_pose)

    arm = make_arm_client(args)
    dex3 = make_hand_controller(args)

    logging.info(
        "TASK2 plan object=%s pregrasp=%s lifted=%s hand=%s grasp=%s approach_dz=%.3f lift_dz=%.3f hold_s=%.1f backend=%s",
        object_pose,
        pregrasp_pose,
        lifted_pose,
        hand,
        grasp,
        args.approach_dz,
        args.lift_dz,
        args.hold_s,
        backend,
    )

    steps = [
        ("open hand", lambda: dex3.open(hand)),
        ("move above object", lambda: arm.move_to(hand, pregrasp_pose)),
        ("descend to object", lambda: arm.move_to(hand, object_pose)),
        ("close hand", lambda: dex3.close(hand, grasp)),
        ("lift object", lambda: arm.move_to(hand, lifted_pose)),
    ]

    for index, (label, action) in enumerate(steps, start=1):
        logging.info("TASK2 STEP %d/%d start: %s", index, len(steps), label)
        ack = action()
        logging.info("TASK2 STEP %d/%d result ok=%s message=%s", index, len(steps), ack.ok, ack.message)
        if not ack.ok:
            logging.error("TASK2 failed at step %d/%d: %s", index, len(steps), label)
            return 2

    logging.info("TASK2 STEP hold start seconds=%.1f", args.hold_s)
    time.sleep(max(0.0, args.hold_s))
    logging.info("TASK2 STEP hold complete seconds=%.1f", args.hold_s)
    logging.info("TASK2 result=OK if object is still held after hold interval")
    return 0


def main() -> int:
    args = parse_args()
    configure_logging(args.log_file)
    try:
        return run_task2(args)
    except ValueError as exc:
        logging.error("TASK2 invalid input: %s", exc)
        return 1
    except RuntimeError as exc:
        logging.error("TASK2 runtime error: %s", exc)
        return 4


if __name__ == "__main__":
    sys.exit(main())
