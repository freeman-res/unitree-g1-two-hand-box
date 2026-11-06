from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from pathlib import Path

from .g1_arm_sdk2 import LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS
from .safety import validate_hand_side, validate_pose
from .sdk2_common import initialize_sdk2
from .types import PoseXYZ


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record current G1 arm joints as an x/y/z calibration point.")
    parser.add_argument("--hand", choices=("left", "right"), required=True)
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--z", type=float, required=True)
    parser.add_argument("--name", default="")
    parser.add_argument("--network-interface", default="", help="Unitree SDK2 network interface, for example eth0.")
    parser.add_argument("--output", default="config/arm_calibration.json")
    parser.add_argument("--log-file", default="")
    return parser.parse_args()


def configure_logging(log_file: str) -> None:
    handlers: list[logging.Handler] = [logging.StreamHandler()]
    if log_file:
        handlers.append(logging.FileHandler(log_file, encoding="utf-8"))
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s", handlers=handlers)


def wait_low_state(network_interface: str, timeout_s: float = 3.0):
    initialize_sdk2(network_interface)
    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

    state_holder = {"msg": None}

    def callback(msg) -> None:
        state_holder["msg"] = msg

    subscriber = ChannelSubscriber("rt/lowstate", LowState_)
    subscriber.Init(callback, 10)
    start_s = time.monotonic()
    while state_holder["msg"] is None:
        if time.monotonic() - start_s > timeout_s:
            raise TimeoutError("No rt/lowstate received from SDK2")
        time.sleep(0.01)
    return state_holder["msg"]


def load_or_create(path: Path) -> dict:
    if path.exists():
        return json.loads(path.read_text(encoding="utf-8"))
    return {"left": [], "right": []}


def main() -> int:
    args = parse_args()
    configure_logging(args.log_file)

    try:
        hand = validate_hand_side(args.hand)
        pose = PoseXYZ(args.x, args.y, args.z)
        validate_pose(pose)
        low_state = wait_low_state(args.network_interface)
        joint_ids = LEFT_ARM_JOINTS if hand == "left" else RIGHT_ARM_JOINTS
        joints = [float(low_state.motor_state[joint_id].q) for joint_id in joint_ids]

        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        data = load_or_create(output)
        data.setdefault("left", [])
        data.setdefault("right", [])
        point = {
            "name": args.name or f"{hand}_{args.x:.3f}_{args.y:.3f}_{args.z:.3f}",
            "x_m": args.x,
            "y_m": args.y,
            "z_m": args.z,
            "joints": joints,
        }
        data[hand].append(point)
        output.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        logging.info("CALIBRATION saved hand=%s pose=%s joints=%s output=%s", hand, pose, joints, output)
        return 0
    except Exception as exc:
        logging.error("CALIBRATION failed: %s", exc)
        return 1


if __name__ == "__main__":
    sys.exit(main())
