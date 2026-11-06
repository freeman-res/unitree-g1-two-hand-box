from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from pathlib import Path
from typing import Dict, List

from .g1_arm_sdk2 import K_NOT_USED_JOINT, LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS, WAIST_JOINTS
from .safety import validate_hand_side
from .sdk2_common import initialize_sdk2
from .types import HandSide, RobotAck

ArmSnapshot = Dict[str, List[float]]


class SafeArmJogController:
    """Jog one G1 arm joint while holding both arms at commanded q."""

    def __init__(
        self,
        network_interface: str,
        duration_s: float = 1.0,
        control_dt_s: float = 0.02,
        kp: float = 60.0,
        kd: float = 1.5,
        max_delta_rad: float = 0.25,
    ) -> None:
        initialize_sdk2(network_interface)
        from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC

        self.duration_s = max(0.2, float(duration_s))
        self.control_dt_s = max(0.005, float(control_dt_s))
        self.kp = float(kp)
        self.kd = float(kd)
        self.max_delta_rad = abs(float(max_delta_rad))
        self._low_state = None
        self._crc = CRC()
        self._make_low_cmd = unitree_hg_msg_dds__LowCmd_
        self._publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self._publisher.Init()
        self._subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self._subscriber.Init(self._low_state_callback, 10)
        logging.info(
            "JOG controller ready topic_cmd=rt/arm_sdk topic_state=rt/lowstate duration_s=%.2f dt=%.3f kp=%.1f kd=%.1f max_delta=%.3f",
            self.duration_s,
            self.control_dt_s,
            self.kp,
            self.kd,
            self.max_delta_rad,
        )

    def snapshot(self) -> ArmSnapshot:
        self._wait_low_state()
        snap = {
            "waist": [float(self._low_state.motor_state[joint_id].q) for joint_id in WAIST_JOINTS],
            "left": [float(self._low_state.motor_state[joint_id].q) for joint_id in LEFT_ARM_JOINTS],
            "right": [float(self._low_state.motor_state[joint_id].q) for joint_id in RIGHT_ARM_JOINTS],
        }
        logging.info("JOG snapshot waist=%s left=%s right=%s", _round_list(snap["waist"]), _round_list(snap["left"]), _round_list(snap["right"]))
        return snap

    def jog(self, hand: HandSide, joint: int, delta_rad: float) -> RobotAck:
        if not 1 <= int(joint) <= 7:
            raise ValueError("--joint must be 1..7 within the selected arm")
        if abs(float(delta_rad)) > self.max_delta_rad:
            raise ValueError(f"delta {delta_rad:.3f} exceeds max_delta {self.max_delta_rad:.3f}")

        start = self.snapshot()
        target = {"waist": list(start["waist"]), "left": list(start["left"]), "right": list(start["right"])}
        local_index = int(joint) - 1
        target[hand][local_index] = start[hand][local_index] + float(delta_rad)
        motor_id = _joint_ids(hand)[local_index]
        logging.info(
            "JOG move hand=%s joint=%d motor_id=%d start=%.4f delta=%.4f target=%.4f",
            hand,
            joint,
            motor_id,
            start[hand][local_index],
            delta_rad,
            target[hand][local_index],
        )
        self._publish_interpolated(start, target)
        return RobotAck(ok=True, message=f"jog hand={hand} joint={joint} delta={delta_rad:.3f}")

    def restore(self, snapshot: ArmSnapshot) -> RobotAck:
        current = self.snapshot()
        if len(snapshot.get("waist", [])) != 3:
            logging.warning("JOG restore snapshot has no waist; keeping current waist=%s", _round_list(current["waist"]))
            snapshot["waist"] = list(current["waist"])
        logging.info("JOG restore target waist=%s left=%s right=%s", _round_list(snapshot["waist"]), _round_list(snapshot["left"]), _round_list(snapshot["right"]))
        self._publish_interpolated(current, snapshot)
        return RobotAck(ok=True, message="arms restored to saved snapshot")

    def _low_state_callback(self, msg) -> None:
        self._low_state = msg

    def _wait_low_state(self, timeout_s: float = 2.0) -> None:
        start_s = time.monotonic()
        while self._low_state is None:
            if time.monotonic() - start_s > timeout_s:
                raise TimeoutError("No rt/lowstate received from SDK2")
            time.sleep(0.01)

    def _publish_interpolated(self, start: ArmSnapshot, target: ArmSnapshot) -> None:
        steps = max(1, int(self.duration_s / self.control_dt_s))
        logging.info("JOG interpolate steps=%d", steps)
        for step in range(steps + 1):
            ratio = step / steps
            pose = {
                "waist": _blend(start["waist"], target["waist"], ratio),
                "left": _blend(start["left"], target["left"], ratio),
                "right": _blend(start["right"], target["right"], ratio),
            }
            cmd = self._make_command(pose)
            cmd.crc = self._crc.Crc(cmd)
            self._publisher.Write(cmd)
            time.sleep(self.control_dt_s)

    def _make_command(self, pose: ArmSnapshot):
        cmd = self._make_low_cmd()
        for hand, joint_ids in (("waist", WAIST_JOINTS), ("left", LEFT_ARM_JOINTS), ("right", RIGHT_ARM_JOINTS)):
            for local_index, joint_id in enumerate(joint_ids):
                motor = cmd.motor_cmd[joint_id]
                motor.q = float(pose[hand][local_index])
                motor.dq = 0.0
                motor.tau = 0.0
                motor.kp = self.kp
                motor.kd = self.kd
        cmd.motor_cmd[K_NOT_USED_JOINT].q = 1.0
        return cmd


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Safely jog one Unitree G1 arm joint by a small delta.")
    parser.add_argument("--hand", choices=("left", "right"), default="right")
    parser.add_argument("--joint", type=int, help="Local arm joint number 1..7. Right motors are 22..28.")
    parser.add_argument("--delta", type=float, default=0.0, help="Delta in radians from current joint q.")
    parser.add_argument("--network-interface", default="eth0")
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--max-delta", type=float, default=0.25)
    parser.add_argument("--snapshot-file", default="arm_jog_snapshot.json")
    parser.add_argument("--restore-only", action="store_true")
    parser.add_argument("--print-only", action="store_true", help="Print current arm joints and exit without moving.")
    parser.add_argument("--log-file", default="")
    return parser.parse_args()


def configure_logging(log_file: str) -> None:
    handlers = [logging.StreamHandler()]
    if log_file:
        handlers.append(logging.FileHandler(log_file, encoding="utf-8"))
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s", handlers=handlers)


def run(args: argparse.Namespace) -> int:
    hand = validate_hand_side(args.hand)
    controller = SafeArmJogController(args.network_interface, duration_s=args.duration, max_delta_rad=args.max_delta)

    if args.restore_only:
        snapshot = _load_snapshot(args.snapshot_file)
        ack = controller.restore(snapshot)
        logging.info("JOG restore_only ok=%s message=%s", ack.ok, ack.message)
        return 0 if ack.ok else 2

    snapshot = controller.snapshot()
    _save_snapshot(args.snapshot_file, hand, snapshot)
    if args.print_only:
        print("WAIST", _round_list(snapshot["waist"]))
        print("LEFT", _round_list(snapshot["left"]))
        print("RIGHT", _round_list(snapshot["right"]))
        return 0

    if args.joint is None:
        raise ValueError("--joint is required unless --print-only or --restore-only is used")
    ack = controller.jog(hand, args.joint, args.delta)
    logging.info("JOG result ok=%s message=%s", ack.ok, ack.message)
    return 0 if ack.ok else 3


def _joint_ids(hand: HandSide) -> tuple[int, ...]:
    return LEFT_ARM_JOINTS if hand == "left" else RIGHT_ARM_JOINTS


def _blend(start: List[float], target: List[float], ratio: float) -> List[float]:
    return [s * (1.0 - ratio) + t * ratio for s, t in zip(start, target)]


def _round_list(values: List[float]) -> List[float]:
    return [round(value, 4) for value in values]


def _save_snapshot(path: str, hand: HandSide, snapshot: ArmSnapshot) -> None:
    payload = {"hand": hand, "created_at": time.strftime("%Y-%m-%d %H:%M:%S"), "snapshot": snapshot}
    Path(path).write_text(json.dumps(payload, indent=2), encoding="utf-8")
    logging.info("JOG saved start snapshot path=%s", path)


def _load_snapshot(path: str) -> ArmSnapshot:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    snapshot = payload["snapshot"]
    if len(snapshot.get("left", [])) != 7 or len(snapshot.get("right", [])) != 7:
        raise ValueError(f"Bad snapshot file: {path}")
    logging.info("JOG loaded snapshot path=%s", path)
    result = {"left": [float(v) for v in snapshot["left"]], "right": [float(v) for v in snapshot["right"]]}
    if len(snapshot.get("waist", [])) == 3:
        result["waist"] = [float(v) for v in snapshot["waist"]]
    return result


def main() -> int:
    args = parse_args()
    configure_logging(args.log_file)
    try:
        return run(args)
    except Exception as exc:
        logging.exception("JOG failed: %s", exc)
        return 1


if __name__ == "__main__":
    sys.exit(main())


