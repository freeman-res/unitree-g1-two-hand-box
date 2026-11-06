from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from pathlib import Path
from typing import Dict, List

from .dex3_hand import Dex3HandController
from .dex3_sdk2_backend import UnitreeSdk2Dex3Backend
from .g1_arm_sdk2 import K_NOT_USED_JOINT, LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS, WAIST_JOINTS
from .safety import validate_hand_side
from .sdk2_common import initialize_sdk2
from .types import HandSide, RobotAck

ArmSnapshot = Dict[str, List[float]]


class SafeWristRollController:
    """Move one wrist-roll joint while keeping both G1 arms stiff at current q."""

    def __init__(
        self,
        network_interface: str,
        duration_s: float = 2.0,
        control_dt_s: float = 0.02,
        kp: float = 60.0,
        kd: float = 1.5,
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
        self._low_state = None
        self._crc = CRC()
        self._make_low_cmd = unitree_hg_msg_dds__LowCmd_
        self._publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self._publisher.Init()
        self._subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self._subscriber.Init(self._low_state_callback, 10)
        logging.info(
            "WRIST controller ready topic_cmd=rt/arm_sdk topic_state=rt/lowstate duration_s=%.2f dt=%.3f kp=%.1f kd=%.1f",
            self.duration_s,
            self.control_dt_s,
            self.kp,
            self.kd,
        )

    def snapshot(self) -> ArmSnapshot:
        self._wait_low_state()
        snap = {
            "waist": [float(self._low_state.motor_state[joint_id].q) for joint_id in WAIST_JOINTS],
            "left": [float(self._low_state.motor_state[joint_id].q) for joint_id in LEFT_ARM_JOINTS],
            "right": [float(self._low_state.motor_state[joint_id].q) for joint_id in RIGHT_ARM_JOINTS],
        }
        logging.info("WRIST snapshot waist=%s left=%s right=%s", _round_list(snap["waist"]), _round_list(snap["left"]), _round_list(snap["right"]))
        return snap

    def move_wrist_roll(self, hand: HandSide, target_rad: float) -> RobotAck:
        start = self.snapshot()
        target = {"waist": list(start["waist"]), "left": list(start["left"]), "right": list(start["right"])}
        target[hand][4] = float(target_rad)
        logging.info(
            "WRIST move hand=%s joint_local=5 motor_id=%d start_rad=%.4f target_rad=%.4f",
            hand,
            _joint_ids(hand)[4],
            start[hand][4],
            target[hand][4],
        )
        self._publish_interpolated(start, target)
        return RobotAck(ok=True, message=f"wrist roll moved hand={hand} target_rad={target_rad:.3f}")

    def restore(self, snapshot: ArmSnapshot) -> RobotAck:
        current = self.snapshot()
        if len(snapshot.get("waist", [])) != 3:
            logging.warning("WRIST restore snapshot has no waist; keeping current waist=%s", _round_list(current["waist"]))
            snapshot["waist"] = list(current["waist"])
        logging.info("WRIST restore target waist=%s left=%s right=%s", _round_list(snapshot["waist"]), _round_list(snapshot["left"]), _round_list(snapshot["right"]))
        self._publish_interpolated(current, snapshot)
        return RobotAck(ok=True, message="arms restored to saved snapshot")

    def hold(self, seconds: float) -> None:
        end_s = time.monotonic() + max(0.0, float(seconds))
        while time.monotonic() < end_s:
            snap = self.snapshot()
            cmd = self._make_command(snap)
            cmd.crc = self._crc.Crc(cmd)
            self._publisher.Write(cmd)
            time.sleep(self.control_dt_s)

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
        logging.info("WRIST interpolate steps=%d", steps)
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
    parser = argparse.ArgumentParser(description="Rotate G1 wrist roll safely, then open/close Dex3 hand.")
    parser.add_argument("--hand", choices=("left", "right"), default="right")
    parser.add_argument("--method", choices=("wrap", "pinch", "side"), default="wrap")
    parser.add_argument("--network-interface", default="eth0")
    parser.add_argument("--wrist-rad", type=float, default=1.57, help="Absolute target for local arm joint 5 / wrist roll.")
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--hold-after-wrist", type=float, default=0.5)
    restore_group = parser.add_mutually_exclusive_group()
    restore_group.add_argument("--restore", dest="restore", action="store_true", help="Restore arms to the saved start snapshot after the hand test.")
    restore_group.add_argument("--no-restore", dest="restore", action="store_false", help="Leave the wrist at --wrist-rad after the hand test.")
    parser.set_defaults(restore=True)
    parser.add_argument("--snapshot-file", default="last_wrist_roll_start.json")
    parser.add_argument("--restore-only", action="store_true", help="Only restore arms from --snapshot-file, without moving wrist or hand.")
    parser.add_argument("--log-file", default="wrist_roll_grasp.log")
    return parser.parse_args()


def configure_logging(log_file: str) -> None:
    handlers: list[logging.Handler] = [logging.StreamHandler()]
    if log_file:
        handlers.append(logging.FileHandler(log_file, encoding="utf-8"))
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s", handlers=handlers)


def run(args: argparse.Namespace) -> int:
    hand = validate_hand_side(args.hand)
    logging.info("WRIST_TASK start args=%s", vars(args))
    arm = SafeWristRollController(args.network_interface, duration_s=args.duration)
    if args.restore_only:
        snapshot = _load_snapshot(args.snapshot_file)
        restore_ack = arm.restore(snapshot)
        logging.info("WRIST_TASK restore_only ok=%s message=%s", restore_ack.ok, restore_ack.message)
        return 0 if restore_ack.ok else 5

    dex3 = Dex3HandController(UnitreeSdk2Dex3Backend(args.network_interface))

    start_snapshot = arm.snapshot()
    _save_snapshot(args.snapshot_file, hand, start_snapshot)
    try:
        wrist_ack = arm.move_wrist_roll(hand, args.wrist_rad)
        logging.info("WRIST_TASK wrist result ok=%s message=%s", wrist_ack.ok, wrist_ack.message)
        arm.hold(args.hold_after_wrist)

        logging.info("WRIST_TASK hand sequence 1/2: open before grasp")
        open_before = dex3.open(hand)
        logging.info("WRIST_TASK open_before ok=%s message=%s", open_before.ok, open_before.message)
        if not open_before.ok:
            return 2

        logging.info("WRIST_TASK hand sequence 2/2: close method=%s", args.method)
        close_ack = dex3.close(hand, args.method)
        logging.info("WRIST_TASK close ok=%s message=%s", close_ack.ok, close_ack.message)
        if not close_ack.ok:
            return 3

        logging.info("WRIST_TASK hand sequence complete: keep hand closed after grasp")
    finally:
        if args.restore:
            restore_ack = arm.restore(start_snapshot)
            logging.info("WRIST_TASK restore ok=%s message=%s", restore_ack.ok, restore_ack.message)

    logging.info("WRIST_TASK result=OK hand=%s method=%s wrist_rad=%.3f", hand, args.method, args.wrist_rad)
    return 0


def _joint_ids(hand: HandSide) -> tuple[int, ...]:
    return LEFT_ARM_JOINTS if hand == "left" else RIGHT_ARM_JOINTS


def _blend(start: list[float], target: list[float], ratio: float) -> list[float]:
    return [s * (1.0 - ratio) + t * ratio for s, t in zip(start, target)]


def _round_list(values: list[float]) -> list[float]:
    return [round(value, 4) for value in values]


def _save_snapshot(path: str, hand: HandSide, snapshot: ArmSnapshot) -> None:
    payload = {"hand": hand, "created_at": time.strftime("%Y-%m-%d %H:%M:%S"), "snapshot": snapshot}
    Path(path).write_text(json.dumps(payload, indent=2), encoding="utf-8")
    logging.info("WRIST_TASK saved start snapshot path=%s", path)


def _load_snapshot(path: str) -> ArmSnapshot:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    snapshot = payload["snapshot"]
    if len(snapshot.get("left", [])) != 7 or len(snapshot.get("right", [])) != 7:
        raise ValueError(f"Bad snapshot file: {path}")
    logging.info("WRIST_TASK loaded snapshot path=%s", path)
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
        logging.exception("WRIST_TASK failed: %s", exc)
        return 1


if __name__ == "__main__":
    sys.exit(main())







