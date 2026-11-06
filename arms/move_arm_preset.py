from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from pathlib import Path
from typing import Dict, List, Tuple

from .g1_arm_sdk2 import K_NOT_USED_JOINT, LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS, WAIST_JOINTS
from .safety import validate_hand_side
from .sdk2_common import initialize_sdk2
from .types import HandSide, RobotAck

ArmSnapshot = Dict[str, List[float]]
JointPose = Tuple[float, float, float, float, float, float, float]

# These are right-arm mirrors of old working left-cup poses from the move project.
# Old left format: Lsp Lsr Lsy Lel LwR LwP LwY.
# Mirror rule used here: pitch same, roll sign flips, yaw sign flips, elbow same,
# wrist roll sign flips, wrist pitch same, wrist yaw sign flips.
PRESETS: Dict[str, JointPose] = {
    "right_cup_side_high_table80": (-0.780, -0.700, 0.010, 0.978, -0.100, -0.760, 0.008),
    "right_cup_forward_left_high_table80": (-0.690, -0.700, 0.010, 0.978, -0.100, -0.600, 0.008),
    "right_cup_forward_right_high_table80": (-0.690, -0.100, 0.010, 0.978, -0.100, -0.600, 0.008),
    "right_cup_side_out_table76": (0.000, -0.700, 0.010, 0.978, -0.100, -0.400, 0.008),
    "right_cup_forward_left_table76": (-0.720, -0.700, 0.010, 0.978, -0.100, -0.660, 0.008),
    "right_cup_forward_right_table76": (-0.850, -0.100, 0.010, 0.978, -0.100, -0.660, 0.008),
}


class SafeArmPoseController:
    """Send one full 7-joint arm pose while keeping both G1 arms stiff."""

    def __init__(
        self,
        network_interface: str,
        duration_s: float = 4.0,
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
            "POSE controller ready topic_cmd=rt/arm_sdk topic_state=rt/lowstate duration_s=%.2f dt=%.3f kp=%.1f kd=%.1f",
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
        logging.info("POSE snapshot waist=%s left=%s right=%s", _round_list(snap["waist"]), _round_list(snap["left"]), _round_list(snap["right"]))
        return snap

    def move_pose(self, hand: HandSide, target_joints: JointPose) -> RobotAck:
        start = self.snapshot()
        target = {"waist": list(start["waist"]), "left": list(start["left"]), "right": list(start["right"])}
        target[hand] = [float(value) for value in target_joints]
        logging.info(
            "POSE move hand=%s start=%s target=%s motor_ids=%s",
            hand,
            _round_list(start[hand]),
            _round_list(target[hand]),
            _joint_ids(hand),
        )
        self._publish_interpolated(start, target)
        return RobotAck(ok=True, message=f"arm pose sent hand={hand} target={_round_list(target[hand])}")

    def move_poses(self, left_joints: JointPose, right_joints: JointPose) -> RobotAck:
        start = self.snapshot()
        target = {
            "waist": list(start["waist"]),
            "left": [float(value) for value in left_joints],
            "right": [float(value) for value in right_joints],
        }
        logging.info(
            "POSE move both start_left=%s start_right=%s target_left=%s target_right=%s",
            _round_list(start["left"]),
            _round_list(start["right"]),
            _round_list(target["left"]),
            _round_list(target["right"]),
        )
        self._publish_interpolated(start, target)
        return RobotAck(
            ok=True,
            message=f"both arms sent left={_round_list(target['left'])} right={_round_list(target['right'])}",
        )

    def restore(self, snapshot: ArmSnapshot) -> RobotAck:
        current = self.snapshot()
        if len(snapshot.get("waist", [])) != 3:
            logging.warning("POSE restore snapshot has no waist; keeping current waist=%s", _round_list(current["waist"]))
            snapshot["waist"] = list(current["waist"])
        logging.info("POSE restore target waist=%s left=%s right=%s", _round_list(snapshot["waist"]), _round_list(snapshot["left"]), _round_list(snapshot["right"]))
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
        logging.info("POSE interpolate steps=%d", steps)
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
    parser = argparse.ArgumentParser(description="Move one G1 arm to a named or explicit 7-joint pose safely.")
    parser.add_argument("--hand", choices=("left", "right"), default="right")
    parser.add_argument("--preset", choices=sorted(PRESETS), help="Named old-move cup pose mirrored for the right arm.")
    parser.add_argument("--joints", default="", help="Comma-separated 7 joint q values, overrides --preset.")
    parser.add_argument("--network-interface", default="eth0")
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--snapshot-file", default="arm_pose_snapshot.json")
    parser.add_argument("--restore-only", action="store_true")
    parser.add_argument("--print-only", action="store_true", help="Print current arm joints and presets without moving.")
    parser.add_argument("--log-file", default="arm_pose.log")
    return parser.parse_args()


def configure_logging(log_file: str) -> None:
    handlers = [logging.StreamHandler()]
    if log_file:
        handlers.append(logging.FileHandler(log_file, encoding="utf-8"))
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s", handlers=handlers)


def run(args: argparse.Namespace) -> int:
    hand = validate_hand_side(args.hand)
    logging.info("POSE_TASK start args=%s", vars(args))
    controller = SafeArmPoseController(args.network_interface, duration_s=args.duration)

    if args.restore_only:
        snapshot = _load_snapshot(args.snapshot_file)
        ack = controller.restore(snapshot)
        logging.info("POSE_TASK restore_only ok=%s message=%s", ack.ok, ack.message)
        return 0 if ack.ok else 2

    snapshot = controller.snapshot()
    _save_snapshot(args.snapshot_file, hand, snapshot)

    if args.print_only:
        print("WAIST", _round_list(snapshot["waist"]))
        print("LEFT", _round_list(snapshot["left"]))
        print("RIGHT", _round_list(snapshot["right"]))
        print("PRESETS")
        for name, joints in sorted(PRESETS.items()):
            print(name, _round_list(list(joints)))
        return 0

    target_joints = _select_target(args)
    logging.info("POSE_TASK selected target=%s", _round_list(list(target_joints)))
    ack = controller.move_pose(hand, target_joints)
    logging.info("POSE_TASK result ok=%s message=%s", ack.ok, ack.message)
    return 0 if ack.ok else 3


def _select_target(args: argparse.Namespace) -> JointPose:
    if args.joints:
        values = [float(part.strip()) for part in args.joints.split(",") if part.strip()]
        if len(values) != 7:
            raise ValueError("--joints must contain exactly 7 comma-separated values")
        return tuple(values)  # type: ignore[return-value]
    if not args.preset:
        raise ValueError("Use --preset, --joints, --print-only, or --restore-only")
    return PRESETS[args.preset]


def _joint_ids(hand: HandSide) -> Tuple[int, ...]:
    return LEFT_ARM_JOINTS if hand == "left" else RIGHT_ARM_JOINTS


def _blend(start: List[float], target: List[float], ratio: float) -> List[float]:
    return [s * (1.0 - ratio) + t * ratio for s, t in zip(start, target)]


def _round_list(values: List[float]) -> List[float]:
    return [round(value, 4) for value in values]


def _save_snapshot(path: str, hand: HandSide, snapshot: ArmSnapshot) -> None:
    payload = {"hand": hand, "created_at": time.strftime("%Y-%m-%d %H:%M:%S"), "snapshot": snapshot}
    Path(path).write_text(json.dumps(payload, indent=2), encoding="utf-8")
    logging.info("POSE_TASK saved start snapshot path=%s", path)


def _load_snapshot(path: str) -> ArmSnapshot:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    snapshot = payload["snapshot"]
    if len(snapshot.get("left", [])) != 7 or len(snapshot.get("right", [])) != 7:
        raise ValueError(f"Bad snapshot file: {path}")
    logging.info("POSE_TASK loaded snapshot path=%s", path)
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
        logging.exception("POSE_TASK failed: %s", exc)
        return 1


if __name__ == "__main__":
    sys.exit(main())


