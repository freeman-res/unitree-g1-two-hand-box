from __future__ import annotations

import json
import logging
import math
import time
from dataclasses import dataclass
from pathlib import Path

from .safety import DEFAULT_LIMITS, WorkspaceLimits, validate_pose
from .sdk2_common import initialize_sdk2
from .types import HandSide, PoseXYZ, RobotAck

WAIST_JOINTS = (12, 13, 14)
LEFT_ARM_JOINTS = (15, 16, 17, 18, 19, 20, 21)
RIGHT_ARM_JOINTS = (22, 23, 24, 25, 26, 27, 28)
K_NOT_USED_JOINT = 29


@dataclass(frozen=True)
class ArmCalibrationPoint:
    pose: PoseXYZ
    joints: tuple[float, float, float, float, float, float, float]


class ArmCalibration:
    def __init__(self, points_by_hand: dict[str, list[ArmCalibrationPoint]]) -> None:
        self.points_by_hand = points_by_hand

    @classmethod
    def load(cls, path: str) -> "ArmCalibration":
        calibration_path = Path(path)
        if not calibration_path.exists():
            raise FileNotFoundError(
                f"Arm calibration file not found: {calibration_path}. Copy config/arm_calibration.example.json "
                "to config/arm_calibration.json and record real safe joint poses first."
            )

        data = json.loads(calibration_path.read_text(encoding="utf-8"))
        points_by_hand: dict[str, list[ArmCalibrationPoint]] = {"left": [], "right": []}
        for hand in ("left", "right"):
            for item in data.get(hand, []):
                joints = tuple(float(v) for v in item["joints"])
                if len(joints) != 7:
                    raise ValueError(f"Calibration point for {hand} must contain exactly 7 joints")
                points_by_hand[hand].append(
                    ArmCalibrationPoint(
                        pose=PoseXYZ(float(item["x_m"]), float(item["y_m"]), float(item["z_m"])),
                        joints=joints,  # type: ignore[arg-type]
                    )
                )
        logging.info(
            "ARM SDK2 calibration loaded path=%s left_points=%d right_points=%d",
            calibration_path,
            len(points_by_hand["left"]),
            len(points_by_hand["right"]),
        )
        return cls(points_by_hand)

    def target_joints(self, hand: HandSide, target: PoseXYZ) -> tuple[float, float, float, float, float, float, float]:
        points = self.points_by_hand.get(hand, [])
        if not points:
            raise ValueError(f"No arm calibration points for hand={hand}")

        ranked = sorted(points, key=lambda point: _distance(point.pose, target))
        if len(ranked) == 1 or _distance(ranked[0].pose, target) < 1e-6:
            logging.info("ARM SDK2 calibration nearest hand=%s point=%s", hand, ranked[0])
            return ranked[0].joints

        neighbors = ranked[: min(3, len(ranked))]
        weights = [1.0 / max(_distance(point.pose, target), 1e-6) for point in neighbors]
        weight_sum = sum(weights)
        joints = []
        for joint_index in range(7):
            value = sum(point.joints[joint_index] * weights[index] for index, point in enumerate(neighbors)) / weight_sum
            joints.append(value)
        logging.info(
            "ARM SDK2 calibration interpolated hand=%s target=%s neighbors=%s joints=%s",
            hand,
            target,
            [point.pose for point in neighbors],
            [round(v, 4) for v in joints],
        )
        return tuple(joints)  # type: ignore[return-value]


class UnitreeSdk2ArmClient:
    """Real Unitree SDK2 joint-space arm client.

    It publishes to rt/arm_sdk and reads rt/lowstate. The x/y/z command is mapped
    through a calibration table, because reliable Cartesian control requires IK
    or recorded safe points on the actual robot.
    """

    def __init__(
        self,
        network_interface: str,
        calibration_path: str,
        duration_s: float = 2.0,
        control_dt_s: float = 0.02,
        kp: float = 60.0,
        kd: float = 1.5,
        limits: WorkspaceLimits = DEFAULT_LIMITS,
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
        self.limits = limits
        self.calibration = ArmCalibration.load(calibration_path)
        self._low_state = None
        self._crc = CRC()
        self._make_low_cmd = unitree_hg_msg_dds__LowCmd_
        self._publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self._publisher.Init()
        self._subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self._subscriber.Init(self._low_state_callback, 10)
        logging.info(
            "ARM SDK2 client ready topic_cmd=rt/arm_sdk topic_state=rt/lowstate duration_s=%.2f dt=%.3f kp=%.1f kd=%.1f",
            self.duration_s,
            self.control_dt_s,
            self.kp,
            self.kd,
        )

    def move_to(self, hand: HandSide, target: PoseXYZ) -> RobotAck:
        logging.info("ARM SDK2 move_to hand=%s target=%s", hand, target)
        validate_pose(target, self.limits)
        try:
            target_joints = self.calibration.target_joints(hand, target)
            self._wait_low_state()
            start_joints = self._current_arm_joints(hand)
            self._publish_interpolated(hand, start_joints, target_joints)
        except Exception as exc:
            logging.error("ARM SDK2 move_to failed hand=%s target=%s error=%s", hand, target, exc)
            return RobotAck(ok=False, message=f"arm sdk2 move_to failed: {exc}")
        return RobotAck(ok=True, message=f"ARM SDK2 move_to sent hand={hand} target={target} joints={target_joints}")

    def named_action(self, action: str) -> RobotAck:
        return RobotAck(ok=False, message=f"ARM SDK2 named_action is not implemented: {action}")

    def lift_relative(self, hand: HandSide, start: PoseXYZ, dz_m: float) -> RobotAck:
        target = PoseXYZ(start.x_m, start.y_m, start.z_m + dz_m)
        return self.move_to(hand, target)

    def _low_state_callback(self, msg) -> None:
        self._low_state = msg

    def _wait_low_state(self, timeout_s: float = 2.0) -> None:
        start_s = time.monotonic()
        while self._low_state is None:
            if time.monotonic() - start_s > timeout_s:
                raise TimeoutError("No rt/lowstate received from SDK2")
            time.sleep(0.01)

    def _current_arm_joints(self, hand: HandSide) -> tuple[float, float, float, float, float, float, float]:
        joint_ids = LEFT_ARM_JOINTS if hand == "left" else RIGHT_ARM_JOINTS
        values = tuple(float(self._low_state.motor_state[joint_id].q) for joint_id in joint_ids)
        logging.info("ARM SDK2 current joints hand=%s q=%s", hand, [round(v, 4) for v in values])
        return values  # type: ignore[return-value]

    def _publish_interpolated(
        self,
        hand: HandSide,
        start_joints: tuple[float, float, float, float, float, float, float],
        target_joints: tuple[float, float, float, float, float, float, float],
    ) -> None:
        joint_ids = LEFT_ARM_JOINTS if hand == "left" else RIGHT_ARM_JOINTS
        steps = max(1, int(self.duration_s / self.control_dt_s))
        logging.info("ARM SDK2 interpolate hand=%s steps=%d joint_ids=%s", hand, steps, joint_ids)
        for step in range(steps + 1):
            ratio = step / steps
            cmd = self._make_base_low_cmd()
            for local_index, joint_id in enumerate(joint_ids):
                q = start_joints[local_index] * (1.0 - ratio) + target_joints[local_index] * ratio
                motor = cmd.motor_cmd[joint_id]
                motor.q = float(q)
                motor.dq = 0.0
                motor.tau = 0.0
                motor.kp = self.kp
                motor.kd = self.kd
            cmd.motor_cmd[K_NOT_USED_JOINT].q = 1.0
            cmd.crc = self._crc.Crc(cmd)
            self._publisher.Write(cmd)
            time.sleep(self.control_dt_s)

    def _make_base_low_cmd(self):
        cmd = self._make_low_cmd()
        if self._low_state is not None:
            for joint_id in WAIST_JOINTS + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS:
                cmd.motor_cmd[joint_id].q = float(self._low_state.motor_state[joint_id].q)
                cmd.motor_cmd[joint_id].dq = 0.0
                cmd.motor_cmd[joint_id].tau = 0.0
                cmd.motor_cmd[joint_id].kp = self.kp
                cmd.motor_cmd[joint_id].kd = self.kd
        return cmd


def _distance(a: PoseXYZ, b: PoseXYZ) -> float:
    return math.sqrt((a.x_m - b.x_m) ** 2 + (a.y_m - b.y_m) ** 2 + (a.z_m - b.z_m) ** 2)



