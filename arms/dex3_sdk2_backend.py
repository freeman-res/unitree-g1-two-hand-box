from __future__ import annotations

import logging
import time
from typing import Dict, Sequence

from .dex3_hand import Dex3Backend, Dex3Pose
from .sdk2_common import initialize_sdk2
from .types import HandSide, RobotAck

# SDK2 Dex3 joint order used by Unitree examples: 3 thumb joints, 2 index, 2 middle.
DEX3_JOINT_ORDER = (
    "thumb_0",
    "thumb_1",
    "thumb_2",
    "index_0",
    "index_1",
    "middle_0",
    "middle_1",
)

# Limits are taken from the official Unitree Dex3 example. The normalized project
# poses 0.0..1.0 are mapped from open q=0 toward these safe close directions.
RIGHT_CLOSE_Q = (1.05, 0.742, -1.75, 1.57, 1.75, 1.57, 1.75)
LEFT_CLOSE_Q = (1.05, -0.724, 1.75, -1.57, -1.75, -1.57, -1.75)


class UnitreeSdk2Dex3Backend(Dex3Backend):
    """Real Unitree SDK2 backend for the G1 Dex3 three-finger hand."""

    def __init__(
        self,
        network_interface: str = "",
        kp: float = 1.5,
        kd: float = 0.1,
        repeat: int = 25,
        period_s: float = 0.02,
    ) -> None:
        initialize_sdk2(network_interface)
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_

        self.kp = float(kp)
        self.kd = float(kd)
        self.repeat = max(1, int(repeat))
        self.period_s = max(0.0, float(period_s))
        self._make_message = unitree_hg_msg_dds__HandCmd_
        self._publishers = {
            "left": ChannelPublisher("rt/dex3/left/cmd", HandCmd_),
            "right": ChannelPublisher("rt/dex3/right/cmd", HandCmd_),
        }
        for side, publisher in self._publishers.items():
            publisher.Init()
            logging.info("DEX3 SDK2 publisher ready hand=%s topic=rt/dex3/%s/cmd", side, side)

    def send_pose(self, hand: HandSide, pose: Dex3Pose, label: str) -> RobotAck:
        q_values = dex3_pose_to_sdk2_q(hand, pose)
        logging.info("DEX3 SDK2 send hand=%s label=%s q=%s", hand, label, [round(v, 4) for v in q_values])

        try:
            msg = self._make_hand_cmd(q_values)
            publisher = self._publishers[hand]
            for _ in range(self.repeat):
                publisher.Write(msg)
                if self.period_s:
                    time.sleep(self.period_s)
        except Exception as exc:  # SDK2 exceptions differ between builds.
            logging.error("DEX3 SDK2 failed hand=%s label=%s error=%s", hand, label, exc)
            return RobotAck(ok=False, message=f"dex3 sdk2 command failed: {exc}")

        return RobotAck(ok=True, message=f"DEX3 SDK2 command sent hand={hand} label={label} q={q_values}")

    def send_poses(self, poses_by_hand: Dict[HandSide, Dex3Pose], label: str) -> RobotAck:
        q_by_hand = {hand: dex3_pose_to_sdk2_q(hand, pose) for hand, pose in poses_by_hand.items()}
        logging.info(
            "DEX3 SDK2 send both label=%s q=%s",
            label,
            {hand: [round(v, 4) for v in q_values] for hand, q_values in q_by_hand.items()},
        )
        try:
            messages = {hand: self._make_hand_cmd(q_values) for hand, q_values in q_by_hand.items()}
            for _ in range(self.repeat):
                for hand, msg in messages.items():
                    self._publishers[hand].Write(msg)
                if self.period_s:
                    time.sleep(self.period_s)
        except Exception as exc:  # SDK2 exceptions differ between builds.
            logging.error("DEX3 SDK2 failed label=%s error=%s", label, exc)
            return RobotAck(ok=False, message=f"dex3 sdk2 command failed: {exc}")

        return RobotAck(ok=True, message=f"DEX3 SDK2 command sent hands={sorted(q_by_hand)} label={label}")

    def _make_hand_cmd(self, q_values: Sequence[float]):
        msg = self._make_message()
        for motor_id, q in enumerate(q_values):
            motor = msg.motor_cmd[motor_id]
            motor.mode = _motor_mode(motor_id)
            motor.q = float(q)
            motor.dq = 0.0
            motor.tau = 0.0
            motor.kp = self.kp
            motor.kd = self.kd
        return msg


def dex3_pose_to_sdk2_q(hand: HandSide, pose: Dex3Pose) -> list[float]:
    normalized = [
        pose.thumb_abduction,
        pose.thumb_flexion_0,
        pose.thumb_flexion_1,
        pose.index_flexion_0,
        pose.index_flexion_1,
        pose.middle_flexion_0,
        pose.middle_flexion_1,
    ]
    close_q = LEFT_CLOSE_Q if hand == "left" else RIGHT_CLOSE_Q
    return [float(value) * close_q[index] for index, value in enumerate(normalized)]


def _motor_mode(motor_id: int, status: int = 1, timeout: int = 0) -> int:
    return (motor_id & 0x0F) | ((status & 0x07) << 4) | ((timeout & 0x01) << 7)
