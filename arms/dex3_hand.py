from __future__ import annotations

import json
import logging
import socket
import time
from dataclasses import dataclass
from typing import Dict, Protocol

from .types import GraspMethod, HandSide, RobotAck

DEX3_JOINTS = (
    "thumb_abduction",
    "thumb_flexion_0",
    "thumb_flexion_1",
    "index_flexion_0",
    "index_flexion_1",
    "middle_flexion_0",
    "middle_flexion_1",
)


@dataclass(frozen=True)
class Dex3Pose:
    """Normalized three-finger hand pose.

    Values are 0.0..1.0 where 0.0 means open and 1.0 means closed.
    The real Unitree Dex3 backend must map these normalized values to the
    actual joint units used by the SDK.
    """

    thumb_abduction: float
    thumb_flexion_0: float
    thumb_flexion_1: float
    index_flexion_0: float
    index_flexion_1: float
    middle_flexion_0: float
    middle_flexion_1: float

    def as_dict(self) -> dict[str, float]:
        return {
            "thumb_abduction": self.thumb_abduction,
            "thumb_flexion_0": self.thumb_flexion_0,
            "thumb_flexion_1": self.thumb_flexion_1,
            "index_flexion_0": self.index_flexion_0,
            "index_flexion_1": self.index_flexion_1,
            "middle_flexion_0": self.middle_flexion_0,
            "middle_flexion_1": self.middle_flexion_1,
        }


OPEN_POSE = Dex3Pose(0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00)
FLAT_PALM_POSE = Dex3Pose(0.00, 1.00, 0.00, 0.00, 0.00, 0.00, 0.00)
FLAT_PALM_THUMB_SQUEEZE_POSE = Dex3Pose(0.00, -0.50, 0.00, 0.00, 0.00, 0.00, 0.00)

GRASP_PRESETS: dict[GraspMethod, Dex3Pose] = {
    # Small or thin objects: thumb closes toward index, middle stays relaxed.
    "pinch": Dex3Pose(0.55, 0.58, 0.48, 0.62, 0.50, 0.10, 0.08),
    # Cup-like objects: thumb opposes index and middle, all fingers participate.
    "wrap": Dex3Pose(0.00, -1.00, 1.00, 0.88, 0.85, 0.88, 0.85),
    # Flat objects or side pickup: more thumb opposition, index/middle partly closed.
    "side": Dex3Pose(0.80, 0.45, 0.30, 0.42, 0.28, 0.42, 0.28),
    # Box squeeze: keep index/middle straight and rotate the thumb base into the palm plane.
    "flat_palm": FLAT_PALM_POSE,
}


class Dex3Backend(Protocol):
    def send_pose(self, hand: HandSide, pose: Dex3Pose, label: str) -> RobotAck:
        ...


class DryRunDex3Backend:
    def send_pose(self, hand: HandSide, pose: Dex3Pose, label: str) -> RobotAck:
        payload = {"hand": hand, "label": label, "pose": pose.as_dict()}
        logging.info("DEX3 DRY_RUN hand=%s label=%s pose=%s", hand, label, pose.as_dict())
        return RobotAck(ok=True, message=f"DRY_RUN dex3 command: {json.dumps(payload, ensure_ascii=True)}")


class JsonUdpDex3Backend:
    """Simple UDP bridge for a robot-side Dex3 controller.

    Use this if you create a small process on the robot that accepts normalized
    Dex3 poses and converts them to Unitree SDK calls.
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 15002, timeout_s: float = 2.0) -> None:
        self.host = host
        self.port = int(port)
        self.timeout_s = float(timeout_s)

    def send_pose(self, hand: HandSide, pose: Dex3Pose, label: str) -> RobotAck:
        command = {
            "action": "dex3_set_pose",
            "hand": hand,
            "label": label,
            "pose": pose.as_dict(),
            "seq": int(time.time() * 1000) % 1_000_000,
        }
        packet = json.dumps(command, ensure_ascii=True).encode("utf-8")
        logging.info(
            "DEX3 TX hand=%s label=%s seq=%s host=%s port=%d pose=%s",
            hand,
            label,
            command["seq"],
            self.host,
            self.port,
            pose.as_dict(),
        )
        start_s = time.monotonic()
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind(("127.0.0.1", 0))
            sock.settimeout(self.timeout_s)
            try:
                sock.sendto(packet, (self.host, self.port))
                data, _ = sock.recvfrom(4096)
            except (OSError, TimeoutError) as exc:
                logging.error("DEX3 RX failed hand=%s label=%s seq=%s error=%s", hand, label, command["seq"], exc)
                return RobotAck(ok=False, message=f"dex3 command failed: {exc}")
        elapsed_ms = (time.monotonic() - start_s) * 1000.0
        message = data.decode("utf-8", errors="replace")
        logging.info("DEX3 RX ok hand=%s label=%s seq=%s elapsed_ms=%.1f ack=%s", hand, label, command["seq"], elapsed_ms, message)
        return RobotAck(ok=True, message=message)


class UnitreeDex3Backend:
    """Placeholder for direct Unitree SDK integration.

    Put the real SDK publisher/client here after confirming the exact Dex3 API
    available on your G1 image.
    """

    def send_pose(self, hand: HandSide, pose: Dex3Pose, label: str) -> RobotAck:
        raise NotImplementedError(
            "Connect Unitree Dex3 SDK here: map normalized Dex3Pose values to real joint commands."
        )


class Dex3HandController:
    def __init__(self, backend: Dex3Backend | None = None, settle_s: float = 0.6) -> None:
        self.backend = backend or DryRunDex3Backend()
        self.settle_s = float(settle_s)

    def open(self, hand: HandSide) -> RobotAck:
        logging.info("DEX3 command=open hand=%s", hand)
        return self.set_pose(hand, OPEN_POSE, "open")

    def open_both(self) -> RobotAck:
        logging.info("DEX3 command=open_both")
        return self.set_poses({"left": OPEN_POSE, "right": OPEN_POSE}, "open_both")

    def flat_both(self) -> RobotAck:
        logging.info("DEX3 command=flat_both")
        return self.set_poses({"left": FLAT_PALM_POSE, "right": FLAT_PALM_POSE}, "flat_both")

    def flat_thumb_squeeze_both(self) -> RobotAck:
        logging.info("DEX3 command=flat_thumb_squeeze_both")
        return self.set_poses({"left": FLAT_PALM_THUMB_SQUEEZE_POSE, "right": FLAT_PALM_THUMB_SQUEEZE_POSE}, "flat_thumb_squeeze_both")

    def close(self, hand: HandSide, method: GraspMethod = "wrap") -> RobotAck:
        logging.info("DEX3 command=close hand=%s method=%s", hand, method)
        return self.set_pose(hand, GRASP_PRESETS[method], f"close_{method}")

    def close_both(self, method: GraspMethod = "wrap") -> RobotAck:
        logging.info("DEX3 command=close_both method=%s", method)
        pose = GRASP_PRESETS[method]
        return self.set_poses({"left": pose, "right": pose}, f"close_both_{method}")

    def set_pose(self, hand: HandSide, pose: Dex3Pose, label: str) -> RobotAck:
        logging.info("DEX3 validate pose hand=%s label=%s pose=%s", hand, label, pose.as_dict())
        self._validate_pose(pose)
        logging.info("DEX3 pose accepted hand=%s label=%s", hand, label)
        ack = self.backend.send_pose(hand, pose, label)
        if ack.ok and self.settle_s > 0:
            logging.info("DEX3 settle label=%s seconds=%.2f", label, self.settle_s)
            time.sleep(self.settle_s)
        logging.info("DEX3 result label=%s ok=%s message=%s", label, ack.ok, ack.message)
        return ack

    def set_poses(self, poses_by_hand: Dict[HandSide, Dex3Pose], label: str) -> RobotAck:
        for hand, pose in poses_by_hand.items():
            logging.info("DEX3 validate pose hand=%s label=%s pose=%s", hand, label, pose.as_dict())
            self._validate_pose(pose)
        send_poses = getattr(self.backend, "send_poses", None)
        if callable(send_poses):
            ack = send_poses(poses_by_hand, label)
        else:
            messages = []
            for hand, pose in poses_by_hand.items():
                ack = self.backend.send_pose(hand, pose, label)
                messages.append(f"{hand}: {ack.message}")
                if not ack.ok:
                    return RobotAck(ok=False, message="; ".join(messages))
            ack = RobotAck(ok=True, message="; ".join(messages))
        if ack.ok and self.settle_s > 0:
            logging.info("DEX3 settle label=%s seconds=%.2f", label, self.settle_s)
            time.sleep(self.settle_s)
        logging.info("DEX3 result label=%s ok=%s message=%s", label, ack.ok, ack.message)
        return ack

    @staticmethod
    def _validate_pose(pose: Dex3Pose) -> None:
        for name, value in pose.as_dict().items():
            min_value = -1.0 if name == "thumb_flexion_0" else 0.0
            if not min_value <= value <= 1.0:
                logging.error("DEX3 unsafe joint name=%s value=%.3f", name, value)
                raise ValueError(
                    f"Dex3 joint {name}={value:.3f} outside normalized range [{min_value:.1f}, 1.0]"
                )














