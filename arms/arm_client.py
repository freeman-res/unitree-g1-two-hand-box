from __future__ import annotations

import json
import logging
import socket
import time
from dataclasses import asdict
from typing import Any

from .safety import DEFAULT_LIMITS, WorkspaceLimits, validate_pose
from .types import HandSide, PoseXYZ, RobotAck


class ArmClient:
    """Client for the G1 arm motion service.

    The current project used `g1_arm_keyframe_player` over UDP for named actions.
    For tasks 1 and 2 we keep the same transport, but send a parameterized
    `move_xyz` action. The robot-side receiver must support this JSON schema.
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 15001,
        timeout_s: float = 5.0,
        dry_run: bool = True,
        limits: WorkspaceLimits = DEFAULT_LIMITS,
    ) -> None:
        self.host = host
        self.port = int(port)
        self.timeout_s = float(timeout_s)
        self.dry_run = dry_run
        self.limits = limits
        logging.info(
            "ARM init host=%s port=%d timeout_s=%.2f dry_run=%s",
            self.host,
            self.port,
            self.timeout_s,
            self.dry_run,
        )

    def move_to(self, hand: HandSide, target: PoseXYZ) -> RobotAck:
        logging.info("ARM validate target hand=%s x=%.3f y=%.3f z=%.3f", hand, target.x_m, target.y_m, target.z_m)
        validate_pose(target, self.limits)
        logging.info("ARM target accepted hand=%s x=%.3f y=%.3f z=%.3f", hand, target.x_m, target.y_m, target.z_m)
        return self._send(
            {
                "action": "move_xyz",
                "hand": hand,
                "target": asdict(target),
            }
        )

    def lift_relative(self, hand: HandSide, start: PoseXYZ, dz_m: float) -> RobotAck:
        target = PoseXYZ(start.x_m, start.y_m, start.z_m + dz_m)
        logging.info("ARM lift_relative hand=%s start=%s dz_m=%.3f target=%s", hand, start, dz_m, target)
        validate_pose(target, self.limits)
        logging.info("ARM lift target accepted hand=%s x=%.3f y=%.3f z=%.3f", hand, target.x_m, target.y_m, target.z_m)
        return self._send(
            {
                "action": "move_xyz",
                "hand": hand,
                "target": asdict(target),
                "motion": {"kind": "lift_relative", "dz_m": dz_m},
            }
        )

    def named_action(self, action: str) -> RobotAck:
        logging.info("ARM named_action action=%s", action)
        return self._send({"action": action})

    def _send(self, command: dict[str, Any]) -> RobotAck:
        command = dict(command)
        command["seq"] = int(time.time() * 1000) % 1_000_000
        packet = json.dumps(command, ensure_ascii=True).encode("utf-8")

        logging.info(
            "ARM TX action=%s seq=%s target=%s host=%s port=%d dry_run=%s",
            command.get("action"),
            command.get("seq"),
            command.get("target", "-"),
            self.host,
            self.port,
            self.dry_run,
        )
        if self.dry_run:
            return RobotAck(ok=True, message=f"DRY_RUN arm command: {packet.decode('utf-8')}")

        start_s = time.monotonic()
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind(("127.0.0.1", 0))
            sock.settimeout(self.timeout_s)
            try:
                sock.sendto(packet, (self.host, self.port))
                data, _ = sock.recvfrom(4096)
            except (OSError, TimeoutError) as exc:
                logging.error("ARM RX failed action=%s seq=%s error=%s", command.get("action"), command.get("seq"), exc)
                return RobotAck(ok=False, message=f"arm command failed: {exc}")

        elapsed_ms = (time.monotonic() - start_s) * 1000.0
        message = data.decode("utf-8", errors="replace")
        logging.info("ARM RX ok action=%s seq=%s elapsed_ms=%.1f ack=%s", command.get("action"), command.get("seq"), elapsed_ms, message)
        return RobotAck(ok=True, message=message)

