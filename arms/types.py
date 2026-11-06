from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

HandSide = Literal["left", "right"]
GripState = Literal["open", "close"]
GraspMethod = Literal["pinch", "wrap", "side", "flat_palm"]


@dataclass(frozen=True)
class PoseXYZ:
    """Target position in the robot base_link frame, meters."""

    x_m: float
    y_m: float
    z_m: float


@dataclass(frozen=True)
class RobotAck:
    ok: bool
    message: str
