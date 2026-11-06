from __future__ import annotations

import logging
from dataclasses import dataclass

from .types import HandSide, PoseXYZ


@dataclass(frozen=True)
class WorkspaceLimits:
    """Conservative starting limits for manual experiments.

    These values are intentionally narrow. Calibrate them on the real G1 before
    increasing the workspace.
    """

    x_min_m: float = 0.15
    x_max_m: float = 0.55
    y_min_m: float = -0.35
    y_max_m: float = 0.35
    z_min_m: float = 0.45
    z_max_m: float = 1.10


DEFAULT_LIMITS = WorkspaceLimits()


def validate_hand_side(hand: str) -> HandSide:
    logging.info("SAFETY validate hand=%s", hand)
    if hand not in ("left", "right"):
        logging.error("SAFETY invalid hand=%s", hand)
        raise ValueError("hand must be 'left' or 'right'")
    logging.info("SAFETY hand accepted=%s", hand)
    return hand  # type: ignore[return-value]


def validate_pose(pose: PoseXYZ, limits: WorkspaceLimits = DEFAULT_LIMITS) -> None:
    logging.info(
        "SAFETY validate pose x=%.3f y=%.3f z=%.3f limits=x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]",
        pose.x_m,
        pose.y_m,
        pose.z_m,
        limits.x_min_m,
        limits.x_max_m,
        limits.y_min_m,
        limits.y_max_m,
        limits.z_min_m,
        limits.z_max_m,
    )
    errors: list[str] = []
    if not limits.x_min_m <= pose.x_m <= limits.x_max_m:
        errors.append(f"x={pose.x_m:.3f} outside [{limits.x_min_m:.3f}, {limits.x_max_m:.3f}]")
    if not limits.y_min_m <= pose.y_m <= limits.y_max_m:
        errors.append(f"y={pose.y_m:.3f} outside [{limits.y_min_m:.3f}, {limits.y_max_m:.3f}]")
    if not limits.z_min_m <= pose.z_m <= limits.z_max_m:
        errors.append(f"z={pose.z_m:.3f} outside [{limits.z_min_m:.3f}, {limits.z_max_m:.3f}]")
    if errors:
        logging.error("SAFETY pose rejected: %s", "; ".join(errors))
        raise ValueError("Unsafe target pose: " + "; ".join(errors))
    logging.info("SAFETY pose accepted")


def pose_with_offset(pose: PoseXYZ, dx_m: float = 0.0, dy_m: float = 0.0, dz_m: float = 0.0) -> PoseXYZ:
    shifted = PoseXYZ(
        x_m=pose.x_m + dx_m,
        y_m=pose.y_m + dy_m,
        z_m=pose.z_m + dz_m,
    )
    logging.info("SAFETY pose offset base=%s dx=%.3f dy=%.3f dz=%.3f result=%s", pose, dx_m, dy_m, dz_m, shifted)
    return shifted
