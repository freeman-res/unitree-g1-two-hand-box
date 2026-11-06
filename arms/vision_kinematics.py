from __future__ import annotations

import logging
import math
from dataclasses import dataclass

from .types import PoseXYZ


@dataclass(frozen=True)
class CameraPoint:
    """Cup point measured by RealSense/YOLO in the camera optical frame.

    x_m: horizontal camera coordinate from the old v3 code.
    y_m: vertical camera coordinate from the old v3 code.
    z_m: depth forward from camera.
    """

    x_m: float
    y_m: float
    z_m: float


@dataclass(frozen=True)
class CameraToArmConfig:
    """Approximate transform from camera coordinates to our arm target coordinates.

    The current robot does not publish ROS tf, so these values are measured and
    kept configurable. Target coordinates follow the same labels used in
    config/arm_calibration.json: x=forward, y=left/right, z=height.
    """

    camera_target_x_m: float = 0.0
    camera_target_y_m: float = -0.15
    camera_target_z_m: float = 0.50
    camera_x_to_target_y_sign: float = -1.0
    camera_y_to_target_z_sign: float = -1.0
    camera_pitch_down_deg: float = 0.0


@dataclass(frozen=True)
class GraspOffset:
    """Offset from the cup center to the desired wrist/palm target."""

    forward_m: float = 0.0
    left_m: float = 0.0
    up_m: float = 0.0


def yolo_camera_point_to_arm_target(
    point: CameraPoint,
    transform: CameraToArmConfig = CameraToArmConfig(),
    grasp_offset: GraspOffset = GraspOffset(),
) -> PoseXYZ:
    """Convert YOLO/RealSense cup coordinates to the arm target coordinate label.

    Old v3 vision uses camera x/y/z. Our arm calibration uses PoseXYZ labels.
    Without ROS tf this is an approximate measured transform plus optional grasp
    offsets. This function does not solve joints; it only produces the target
    coordinate that the calibrated IK table should consume.
    """

    if abs(transform.camera_pitch_down_deg) < 1e-9:
        delta_x = point.z_m
        delta_y = transform.camera_x_to_target_y_sign * point.x_m
        delta_z = transform.camera_y_to_target_z_sign * point.y_m
    else:
        if transform.camera_x_to_target_y_sign != -1.0 or transform.camera_y_to_target_z_sign != -1.0:
            logging.warning("KINEMATICS pitch transform assumes optical axes x->-y, y->-z, z->+x")
        theta = math.radians(transform.camera_pitch_down_deg)
        delta_x = point.z_m * math.cos(theta) - point.y_m * math.sin(theta)
        delta_y = -point.x_m
        delta_z = -point.z_m * math.sin(theta) - point.y_m * math.cos(theta)

    target = PoseXYZ(
        x_m=transform.camera_target_x_m + delta_x + grasp_offset.forward_m,
        y_m=transform.camera_target_y_m + delta_y + grasp_offset.left_m,
        z_m=transform.camera_target_z_m + delta_z + grasp_offset.up_m,
    )
    logging.info(
        "KINEMATICS camera point x=%.3f y=%.3f z=%.3f -> target x=%.3f y=%.3f z=%.3f "
        "camera_origin=(%.3f, %.3f, %.3f) signs=(x_to_y %.1f, y_to_z %.1f) pitch_down_deg=%.1f delta=(%.3f, %.3f, %.3f) grasp_offset=(%.3f, %.3f, %.3f)",
        point.x_m,
        point.y_m,
        point.z_m,
        target.x_m,
        target.y_m,
        target.z_m,
        transform.camera_target_x_m,
        transform.camera_target_y_m,
        transform.camera_target_z_m,
        transform.camera_x_to_target_y_sign,
        transform.camera_y_to_target_z_sign,
        transform.camera_pitch_down_deg,
        delta_x,
        delta_y,
        delta_z,
        grasp_offset.forward_m,
        grasp_offset.left_m,
        grasp_offset.up_m,
    )
    return target

