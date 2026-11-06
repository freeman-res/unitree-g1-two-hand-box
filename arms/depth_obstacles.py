from __future__ import annotations

import json
import logging
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

from .types import PoseXYZ
from .urdf_kinematics import (
    Joint,
    Matrix4,
    active_joints,
    identity4,
    matmul4,
    origin_matrix,
    position,
    rotation_axis,
    translation,
)
from .vision_kinematics import CameraPoint, CameraToArmConfig

Vector3 = Tuple[float, float, float]
COLLISION_DISTANCE_EPS_M = 0.005


@dataclass(frozen=True)
class ObstacleCloud:
    points: List[Vector3]
    source: str


@dataclass(frozen=True)
class CollisionCheckResult:
    ok: bool
    message: str
    min_distance_m: float
    sample_index: int = -1
    segment_name: str = ""
    obstacle: Optional[Vector3] = None


def capture_realsense_obstacle_cloud(
    transform: CameraToArmConfig,
    target: PoseXYZ,
    *,
    capture_s: float = 0.8,
    stride_px: int = 12,
    min_depth_m: float = 0.15,
    max_depth_m: float = 1.50,
    voxel_size_m: float = 0.025,
    max_points: int = 2500,
    exclude_target_radius_m: float = 0.13,
) -> ObstacleCloud:
    """Capture visible RealSense depth points and convert them into torso_link."""

    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        raise RuntimeError("pyrealsense2 is not installed; cannot use --obstacle-check realsense") from exc

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    logging.info(
        "OBSTACLE RealSense start capture_s=%.2f stride_px=%d depth=[%.2f, %.2f]",
        capture_s,
        stride_px,
        min_depth_m,
        max_depth_m,
    )

    profile = pipeline.start(config)
    try:
        end_s = time.monotonic() + max(0.1, float(capture_s))
        depth_frame = None
        while time.monotonic() < end_s:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
        if depth_frame is None:
            raise RuntimeError("RealSense did not return a depth frame")

        intr = depth_frame.profile.as_video_stream_profile().intrinsics
        points: List[Vector3] = []
        stride_px = max(1, int(stride_px))
        for v in range(0, intr.height, stride_px):
            for u in range(0, intr.width, stride_px):
                depth_m = float(depth_frame.get_distance(u, v))
                if not min_depth_m <= depth_m <= max_depth_m:
                    continue
                camera_x_m = (u - intr.ppx) * depth_m / intr.fx
                camera_y_m = (v - intr.ppy) * depth_m / intr.fy
                torso_point = camera_point_to_torso(
                    CameraPoint(camera_x_m, camera_y_m, depth_m),
                    transform,
                )
                if _distance(torso_point, (target.x_m, target.y_m, target.z_m)) <= exclude_target_radius_m:
                    continue
                points.append(torso_point)
    finally:
        pipeline.stop()

    downsampled = voxel_downsample(points, voxel_size_m, max_points)
    logging.info(
        "OBSTACLE RealSense captured raw_points=%d downsampled=%d target_exclude_radius=%.3f",
        len(points),
        len(downsampled),
        exclude_target_radius_m,
    )
    return ObstacleCloud(points=downsampled, source="realsense")


def camera_point_to_torso(point: CameraPoint, transform: CameraToArmConfig) -> Vector3:
    if abs(transform.camera_pitch_down_deg) < 1e-9:
        delta_x = point.z_m
        delta_y = transform.camera_x_to_target_y_sign * point.x_m
        delta_z = transform.camera_y_to_target_z_sign * point.y_m
    else:
        theta = math.radians(transform.camera_pitch_down_deg)
        delta_x = point.z_m * math.cos(theta) - point.y_m * math.sin(theta)
        delta_y = -point.x_m
        delta_z = -point.z_m * math.sin(theta) - point.y_m * math.cos(theta)
    return (
        transform.camera_target_x_m + delta_x,
        transform.camera_target_y_m + delta_y,
        transform.camera_target_z_m + delta_z,
    )


def load_obstacle_cloud_json(path: str) -> ObstacleCloud:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    raw_points = payload["points"] if isinstance(payload, dict) else payload
    points: List[Vector3] = []
    for item in raw_points:
        if isinstance(item, dict):
            points.append((float(item["x_m"]), float(item["y_m"]), float(item["z_m"])))
        else:
            points.append((float(item[0]), float(item[1]), float(item[2])))
    logging.info("OBSTACLE loaded json path=%s points=%d", path, len(points))
    return ObstacleCloud(points=points, source=path)


def save_obstacle_cloud_json(path: str, cloud: ObstacleCloud) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "source": cloud.source,
        "points": [{"x_m": p[0], "y_m": p[1], "z_m": p[2]} for p in cloud.points],
    }
    output.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    logging.info("OBSTACLE saved json path=%s points=%d", output, len(cloud.points))


def voxel_downsample(points: Sequence[Vector3], voxel_size_m: float, max_points: int) -> List[Vector3]:
    if voxel_size_m <= 0:
        result = list(points)
    else:
        buckets: Dict[Tuple[int, int, int], Vector3] = {}
        inv = 1.0 / voxel_size_m
        for point in points:
            key = (round(point[0] * inv), round(point[1] * inv), round(point[2] * inv))
            buckets.setdefault(key, point)
        result = list(buckets.values())
    max_points = max(1, int(max_points))
    if len(result) <= max_points:
        return result
    step = len(result) / max_points
    return [result[int(index * step)] for index in range(max_points)]


def check_joint_path_collision(
    chain: Sequence[Joint],
    q_waypoints: Sequence[Sequence[float]],
    obstacle_points: Sequence[Vector3],
    *,
    clearance_m: float = 0.08,
    samples_per_leg: int = 24,
) -> CollisionCheckResult:
    """Check whether arm link segments pass too close to obstacle points."""

    if not obstacle_points:
        return CollisionCheckResult(True, "no obstacle points", float("inf"))
    if len(q_waypoints) < 2:
        return CollisionCheckResult(True, "not enough waypoints to check", float("inf"))

    min_distance = float("inf")
    min_sample = -1
    min_segment = ""
    min_obstacle: Optional[Vector3] = None
    sample_index = 0

    for start_q, end_q in zip(q_waypoints, q_waypoints[1:]):
        for q in _sample_joint_leg(start_q, end_q, max(2, int(samples_per_leg))):
            segments = arm_segments_for_q(chain, q)
            for segment_name, a, b in segments:
                near_obstacles = _points_near_segment_box(obstacle_points, a, b, clearance_m)
                if not near_obstacles:
                    for obstacle in obstacle_points:
                        distance_m = _point_segment_distance(obstacle, a, b)
                        if distance_m < min_distance:
                            min_distance = distance_m
                            min_sample = sample_index
                            min_segment = segment_name
                            min_obstacle = obstacle
                    continue
                for obstacle in near_obstacles:
                    distance_m = _point_segment_distance(obstacle, a, b)
                    if distance_m < min_distance:
                        min_distance = distance_m
                        min_sample = sample_index
                        min_segment = segment_name
                        min_obstacle = obstacle
                    if distance_m < clearance_m - COLLISION_DISTANCE_EPS_M:
                        return CollisionCheckResult(
                            ok=False,
                            message=(
                                f"collision risk: segment={segment_name} sample={sample_index} "
                                f"distance={distance_m:.3f}m clearance={clearance_m:.3f}m eps={COLLISION_DISTANCE_EPS_M:.3f}m"
                            ),
                            min_distance_m=distance_m,
                            sample_index=sample_index,
                            segment_name=segment_name,
                            obstacle=obstacle,
                        )
            sample_index += 1

    return CollisionCheckResult(
        ok=True,
        message=f"path clear: min_distance={min_distance:.3f}m clearance={clearance_m:.3f}m",
        min_distance_m=min_distance,
        sample_index=min_sample,
        segment_name=min_segment,
        obstacle=min_obstacle,
    )


def arm_segments_for_q(chain: Sequence[Joint], q: Sequence[float]) -> List[Tuple[str, Vector3, Vector3]]:
    movable = active_joints(list(chain))
    if len(q) != len(movable):
        raise ValueError(f"q must contain {len(movable)} values")
    values_by_name = {joint.name: float(q[index]) for index, joint in enumerate(movable)}
    link_points = chain_link_positions(chain, values_by_name)
    ordered_points = [point for _, point in link_points if point is not None]
    segments: List[Tuple[str, Vector3, Vector3]] = []
    for index, (a, b) in enumerate(zip(ordered_points, ordered_points[1:])):
        if _distance(a, b) < 0.03:
            continue
        segments.append((f"arm_segment_{index}", a, b))
    return segments


def chain_link_positions(chain: Sequence[Joint], values_by_name: Dict[str, float]) -> List[Tuple[str, Vector3]]:
    transform: Matrix4 = identity4()
    result: List[Tuple[str, Vector3]] = []
    for joint in chain:
        transform = matmul4(transform, origin_matrix(joint.xyz, joint.rpy))
        q = float(values_by_name.get(joint.name, 0.0))
        if joint.joint_type in ("revolute", "continuous"):
            transform = matmul4(transform, rotation_axis(joint.axis, q))
        elif joint.joint_type == "prismatic":
            transform = matmul4(transform, translation(tuple(v * q for v in joint.axis)))
        result.append((joint.child, position(transform)))
    return result


def _sample_joint_leg(start_q: Sequence[float], end_q: Sequence[float], count: int) -> List[List[float]]:
    if len(start_q) != len(end_q):
        raise ValueError("joint waypoints must have the same length")
    result: List[List[float]] = []
    for index in range(count):
        ratio = index / (count - 1)
        result.append([float(a) * (1.0 - ratio) + float(b) * ratio for a, b in zip(start_q, end_q)])
    return result


def _points_near_segment_box(
    points: Sequence[Vector3],
    a: Vector3,
    b: Vector3,
    margin_m: float,
) -> List[Vector3]:
    min_x, max_x = sorted((a[0], b[0]))
    min_y, max_y = sorted((a[1], b[1]))
    min_z, max_z = sorted((a[2], b[2]))
    margin_m = max(0.0, float(margin_m))
    return [
        point
        for point in points
        if min_x - margin_m <= point[0] <= max_x + margin_m
        and min_y - margin_m <= point[1] <= max_y + margin_m
        and min_z - margin_m <= point[2] <= max_z + margin_m
    ]


def _point_segment_distance(point: Vector3, a: Vector3, b: Vector3) -> float:
    ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    ap = (point[0] - a[0], point[1] - a[1], point[2] - a[2])
    ab_len2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2]
    if ab_len2 < 1e-12:
        return _distance(point, a)
    t = max(0.0, min(1.0, (ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / ab_len2))
    closest = (a[0] + ab[0] * t, a[1] + ab[1] * t, a[2] + ab[2] * t)
    return _distance(point, closest)


def _distance(a: Vector3, b: Vector3) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)
