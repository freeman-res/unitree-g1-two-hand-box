from types import SimpleNamespace

from arms.depth_obstacles import check_joint_path_collision
from arms.task5_ik_reach import _check_obstacles_if_needed
from arms.types import PoseXYZ
from arms.urdf_kinematics import Joint


def _fixed_line_chain() -> list[Joint]:
    return [
        Joint("base_to_a", "fixed", "base", "a", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), None, None),
        Joint("a_to_b", "fixed", "a", "b", (1.0, 0.0, 0.0), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), None, None),
    ]


def test_collision_check_blocks_point_near_arm_segment() -> None:
    result = check_joint_path_collision(
        _fixed_line_chain(),
        [[], []],
        [(0.5, 0.02, 0.0)],
        clearance_m=0.05,
        samples_per_leg=2,
    )

    assert not result.ok
    assert result.segment_name == "arm_segment_0"
    assert round(result.min_distance_m, 3) == 0.02


def test_collision_check_allows_point_far_from_arm_segment() -> None:
    result = check_joint_path_collision(
        _fixed_line_chain(),
        [[], []],
        [(0.5, 0.20, 0.0)],
        clearance_m=0.05,
        samples_per_leg=2,
    )

    assert result.ok
    assert round(result.min_distance_m, 3) == 0.20


def test_obstacle_extra_clearance_rejects_weak_clear_path(tmp_path) -> None:
    obstacle_json = tmp_path / "obstacles.json"
    obstacle_json.write_text('{"points": [{"x_m": 0.5, "y_m": 0.19, "z_m": 0.0}]}', encoding="utf-8")
    args = SimpleNamespace(
        obstacle_check="json",
        obstacle_json=str(obstacle_json),
        obstacle_save_json="",
        obstacle_clearance=0.18,
        obstacle_extra_clearance=0.12,
        obstacle_samples_per_leg=2,
        avoidance_mode="none",
        avoidance_replans=1,
        avoidance_replan_scale=1.6,
    )

    code, planned_path = _check_obstacles_if_needed(args, _fixed_line_chain(), PoseXYZ(1.0, 0.0, 0.0), [[], []])

    assert code == 6
    assert planned_path == [[], []]
