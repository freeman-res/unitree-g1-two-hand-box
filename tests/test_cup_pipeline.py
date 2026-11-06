from pathlib import Path

from arms.cup_pipeline import parse_yolo_log
from arms.vision_kinematics import CameraPoint, GraspOffset, yolo_camera_point_to_arm_target


def test_parse_yolo_reject_line_with_xyz_and_low_conf(tmp_path: Path) -> None:
    log_path = tmp_path / "yolo.log"
    log_path.write_text(
        "2026 INFO Reject detection: low conf 0.12 < 0.20. frame=83 X=+0.042 Y=-0.064 Z=0.562 samples=5544\n",
        encoding="utf-8",
    )

    detection = parse_yolo_log(str(log_path), min_confidence=0.20, allow_low_confidence=True, default_y=None)

    assert detection.frame == 83
    assert detection.confidence == 0.12
    assert detection.x_m == 0.042
    assert detection.y_m == -0.064
    assert detection.z_m == 0.562


def test_camera_point_to_right_arm_target_with_forward_offset() -> None:
    target = yolo_camera_point_to_arm_target(
        CameraPoint(x_m=0.042, y_m=-0.064, z_m=0.562),
        grasp_offset=GraspOffset(forward_m=-0.10),
    )

    assert round(target.x_m, 3) == 0.462
    assert round(target.y_m, 3) == -0.192
    assert round(target.z_m, 3) == 0.564
