from __future__ import annotations

import pytest

from arms.safety import validate_pose
from arms.types import PoseXYZ


def test_pose_inside_default_workspace() -> None:
    validate_pose(PoseXYZ(0.35, 0.0, 0.75))


def test_pose_outside_default_workspace_raises() -> None:
    with pytest.raises(ValueError):
        validate_pose(PoseXYZ(0.90, 0.0, 0.75))
