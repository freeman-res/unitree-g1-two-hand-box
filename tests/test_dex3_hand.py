from __future__ import annotations

import pytest

from arms.dex3_hand import Dex3HandController, Dex3Pose, DryRunDex3Backend


def test_open_pose_dry_run() -> None:
    controller = Dex3HandController(DryRunDex3Backend(), settle_s=0.0)
    ack = controller.open("right")
    assert ack.ok
    assert "open" in ack.message


def test_invalid_pose_rejected() -> None:
    controller = Dex3HandController(DryRunDex3Backend(), settle_s=0.0)
    with pytest.raises(ValueError):
        controller.set_pose("right", Dex3Pose(0, 0, 0, 0, 0, 0, 1.2), "bad")
