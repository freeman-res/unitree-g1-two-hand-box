from __future__ import annotations

from arms.task3_grasp_methods import SCENARIOS, selected_methods


def test_task3_has_three_required_methods() -> None:
    assert selected_methods("all") == ["pinch", "wrap", "side"]
    assert set(SCENARIOS) == {"pinch", "wrap", "side"}
