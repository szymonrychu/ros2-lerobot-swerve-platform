"""serve_filter_node builds the guard on the accepted arm calibration and swaps a new Accept into the running guard on
its calibration timer (filter_node.arm_calibration), without a restart."""

from collections.abc import Callable
from pathlib import Path

import pytest
from arm_collision.settings import ArmBaseOffset

from filter_node.arm_calibration import CALIBRATION_POLL_S
from filter_node.config import FilterConfig, GuardSettings

from .arm_fixture import ARM, HARD_MARGIN_M, PARKED_HOME
from .test_arm_calibration import BASE, OFFSETS, accept, accepted_arm, checker_for
from .test_guard_wiring import AUTONOMY, FEEDBACK, HEAD, OUTPUT, WEB_UI, Harness, harness, joint_msg

__all__ = ["harness"]  # the fixture, imported for this module's tests


def serve(harness: Harness, applied_dir: Path | None, timers: list[tuple[float, Callable[[], None]]]) -> None:
    """Run serve_filter_node with the guard on applied_dir until the harness steps run out; record its timers.

    Args:
        harness (Harness): Harness with its steps set.
        applied_dir (Path | None): self_collision_guard.applied_dir.
        timers (list[tuple[float, Callable[[], None]]]): Receives (period, callback) of every created timer.
    """
    harness.node.create_timer.side_effect = lambda period, callback: timers.append((period, callback))
    config = FilterConfig(
        input_topic="/in",
        output_topic=OUTPUT,
        algorithm="kalman",
        algorithm_params={},
        control_loop_hz=100.0,
        joint_names=list(PARKED_HOME),
        web_ui_input_topic=WEB_UI,
        follower_feedback_topic=FEEDBACK,
        autonomy_input_topic=AUTONOMY,
        self_collision_guard=GuardSettings(
            hard_margin_m=HARD_MARGIN_M, head_state_topic=HEAD, arm=ARM, applied_dir=applied_dir
        ),
    )
    harness.module.serve_filter_node(config, harness.node)


def calibration_timers(timers: list[tuple[float, Callable[[], None]]]) -> list[Callable[[], None]]:
    """Callbacks of the timers with the calibration period.

    Args:
        timers (list[tuple[float, Callable[[], None]]]): Created timers.

    Returns:
        list[Callable[[], None]]: Their callbacks.
    """
    return [callback for period, callback in timers if period == CALIBRATION_POLL_S]


def test_the_guard_is_built_on_an_accepted_calibration(harness: Harness, tmp_path: Path) -> None:
    accept(tmp_path)
    harness.steps = [harness.at(0.1)]
    serve(harness, tmp_path, [])
    (guard,) = harness.guards
    assert guard.checker.mount == ArmBaseOffset(**BASE)
    assert guard.checker.kin.offsets == OFFSETS


def test_an_accept_while_running_reaches_the_guard_on_the_next_timer_tick(harness: Harness, tmp_path: Path) -> None:
    timers: list[tuple[float, Callable[[], None]]] = []

    def tick() -> None:
        for callback in calibration_timers(timers):
            callback()

    harness.steps = [
        harness.at(0.1, FEEDBACK, joint_msg(PARKED_HOME)),
        harness.call(0.2, lambda: accept(tmp_path)),
        harness.call(0.3, tick),
        harness.at(0.4, WEB_UI, joint_msg(PARKED_HOME)),
    ]
    serve(harness, tmp_path, timers)
    (guard,) = harness.guards
    reference = checker_for(accepted_arm())
    assert guard.checker.mount == ArmBaseOffset(**BASE) and guard.checker.kin.offsets == OFFSETS
    assert guard.checker.pose(PARKED_HOME).min_clearance == pytest.approx(
        reference.pose(PARKED_HOME).min_clearance, abs=1e-9
    )
    assert harness.published and harness.published[-1][0] == "web_ui"


def test_no_applied_dir_creates_no_calibration_timer_and_keeps_the_configured_arm(harness: Harness) -> None:
    timers: list[tuple[float, Callable[[], None]]] = []
    harness.steps = [harness.at(0.1)]
    serve(harness, None, timers)
    assert calibration_timers(timers) == []
    (guard,) = harness.guards
    assert guard.checker.mount == ARM.base_in_base_link
    assert guard.checker.kin.offsets == ARM.joint_offsets_rad.model_dump()


def test_the_calibration_timer_is_created_with_an_applied_dir(harness: Harness, tmp_path: Path) -> None:
    timers: list[tuple[float, Callable[[], None]]] = []
    harness.steps = [harness.at(0.1)]
    serve(harness, tmp_path, timers)
    assert len(calibration_timers(timers)) == 1
