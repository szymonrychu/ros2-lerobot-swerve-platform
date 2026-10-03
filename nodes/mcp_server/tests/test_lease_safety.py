"""Lease safety: home releases, release serialized against stream/keepalive, stop leaves an unheld arm alone,
orphaned lease released on startup."""

import threading
from pathlib import Path

import pytest

from mcp_server.arm import ORPHAN_LEASE_WINDOW_S, ArmController
from mcp_server.base_motion import run_stop
from mcp_server.config import McpServerConfig
from mcp_server.ik import ArmKinematics, load_joint_limits

from .fakes import FakeArmBackend

CONFIG = McpServerConfig()
KIN = ArmKinematics(CONFIG.arm.urdf_path, margin=CONFIG.limits.arm_limit_margin_rad)


def make(tmp_path: Path) -> tuple[ArmController, FakeArmBackend]:
    be = FakeArmBackend()
    cfg = CONFIG.model_copy(deep=True)
    cfg.arm.home_file = tmp_path / "arm" / "home.yaml"
    return ArmController(be, KIN, load_joint_limits(cfg.arm.urdf_path), cfg), be


def store_home(arm: ArmController, be: FakeArmBackend) -> None:
    be.positions["elbow_flex"] = 0.3
    arm.set_home()
    be.positions["elbow_flex"] = -0.2


def no_command_after_release(be: FakeArmBackend) -> bool:
    kinds = [kind for kind, _ in be.events]
    return "release" in kinds and "command" not in kinds[kinds.index("release") :]


def release_from_other_thread(arm: ArmController) -> None:
    worker = threading.Thread(target=arm.release)
    worker.start()
    worker.join(timeout=5.0)
    assert not worker.is_alive(), "release deadlocked"


# --- (a) home releases -----------------------------------------------------------------------------------------


def test_home_service_releases_after_success(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    store_home(arm, be)
    ok, message = arm.home_service_call()
    assert ok, message
    assert be.positions["elbow_flex"] == pytest.approx(0.3)
    assert not arm.control_held
    assert be.events[-1] == ("release", None)
    n = len(be.commands)
    arm.keepalive_tick()
    assert len(be.commands) == n  # keepalive stopped with the lease


def test_home_service_releases_after_failed_motion(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    store_home(arm, be)
    be.stale_after = be.t + 0.2
    ok, message = arm.home_service_call()
    assert not ok
    assert "aborted_stale" in message
    assert not arm.control_held
    assert be.events[-1] == ("release", None)


def test_home_service_releases_even_when_control_was_held(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    store_home(arm, be)
    arm.acquire()
    ok, _ = arm.home_service_call()
    assert ok
    assert not arm.control_held
    assert be.events[-1] == ("release", None)


def test_home_service_without_pose_reports_error_and_touches_nothing(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    ok, message = arm.home_service_call()
    assert not ok
    assert "home" in message
    assert be.events == []


def test_home_service_busy_does_not_release_running_motion(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    store_home(arm, be)
    results: list[tuple[bool, str]] = []

    def call_home_during_motion(b: FakeArmBackend) -> None:
        if not results:
            results.append(arm.home_service_call())

    be.on_sleep = call_home_during_motion
    res = arm.move_joints({"wrist_flex": 0.3}, speed_scale=0.5)
    assert res.status == "converged"
    assert results[0][0] is False and "another arm motion" in results[0][1]
    assert be.releases == 0
    assert arm.control_held


def test_tool_home_releases_when_control_was_not_held(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    store_home(arm, be)
    res = arm.home()
    assert res.status == "converged"
    assert not arm.control_held
    assert be.events[-1] == ("release", None)


def test_tool_home_keeps_control_that_was_held(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    store_home(arm, be)
    arm.acquire()
    res = arm.home()
    assert res.status == "converged"
    assert arm.control_held
    assert be.releases == 0


def test_tool_home_failure_releases_when_control_was_not_held(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    store_home(arm, be)
    be.stale_after = be.t + 0.2
    res = arm.home()
    assert res.status == "aborted_stale"
    assert not arm.control_held
    assert be.events[-1] == ("release", None)


# --- (b) release serialized against stream and keepalive -----------------------------------------------------------


def test_release_during_stream_blocks_further_commands(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    arm.acquire()

    def release_between_check_and_command(b: FakeArmBackend) -> None:
        if len(b.commands) >= 5:
            release_from_other_thread(arm)
        else:
            b.on_active_source = release_between_check_and_command

    be.on_active_source = release_between_check_and_command
    res = arm.move_joints({"elbow_flex": 1.0}, speed_scale=0.5)
    assert res.status == "stopped"
    assert not arm.control_held
    assert no_command_after_release(be), be.events[-3:]


def test_release_during_keepalive_blocks_republish(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    arm.move_joints({"elbow_flex": 0.2}, speed_scale=0.5)
    be.t += 1.0  # past the lease grace
    workers: list[threading.Thread] = []

    def release_while_keepalive_publishes(_b: FakeArmBackend) -> None:
        # Release races the keepalive publish; it must either finish first (and suppress it) or wait for it.
        worker = threading.Thread(target=arm.release)
        worker.start()
        workers.append(worker)
        worker.join(timeout=0.2)

    be.on_publish_command = release_while_keepalive_publishes
    arm.keepalive_tick()
    for worker in workers:
        worker.join(timeout=5.0)
    assert not arm.control_held
    assert no_command_after_release(be), be.events[-3:]


def test_hold_after_release_publishes_nothing_until_reacquired(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    arm.acquire()
    arm.release()
    assert arm.command({"elbow_flex": 0.1}) is False
    assert no_command_after_release(be)


# --- (c) stop leaves the arm alone unless controlled -------------------------------------------------------------


def test_stop_arm_untouched_when_control_not_held(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.source = "leader"
    held, note = arm.stop_hold()
    assert held is False
    assert "not touched" in note
    assert be.events == []
    assert not arm.control_held


def test_stop_holds_arm_when_control_held(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    arm.acquire()
    be.positions["gripper"] = 0.4
    held, note = arm.stop_hold()
    assert held is True and note == ""
    assert be.commands[-1]["gripper"] == 0.4
    assert arm.control_held


def test_stop_during_motion_holds_and_aborts(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    outcome: list[tuple[bool, str]] = []

    def stop_midway(b: FakeArmBackend) -> None:
        if len(b.commands) == 5 and not outcome:
            outcome.append(arm.stop_hold())

    be.on_sleep = stop_midway
    res = arm.move_joints({"elbow_flex": 1.0}, speed_scale=0.5)
    assert res.status == "stopped"
    assert outcome[0][0] is True


def test_run_stop_zeroes_base_and_cancels_without_touching_free_arm(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    twists: list[tuple[float, float, float]] = []
    flagged: list[bool] = []
    res = run_stop(lambda: flagged.append(True), lambda *v: twists.append(v), lambda: True, arm.stop_hold)
    assert flagged == [True]
    assert twists and all(t == (0.0, 0.0, 0.0) for t in twists)
    assert res.nav_goals_cancelled and res.base_zeroed
    assert res.arm_held is False
    assert "not touched" in res.message
    assert be.events == []


def test_run_stop_reports_cancel_failure_and_holds_controlled_arm(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    arm.acquire()
    twists: list[tuple[float, float, float]] = []
    res = run_stop(lambda: None, lambda *v: twists.append(v), lambda: False, arm.stop_hold)
    assert res.arm_held is True
    assert not res.nav_goals_cancelled
    assert "Nav2 cancel service unavailable" in res.message
    assert twists


# --- (e) orphaned lease ---------------------------------------------------------------------------------------------


def test_orphan_lease_released_on_startup(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    started = be.t
    assert arm.check_orphan_lease(started) == "released"
    assert be.events == [("release", None)]
    assert not arm.control_held


def test_orphan_check_pending_until_source_seen_then_clear_after_window(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.source = None
    started = be.t
    assert arm.check_orphan_lease(started) == "pending"
    be.t += ORPHAN_LEASE_WINDOW_S + 0.1
    assert arm.check_orphan_lease(started) == "clear"
    assert be.events == []


def test_orphan_check_clear_when_other_source_active(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.source = "leader"
    started = be.t
    assert arm.check_orphan_lease(started) == "pending"
    be.t += ORPHAN_LEASE_WINDOW_S
    assert arm.check_orphan_lease(started) == "clear"
    assert be.releases == 0


def test_orphan_check_does_not_release_own_lease(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    started = be.t
    arm.acquire()
    assert arm.check_orphan_lease(started) == "clear"
    assert be.releases == 0
    assert arm.control_held
