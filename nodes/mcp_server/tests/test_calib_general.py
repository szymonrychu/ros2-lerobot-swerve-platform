"""Calibrate page General section services: /arm/gripper/open, /arm/gripper/close, /head/home, /base/steer_home.

Covers the gripper service body (ArmController.gripper_service_call: refused while any arm lease is held or the arm
torque is off, open / close, always releases its own lease, never another's), the head home and steering home bodies
(calib_general.head_home_call / steer_home_call) and their calib_event lines (flow calib_general), the Topics defaults
and the ros_iface service wiring.
"""

import json
import logging
from pathlib import Path
from types import ModuleType
from typing import Any

import pytest
from arm_collision.ik import ArmKinematics, load_joint_limits

from mcp_server.arm import (
    CALIB_GENERAL_FLOW,
    GRIPPER_SERVICE_RELEASE_NOTE,
    ArmBusyError,
    ArmController,
)
from mcp_server.calib_event import CALIB_EVENT_PREFIX
from mcp_server.calib_general import (
    HEAD_HOME_STEP,
    STEER_HOME_STEP,
    head_home_call,
    steer_home_call,
)
from mcp_server.config import McpServerConfig, TopicSettings
from mcp_server.head import AppliedHeadKinematics, HeadController, HeadError, HeadMoveResult

from .fakes import FakeArmBackend, FakeHeadBackend
from .test_calibration_marker import POSE

CONFIG = McpServerConfig()
KIN = ArmKinematics(CONFIG.arm.urdf_path, margin=CONFIG.limits.arm_limit_margin_rad)
GRIPPER = CONFIG.arm.gripper_joint
STEER = {"fl_steer": 0.0, "fr_steer": 0.0, "rl_steer": 0.0, "rr_steer": 0.0}


def make(tmp_path: Path) -> tuple[ArmController, FakeArmBackend]:
    """Controller at POSE (gripper partly open) with a temporary home file.

    Args:
        tmp_path (Path): Test directory.

    Returns:
        tuple[ArmController, FakeArmBackend]: Controller and backend.
    """
    cfg = CONFIG.model_copy(deep=True)
    cfg.arm.home_file = tmp_path / "arm" / "home.yaml"
    be = FakeArmBackend(dict(POSE))
    return ArmController(be, KIN, load_joint_limits(cfg.arm.urdf_path), cfg), be


def events(caplog: pytest.LogCaptureFixture) -> list[tuple[int, dict[str, Any]]]:
    """calib_event lines logged so far.

    Args:
        caplog (pytest.LogCaptureFixture): Log capture.

    Returns:
        list[tuple[int, dict[str, Any]]]: (level, parsed JSON) per calib_event line.
    """
    prefix = f"{CALIB_EVENT_PREFIX} "
    return [
        (r.levelno, json.loads(r.getMessage()[len(prefix) :]))
        for r in caplog.records
        if r.getMessage().startswith(prefix)
    ]


class Clock:
    """Fake monotonic clock advanced by hand."""

    def __init__(self) -> None:
        self.t = 50.0

    def __call__(self) -> float:
        return self.t


# --- Topics defaults and ros_iface wiring ----------------------------------------------------------------------------


def test_topics_defaults() -> None:
    topics = TopicSettings()
    assert topics.gripper_open_service == "/arm/gripper/open"
    assert topics.gripper_close_service == "/arm/gripper/close"
    assert topics.head_home_service == "/head/home"
    assert topics.steer_home_service == "/base/steer_home"


def test_ros_iface_serves_the_general_services(ros_iface: ModuleType) -> None:
    robot = ros_iface.RosRobot(McpServerConfig())
    served = {c.args[1]: c for c in robot.node.create_service.call_args_list}
    for name in ("/arm/gripper/open", "/arm/gripper/close", "/head/home", "/base/steer_home"):
        assert name in served, name
        assert served[name].kwargs["callback_group"] is robot.group


# --- gripper ---------------------------------------------------------------------------------------------------------


def test_open_gripper_opens_fully_and_releases(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    ok, message = arm.gripper_service_call(open_gripper=True)
    assert ok is True, message
    assert message.startswith("converged")
    assert be.positions[GRIPPER] == pytest.approx(CONFIG.arm.gripper_open_rad, abs=0.02)
    assert arm.control_held is False and be.events[-1] == ("release", None)
    assert arm.lost_note == GRIPPER_SERVICE_RELEASE_NOTE


def test_close_gripper_closes_and_releases(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    ok, message = arm.gripper_service_call(open_gripper=False)
    assert ok is True, message
    assert be.commands[-1][GRIPPER] == pytest.approx(CONFIG.arm.gripper_closed_rad)
    assert arm.control_held is False and be.events[-1] == ("release", None)


def test_gripper_moves_only_the_gripper(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    arm.gripper_service_call(open_gripper=True)
    for joint, value in POSE.items():
        if joint != GRIPPER:
            assert be.positions[joint] == pytest.approx(value, abs=1e-6)


@pytest.mark.parametrize("status", ["converged", "grasped", "blocked"])
def test_close_counts_every_completed_close_as_success(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, status: str
) -> None:
    arm, _ = make(tmp_path)
    real = arm.set_gripper

    def fake(**kwargs: Any) -> Any:
        return real(**kwargs).model_copy(update={"status": status})

    monkeypatch.setattr(arm, "set_gripper", fake)
    ok, message = arm.gripper_service_call(open_gripper=False)
    assert ok is True and message.startswith(status)


def test_open_needs_convergence(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    arm, _ = make(tmp_path)
    real = arm.set_gripper
    monkeypatch.setattr(arm, "set_gripper", lambda **k: real(**k).model_copy(update={"status": "blocked"}))
    ok, _ = arm.gripper_service_call(open_gripper=True)
    assert ok is False


def test_failed_motion_still_releases(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.stale_after = be.t + 0.2
    ok, message = arm.gripper_service_call(open_gripper=True)
    assert ok is False
    assert "aborted_stale" in message
    assert arm.control_held is False and be.events[-1] == ("release", None)


@pytest.mark.parametrize("open_gripper", [True, False])
def test_refused_while_the_arm_lease_is_held(tmp_path: Path, open_gripper: bool) -> None:
    arm, be = make(tmp_path)
    arm.acquire()  # a running calibration sequence, the agent or the web UI Arm tab
    commands = len(be.commands)
    ok, message = arm.gripper_service_call(open_gripper=open_gripper)
    assert ok is False
    assert "Abort" in message and "release control" in message
    assert len(be.commands) == commands
    assert arm.control_held is True and be.releases == 0


def test_a_lease_taken_during_the_motion_is_never_released(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    arm, be = make(tmp_path)
    real = arm.set_gripper

    def taken_over(**kwargs: Any) -> Any:
        result = real(**kwargs)
        arm.drop_lease("agent")  # the service's lease ended meanwhile ...
        arm.acquire()  # ... and a calibration sequence or the agent took a new one
        return result

    monkeypatch.setattr(arm, "set_gripper", taken_over)
    releases = be.releases
    arm.gripper_service_call(open_gripper=True)
    assert arm.control_held is True
    assert be.releases == releases


def test_the_service_claims_the_lease_before_moving(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    arm, _ = make(tmp_path)
    held: list[bool] = []
    real = arm.set_gripper

    def check(**kwargs: Any) -> Any:
        held.append(arm.control_held)
        return real(**kwargs)

    monkeypatch.setattr(arm, "set_gripper", check)
    arm.gripper_service_call(open_gripper=True)
    assert held == [True]
    assert arm.control_held is False


def test_refused_while_the_arm_torque_is_off(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.released = True
    ok, message = arm.gripper_service_call(open_gripper=True)
    assert ok is False
    assert "torque" in message
    assert be.commands == [] and be.releases == 0


def test_busy_arm_is_refused_without_releasing_the_running_motion(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    arm, _ = make(tmp_path)
    released: list[bool] = []
    monkeypatch.setattr(arm, "release", lambda *a, **k: released.append(True))

    def busy(**_kwargs: Any) -> Any:
        arm._held = True  # the other motion took the lease meanwhile
        raise ArmBusyError("another arm motion is running; call stop first")

    monkeypatch.setattr(arm, "set_gripper", busy)
    ok, message = arm.gripper_service_call(open_gripper=True)
    assert ok is False and "another arm motion" in message
    assert released == []


def test_gripper_logs_start_and_result(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    arm, _ = make(tmp_path)
    with caplog.at_level(logging.INFO, logger="mcp_server.arm"):
        arm.gripper_service_call(open_gripper=True)
    logged = events(caplog)
    assert [(e["step"], e["outcome"]) for _, e in logged] == [("gripper_open", "start"), ("gripper_open", "ok")]
    assert all(e["flow"] == CALIB_GENERAL_FLOW == "calib_general" for _, e in logged)
    assert len({e["run_id"] for _, e in logged}) == 1
    start, result = (e for _, e in logged)
    assert start["data"] == {"action": "open", "gripper": pytest.approx(POSE[GRIPPER])}
    assert result["data"]["status"] == "converged"
    assert result["data"]["gripper_before"] == pytest.approx(POSE[GRIPPER])
    assert result["data"]["gripper_after"] == pytest.approx(CONFIG.arm.gripper_open_rad, abs=0.02)
    assert result["duration_s"] is not None


def test_gripper_logs_a_refusal_at_warning(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    arm, _ = make(tmp_path)
    arm.acquire()
    with caplog.at_level(logging.INFO, logger="mcp_server.arm"):
        arm.gripper_service_call(open_gripper=False)
    logged = events(caplog)
    assert [(lvl, e["step"], e["outcome"]) for lvl, e in logged] == [
        (logging.INFO, "gripper_close", "start"),
        (logging.WARNING, "gripper_close", "refused"),
    ]


def test_gripper_logs_a_failure_at_error(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    arm, be = make(tmp_path)
    be.stale_after = be.t + 0.2
    with caplog.at_level(logging.INFO, logger="mcp_server.arm"):
        arm.gripper_service_call(open_gripper=True)
    level, last = events(caplog)[-1]
    assert level == logging.ERROR and last["outcome"] == "failed"
    assert last["data"]["status"] == "aborted_stale"


# --- head home -------------------------------------------------------------------------------------------------------


def head(backend: FakeHeadBackend) -> HeadController:
    """Head controller on the default settings.

    Args:
        backend (FakeHeadBackend): Simulated head.

    Returns:
        HeadController: The controller.
    """
    return HeadController(backend, AppliedHeadKinematics(CONFIG.head.urdf_path, None), CONFIG.head)


def call_head(backend: FakeHeadBackend) -> tuple[bool, str]:
    """Run head_home_call on a simulated head.

    Args:
        backend (FakeHeadBackend): Simulated head.

    Returns:
        tuple[bool, str]: The service answer.
    """
    controller = head(backend)
    return head_home_call(controller.home, controller.measured, lambda: backend.servo_released("head"), backend.now)


def test_head_home_moves_to_the_home_angles() -> None:
    be = FakeHeadBackend({"head_pan": 0.3, "head_tilt": -0.1})
    ok, message = call_head(be)
    assert ok is True, message
    assert be.commands == [{"head_pan": CONFIG.head.home_pan_rad, "head_tilt": CONFIG.head.home_tilt_rad}]
    assert message.startswith("converged")


def test_head_home_fails_unless_converged() -> None:
    be = FakeHeadBackend({"head_pan": 0.3, "head_tilt": -0.1}, speed=0.0)  # stuck follower
    ok, message = call_head(be)
    assert ok is False
    assert message.startswith("timeout")


def test_head_home_without_feedback_fails() -> None:
    be = FakeHeadBackend()
    be.no_samples = True
    ok, message = call_head(be)
    assert ok is False and message.startswith("no_feedback")


def test_head_home_refused_while_the_head_torque_is_off() -> None:
    be = FakeHeadBackend()
    be.released = True
    ok, message = call_head(be)
    assert ok is False and "torque" in message
    assert be.commands == []


def test_head_error_is_refused() -> None:
    def bad() -> HeadMoveResult:
        raise HeadError("head_tilt 99.0 deg is outside the limits [-10.0, 60.0] deg")

    ok, message = head_home_call(bad, lambda: None, lambda: None, lambda: 0.0)
    assert ok is False and "outside the limits" in message


def test_head_home_logs(caplog: pytest.LogCaptureFixture) -> None:
    be = FakeHeadBackend({"head_pan": 0.3, "head_tilt": -0.1})
    with caplog.at_level(logging.INFO, logger="mcp_server.calib_general"):
        call_head(be)
    logged = events(caplog)
    assert [(e["flow"], e["step"], e["outcome"]) for _, e in logged] == [
        ("calib_general", HEAD_HOME_STEP, "start"),
        ("calib_general", HEAD_HOME_STEP, "ok"),
    ]
    start, result = (e for _, e in logged)
    assert start["data"]["measured"] == {"head_pan": 0.3, "head_tilt": -0.1}
    assert result["data"]["status"] == "converged"
    assert result["data"]["target"] == {"head_pan": CONFIG.head.home_pan_rad, "head_tilt": CONFIG.head.home_tilt_rad}


def test_head_home_logs_refusal_and_failure(caplog: pytest.LogCaptureFixture) -> None:
    released = FakeHeadBackend()
    released.released = True
    stuck = FakeHeadBackend({"head_pan": 0.3, "head_tilt": -0.1}, speed=0.0)
    with caplog.at_level(logging.INFO, logger="mcp_server.calib_general"):
        call_head(released)
        call_head(stuck)
    outcomes = [(lvl, e["outcome"]) for lvl, e in events(caplog)]
    assert outcomes == [
        (logging.INFO, "start"),
        (logging.WARNING, "refused"),
        (logging.INFO, "start"),
        (logging.ERROR, "failed"),
    ]


# --- steering home ---------------------------------------------------------------------------------------------------


class FakeBase:
    """Steering that recentres on reset_base; records the calls."""

    def __init__(self, steer: dict[str, float] | None = None, recentre: bool = True, cancel_ok: bool = True) -> None:
        self.steer = dict(
            steer if steer is not None else {"fl_steer": 0.4, "fr_steer": 0.0, "rl_steer": 0.0, "rr_steer": 0.0}
        )
        self.recentre = recentre
        self.cancel_ok = cancel_ok
        self.resets = 0
        self.held = False
        self.released: bool | None = None

    def reset_base(self) -> bool:
        self.resets += 1
        if self.recentre:
            self.steer = dict(STEER)
        return self.cancel_ok

    def steer_positions(self) -> dict[str, float] | None:
        return dict(self.steer)

    def wait(self) -> tuple[bool, str]:
        if all(abs(v) <= CONFIG.control.steer_neutral_tolerance_rad for v in self.steer.values()):
            return True, ""
        worst = max(self.steer, key=lambda j: abs(self.steer[j]))
        return False, f"steering not neutral within 6 s ({worst} {self.steer[worst]:.2f} rad, tolerance 0.05 rad)"


def call_steer(base: FakeBase) -> tuple[bool, str]:
    """Run steer_home_call on a fake base.

    Args:
        base (FakeBase): Fake base.

    Returns:
        tuple[bool, str]: The service answer.
    """
    return steer_home_call(
        lambda: base.held, lambda: base.released, base.reset_base, base.wait, base.steer_positions, Clock()
    )


def test_steer_home_resets_and_waits_for_neutral() -> None:
    base = FakeBase()
    ok, message = call_steer(base)
    assert ok is True, message
    assert base.resets == 1
    assert "neutral" in message


def test_steer_home_refused_while_the_base_lease_is_held() -> None:
    base = FakeBase()
    base.held = True
    ok, message = call_steer(base)
    assert ok is False
    assert "base lease" in message and "release control" in message
    assert base.resets == 0


def test_steer_home_refused_while_the_base_torque_is_off() -> None:
    base = FakeBase()
    base.released = True
    ok, message = call_steer(base)
    assert ok is False and "torque" in message
    assert base.resets == 0


def test_steer_home_names_the_worst_joint_when_not_neutral() -> None:
    base = FakeBase(recentre=False)
    ok, message = call_steer(base)
    assert ok is False
    assert "fl_steer" in message


def test_steer_home_fails_when_the_nav2_cancel_did_not_answer() -> None:
    base = FakeBase(cancel_ok=False)
    ok, message = call_steer(base)
    assert ok is False
    assert "Nav2 cancel" in message


def test_steer_home_logs(caplog: pytest.LogCaptureFixture) -> None:
    held = FakeBase()
    held.held = True
    stuck = FakeBase(recentre=False)
    with caplog.at_level(logging.INFO, logger="mcp_server.calib_general"):
        call_steer(FakeBase())
        call_steer(held)
        call_steer(stuck)
    logged = events(caplog)
    assert [(lvl, e["step"], e["outcome"]) for lvl, e in logged] == [
        (logging.INFO, STEER_HOME_STEP, "start"),
        (logging.INFO, STEER_HOME_STEP, "ok"),
        (logging.INFO, STEER_HOME_STEP, "start"),
        (logging.WARNING, STEER_HOME_STEP, "refused"),
        (logging.INFO, STEER_HOME_STEP, "start"),
        (logging.ERROR, STEER_HOME_STEP, "failed"),
    ]
    assert all(e["flow"] == "calib_general" for _, e in logged)
    assert logged[0][1]["data"]["steer"]["fl_steer"] == pytest.approx(0.4)
    assert logged[1][1]["data"]["steer"] == STEER
