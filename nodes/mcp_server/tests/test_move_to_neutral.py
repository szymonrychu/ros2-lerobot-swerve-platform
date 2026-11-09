"""/arm/move_to_neutral: every arm joint (gripper excluded) to 0.0 measured via the home routing, with its refusals."""

import json
import logging
from pathlib import Path
from typing import Any

import pytest
from arm_collision.ik import ArmKinematics, load_joint_limits

from mcp_server.arm import MOVE_TO_NEUTRAL_RELEASE_NOTE, ArmController, ArmError
from mcp_server.calib_event import CALIB_EVENT_PREFIX
from mcp_server.config import McpServerConfig

from .fakes import FakeArmBackend
from .test_calibration_marker import POSE, marker_doc

CONFIG = McpServerConfig()
KIN = ArmKinematics(CONFIG.arm.urdf_path, margin=CONFIG.limits.arm_limit_margin_rad)
GRIPPER = CONFIG.arm.gripper_joint
NEUTRAL = {j: 0.0 for j in CONFIG.arm.joint_names if j != GRIPPER}
OFFSETS_REFUSAL = "calibration stale marker lists joint_offsets"


def make(tmp_path: Path, marker: dict | str | None = None) -> tuple[ArmController, FakeArmBackend]:
    """Controller at POSE with the stale marker path configured (and written when given).

    Args:
        tmp_path (Path): Test directory.
        marker (dict | str | None): Marker document, raw text, or None for no file.

    Returns:
        tuple[ArmController, FakeArmBackend]: Controller and backend.
    """
    cfg = CONFIG.model_copy(deep=True)
    cfg.arm.home_file = tmp_path / "arm" / "home.yaml"
    cfg.arm.calibration_stale_marker = tmp_path / "marker.json"
    if marker is not None:
        cfg.arm.calibration_stale_marker.write_text(marker if isinstance(marker, str) else json.dumps(marker))
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


def test_moves_every_arm_joint_to_zero_and_keeps_the_gripper(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    ok, message = arm.move_to_neutral_service_call()
    assert ok is True, message
    assert message.startswith("converged")
    for joint, value in NEUTRAL.items():
        assert be.positions[joint] == pytest.approx(value, abs=0.02)
    assert be.positions[GRIPPER] == pytest.approx(POSE[GRIPPER])
    assert all(GRIPPER not in c or c[GRIPPER] == pytest.approx(POSE[GRIPPER]) for c in be.commands)


def test_uses_the_home_routing_with_the_zero_pose(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    arm, _ = make(tmp_path)
    seen: list[tuple[dict[str, float], Any, Any, Any]] = []
    real = arm.home_motion

    def spy(pose: dict[str, float], floor: Any, speed_scale: Any = None, board: Any = None) -> Any:
        seen.append((dict(pose), floor, speed_scale, board))
        return real(pose, floor, speed_scale, board)

    monkeypatch.setattr(arm, "home_motion", spy)
    arm.move_to_neutral()
    assert seen == [(NEUTRAL, None, None, None)]


def test_always_releases_control_afterwards(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    arm.acquire()
    ok, _ = arm.move_to_neutral_service_call()
    assert ok is True
    assert arm.control_held is False and be.releases == 1
    assert arm.lost_note == MOVE_TO_NEUTRAL_RELEASE_NOTE


@pytest.mark.parametrize("marker", [marker_doc(), marker_doc(["joint_offsets"]), "{not json"])
def test_refused_while_joint_offsets_are_stale(tmp_path: Path, marker: dict | str) -> None:
    arm, be = make(tmp_path, marker)
    ok, message = arm.move_to_neutral_service_call()
    assert ok is False
    assert OFFSETS_REFUSAL in message
    assert "old zero" in message
    assert be.commands == [] and arm.control_held is False


def test_allowed_when_only_home_is_stale(tmp_path: Path) -> None:
    arm, _ = make(tmp_path, marker_doc(["home"]))
    ok, message = arm.move_to_neutral_service_call()
    assert ok is True, message


def test_refused_while_the_arm_torque_is_released(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.released = True
    ok, message = arm.move_to_neutral_service_call()
    assert ok is False
    assert "torque" in message
    assert be.commands == []


def test_refusal_raises_arm_error_from_move_to_neutral(tmp_path: Path) -> None:
    arm, _ = make(tmp_path, marker_doc(["joint_offsets"]))
    with pytest.raises(ArmError, match=OFFSETS_REFUSAL):
        arm.move_to_neutral()


def test_logs_start_route_and_result_events(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    arm, _ = make(tmp_path)
    with caplog.at_level(logging.INFO, logger="mcp_server.arm"):
        ok, _ = arm.move_to_neutral_service_call()
    assert ok
    logged = events(caplog)
    assert [(e["step"], e["outcome"]) for _, e in logged] == [("request", "start"), ("route", "ok"), ("result", "ok")]
    assert all(e["flow"] == "move_to_neutral" for _, e in logged)
    assert len({e["run_id"] for _, e in logged}) == 1
    start, route, result = (e for _, e in logged)
    assert start["data"]["target"] == NEUTRAL
    assert start["data"]["measured"]["elbow_flex"] == pytest.approx(POSE["elbow_flex"])
    assert route["data"]["note"] == "direct" and route["data"]["legs"]
    assert result["data"]["status"] == "converged"
    assert result["data"]["measured"]["elbow_flex"] == pytest.approx(0.0, abs=0.02)
    assert result["duration_s"] is not None


def test_logs_a_refusal_at_warning(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    arm, be = make(tmp_path)
    be.released = True
    with caplog.at_level(logging.INFO, logger="mcp_server.arm"):
        arm.move_to_neutral_service_call()
    logged = events(caplog)
    assert [(lvl, e["step"], e["outcome"]) for lvl, e in logged] == [
        (logging.INFO, "request", "start"),
        (logging.WARNING, "request", "refused"),
    ]
    assert "torque" in logged[-1][1]["reason"]
