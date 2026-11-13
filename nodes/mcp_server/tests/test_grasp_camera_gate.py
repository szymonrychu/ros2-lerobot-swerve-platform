"""Direct grasps from a camera-derived position are gated on the camera-to-arm residual: the accepted arm to camera
calibration's held-out tool error when it was fitted against the applied head chain, else the head-from-arm residual."""

import json
from pathlib import Path
from typing import Any

import pytest
from mcp.server.mcpserver.exceptions import ToolError
from pydantic import ValidationError
from ros2_common.head_rgbd_calibration.urdf_text import ChainResidual

from mcp_server.config import McpServerConfig
from mcp_server.grasp import ObjectSpec, grasp_params
from mcp_server.grasp_tools import REZERO_WARNING, GraspExecutor, GraspService
from mcp_server.head import AppliedHeadKinematics, HeadController
from mcp_server.tools import build_mcp_server

from .fakes import FakeHeadBackend
from .test_calibration_marker import marker_doc
from .test_grasp_tools import OBJECT, object_in_jaws
from .test_tools import TOKEN, FakeRobot, call, take

from .depth_helpers import write_applied, write_arm_calibration  # isort: skip

CAMERA_OBJECT = {**OBJECT, "position_source": "camera"}


def with_calibration(robot: FakeRobot, directory: Path, residual: ChainResidual | None) -> None:
    """Put an applied head chain (with or without an arm-frame residual) behind the fake robot's head."""
    applied = write_applied(directory, residual, source="head_arm" if residual else "floor")
    cfg = McpServerConfig()
    robot.head = HeadController(FakeHeadBackend(), AppliedHeadKinematics(cfg.head.urdf_path, applied), cfg.head)


def with_arm_calibration(
    robot: FakeRobot, directory: Path, tool_mm: float, head_chain: tuple[str, str] = ("calib-1", "r-1")
) -> None:
    """A ground-board head chain (no residual of its own) plus an accepted fake arm to camera calibration."""
    applied = write_applied(directory, None, source="ground_board")
    write_arm_calibration(applied, tool_mm, head_chain)
    cfg = McpServerConfig()
    robot.head = HeadController(FakeHeadBackend(), AppliedHeadKinematics(cfg.head.urdf_path, applied), cfg.head)


def server_for(robot: FakeRobot, **camera_arm: Any) -> Any:
    cfg = McpServerConfig()
    for key, value in camera_arm.items():
        setattr(cfg.camera_arm, key, value)
    server = build_mcp_server(robot, cfg, TOKEN)
    take(server, "arm")
    return server


@pytest.fixture
def robot(tmp_path: Path) -> FakeRobot:
    return FakeRobot(tmp_path)


def test_position_source_defaults_to_manual_and_is_validated() -> None:
    assert ObjectSpec(**OBJECT).position_source == "manual"
    assert ObjectSpec(**CAMERA_OBJECT).position_source == "camera"
    with pytest.raises(ValidationError):
        ObjectSpec(**{**OBJECT, "position_source": "guess"})


def test_camera_grasp_without_an_arm_frame_residual_is_refused_before_any_motion(
    robot: FakeRobot, tmp_path: Path
) -> None:
    with_calibration(robot, tmp_path / "applied", None)
    server = server_for(robot)
    commands_before = len(robot.arm_backend.commands)
    with pytest.raises(ToolError, match=r"no arm-frame residual to check against the 5 mm gate"):
        call(server, "grasp_object", {"object": CAMERA_OBJECT, "strategy": "top_down"})
    assert len(robot.arm_backend.commands) == commands_before


def test_camera_grasp_with_a_residual_above_the_gate_is_refused(robot: FakeRobot, tmp_path: Path) -> None:
    with_calibration(robot, tmp_path / "applied", ChainResidual("head_arm", 6.2, 3.0, 3.0))
    server = server_for(robot)
    commands_before = len(robot.arm_backend.commands)
    with pytest.raises(ToolError, match=r"residual 6\.2 mm is above the 5 mm gate"):
        call(server, "grasp_object", {"object": CAMERA_OBJECT, "strategy": "top_down"})
    assert len(robot.arm_backend.commands) == commands_before


def test_camera_grasp_within_the_gate_runs(robot: FakeRobot, tmp_path: Path) -> None:
    with_calibration(robot, tmp_path / "applied", ChainResidual("head_arm", 2.9, 2.5, 2.7))
    server = server_for(robot)
    commands_before = len(robot.arm_backend.commands)
    object_in_jaws(robot.arm_backend)
    data = call(server, "grasp_object", {"object": CAMERA_OBJECT, "strategy": "top_down"}).structured_content
    assert data["outcome"] != "infeasible" and len(robot.arm_backend.commands) > commands_before


def test_camera_grasp_with_a_matching_arm_calibration_within_the_gate_runs(robot: FakeRobot, tmp_path: Path) -> None:
    """The robot's case: a ground-board head chain and an arm calibration fitted against it (2.67 mm)."""
    with_arm_calibration(robot, tmp_path / "applied", 2.67)
    server = server_for(robot)
    commands_before = len(robot.arm_backend.commands)
    object_in_jaws(robot.arm_backend)
    data = call(server, "grasp_object", {"object": CAMERA_OBJECT, "strategy": "top_down"}).structured_content
    assert data["outcome"] != "infeasible" and len(robot.arm_backend.commands) > commands_before


def test_camera_grasp_with_a_matching_arm_calibration_above_the_gate_is_refused(
    robot: FakeRobot, tmp_path: Path
) -> None:
    with_arm_calibration(robot, tmp_path / "applied", 6.5)
    server = server_for(robot)
    commands_before = len(robot.arm_backend.commands)
    with pytest.raises(
        ToolError, match=r"arm to camera calibration's camera-to-arm residual 6\.5 mm is above the 5 mm"
    ):
        call(server, "grasp_object", {"object": CAMERA_OBJECT, "strategy": "top_down"})
    assert len(robot.arm_backend.commands) == commands_before


def test_camera_grasp_with_an_arm_calibration_of_another_head_chain_is_refused_as_stale(
    robot: FakeRobot, tmp_path: Path
) -> None:
    with_arm_calibration(robot, tmp_path / "applied", 2.0, ("fake-other-ground-1", "fake-other-r-1"))
    server = server_for(robot)
    commands_before = len(robot.arm_backend.commands)
    with pytest.raises(ToolError, match=r"arm to camera calibration is stale for the applied head chain"):
        call(server, "grasp_object", {"object": CAMERA_OBJECT, "strategy": "top_down"})
    assert len(robot.arm_backend.commands) == commands_before


def test_the_gate_threshold_is_configurable(robot: FakeRobot, tmp_path: Path) -> None:
    with_calibration(robot, tmp_path / "applied", ChainResidual("head_arm", 6.2, 3.0, 3.0))
    server = server_for(robot, max_residual_mm=7.0)
    commands_before = len(robot.arm_backend.commands)
    object_in_jaws(robot.arm_backend)
    data = call(server, "grasp_object", {"object": CAMERA_OBJECT, "strategy": "top_down"}).structured_content
    assert data["outcome"] != "infeasible" and len(robot.arm_backend.commands) > commands_before
    tight = server_for(robot, max_residual_mm=3.0)
    with pytest.raises(ToolError, match=r"above the 3 mm gate"):
        call(tight, "grasp_object", {"object": CAMERA_OBJECT, "strategy": "top_down"})


def test_a_manual_position_is_not_gated(robot: FakeRobot, tmp_path: Path) -> None:
    with_calibration(robot, tmp_path / "applied", None)
    server = server_for(robot)
    commands_before = len(robot.arm_backend.commands)
    object_in_jaws(robot.arm_backend)
    data = call(server, "grasp_object", {"object": OBJECT, "strategy": "top_down"}).structured_content
    assert data["outcome"] != "infeasible" and len(robot.arm_backend.commands) > commands_before


def test_plan_grasp_still_plans_but_says_grasp_object_will_refuse(robot: FakeRobot, tmp_path: Path) -> None:
    with_calibration(robot, tmp_path / "applied", ChainResidual("head_arm", 6.2, 3.0, 3.0))
    server = server_for(robot)
    commands_before = len(robot.arm_backend.commands)
    data = call(server, "plan_grasp", {"object": CAMERA_OBJECT, "strategy": "top_down"}).structured_content
    assert data["outcome"] == "planned"
    assert any("grasp_object will refuse this camera position" in reason for reason in data["reasons"])
    assert len(robot.arm_backend.commands) == commands_before


def test_plan_grasp_within_the_gate_adds_no_refusal_note(robot: FakeRobot, tmp_path: Path) -> None:
    with_calibration(robot, tmp_path / "applied", ChainResidual("head_arm", 2.9, 2.5, 2.7))
    server = server_for(robot)
    data = call(server, "plan_grasp", {"object": CAMERA_OBJECT, "strategy": "top_down"}).structured_content
    assert data["outcome"] == "planned" and not any("will refuse" in reason for reason in data["reasons"])


def test_a_queued_camera_grasp_is_gated_too(robot: FakeRobot, tmp_path: Path) -> None:
    """enqueue_motions grasp steps run the same executor: the gate must hold there as well as in grasp_object."""
    with_calibration(robot, tmp_path / "applied", ChainResidual("head_arm", 6.2, 3.0, 3.0))
    server = server_for(robot)
    commands_before = len(robot.arm_backend.commands)
    call(server, "enqueue_motions", {"steps": [{"kind": "grasp", "object": CAMERA_OBJECT, "strategy": "top_down"}]})
    waited = call(server, "wait_for_event", {"timeout_s": 5.0}).structured_content
    assert waited["reason"] != "timeout", waited
    failures = [e for e in waited["events"] if e["type"] == "step_failed"]
    assert failures and "above the 5 mm gate" in json.dumps(failures), waited["events"]
    assert len(robot.arm_backend.commands) == commands_before


def test_a_camera_grasp_on_the_web_service_is_gated_too(robot: FakeRobot, tmp_path: Path) -> None:
    """GraspService (/grasp/command) executes through the same executor."""
    with_calibration(robot, tmp_path / "applied", None)
    cfg = McpServerConfig()
    service = GraspService(robot.arm, cfg, calibration=robot.head.calibration)
    commands_before = len(robot.arm_backend.commands)
    answer = service.handle({"action": "execute", "object": CAMERA_OBJECT, "strategy": "top_down"})
    assert answer is not None and answer["ok"] is False
    assert "no arm-frame residual" in answer["error"]
    assert len(robot.arm_backend.commands) == commands_before


def test_an_executor_without_a_calibration_source_refuses_a_camera_position(robot: FakeRobot) -> None:
    executor = GraspExecutor(robot.arm, McpServerConfig())
    with pytest.raises(ValueError, match="no head calibration"):
        executor.grasp(
            ObjectSpec(**CAMERA_OBJECT),
            "top_down",
            grasp_params(McpServerConfig().grasp, None),
            None,
            None,
            lambda: False,
        )


def rezeroed_robot(robot: FakeRobot, tmp_path: Path, stale: list[str] | None = None) -> Any:
    """A server whose arm re-zero marker lists `stale` (both items when None) and whose camera calibration passes."""
    with_arm_calibration(robot, tmp_path / "applied", 2.67)
    marker = tmp_path / "marker.json"
    marker.write_text(json.dumps(marker_doc(stale)))
    robot.arm.cfg.arm.calibration_stale_marker = marker
    return server_for(robot)


def test_a_camera_grasp_after_an_arm_rezero_warns_and_still_runs(robot: FakeRobot, tmp_path: Path) -> None:
    server = rezeroed_robot(robot, tmp_path)
    object_in_jaws(robot.arm_backend)
    commands_before = len(robot.arm_backend.commands)
    data = call(server, "grasp_object", {"object": CAMERA_OBJECT, "strategy": "top_down"}).structured_content
    assert data["outcome"] != "infeasible" and len(robot.arm_backend.commands) > commands_before
    assert REZERO_WARNING in data["warnings"]


def test_grasp_approach_after_an_arm_rezero_warns_and_still_lands(robot: FakeRobot, tmp_path: Path) -> None:
    server = rezeroed_robot(robot, tmp_path)
    data = call(server, "grasp_approach", {"object": CAMERA_OBJECT, "strategy": "top_down"}).structured_content
    assert data["outcome"] == "landed" and REZERO_WARNING in data["warnings"]


def test_plan_grasp_after_an_arm_rezero_notes_the_warning(robot: FakeRobot, tmp_path: Path) -> None:
    server = rezeroed_robot(robot, tmp_path)
    data = call(server, "plan_grasp", {"object": CAMERA_OBJECT, "strategy": "top_down"}).structured_content
    assert data["outcome"] == "planned" and REZERO_WARNING in data["reasons"]


def test_a_manual_position_gets_no_rezero_warning(robot: FakeRobot, tmp_path: Path) -> None:
    server = rezeroed_robot(robot, tmp_path)
    assert (
        REZERO_WARNING
        not in call(server, "grasp_approach", {"object": OBJECT, "strategy": "top_down"}).structured_content["warnings"]
    )
    assert (
        REZERO_WARNING
        not in call(server, "plan_grasp", {"object": OBJECT, "strategy": "top_down"}).structured_content["reasons"]
    )


def test_a_camera_position_without_a_rezero_marker_or_with_home_only_stale_gets_no_warning(
    robot: FakeRobot, tmp_path: Path
) -> None:
    server = rezeroed_robot(robot, tmp_path, stale=["home"])
    data = call(server, "grasp_approach", {"object": CAMERA_OBJECT, "strategy": "top_down"}).structured_content
    assert REZERO_WARNING not in data["warnings"]
    executor = GraspExecutor(robot.arm, McpServerConfig())
    assert executor.rezero_warning(ObjectSpec(**CAMERA_OBJECT)) is None
    robot.arm.cfg.arm.calibration_stale_marker = tmp_path / "absent.json"
    assert executor.rezero_warning(ObjectSpec(**CAMERA_OBJECT)) is None


def test_an_unreadable_rezero_marker_counts_as_stale(robot: FakeRobot, tmp_path: Path) -> None:
    server = rezeroed_robot(robot, tmp_path)
    assert robot.arm.cfg.arm.calibration_stale_marker is not None
    robot.arm.cfg.arm.calibration_stale_marker.write_text("not json")
    data = call(server, "grasp_approach", {"object": CAMERA_OBJECT, "strategy": "top_down"}).structured_content
    assert REZERO_WARNING in data["warnings"]
