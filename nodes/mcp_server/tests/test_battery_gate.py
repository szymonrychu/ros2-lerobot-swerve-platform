"""Battery cut-off gate: motion/effector tools are refused in cut-off, stop and read-only tools keep working."""

import json
from pathlib import Path
from typing import Any

import pytest
from mcp.server.mcpserver.exceptions import ToolError
from ros2_common.battery import BatteryConfig, BatteryGuard
from ros2_common.maps import Localisation, MapStatus

from mcp_server.approach_tools import SENSOR_TOOL_NAMES as APPROACH_TOOL_NAMES
from mcp_server.camera_tools import TOOL_NAMES as CAMERA_TOOL_NAMES
from mcp_server.config import McpServerConfig, load_config
from mcp_server.tools import ALWAYS_ALLOWED_TOOLS, MOTION_TOOLS, TOOL_NAMES, build_mcp_server

from .test_frontier_tools import room
from .test_tools import TOKEN, FakeRobot, call, take

LOW_V = 8.21  # 2.74 V/cell
OK_V = 11.4
REFUSAL = "battery below cut-off: 8.21 V (2.74 V/cell < 2.80 V/cell); motion refused"
MOTION_ARGS: dict[str, dict[str, Any]] = {
    "navigate_to_pose": {"x": 1.0, "y": 0.0},
    "move_relative": {"dx": 0.5},
    "drive": {"vx": 0.1},
    "move_arm_joints": {"targets": {"elbow_flex": 0.2}},
    "move_arm_cartesian": {"x": 0.2, "y": 0.0, "z": 0.15},
    "set_gripper": {"open_fraction": 0.5},
    "arm_home": {},
    "arm_set_home": {},
    "look_around": {},
    "grasp_object": {
        "object": {"x": 0.22, "y": 0.0, "support_z": -0.15, "width_m": 0.03, "depth_m": 0.03, "height_m": 0.03}
    },
    "release_object": {},
    "enqueue_motions": {"steps": [{"kind": "wait_s", "seconds": 0.1}]},
    "resume_motions": {},
    "go_to_frontier": {"id": 1},
    "look_toward_frontier": {"id": 1},
    "approach_for_grasp": {"x": 1.5, "y": 2.0, "z": 0.05},
}
ALLOWED_ARGS: dict[str, dict[str, Any]] = {
    "get_robot_state": {},
    "get_camera_image": {"camera": "gripper"},
    "get_map_summary": {},
    "stop": {},
    "get_arm_state": {},
    "take_control": {"subsystem": "arm"},
    "release_control": {},
    "get_control_state": {},
    "get_body_state": {},
    "get_arm_reach": {},
    "pixel_to_ground": {"camera": "front", "u": 100.0, "v": 100.0},
    "get_annotated_camera_image": {"camera": "front"},
    "mark_candidate_points": {"camera": "front"},
    "resolve_candidate": {"set_id": "c1", "n": 1},
    "capture_calibration_sample": {"camera": "front", "u": 100.0, "v": 100.0, "ground_x": 1.0, "ground_y": 0.0},
    "solve_camera_calibration": {"camera": "front"},
    "clear_calibration_samples": {"camera": "front"},
    "get_topdown_view": {},
    "remember_object": {"label": "cup", "x": 1.0, "y": 1.0},
    "list_objects": {},
    "forget_object": {"id": "placeholder"},
    "list_pois": {},
    "add_poi": {"kind": "point", "name": "dock"},
    "update_poi": {"id": "id0", "note": "n"},
    "delete_poi": {"id": "id0"},
    "clear_pois": {},
    "plan_grasp": {
        "object": {"x": 0.22, "y": 0.0, "support_z": -0.15, "width_m": 0.03, "depth_m": 0.03, "height_m": 0.03}
    },
    "confirm_grasp": {"grasp_id": "0" * 12, "object_between_fingers": "yes", "evidence": "e"},
    "get_motion_status": {},
    "cancel_motions": {},
    "wait_for_event": {"timeout_s": 0.1},
    "check_guard_rules": {"tool": "navigate_to_pose"},
    "save_map": {"name": "m"},
    "list_maps": {},
    "load_map": {"name": "m"},
    "delete_map": {"name": "m", "confirm": True},
    "list_lookups": {},
    "get_lookup_image": {"lookup_id": "0" * 32},
    "annotate_lookup": {"lookup_id": "0" * 32, "note": "n"},
    "delete_photos": {"keep_newest": 1000, "dry_run": True},
    "list_frontiers": {},
    "plan_frontier_visit": {"id": 1},
    "suggest_approach_pose": {"x": 1.5, "y": 2.0, "z": 0.05},
}


def make_guard(voltage: float | None) -> BatteryGuard:
    """Guard with default thresholds, optionally already fed one reading."""
    guard = BatteryGuard.from_config(BatteryConfig())
    if voltage is not None:
        guard.update(voltage)
    return guard


def make_server(robot: FakeRobot, guard: BatteryGuard | None) -> Any:
    cfg = McpServerConfig(objects={"store_path": robot.arm.cfg.arm.home_file.parent / "objects.json"})
    cfg.cameras.calibration_dir = robot.arm.cfg.arm.home_file.parent / "calibration"
    server = build_mcp_server(robot, cfg, TOKEN, guard)
    take(server)  # both leases (allowed in cut-off), so the battery gate is what refuses
    return server


def test_tool_classification_covers_every_tool_exactly_once() -> None:
    assert MOTION_TOOLS | ALWAYS_ALLOWED_TOOLS == set(TOOL_NAMES)
    assert not MOTION_TOOLS & ALWAYS_ALLOWED_TOOLS
    assert MOTION_TOOLS == set(MOTION_ARGS)
    assert ALWAYS_ALLOWED_TOOLS == set(ALLOWED_ARGS)
    assert "stop" in ALWAYS_ALLOWED_TOOLS


@pytest.mark.parametrize("name", sorted(MOTION_ARGS))
def test_motion_tool_refused_in_cutoff_without_touching_robot(
    tmp_path: Path, name: str, caplog: pytest.LogCaptureFixture
) -> None:
    robot = FakeRobot(tmp_path)
    server = make_server(robot, make_guard(LOW_V))
    with caplog.at_level("WARNING"), pytest.raises(ToolError, match="battery below cut-off"):
        call(server, name, MOTION_ARGS[name])
    assert robot.calls == []
    assert any(name in r.getMessage() and "refused" in r.getMessage() for r in caplog.records)


def test_refusal_message_is_exact(tmp_path: Path) -> None:
    server = make_server(FakeRobot(tmp_path), make_guard(LOW_V))
    with pytest.raises(ToolError) as exc:
        call(server, "drive", {"vx": 0.1})
    assert REFUSAL in str(exc.value)


@pytest.mark.parametrize("name", sorted(ALLOWED_ARGS))
def test_stop_and_read_only_tools_work_in_cutoff(tmp_path: Path, name: str) -> None:
    robot = FakeRobot(tmp_path)
    robot.pois.append({"id": "id0", "name": "dock", "kind": "point", "x": 0.0, "y": 0.0, "status": "open"})
    robot.map = room()  # a map with frontier 1 for the frontier sensor tools
    robot.map_status_msg = MapStatus(maps=[], active_map=None, load_seq=0, localisation=Localisation(state="unknown"))
    server = make_server(robot, make_guard(LOW_V))
    args = dict(ALLOWED_ARGS[name])
    if name == "forget_object":
        args["id"] = call(server, "remember_object", ALLOWED_ARGS["remember_object"]).structured_content["id"]
    if name in ("get_lookup_image", "annotate_lookup"):  # a real lookup, recorded by a camera call in cut-off
        taken = call(server, "get_camera_image", {"camera": "front"})
        args["lookup_id"] = json.loads(next(c.text for c in taken.content if c.type == "text"))["lookup_id"]
    try:
        call(server, name, args)
    except ToolError as exc:  # uncalibrated cameras, an unknown grasp_id or a missing costmap fail on their own terms
        assert "battery below cut-off" not in str(exc)
        assert name in CAMERA_TOOL_NAMES or name == "confirm_grasp" or name in APPROACH_TOOL_NAMES


@pytest.mark.parametrize("voltage", [OK_V, None])
def test_motion_works_with_good_or_unknown_battery(tmp_path: Path, voltage: float | None) -> None:
    robot = FakeRobot(tmp_path)
    server = make_server(robot, make_guard(voltage))
    call(server, "drive", {"vx": 0.1})
    call(server, "navigate_to_pose", {"x": 1.0, "y": 0.0})
    assert [c[0] for c in robot.calls] == ["drive", "navigate"]


def test_motion_works_without_battery_guard(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    server = make_server(robot, None)
    call(server, "drive", {"vx": 0.1})
    assert robot.calls[-1][0] == "drive"


def test_motion_resumes_after_recovery(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    guard = make_guard(LOW_V)
    server = make_server(robot, guard)
    with pytest.raises(ToolError):
        call(server, "drive", {"vx": 0.1})
    guard.update(OK_V)
    call(server, "drive", {"vx": 0.1})
    assert robot.calls[-1][0] == "drive"


def test_battery_config_section_is_optional(tmp_path: Path) -> None:
    assert McpServerConfig().battery is None
    path = tmp_path / "c.yaml"
    path.write_text("battery:\n  topic: /battery_state\n  cells: 4\n")
    cfg = load_config(path)
    assert cfg.battery is not None and cfg.battery.cells == 4 and cfg.battery.cutoff_cell_v == 2.8


def test_battery_config_validated(tmp_path: Path) -> None:
    path = tmp_path / "c.yaml"
    path.write_text("battery:\n  cutoff_cell_v: 3.0\n  resume_cell_v: 2.9\n")
    with pytest.raises(ValueError):
        load_config(path)
