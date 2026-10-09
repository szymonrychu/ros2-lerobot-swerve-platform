"""Tests for the motion queue MCP tools (enqueue_motions, get_motion_status, cancel_motions, wait_for_event) and the
single-owner rule between the queue and the blocking motion tools."""

import threading
from pathlib import Path
from typing import Any

import pytest
from mcp.server.mcpserver.exceptions import ToolError

from mcp_server.config import McpServerConfig
from mcp_server.tools import (
    ALWAYS_ALLOWED_TOOLS,
    MOTION_TOOLS,
    QUEUE_EXCLUSIVE_TOOLS,
    TOOL_NAMES,
    build_mcp_server,
    stop_queued_motion,
)

from .test_motion_queue import GateRobot
from .test_tools import TOKEN, call

WAIT = {"timeout_s": 5.0}


@pytest.fixture
def robot(tmp_path: Path) -> GateRobot:
    r = GateRobot(tmp_path)
    r.gate.set()
    return r


@pytest.fixture
def server(robot: GateRobot) -> Any:
    cfg = McpServerConfig(objects={"store_path": robot.arm.cfg.arm.home_file.parent / "objects.json"})
    return build_mcp_server(robot, cfg, TOKEN)


def structured(result: Any) -> dict[str, Any]:
    return result.structured_content


def test_queue_tools_are_registered_and_classified() -> None:
    names = {"enqueue_motions", "get_motion_status", "cancel_motions", "wait_for_event"}
    assert names <= set(TOOL_NAMES)
    assert "enqueue_motions" in MOTION_TOOLS
    assert {"get_motion_status", "cancel_motions", "wait_for_event"} <= ALWAYS_ALLOWED_TOOLS
    assert "enqueue_motions" not in QUEUE_EXCLUSIVE_TOOLS and "stop" not in QUEUE_EXCLUSIVE_TOOLS
    assert {"move_arm_joints", "navigate_to_pose", "look_around", "grasp_object", "drive"} <= QUEUE_EXCLUSIVE_TOOLS


def test_enqueue_returns_job_ids_and_wait_returns_events_and_state(server: Any, robot: GateRobot) -> None:
    out = structured(
        call(
            server,
            "enqueue_motions",
            {
                "steps": [
                    {"kind": "arm_joints", "targets": {"shoulder_pan": 0.2}},
                    {"kind": "arm_joints", "targets": {"shoulder_pan": 0.4}},
                    {"kind": "gripper", "open_fraction": 0.4},
                ]
            },
        )
    )
    assert len(out["job_ids"]) == 3
    assert out["blend_groups"] == [out["job_ids"][:2], out["job_ids"][2:]]
    waited = structured(call(server, "wait_for_event", WAIT))
    assert waited["reason"] == "queue_empty" and waited["timed_out"] is False
    assert [e["type"] for e in waited["events"]][-1] == "queue_empty"
    state = waited["state"]
    assert state["arm_joints"]["shoulder_pan"] == pytest.approx(0.4, abs=1e-3)
    assert state["gripper"]["open_fraction"] == pytest.approx(0.4, abs=0.01)
    assert state["base_pose"]["frame"] == "map"
    assert "robot_events_since_last_call" in waited  # the digest of every tool call is still attached


def test_blocking_motion_tools_are_refused_while_the_queue_runs(server: Any, robot: GateRobot) -> None:
    robot.gate.clear()
    call(server, "enqueue_motions", {"steps": [{"kind": "navigate_to_pose", "x": 1.0, "y": 0.0}]})
    assert robot.entered.wait(5.0)
    for name, args in (
        ("move_arm_joints", {"targets": {"elbow_flex": 0.2}}),
        ("move_relative", {"dx": 0.1}),
        ("set_gripper", {"open_fraction": 0.5}),
    ):
        with pytest.raises(ToolError, match="motion queue"):
            call(server, name, args)
    # Read-only tools and the queue tools keep working.
    status = structured(call(server, "get_motion_status", {}))
    assert status["running"] is True and status["current"]["kind"] == "navigate_to_pose"
    call(server, "get_robot_state", {})
    robot.gate.set()
    call(server, "wait_for_event", WAIT)
    call(server, "move_arm_joints", {"targets": {"elbow_flex": 0.2}})  # allowed again once drained


def test_enqueue_is_refused_while_a_blocking_motion_tool_runs(server: Any, robot: GateRobot) -> None:
    robot.gate.clear()
    errors: list[Exception] = []

    def blocking() -> None:
        try:
            call(server, "navigate_to_pose", {"x": 1.0, "y": 0.0})
        except Exception as exc:  # noqa: BLE001 - surfaced through the list
            errors.append(exc)

    worker = threading.Thread(target=blocking)
    worker.start()
    assert robot.entered.wait(5.0)
    with pytest.raises(ToolError, match="navigate_to_pose"):
        call(server, "enqueue_motions", {"steps": [{"kind": "wait_s", "seconds": 0.1}]})
    robot.gate.set()
    worker.join(5.0)
    assert errors == []
    call(server, "enqueue_motions", {"steps": [{"kind": "wait_s", "seconds": 0.01}]})


def test_stop_clears_the_queue_and_reports_dropped_steps(server: Any, robot: GateRobot) -> None:
    robot.gate.clear()
    jobs = structured(
        call(
            server,
            "enqueue_motions",
            {
                "steps": [
                    {"kind": "navigate_to_pose", "x": 1.0, "y": 0.0},
                    {"kind": "arm_joints", "targets": {"shoulder_pan": 0.3}},
                ]
            },
        )
    )["job_ids"]
    assert robot.entered.wait(5.0)
    stopped = structured(call(server, "stop", {}))
    assert stopped["motion_queue_dropped"] == [jobs[1]]
    waited = structured(call(server, "wait_for_event", WAIT))
    assert "stopped" in [e["type"] for e in waited["events"]]
    assert waited["status"]["running"] is False
    assert robot.arm_backend.commands == [] or all(c["shoulder_pan"] == 0.0 for c in robot.arm_backend.commands)


def test_cancel_motions_tool(server: Any, robot: GateRobot) -> None:
    robot.gate.clear()
    call(
        server,
        "enqueue_motions",
        {"steps": [{"kind": "navigate_to_pose", "x": 1.0, "y": 0.0}, {"kind": "wait_s", "seconds": 1.0}]},
    )
    assert robot.entered.wait(5.0)
    out = structured(call(server, "cancel_motions", {}))
    assert len(out["dropped"]) == 1 and out["running"]
    waited = structured(call(server, "wait_for_event", WAIT))
    assert waited["status"]["running"] is False


def test_enqueue_reports_infeasible_steps_as_a_tool_error(server: Any) -> None:
    with pytest.raises(ToolError, match="unreachable"):
        call(server, "enqueue_motions", {"steps": [{"kind": "arm_cartesian", "x": 2.0, "y": 0.0, "z": 0.0}]})


def test_enqueue_replace_flag(server: Any, robot: GateRobot) -> None:
    robot.gate.clear()
    first = structured(
        call(
            server,
            "enqueue_motions",
            {"steps": [{"kind": "navigate_to_pose", "x": 1.0, "y": 0.0}, {"kind": "wait_s", "seconds": 1.0}]},
        )
    )
    assert robot.entered.wait(5.0)
    second = structured(
        call(server, "enqueue_motions", {"steps": [{"kind": "wait_s", "seconds": 0.01}], "replace": True})
    )
    assert second["replaced"] == [first["job_ids"][1]]
    robot.gate.set()
    call(server, "wait_for_event", WAIT)


def test_wait_for_event_until_failure_returns_idle_at_once(server: Any) -> None:
    waited = structured(call(server, "wait_for_event", {"timeout_s": 5.0, "until": "failure"}))
    assert waited["reason"] == "idle"


def test_shutdown_halts_a_busy_queue_and_stops_the_robot(server: Any, robot: GateRobot) -> None:
    robot.gate.clear()
    call(server, "enqueue_motions", {"steps": [{"kind": "navigate_to_pose", "x": 1.0, "y": 0.0}]})
    assert robot.entered.wait(5.0)
    assert stop_queued_motion(server, robot) is True
    assert robot.stopped.is_set()
    assert not server.motion_queue.wait(5.0, "queue_empty").timed_out


def test_shutdown_leaves_an_idle_robot_alone(server: Any, robot: GateRobot) -> None:
    assert stop_queued_motion(server, robot) is False
    assert not robot.stopped.is_set()
