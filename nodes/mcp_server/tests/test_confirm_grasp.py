"""Tests for the confirm_grasp tool: the agent's visual verdict fused with the close's sensor band."""

import json
from pathlib import Path
from typing import Any

import anyio
import pytest
from mcp.server.mcpserver.exceptions import ToolError
from prometheus_client import REGISTRY

from mcp_server.config import McpServerConfig
from mcp_server.grasp_check import Band, ImageVerdict, fuse
from mcp_server.tools import ALWAYS_ALLOWED_TOOLS, MOTION_TOOLS, build_mcp_server

from .test_grasp_tools import OBJECT, START, object_in_jaws
from .test_tools import TOKEN, FakeRobot, call, take

EVIDENCE = "the block sits between both finger pads"
BANDS: tuple[Band, ...] = ("object", "empty", "ambiguous")
VERDICTS: tuple[ImageVerdict, ...] = ("yes", "no", "unsure")


@pytest.fixture
def robot(tmp_path: Path) -> FakeRobot:
    fake = FakeRobot(tmp_path)
    fake.arm_backend.positions.update(START)
    return fake


@pytest.fixture
def server(robot: FakeRobot) -> Any:
    server = build_mcp_server(robot, McpServerConfig(), TOKEN)
    take(server, "arm")
    return server


def held_grasp(server: Any, robot: FakeRobot, band: str | None = None) -> str:
    """Run grasp_object on an object in the jaws and return its grasp_id; band forces the stored sensor band."""
    object_in_jaws(robot.arm_backend)
    data = call(server, "grasp_object", {"object": OBJECT, "strategy": "top_down"}).structured_content
    assert data["outcome"] == "unverified", data["reasons"]
    grasp_id = data["grasp_check"]["grasp_id"]
    if band is not None:
        robot.arm.carry.update(band=band)
        robot.arm.note_check(grasp_id, band=band)
    return grasp_id


def confirm(server: Any, grasp_id: str, verdict: str, evidence: str = EVIDENCE) -> dict[str, Any]:
    res = call(
        server,
        "confirm_grasp",
        {"grasp_id": grasp_id, "object_between_fingers": verdict, "evidence": evidence},
    )
    return res.structured_content


def count(band: str, image: str, fused: str) -> float:
    labels = {"band": band, "image": image, "fused": fused}
    return REGISTRY.get_sample_value("mcp_grasp_verdicts_total", labels) or 0.0


def test_confirm_grasp_is_a_sensor_tool_not_a_motion_tool() -> None:
    assert "confirm_grasp" in ALWAYS_ALLOWED_TOOLS
    assert "confirm_grasp" not in MOTION_TOOLS


@pytest.mark.parametrize("band", BANDS)
@pytest.mark.parametrize("verdict", VERDICTS)
def test_every_fuse_cell_updates_the_reply_and_the_carry_record(
    server: Any, robot: FakeRobot, band: Band, verdict: ImageVerdict
) -> None:
    grasp_id = held_grasp(server, robot, band)
    fused = fuse(band, verdict)
    before = count(band, verdict, fused)
    reply = confirm(server, grasp_id, verdict)
    assert reply["grasp_id"] == grasp_id and reply["band"] == band
    assert reply["image"] == verdict and reply["fused"] == fused
    assert set(reply["evidence"]) >= {"grasp_id", "band", "jaw_gap_rad", "in_empty_band"}
    assert reply["next"]
    record = robot.arm.carry.get()
    assert record.image == verdict and record.fused == fused
    assert any("last grasp" in r for r in reply["carrying"]["reasons"]) is (fused != "missed")
    if fused == "grasped":
        assert record.grasp_kind == "grasped"
        assert robot.arm._unverified_since is None
        assert "complete_phase" in reply["next"]
    elif fused == "unverified":
        assert record.grasp_kind == "unverified"
    else:
        assert record.grasp_id == grasp_id and not record.grasp_held
        assert "release_object" in reply["next"]
    assert count(band, verdict, fused) == before + 1


def test_an_unknown_grasp_id_is_refused_and_changes_nothing(server: Any, robot: FakeRobot) -> None:
    held_grasp(server, robot, "object")
    before = robot.arm.carry.get()
    with pytest.raises(ToolError, match="unknown or superseded grasp_id"):
        confirm(server, "deadbeef0000", "yes")
    assert robot.arm.carry.get() == before


def test_a_later_close_supersedes_the_earlier_grasp_id(server: Any, robot: FakeRobot) -> None:
    old = held_grasp(server, robot, "object")
    call(server, "set_gripper", {"open_fraction": 1.0})
    with pytest.raises(ToolError, match="unknown or superseded"):
        confirm(server, old, "yes")
    assert robot.arm.carry.get().grasp_held is False


def test_a_later_grasp_supersedes_the_earlier_grasp_id(server: Any, robot: FakeRobot) -> None:
    old = held_grasp(server, robot)
    call(server, "release_object", {})
    new = held_grasp(server, robot)
    assert new != old
    with pytest.raises(ToolError, match="unknown or superseded"):
        confirm(server, old, "yes")
    assert robot.arm.carry.get().image == "pending"  # untouched


def test_reconfirming_overwrites_the_earlier_verdict(server: Any, robot: FakeRobot) -> None:
    grasp_id = held_grasp(server, robot, "object")
    assert confirm(server, grasp_id, "no")["fused"] == "unverified"
    second = confirm(server, grasp_id, "yes")
    assert second["fused"] == "grasped" and second["image"] == "yes"
    record = robot.arm.carry.get()
    assert record.fused == "grasped" and record.image == "yes"
    assert robot.arm.last_grasp_check is not None and robot.arm.last_grasp_check.fused == "grasped"


def test_evidence_text_is_bounded() -> None:
    async def run() -> Any:
        return await build_mcp_server(FakeRobot(Path("/nonexistent")), McpServerConfig(), TOKEN).list_tools()

    tool = next(t for t in anyio.run(run) if t.name == "confirm_grasp")
    props = tool.input_schema["properties"]
    assert props["evidence"]["minLength"] == 1 and props["evidence"]["maxLength"] == 500
    assert props["object_between_fingers"]["enum"] == ["yes", "no", "unsure"]
    assert set(tool.input_schema["required"]) == {"grasp_id", "object_between_fingers", "evidence"}


@pytest.mark.parametrize("evidence", ["", "x" * 501])
def test_evidence_outside_1_to_500_chars_is_rejected(server: Any, robot: FakeRobot, evidence: str) -> None:
    grasp_id = held_grasp(server, robot, "object")
    with pytest.raises(ToolError):
        confirm(server, grasp_id, "yes", evidence)
    assert robot.arm.carry.get().image == "pending"


def test_confirming_grasped_after_the_relax_timer_fired_says_so(server: Any, robot: FakeRobot) -> None:
    grasp_id = held_grasp(server, robot, "object")
    relax_s = robot.arm.cfg.grip_profiles.unverified_hold_relax_s
    assert relax_s is not None
    started = robot.arm._unverified_since
    assert started is not None
    robot.arm_backend.t = started + relax_s + 1.0
    robot.arm.keepalive_tick()
    assert robot.arm._unverified_since is None  # the relax fired
    reply = confirm(server, grasp_id, "yes")
    assert reply["fused"] == "grasped"
    assert "default torque" in reply["next"] and "re-grip" in reply["next"]


def test_confirm_grasp_needs_no_lease_and_moves_nothing(server: Any, robot: FakeRobot) -> None:
    grasp_id = held_grasp(server, robot, "object")
    commands = len(robot.arm_backend.commands)
    call(server, "release_control", {"subsystem": "arm", "keep_position": True})
    reply = confirm(server, grasp_id, "unsure")
    assert reply["fused"] == "unverified"
    assert len(robot.arm_backend.commands) == commands


def test_the_reply_is_json_serialisable_and_describes_the_fuse_table(server: Any) -> None:
    async def run() -> Any:
        return await server.list_tools()

    text = next(t for t in anyio.run(run) if t.name == "confirm_grasp").description or ""
    for needle in ("object", "empty", "ambiguous", "grasped", "unverified", "missed", "release_object"):
        assert needle in text, needle
    json.dumps(text)
