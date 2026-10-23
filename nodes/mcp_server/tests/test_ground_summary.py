"""get_map_summary `ground` from /ground/state: the GroundState contract model, region conversion, summary mapping.

Covers: ground_regions_to_surfaces (drops kind/class/distance_m/points, base_link SurfaceRegion, 16 cap keeping the
nearest, invalid region skipped); the ground_estimator_cpp contract fixture parses; unknown/missing keys rejected; stale
and absent states give the note, plane null gives the reason note; summary fields mapped and rounded; RosRobot
subscription parsing (invalid logged once per distinct error, not stored; stale -> None); config defaults.
"""

import json
import logging
import time
from pathlib import Path
from types import ModuleType
from typing import Any
from unittest.mock import MagicMock

import pytest
import yaml
from pydantic import ValidationError

from mcp_server.config import McpServerConfig
from mcp_server.models import GroundState, ground_regions_to_surfaces, summarize_ground
from mcp_server.staleness import Stamped
from mcp_server.surfaces import MAX_REGIONS, SurfaceRegion

from .test_tools import TOKEN, FakeRobot, build_mcp_server, call, tool_descriptions

REPO = Path(__file__).resolve().parents[3]
FIXTURE = REPO / "nodes/ground_estimator_cpp/test/data/state_example.json"
NO_STATE_NOTE = "ground estimate unavailable: no fresh /ground/state"


def example() -> dict[str, Any]:
    return json.loads(FIXTURE.read_text())


def region(name: str, distance: float, **over: Any) -> dict[str, Any]:
    base = {
        "name": name,
        "kind": "step",
        "class": "obstacle",
        "height_m": 0.1,
        "frame": "base_link",
        "edge": {"point": [distance, 0.0], "direction": [0.0, 1.0], "side": "left"},
        "distance_m": distance,
        "points": 10,
    }
    return base | over


def state_with(regions: list[dict[str, Any]]) -> GroundState:
    return GroundState.model_validate(example() | {"regions": regions})


def test_fixture_parses() -> None:
    state = GroundState.model_validate(example())
    assert [r.name for r in state.regions] == ["step_0", "drop_0"]
    assert state.regions[0].cls == "obstacle" and state.traversability.max_step_m == 0.04
    assert state.warnings[0].kind == "ramp" and state.plane is not None


def test_unknown_and_missing_keys_rejected() -> None:
    with pytest.raises(ValidationError):
        GroundState.model_validate(example() | {"extra": 1})
    missing = example()
    del missing["slope_level"]
    with pytest.raises(ValidationError):
        GroundState.model_validate(missing)
    bad_region = example()
    bad_region["regions"][0]["surprise"] = 1
    with pytest.raises(ValidationError):
        GroundState.model_validate(bad_region)


def test_regions_convert_to_surface_regions() -> None:
    surfaces = ground_regions_to_surfaces(GroundState.model_validate(example()))
    assert [s.name for s in surfaces] == ["step_0", "drop_0"]
    first = surfaces[0]
    assert isinstance(first, SurfaceRegion) and first.frame == "base_link" and first.height_m == 0.0988
    assert first.edge is not None and first.edge.point == (0.8, 0.0) and first.edge.side == "left"
    dumped = first.model_dump()
    assert not {"kind", "class", "distance_m", "points"} & set(dumped)
    SurfaceRegion.model_validate(dumped)


def test_regions_capped_keeping_nearest() -> None:
    regions = [region(f"r{i}", 20.0 - i) for i in range(20)]  # r19 is nearest (1.0 m), r0 farthest
    surfaces = ground_regions_to_surfaces(state_with(regions))
    assert len(surfaces) == MAX_REGIONS
    assert {s.name for s in surfaces} == {f"r{i}" for i in range(4, 20)}
    assert [s.name for s in surfaces] == [f"r{i}" for i in range(4, 20)]  # state order kept


def test_invalid_region_skipped_and_logged(caplog: pytest.LogCaptureFixture) -> None:
    bad_dir = region("zero", 1.0, edge={"point": [1.0, 0.0], "direction": [0.0, 0.0], "side": "left"})
    bad_height = region("tall", 1.0, height_m=2.0)
    with caplog.at_level(logging.WARNING):
        surfaces = ground_regions_to_surfaces(state_with([bad_dir, region("ok", 2.0), bad_height]))
    assert [s.name for s in surfaces] == ["ok"] and surfaces[0].height_m == 0.1
    assert "zero" in caplog.text and "tall" in caplog.text


def test_summary_maps_and_rounds() -> None:
    raw = example()
    raw["plane"]["height_m"] = -0.04126
    raw["slope_deg"] = 3.04
    raw["colour"]["mismatch_ratio"] = 0.01234
    summary = summarize_ground(GroundState.model_validate(raw), 0.34567)
    assert summary.floor_height_m == -0.041 and summary.plane_source == "held"
    assert summary.slope_deg == 3.0 and summary.slope_level == "warning" and summary.plane_vs_base_deg == 3.0
    assert summary.colour_mismatch_ratio == 0.012 and summary.state_age_s == 0.346
    assert summary.obstacle_rule.max_step_m == 0.04 and summary.obstacle_rule.max_slope_deg == 8.0
    step = summary.regions[0]
    assert (step.kind, step.classification, step.height_m, step.distance_m) == ("step", "obstacle", 0.099, 0.8)
    assert step.edge.point == (0.8, 0.0)
    ramp = summary.warnings[0]
    assert ramp.kind == "ramp" and ramp.slope_deg == 5.6 and ramp.nearest_distance_m == 0.5


def test_defaults_and_client_yml() -> None:
    cfg = McpServerConfig()
    assert cfg.topics.ground_state == "/ground/state" and cfg.timeouts.ground_state_stale_s == 1.0
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"timeouts": {"ground_state_stale_s": 0}})
    text = (REPO / "ansible/group_vars/client.yml").read_text()
    assert "ground_state: /ground/state" in text
    ground_cfg = yaml.safe_load((REPO / "nodes/ground_estimator_cpp/config/ground_estimator.yaml").read_text())
    assert ground_cfg["output_namespace"] + "/state" == cfg.topics.ground_state


# --- RosRobot subscription ---------------------------------------------------------------------------------------


def ros_robot(ros_iface: ModuleType) -> Any:
    robot = ros_iface.RosRobot.__new__(ros_iface.RosRobot)
    robot.cfg = McpServerConfig()
    robot.node = MagicMock()
    robot.lock = __import__("threading").Lock()
    robot.latest = {}
    robot.ground_state_error = None
    robot.log = MagicMock()
    robot.monitor = MagicMock()
    return robot


def test_ros_robot_stores_valid_and_returns_age(ros_iface: ModuleType) -> None:
    robot = ros_robot(ros_iface)
    assert robot.ground_state() is None
    robot.on_ground_state(FIXTURE.read_text())
    state, age = robot.ground_state()
    assert state.frame == "base_link" and 0.0 <= age < 0.5


def test_ros_robot_invalid_logged_once_and_not_stored(ros_iface: ModuleType) -> None:
    robot = ros_robot(ros_iface)
    robot.on_ground_state("{not json")
    robot.on_ground_state("{not json")
    robot.on_ground_state(json.dumps({"bogus": 1}))
    assert robot.ground_state() is None
    assert robot.log.warning.call_count == 2


def test_ros_robot_stale_state_is_none(ros_iface: ModuleType) -> None:
    robot = ros_robot(ros_iface)
    robot.on_ground_state(FIXTURE.read_text())
    stamped = robot.latest["ground_state"]
    robot.latest["ground_state"] = Stamped(value=stamped.value, stamp=time.monotonic() - 5.0)
    assert robot.ground_state() is None
    state, age = robot.ground_state(max_age_s=None)
    assert age >= 5.0 and state is stamped.value  # floor_guard applies its own ground_max_age_s


# --- tool ---------------------------------------------------------------------------------------------------------


@pytest.fixture
def robot(tmp_path: Path) -> FakeRobot:
    return FakeRobot(tmp_path)


def summary_of(robot: FakeRobot) -> dict[str, Any]:
    return call(build_mcp_server(robot, McpServerConfig(), TOKEN), "get_map_summary", {}).structured_content


def test_tool_fills_ground(robot: FakeRobot) -> None:
    robot.ground = (GroundState.model_validate(example()), 0.25)
    summary = summary_of(robot)
    assert summary["ground"]["plane_source"] == "held" and summary["ground"]["state_age_s"] == 0.25
    assert NO_STATE_NOTE not in summary["notes"]


def test_tool_absent_state_gives_note(robot: FakeRobot) -> None:
    robot.ground = None
    summary = summary_of(robot)
    assert summary["ground"] is None and NO_STATE_NOTE in summary["notes"]


def test_tool_plane_null_gives_reason_note(robot: FakeRobot) -> None:
    raw = example() | {
        "plane": None, "reason": "too_few_points", "slope_deg": None, "slope_roll_deg": None,
        "slope_pitch_deg": None, "slope_level": None, "plane_vs_base_deg": None, "regions": [], "warnings": [],
    }  # fmt: skip
    robot.ground = (GroundState.model_validate(raw), 0.1)
    summary = summary_of(robot)
    assert summary["ground"] is None and "no floor plane: too_few_points" in summary["notes"]


def test_description_documents_ground(robot: FakeRobot) -> None:
    doc = tool_descriptions(build_mcp_server(robot, McpServerConfig(), TOKEN))["get_map_summary"]
    assert "ground" in doc and "obstacles_3d" in doc and "fixed" in doc


def test_no_return_warning_with_null_heights() -> None:
    """A no_return warning has null height and slope: the state parses and the summary keeps them null."""
    no_return = {
        "kind": "no_return",
        "polygon": [[0.5, -0.2], [0.9, -0.2], [0.9, 0.3], [0.5, 0.3]],
        "height_m": None,
        "slope_deg": None,
        "source": "no_return",
    }
    state = GroundState.model_validate(example() | {"warnings": [no_return]})
    summary = summarize_ground(state, 0.0)
    warning = summary.warnings[0]
    assert warning.kind == "no_return"
    assert warning.height_m is None and warning.slope_deg is None
    assert warning.nearest_distance_m == 0.5
