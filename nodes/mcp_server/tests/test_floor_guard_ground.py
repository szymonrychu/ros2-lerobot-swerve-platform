"""floor_guard with the measured ground estimate (use_ground_estimate): terrain from /ground/state, freshness
refusal, explicit surfaces winning, MAX_REGIONS trimming, default-off behaviour and the tool parameter."""

from pathlib import Path
from typing import Any

import numpy as np
import pytest
from mcp.server.mcpserver.exceptions import ToolError

from mcp_server.config import McpServerConfig
from mcp_server.floor_guard import FloorOverride, GroundEstimateError
from mcp_server.grasp_tools import floor_override
from mcp_server.models import GroundState
from mcp_server.surfaces import MAX_REGIONS, HalfPlaneEdge, SurfaceRegion

from .test_floor_guard import NOW, guard
from .test_ground_summary import example, region, state_with
from .test_tools import TOKEN, FakeRobot, build_mcp_server, call, take, tool_descriptions

USE = FloorOverride(use_ground_estimate=True)
DROP = region(
    "drop_0", 0.3, kind="drop", height_m=-0.15, edge={"point": [0.3, 0.0], "direction": [0.0, 1.0], "side": "right"}
)
EXPLICIT = SurfaceRegion(
    name="table",
    height_m=0.2,
    frame="base_link",
    edge=HalfPlaneEdge(point=(0.3, 0.0), direction=(0.0, 1.0), side="right"),
)


def terrain_height(model: object, x: float) -> float:
    terrain = model.terrain  # type: ignore[attr-defined]
    assert terrain is not None
    return terrain.lookup(np.array([x, 0.0]))[0]


def test_drop_region_ahead_sets_the_height_beyond_the_edge() -> None:
    model = guard().surface(USE, None, NOW, (state_with([DROP]), 0.2))
    assert terrain_height(model, 0.5) == pytest.approx(-0.15)
    assert terrain_height(model, 0.1) == pytest.approx(0.0)


def test_explicit_surfaces_override_an_overlapping_ground_region() -> None:
    override = FloorOverride(use_ground_estimate=True, surfaces=[EXPLICIT])
    model = guard().surface(override, None, NOW, (state_with([DROP]), 0.2))
    assert terrain_height(model, 0.5) == pytest.approx(0.2)


def test_region_count_is_trimmed_keeping_explicit_ones() -> None:
    far = [region(f"s{i}", 1.0 + i, height_m=-0.05 * (i + 1)) for i in range(MAX_REGIONS)]
    explicit = [EXPLICIT.model_copy(update={"name": f"e{i}"}) for i in range(3)]
    override = FloorOverride(use_ground_estimate=True, surfaces=explicit)
    model = guard().surface(override, None, NOW, (state_with(far), 0.2))
    names = [r.name for r in model.terrain.regions]  # type: ignore[union-attr]
    assert len(names) == MAX_REGIONS
    assert names[-3:] == ["e0", "e1", "e2"]


@pytest.mark.parametrize(
    ("ground", "cause"),
    [
        (None, "no fresh"),
        ((state_with([DROP]), 1.5), "older than"),
        ((GroundState.model_validate(example() | {"plane": None, "reason": "no points"}), 0.1), "no ground plane"),
    ],
)
def test_stale_absent_or_planeless_estimate_is_refused(ground: tuple[GroundState, float] | None, cause: str) -> None:
    with pytest.raises(GroundEstimateError) as err:
        guard().surface(USE, None, NOW, ground)
    assert "/ground/state" in str(err.value) and cause in str(err.value)


def test_max_age_is_configurable() -> None:
    model = guard(ground_max_age_s=3.0).surface(USE, None, NOW, (state_with([DROP]), 2.5))
    assert terrain_height(model, 0.5) == pytest.approx(-0.15)


def test_default_false_ignores_the_ground_state() -> None:
    g = guard()
    plain = g.surface(FloorOverride(surfaces=[EXPLICIT]), None, NOW)
    with_state = g.surface(FloorOverride(surfaces=[EXPLICIT]), None, NOW, (state_with([DROP]), 0.2))
    assert [r.name for r in with_state.terrain.regions] == ["table"]  # type: ignore[union-attr]
    assert with_state.surface_z_m == plain.surface_z_m and with_state.tilt == plain.tilt
    assert g.surface(None, None, NOW, None).terrain is None
    assert g.surface(None, None, NOW, (state_with([DROP]), 0.2)).terrain is None


def test_tool_parameter_is_passed_through() -> None:
    assert floor_override(None, None, None, True) == USE
    assert floor_override(None, None, None) is None
    assert floor_override(None, None, None, False) is None


def test_tools_document_and_refuse_without_a_fresh_estimate(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    server: Any = build_mcp_server(robot, McpServerConfig(), TOKEN)
    docs = tool_descriptions(server)
    assert "use_ground_estimate" in docs["move_arm_joints"]
    take(server, "arm")
    with pytest.raises(ToolError, match="/ground/state"):
        call(server, "move_arm_joints", {"targets": {"elbow_flex": 0.1}, "use_ground_estimate": True})
    robot.arm.ground_source = lambda: (state_with([DROP]), 0.2)
    result = call(server, "move_arm_joints", {"targets": {"elbow_flex": 0.1}, "use_ground_estimate": True})
    assert result.structured_content["status"] == "converged"
