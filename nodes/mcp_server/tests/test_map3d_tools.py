"""Tests for map3d_tools: get_3d_map_status, query_3d_map_at and get_3d_free_space (FakeRobot with a synthetic cloud)."""

import math
import struct
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest
from mcp.server.mcpserver.exceptions import ToolError

from mcp_server import map3d_tools
from mcp_server.config import Map3dSettings, McpServerConfig
from mcp_server.map3d import VoxelMap, cloud_from_message
from mcp_server.models import Map3dCloud
from mcp_server.tools import ALWAYS_ALLOWED_TOOLS, build_mcp_server

from .test_map3d import (
    FRAME,
    OCCUPIED_VALUE,
    PLANE,
    TABLE_TOP,
    VOXEL,
    cell_of,
    centre_of,
    floor,
    free_grid,
    grid_of,
    lattice,
    post,
    room_grid,
    room_wall_cloud,
    table,
)
from .test_tools import TOKEN, FakeRobot, call

NAMES = ("get_3d_map_status", "query_3d_map_at", "get_3d_free_space")
FRESH_S = 1.0


def cloud_of(
    points: np.ndarray, age_s: float = FRESH_S, rgb: np.ndarray | None = None, frame: str = FRAME
) -> Map3dCloud:
    """A Map3dCloud stamped age_s seconds ago."""
    return Map3dCloud(frame_id=frame, stamp_s=time.time() - age_s, xyz=points, rgb=rgb)


@pytest.fixture
def robot(tmp_path: Path) -> FakeRobot:
    return FakeRobot(tmp_path)


@pytest.fixture
def server(robot: FakeRobot) -> Any:
    return build_mcp_server(robot, McpServerConfig(), TOKEN)


def out(server: Any, name: str, args: dict[str, Any] | None = None) -> dict[str, Any]:
    return call(server, name, args or {}).structured_content


# --- config / registration ----------------------------------------------------------------------------------------


def test_config_defaults() -> None:
    cfg = McpServerConfig()
    assert cfg.topics.map3d_cloud == "/map3d/cloud"
    assert cfg.topics.map3d_status == "/map3d/status"
    m = cfg.map3d
    assert isinstance(m, Map3dSettings)
    assert (m.frame_id, m.voxel_m, m.lidar_plane_height_m, m.robot_height_m, m.max_age_s) == (
        "rtabmap_map",
        0.05,
        0.20,
        0.55,
        120.0,
    )


def test_tools_are_read_only_sensor_tools() -> None:
    assert set(map3d_tools.TOOL_NAMES) == set(NAMES) == set(map3d_tools.SENSOR_TOOL_NAMES)
    assert set(NAMES) <= ALWAYS_ALLOWED_TOOLS


# --- no data ------------------------------------------------------------------------------------------------------


@pytest.mark.parametrize("name", ["query_3d_map_at", "get_3d_free_space"])
def test_query_tools_without_a_cloud_raise(server: Any, name: str) -> None:
    with pytest.raises(ToolError, match="no 3D map received yet"):
        call(server, name, {"x": 0.0, "y": 0.0})


def test_status_without_data_is_null_with_a_note(server: Any) -> None:
    res = out(server, "get_3d_map_status")
    assert res["status"] is None and res["voxel_count"] is None and res["bounds"] is None
    assert res["age_s"] is None and res["alignment_agreement"] is None
    assert any("no 3D map received yet" in n for n in res["notes"])
    assert any("/map3d/status" in n for n in res["notes"])


# --- get_3d_map_status --------------------------------------------------------------------------------------------


def test_status_reports_cloud_bounds_age_and_alignment(robot: FakeRobot, server: Any) -> None:
    robot.map3d_cloud_msg = cloud_of(room_wall_cloud(), age_s=2.0)
    robot.map3d_status_msg = {"state": "mapping", "nodes": 12}
    robot.map = grid_of(room_grid()).grid
    res = out(server, "get_3d_map_status")
    assert res["status"] == {"state": "mapping", "nodes": 12}
    assert res["voxel_count"] > 0 and res["voxel_m"] == VOXEL and res["frame_id"] == FRAME
    assert res["bounds"]["min"][2] <= 0.1 and res["bounds"]["max"][2] >= 0.95
    assert 1.5 <= res["age_s"] <= 3.5
    assert res["alignment_agreement"] > 0.9
    assert res["stale"] is False
    assert res["lidar"]["occupied_cells"] > 0


def test_status_alignment_null_without_map_is_noted(robot: FakeRobot, server: Any) -> None:
    robot.map3d_cloud_msg = cloud_of(room_wall_cloud())
    res = out(server, "get_3d_map_status")
    assert res["alignment_agreement"] is None and res["lidar"] is None
    assert any("/map" in n for n in res["notes"])


def test_status_stale_note_after_max_age(robot: FakeRobot, server: Any) -> None:
    robot.map3d_cloud_msg = cloud_of(floor(-1, 1, -1, 1), age_s=300.0)
    res = out(server, "get_3d_map_status")
    assert res["stale"] is True
    assert any("stale" in n for n in res["notes"])


# --- query_3d_map_at ----------------------------------------------------------------------------------------------


def test_query_reports_table_height_rounded_with_source_and_lidar_note(robot: FakeRobot, server: Any) -> None:
    robot.map3d_cloud_msg = cloud_of(np.vstack([floor(-1.5, 1.5, -1.5, 1.5), table(0.0, 0.0)]))
    res = out(server, "query_3d_map_at", {"x": 0.0, "y": 0.0, "radius_m": 0.5})
    assert res["source"] == "depth_3d"
    assert abs(res["height_m"] - TABLE_TOP) <= VOXEL
    assert res["height_m"] == round(res["height_m"], 3)
    assert res["radius_m"] == 0.5
    assert len(res["highest_point"]) == 3
    assert any("lidar" in n and "unknown" in n for n in res["notes"])
    assert not any("stale" in n for n in res["notes"])


def test_query_default_radius_and_lidar_only_height_is_null(robot: FakeRobot, server: Any) -> None:
    data = free_grid()
    wall_col = cell_of(2.0, 0.0)[1]
    data[:, wall_col] = OCCUPIED_VALUE
    robot.map = grid_of(data).grid
    robot.map3d_cloud_msg = cloud_of(floor(-1.0, 1.0, -1.0, 1.0))
    res = out(server, "query_3d_map_at", {"x": 2.0, "y": 0.0})
    assert res["radius_m"] == 0.3
    assert res["source"] == "lidar_2d" and res["height_m"] is None
    assert res["lidar_only_cells"] > 0 and res["lidar_min_height_m"] == PLANE


def test_query_outside_everything_is_unknown(robot: FakeRobot, server: Any) -> None:
    robot.map3d_cloud_msg = cloud_of(floor(-1.0, 1.0, -1.0, 1.0))
    res = out(server, "query_3d_map_at", {"x": 4.0, "y": 4.0})
    assert res["source"] == "unknown" and res["height_m"] is None


def test_query_stale_cloud_carries_a_note(robot: FakeRobot, server: Any) -> None:
    robot.map3d_cloud_msg = cloud_of(floor(-1.0, 1.0, -1.0, 1.0), age_s=500.0)
    res = out(server, "query_3d_map_at", {"x": 0.0, "y": 0.0})
    assert any("stale" in n for n in res["notes"])


@pytest.mark.parametrize("radius", [0.04, 2.01, 0.0, -1.0])
def test_query_radius_bounds(robot: FakeRobot, server: Any, radius: float) -> None:
    robot.map3d_cloud_msg = cloud_of(floor(-1.0, 1.0, -1.0, 1.0))
    with pytest.raises(ToolError):
        call(server, "query_3d_map_at", {"x": 0.0, "y": 0.0, "radius_m": radius})


@pytest.mark.parametrize("radius", [0.05, 2.0])
def test_query_radius_bounds_inclusive(robot: FakeRobot, server: Any, radius: float) -> None:
    robot.map3d_cloud_msg = cloud_of(floor(-1.0, 1.0, -1.0, 1.0))
    assert out(server, "query_3d_map_at", {"x": 0.0, "y": 0.0, "radius_m": radius})["radius_m"] == radius


def test_cloud_in_another_frame_is_refused(robot: FakeRobot, server: Any) -> None:
    robot.map3d_cloud_msg = cloud_of(floor(-1.0, 1.0, -1.0, 1.0), frame="odom")
    with pytest.raises(ToolError, match="frame"):
        call(server, "query_3d_map_at", {"x": 0.0, "y": 0.0})


# --- get_3d_free_space --------------------------------------------------------------------------------------------


def sector(res: dict[str, Any], bearing_deg: float) -> dict[str, Any]:
    return min(res["sectors"], key=lambda s: abs(math.remainder(s["bearing_deg"] - bearing_deg, 360.0)))


def test_free_space_per_sector_with_bearings_in_degrees(robot: FakeRobot, server: Any) -> None:
    robot.map3d_cloud_msg = cloud_of(np.vstack([floor(-2.0, 2.0, -2.0, 2.0), table(1.4, 0.0)]))
    res = out(server, "get_3d_free_space", {"x": 0.0, "y": 0.0, "radius_m": 2.0, "z_max_m": 0.9})
    assert len(res["sectors"]) == 8
    assert sorted(round(s["bearing_deg"]) for s in res["sectors"]) == [0, 45, 90, 135, 180, 225, 270, 315]
    east, west = sector(res, 0.0), sector(res, 180.0)
    assert east["observed"] >= 0.9 and abs(east["free_m"] - 1.0) <= 2 * VOXEL
    assert west["free_m"] == pytest.approx(2.0)
    assert res["z_min_m"] == 0.02 and res["z_max_m"] == 0.9 and res["radius_m"] == 2.0


def test_free_space_z_max_defaults_to_robot_height(robot: FakeRobot, server: Any) -> None:
    low_box = np.vstack([post(1.0, y, 0.06, 0.3, step=0.025) for y in np.arange(-0.4, 0.41, 0.025)])
    high_board = lattice(-1.4, -1.0, -0.3, 0.3, 0.7)  # above robot_height_m 0.55
    robot.map3d_cloud_msg = cloud_of(np.vstack([floor(-2.0, 2.0, -2.0, 2.0), low_box, high_board]))
    res = out(server, "get_3d_free_space", {"x": 0.0, "y": 0.0})
    assert res["radius_m"] == 1.0 and res["z_max_m"] == 0.55
    assert abs(sector(res, 0.0)["free_m"] - 0.975) <= 2 * VOXEL
    assert sector(res, 180.0)["free_m"] == pytest.approx(1.0)  # the board is above the robot
    raised = out(server, "get_3d_free_space", {"x": 0.0, "y": 0.0, "z_max_m": 0.9})
    assert abs(sector(raised, 180.0)["free_m"] - 1.0) <= 2 * VOXEL or sector(raised, 180.0)["free_m"] < 1.0


def test_free_space_unobserved_sector_is_null(robot: FakeRobot, server: Any) -> None:
    robot.map3d_cloud_msg = cloud_of(floor(0.05, 2.0, -2.0, 2.0))
    res = out(server, "get_3d_free_space", {"x": 0.0, "y": 0.0, "radius_m": 1.5})
    assert sector(res, 0.0)["free_m"] == pytest.approx(1.5)
    assert sector(res, 180.0)["free_m"] is None and sector(res, 180.0)["observed"] < 0.2


def test_free_space_lidar_wall_blocks(robot: FakeRobot, server: Any) -> None:
    data = free_grid()
    wall_col = cell_of(1.0, 0.0)[1]
    data[:, wall_col] = OCCUPIED_VALUE
    robot.map = grid_of(data).grid
    robot.map3d_cloud_msg = cloud_of(np.zeros((0, 3)))
    res = out(server, "get_3d_free_space", {"x": 0.0, "y": 0.0, "radius_m": 2.0})
    east = sector(res, 0.0)
    assert abs(east["free_m"] - 1.0) <= 0.06 and east["obstacle_source"] == "lidar_2d"
    assert centre_of(0, wall_col)[0] > 0.9


@pytest.mark.parametrize(
    "args",
    [
        {"radius_m": 0.19},
        {"radius_m": 5.01},
        {"z_min_m": 0.5, "z_max_m": 0.5},
        {"z_min_m": 0.6},  # above the default z_max_m 0.55
    ],
)
def test_free_space_argument_bounds(robot: FakeRobot, server: Any, args: dict[str, Any]) -> None:
    robot.map3d_cloud_msg = cloud_of(floor(-1.0, 1.0, -1.0, 1.0))
    with pytest.raises(ToolError):
        call(server, "get_3d_free_space", {"x": 0.0, "y": 0.0, **args})


def test_free_space_stale_note(robot: FakeRobot, server: Any) -> None:
    robot.map3d_cloud_msg = cloud_of(floor(-1.0, 1.0, -1.0, 1.0), age_s=500.0)
    res = out(server, "get_3d_free_space", {"x": 0.0, "y": 0.0})
    assert any("stale" in n for n in res["notes"])


# --- caching ------------------------------------------------------------------------------------------------------


def test_voxel_map_rebuilt_once_per_stamp(robot: FakeRobot, server: Any, monkeypatch: pytest.MonkeyPatch) -> None:
    builds: list[float] = []
    original = VoxelMap.from_points.__func__  # type: ignore[attr-defined]

    def counting(cls: type, xyz: np.ndarray, voxel_m: float, stamp_s: float, *a: Any, **kw: Any) -> VoxelMap:
        builds.append(stamp_s)
        return original(cls, xyz, voxel_m, stamp_s, *a, **kw)

    monkeypatch.setattr(VoxelMap, "from_points", classmethod(counting))
    robot.map3d_cloud_msg = cloud_of(floor(-1.0, 1.0, -1.0, 1.0))
    for _ in range(3):
        out(server, "query_3d_map_at", {"x": 0.0, "y": 0.0})
        out(server, "get_3d_free_space", {"x": 0.0, "y": 0.0})
        out(server, "get_3d_map_status")
    assert len(builds) == 1
    stamp = robot.map3d_cloud_msg.stamp_s
    robot.map3d_cloud_msg = Map3dCloud(FRAME, stamp + 10.0, floor(-1.0, 1.0, -1.0, 1.0), None)
    out(server, "query_3d_map_at", {"x": 0.0, "y": 0.0})
    out(server, "query_3d_map_at", {"x": 0.0, "y": 0.0})
    assert len(builds) == 2


# --- cloud decoding -----------------------------------------------------------------------------------------------


def packed_rgb(r: int, g: int, b: int) -> float:
    return struct.unpack("<f", struct.pack("<I", (r << 16) | (g << 8) | b))[0]


def cloud_message(rows: list[tuple[float, float, float, float]], with_rgb: bool = True) -> SimpleNamespace:
    names = ["x", "y", "z", "rgb"] if with_rgb else ["x", "y", "z"]
    step = 4 * len(names)
    data = b"".join(struct.pack("<" + "f" * len(names), *row[: len(names)]) for row in rows)
    fields = [SimpleNamespace(name=n, offset=4 * i) for i, n in enumerate(names)]
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=FRAME, stamp=SimpleNamespace(sec=100, nanosec=500_000_000)),
        fields=fields,
        point_step=step,
        width=len(rows),
        height=1,
        is_bigendian=False,
        data=data,
    )


def test_cloud_from_message_reads_xyz_rgb_and_drops_non_finite() -> None:
    rows = [
        (1.0, 2.0, 0.5, packed_rgb(10, 20, 30)),
        (float("nan"), 0.0, 0.0, packed_rgb(1, 2, 3)),
        (3.0, 4.0, 1.0, packed_rgb(200, 100, 0)),
    ]
    cloud = cloud_from_message(cloud_message(rows))
    assert cloud.frame_id == FRAME and cloud.stamp_s == pytest.approx(100.5)
    assert cloud.xyz.tolist() == [[1.0, 2.0, 0.5], [3.0, 4.0, 1.0]]
    assert cloud.rgb is not None and cloud.rgb.tolist() == [[10, 20, 30], [200, 100, 0]]


def test_cloud_from_message_without_colour_field() -> None:
    cloud = cloud_from_message(cloud_message([(1.0, 2.0, 0.5, 0.0)], with_rgb=False))
    assert cloud.rgb is None and cloud.xyz.shape == (1, 3)
