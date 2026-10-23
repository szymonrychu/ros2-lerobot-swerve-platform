"""Calibration board park (D63): mcp_server's board geometry equals the calibration node's; the route token is private.

mcp_server owns the board outline, margins and floor height of POST /admin/park_calibration_board (the
`calibration_board:` block of its client.yml config). head_rgbd_calibration checks the same board with its own copy.
This pins the two together, and pins that the route's token is a separate file readable by the
head_rgbd_calibration group only, never by claude_agent.
"""

import sys
from pathlib import Path
from typing import Any

import numpy as np
import pytest
import yaml

pytest.importorskip("pydantic")

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
MCP_SETUP = REPO_ROOT / "ansible" / "playbooks" / "tasks" / "mcp_server_setup.yml"
CALIB_SETUP = REPO_ROOT / "ansible" / "playbooks" / "tasks" / "head_rgbd_calibration_setup.yml"
BOARD_GROUP = "board-park"
BOARD_TOKEN_FILE = "/etc/ros2/mcp_server/board_park_token"
GRID = 5

sys.path[:0] = [
    str(REPO_ROOT / "nodes" / "mcp_server"),
    str(REPO_ROOT / "nodes" / "head_rgbd_calibration"),
    str(REPO_ROOT / "shared"),
]
try:
    from head_rgbd_calibration.config import NodeConfig
    from mcp_server.calibration_board import BoardPark
    from mcp_server.config import CalibrationBoardSettings, McpServerConfig
    from ros2_common.head_rgbd_calibration.arm_pose_check import board_points_local
finally:
    del sys.path[:3]


def client_node(name: str) -> dict[str, Any]:
    nodes = yaml.safe_load(CLIENT_VARS.read_text())["ros2_nodes"]
    return yaml.safe_load(next(n for n in nodes if n["name"] == name)["config"])


def test_mcp_server_board_equals_the_calibration_fit_board_and_mount() -> None:
    mcp = client_node("mcp_server")["calibration_board"]
    calib = NodeConfig.model_validate(client_node("head_rgbd_calibration"))
    fit, seq = calib.fit, calib.arm_sequence
    assert (mcp["squares_x"], mcp["squares_y"], mcp["square_mm"]) == (
        fit.board.squares_x,
        fit.board.squares_y,
        fit.board.square_mm,
    )
    assert mcp["outline_margin_m"] == fit.board_mount.outline_margin_m
    assert mcp["lattice_points"] == GRID == seq.pose_check.board_grid
    assert tuple(mcp["mount_xyz_m"]) == fit.board_mount.xyz_init
    assert tuple(mcp["mount_rpy_rad"]) == fit.board_mount.rpy_init
    # The strictest clearances the node's park check uses: sequence margin + nominal-mount allowance + sag.
    assert mcp["margin_m"] == pytest.approx(seq.board_margin_m(True) + seq.park_board_sag_margin_m)
    assert mcp["min_height_m"] == pytest.approx(seq.pose_check.min_board_height_m + seq.park_board_sag_margin_m)


def test_deployed_block_and_code_defaults_agree() -> None:
    deployed = CalibrationBoardSettings.model_validate(client_node("mcp_server")["calibration_board"])
    assert deployed == CalibrationBoardSettings() == McpServerConfig().calibration_board


def test_mcp_server_points_equal_the_node_board_points_at_the_configured_mount() -> None:
    calib = NodeConfig()
    settings = CalibrationBoardSettings()
    expected = board_points_local(calib.fit, GRID)
    park = BoardPark.from_mount(settings, list(settings.mount_xyz_m), list(settings.mount_rpy_rad))
    assert np.allclose(np.sort(park.points, axis=0), np.sort(expected, axis=0), atol=1e-9)
    assert np.allclose(park.points, expected, atol=1e-9)


def test_board_park_token_is_a_separate_file_for_the_calibration_group_only() -> None:
    client = yaml.safe_load(CLIENT_VARS.read_text())
    assert client_node("mcp_server")["server"]["board_park_token_file"] == BOARD_TOKEN_FILE
    assert client_node("head_rgbd_calibration")["robot"]["board_park_token_file"] == BOARD_TOKEN_FILE
    defaults = client["ros2_node_type_defaults"]
    assert BOARD_GROUP in defaults["head_rgbd_calibration"]["supplementary_groups"]
    assert BOARD_GROUP not in defaults["claude_agent"]["supplementary_groups"]
    assert client_node("claude_agent")["mcp_token_file"] != BOARD_TOKEN_FILE
    tasks = yaml.safe_load(MCP_SETUP.read_text())
    file_tasks = [t["ansible.builtin.copy"] for t in tasks if "ansible.builtin.copy" in t]
    token = next(t for t in file_tasks if t["dest"] == BOARD_TOKEN_FILE)
    assert token["group"] == BOARD_GROUP and token["mode"] == "0640" and token["force"] is False
    modes = [
        t["ansible.builtin.file"]
        for t in tasks
        if "ansible.builtin.file" in t and t["ansible.builtin.file"].get("path") == BOARD_TOKEN_FILE
    ]
    assert modes and all(m["group"] == BOARD_GROUP and m["mode"] == "0640" for m in modes)
    for path in (MCP_SETUP, CALIB_SETUP):  # the service starts only if the group exists, whichever setup runs first
        groups = [t["ansible.builtin.group"] for t in yaml.safe_load(path.read_text()) if "ansible.builtin.group" in t]
        assert any(g["name"] == BOARD_GROUP for g in groups)
