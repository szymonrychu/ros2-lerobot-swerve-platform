"""Jaw (TCP) calibration: the sim jaws close where the real robot's measured tool_offset_m says they do."""

import hashlib
import importlib.util
from types import ModuleType

import mujoco
import numpy as np
import pytest

from grasp_sim.config import BoxObjectConfig, SceneConfig
from grasp_sim.replay import Rig
from grasp_sim.scene import ARM_XML, build_model
from grasp_sim.tcp import (
    REPO_ROOT,
    STOCK_CLOSING_POINT_GFL,
    client_tool_offset,
    closing_point_gfl,
    gfl_to_gripper,
    gripper_to_gfl,
    moving_jaw_inner_profile,
)

MEASURED_TOOL_OFFSET = (0.0010, -0.0056, -0.0014)
CALIBRATION_TOLERANCE_M = 0.002


def test_gripper_frame_link_round_trip() -> None:
    point = np.array([0.0010, -0.0056, -0.0014])
    assert np.allclose(gripper_to_gfl(gfl_to_gripper(point)), point)
    # gripper_frame_link is gripper_link turned pi about y, 9.8 cm down the jaws.
    assert np.allclose(gfl_to_gripper(np.zeros(3)), [-0.0079, -0.000218121, -0.0981274])
    assert np.allclose(gfl_to_gripper(np.array([0.01, 0.0, 0.0])) - gfl_to_gripper(np.zeros(3)), [-0.01, 0.0, 0.0])


def test_stock_menagerie_jaws_close_near_the_gripper_frame_origin() -> None:
    point = closing_point_gfl(build_model(SceneConfig(object=None)))
    assert np.allclose(point, STOCK_CLOSING_POINT_GFL, atol=1e-4)
    assert np.allclose(point, (-0.0015, 0.0002, 0.003), atol=1e-3)


def test_client_yml_tool_offset_is_read_from_ansible() -> None:
    assert client_tool_offset() == pytest.approx(MEASURED_TOOL_OFFSET)


def test_calibrated_jaws_close_at_the_measured_tool_offset() -> None:
    model = build_model(SceneConfig(object=None, tool_offset_m=MEASURED_TOOL_OFFSET))
    point = closing_point_gfl(model)
    assert float(np.linalg.norm(point - np.array(MEASURED_TOOL_OFFSET))) < CALIBRATION_TOLERANCE_M


def test_calibration_keeps_the_vendored_model_file_unmodified() -> None:
    before = hashlib.sha256(ARM_XML.read_bytes()).hexdigest()
    build_model(SceneConfig(object=None, tool_offset_m=MEASURED_TOOL_OFFSET))
    assert hashlib.sha256(ARM_XML.read_bytes()).hexdigest() == before


def test_calibration_moves_only_the_fingers_not_the_wrist_housing() -> None:
    stock = build_model(SceneConfig(object=None))
    calibrated = build_model(SceneConfig(object=None, tool_offset_m=MEASURED_TOOL_OFFSET))
    assert np.allclose(stock.geom("fixed_jaw_box1").pos, calibrated.geom("fixed_jaw_box1").pos)
    shift = calibrated.geom("fixed_jaw_sph_tip1").pos - stock.geom("fixed_jaw_sph_tip1").pos
    assert 0.002 < float(np.linalg.norm(shift)) < 0.02  # the physical fixed-jaw tip is close to the stock jaws
    moving = calibrated.body("moving_jaw_so101_v1").pos - stock.body("moving_jaw_so101_v1").pos
    assert np.allclose(moving, shift)


def test_rig_grip_point_follows_the_calibrated_jaws() -> None:
    stock = Rig(SceneConfig(object=BoxObjectConfig()))
    calibrated = Rig(SceneConfig(object=BoxObjectConfig(), tool_offset_m=MEASURED_TOOL_OFFSET))
    for rig in (stock, calibrated):
        mujoco.mj_forward(rig.model, rig.data)
    delta = calibrated.grip_point() - stock.grip_point()
    expected = gfl_to_gripper(np.array(MEASURED_TOOL_OFFSET)) - gfl_to_gripper(np.array(STOCK_CLOSING_POINT_GFL))
    rot = stock.data.body("gripper").xmat.reshape(3, 3)
    assert np.allclose(delta, rot @ expected, atol=1e-6)


MCP_JAW_PROFILE = REPO_ROOT / "nodes" / "mcp_server" / "mcp_server" / "jaw_profile.py"


def load_mcp_jaw_profile() -> ModuleType:
    """mcp_server's jaw_profile module (numpy-free constants), loaded by path from the other uv environment."""
    spec = importlib.util.spec_from_file_location("mcp_jaw_profile", MCP_JAW_PROFILE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_moving_jaw_inner_profile_flares_away_from_the_fixed_jaw_with_depth() -> None:
    profile = moving_jaw_inner_profile(build_model(SceneConfig(object=None, tool_offset_m=None)))
    depths = [d for _, d in profile]
    assert depths == sorted(depths) and depths[0] < 0.0 < depths[-1]
    near_tip = [o for o, d in profile if 0.0 <= d <= 0.006]
    deep = [o for o, d in profile if 0.04 <= d <= 0.05]
    assert max(near_tip) < 0.003  # the tip meets the fixed jaw
    assert min(deep) > 0.01  # 4 cm in, the closed jaws stand more than 1 cm apart


def test_mcp_server_jaw_profile_is_the_sim_jaw_mesh_silhouette() -> None:
    """mcp_server plans the moving jaw clearance with MOVING_JAW_INNER_PROFILE: it must be this model's silhouette."""
    sim = moving_jaw_inner_profile(build_model(SceneConfig(object=None, tool_offset_m=None)))
    mcp = load_mcp_jaw_profile().MOVING_JAW_INNER_PROFILE
    assert len(mcp) == len(sim)
    for (o_mcp, d_mcp), (o_sim, d_sim) in zip(mcp, sim, strict=True):
        assert d_mcp == pytest.approx(d_sim, abs=1e-4)
        assert o_mcp == pytest.approx(o_sim, abs=2e-4)
