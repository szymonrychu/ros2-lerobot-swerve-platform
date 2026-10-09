"""Jaw (TCP) calibration: the sim jaws close where the real robot's measured tool_offset_m says they do."""

import hashlib

import mujoco
import numpy as np
import pytest

from grasp_sim.config import BoxObjectConfig, SceneConfig
from grasp_sim.replay import Rig
from grasp_sim.scene import ARM_XML, build_model
from grasp_sim.tcp import (
    STOCK_CLOSING_POINT_GFL,
    client_tool_offset,
    closing_point_gfl,
    gfl_to_gripper,
    gripper_to_gfl,
)

MEASURED_TOOL_OFFSET = (0.0104, -0.0282, -0.0017)
CALIBRATION_TOLERANCE_M = 0.002


def test_gripper_frame_link_round_trip() -> None:
    point = np.array([0.0104, -0.0282, -0.0017])
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
    assert float(np.linalg.norm(shift)) > 0.02
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
