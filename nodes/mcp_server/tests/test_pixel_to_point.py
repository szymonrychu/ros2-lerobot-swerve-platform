"""pixel_to_point and camera_point_to_arm: D435 depth pixel -> camera, base_link, arm and map frames at the image stamp."""

import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from mcp.server.mcpserver.exceptions import ToolError
from ros2_common.head_rgbd_calibration.urdf_text import ChainResidual

from mcp_server.camera_scene import base_to_map
from mcp_server.camera_tools import TOOL_NAMES
from mcp_server.config import McpServerConfig
from mcp_server.head_kinematics import CAMERA_FRAME, DEPTH_FRAME
from mcp_server.tools import build_mcp_server

from .depth_helpers import (
    ARM_JOINT_SETS,
    CX,
    CY,
    FX,
    FY,
    HEAD_RATE_HZ,
    HEIGHT,
    INFO,
    STAMP,
    WIDTH,
    DepthRobot,
    d435_sigma_m,
    depth_config,
    depth_frame,
    feed_history,
    flat_depth,
    install_head,
    object_depth,
    project_arm_point,
    write_applied,
)
from .test_camera_arm_chain import ARM_BASE, calibration_chain
from .test_tools import TOKEN, call

RESIDUAL = ChainResidual(kind="head_arm", holdout_board_mm=2.9, holdout_rms_px=2.5, rms_px=2.7)
PAN, TILT = -0.45, 0.10
POINT_ARM = np.array([0.39, 0.0, 0.22])  # 0.3 m from the camera, a little right of its axis


def make(
    tmp_path: Path, residual: ChainResidual | None = RESIDUAL, source: str = "head_arm", **arm: object
) -> tuple[Any, DepthRobot, McpServerConfig, Path]:
    cfg = depth_config(**arm)
    cfg.cameras.calibration_dir = tmp_path / "calib"
    applied = write_applied(tmp_path / "applied", residual, source=source)
    robot = DepthRobot(tmp_path)
    install_head(robot, cfg, applied, PAN, TILT)
    return build_mcp_server(robot, cfg, TOKEN), robot, cfg, applied


def pixel_call(server: Any, u: float, v: float, **extra: Any) -> dict[str, Any]:
    return call(server, "pixel_to_point", {"camera": "head", "u": u, "v": v, **extra}).structured_content


def arm_xyz(block: dict[str, Any]) -> np.ndarray:
    return np.array([block["x"], block["y"], block["z"]])


def test_tool_names_include_the_depth_tools() -> None:
    assert "pixel_to_point" in TOOL_NAMES and "camera_point_to_arm" in TOOL_NAMES


def test_description_says_the_pixel_must_come_from_a_current_picture(tmp_path: Path) -> None:
    """The depth is read now, not at the stamp of the picture the pixel came from: a stale pixel lands on another spot."""
    import anyio

    server, *_ = make(tmp_path)
    tools = anyio.run(server.list_tools)
    text = next(t.description for t in tools if t.name == "pixel_to_point")
    assert "just taken" in text and "get_camera_image" in text and "not moved since" in text


def test_reports_every_frame_the_depth_and_the_calibration(tmp_path: Path) -> None:
    server, robot, _, applied = make(tmp_path)
    p_arm = POINT_ARM
    u, v, z = project_arm_point(applied, p_arm, PAN, TILT)
    robot.depth_frame = depth_frame(object_depth(u, v, z))
    data = pixel_call(server, u, v)
    assert data["camera"] == "head" and data["pixel"] == {"u": u, "v": v}
    assert data["camera_optical"]["frame"] == CAMERA_FRAME
    assert data["base_link"]["frame"] == "base_link" and data["arm"]["frame"] == "arm"
    assert data["camera_optical"]["z"] == pytest.approx(z, abs=0.001)
    assert arm_xyz(data["arm"]) == pytest.approx(p_arm, abs=0.002)
    pose = robot.pose
    mx, my = base_to_map(pose, data["base_link"]["x"], data["base_link"]["y"])
    assert data["map"] == {
        "frame": "map",
        "x": pytest.approx(mx, abs=0.002),
        "y": pytest.approx(my, abs=0.002),
        "z": data["base_link"]["z"],
    }
    assert data["depth"]["z_m"] == pytest.approx(z, abs=0.001) and data["depth"]["valid_fraction"] == 1.0
    assert data["depth"]["window_px"] == 5 and data["depth"]["spread_m"] == pytest.approx(0.0, abs=0.0011)
    assert data["uncertainty_m"] == pytest.approx(d435_sigma_m(z), abs=1e-4)
    assert data["head_pose_at_capture"]["pan_deg"] == pytest.approx(math.degrees(PAN), abs=1e-3)
    assert data["head_pose_at_capture"]["tilt_deg"] == pytest.approx(math.degrees(TILT), abs=1e-3)
    assert data["head_pose_at_capture"]["stamp_s"] == STAMP and data["head_moving"] is False
    assert robot.depth_calls == ["head"]


def test_calibration_block_names_the_residual_and_the_gate(tmp_path: Path) -> None:
    server, robot, _, applied = make(tmp_path)
    u, v, z = project_arm_point(applied, POINT_ARM, PAN, TILT)
    robot.depth_frame = depth_frame(object_depth(u, v, z))
    data = pixel_call(server, u, v)
    cal = data["calibration"]
    assert cal["applied"] is True and cal["source"] == "head_arm" and cal["residual_mm"] == 2.9
    assert (cal["session"], cal["result"]) == ("calib-1", "r-1") and cal["gate_mm"] == 5.0
    assert cal["within_gate"] is True and data["grasp_ready"] is True
    assert data["error_budget_mm"] == pytest.approx(math.hypot(2.9, d435_sigma_m(z) * 1000), abs=0.01)


def test_residual_above_the_gate_is_reported_and_blocks_grasp_ready(tmp_path: Path) -> None:
    server, robot, _, applied = make(tmp_path, ChainResidual("head_arm", 6.2, 3.0, 3.0))
    u, v, z = project_arm_point(applied, POINT_ARM, PAN, TILT)
    robot.depth_frame = depth_frame(object_depth(u, v, z))
    data = pixel_call(server, u, v)
    assert data["calibration"]["residual_mm"] == 6.2 and data["calibration"]["within_gate"] is False
    assert data["grasp_ready"] is False
    assert any("above the 5 mm gate" in note for note in data["notes"])


def test_floor_fit_chain_has_no_arm_frame_residual_so_it_is_not_grasp_ready(tmp_path: Path) -> None:
    server, robot, _, applied = make(tmp_path, None, source="floor")
    u, v, z = project_arm_point(applied, POINT_ARM, PAN, TILT)
    robot.depth_frame = depth_frame(object_depth(u, v, z))
    data = pixel_call(server, u, v)
    assert data["calibration"]["residual_mm"] is None and data["calibration"]["within_gate"] is None
    assert data["grasp_ready"] is False
    assert any("no arm-frame residual" in note for note in data["notes"])
    assert arm_xyz(data["arm"]) == pytest.approx(list(POINT_ARM), abs=0.002)  # the point itself is still reported


def test_gate_is_configurable(tmp_path: Path) -> None:
    cfg = depth_config()
    cfg.camera_arm.max_residual_mm = 3.5
    applied = write_applied(tmp_path / "applied", ChainResidual("head_arm", 4.0, 3.0, 3.0))
    robot = DepthRobot(tmp_path)
    install_head(robot, cfg, applied, PAN, TILT)
    server = build_mcp_server(robot, cfg, TOKEN)
    u, v, z = project_arm_point(applied, POINT_ARM, PAN, TILT)
    robot.depth_frame = depth_frame(object_depth(u, v, z))
    data = pixel_call(server, u, v)
    assert data["calibration"]["gate_mm"] == 3.5 and data["calibration"]["within_gate"] is False


@pytest.mark.parametrize("joints", ARM_JOINT_SETS, ids=range(len(ARM_JOINT_SETS)))
@pytest.mark.parametrize(("aim_pan", "aim_tilt"), [(0.0, 0.0), (0.3, 0.1), (-0.4, -0.1)])
def test_round_trip_arm_fk_to_pixel_to_arm_within_2_mm(
    tmp_path: Path, joints: dict[str, float], aim_pan: float, aim_tilt: float
) -> None:
    """arm FK tool point -> colour pixel through the calibration chain -> depth of a small object -> arm frame."""
    server, robot, cfg, applied = make(tmp_path)
    kin = robot.arm.kin
    pose = kin.forward(joints)
    p_arm = np.array([pose.x, pose.y, pose.z])
    head_kin = robot.head.kinematics
    t_base_arm = np.linalg.inv(_arm_in_base())
    p_base = (t_base_arm @ np.append(p_arm, 1.0))[:3]
    pan, tilt = head_kin.look_at(p_base)
    pan, tilt = pan + aim_pan * 0.25, tilt + aim_tilt * 0.25  # off-centre pixels too
    install_head(robot, cfg, applied, pan, tilt)
    u, v, z = project_arm_point(applied, p_arm, pan, tilt)
    assert 0.2 < z < 0.9 and 20 < u < WIDTH - 20 and 20 < v < HEIGHT - 20, (u, v, z)
    robot.depth_frame = depth_frame(object_depth(u, v, z))
    data = pixel_call(server, u, v)
    assert np.linalg.norm(arm_xyz(data["arm"]) - p_arm) < 0.002
    assert data["head_pose_at_capture"]["pan_deg"] == pytest.approx(math.degrees(pan), abs=1e-3)


def _arm_in_base() -> np.ndarray:
    """T_arm_base: base_link in the arm base frame (the inverse of the arm mount pose)."""
    from ros2_common.head_rgbd_calibration.chain import arm_in_robot_base

    return np.linalg.inv(arm_in_robot_base(ARM_BASE))


@pytest.mark.parametrize("z_target", [0.35, 0.6, 0.9])
def test_round_trip_with_d435_noise_stays_inside_the_z_squared_bound(tmp_path: Path, z_target: float) -> None:
    server, robot, cfg, applied = make(tmp_path)
    pan, tilt = 0.1, 0.45
    install_head(robot, cfg, applied, pan, tilt)
    # An arm-frame point whose camera depth is about z_target: walk along the optical axis from the camera.
    chain = calibration_chain(applied)
    t_base_cam = chain.transform(CAMERA_FRAME, {"head_pan": pan, "head_tilt": tilt})
    p_base = (t_base_cam @ np.array([0.02, -0.01, z_target, 1.0]))[:3]
    p_arm = (_arm_in_base() @ np.append(p_base, 1.0))[:3]
    u, v, z = project_arm_point(applied, p_arm, pan, tilt)
    sigma = d435_sigma_m(z)
    robot.depth_frame = depth_frame(object_depth(u, v, z, noise_sigma_m=sigma, seed=3))
    data = pixel_call(server, u, v)
    error = np.linalg.norm(arm_xyz(data["arm"]) - p_arm)
    assert error < 3.0 * sigma + 0.001  # depth error (median of 25 pixels, so well inside 3 sigma) plus quantisation
    assert data["uncertainty_m"] == pytest.approx(sigma, rel=0.02)


def test_pixel_picked_on_a_smaller_picture_is_scaled(tmp_path: Path) -> None:
    server, robot, _, applied = make(tmp_path)
    u, v, z = project_arm_point(applied, POINT_ARM, PAN, TILT)
    robot.depth_frame = depth_frame(object_depth(u, v, z))
    scale = 384 / WIDTH
    data = pixel_call(server, u * scale, v * scale, image_width=384, image_height=216)
    assert data["pixel"] == {"u": pytest.approx(u, abs=1e-6), "v": pytest.approx(v, abs=1e-6)}
    assert data["pixel_input"]["image_width"] == 384
    assert arm_xyz(data["arm"]) == pytest.approx(list(POINT_ARM), abs=0.002)


def test_head_moving_at_the_frame_stamp_is_refused(tmp_path: Path) -> None:
    server, robot, _, applied = make(tmp_path)
    robot.head.history.stamps.clear(), robot.head.history.pans.clear(), robot.head.history.tilts.clear()
    for i in range(int(2 * HEAD_RATE_HZ) + 1):  # a 0.5 rad/s pan sweep through the stamp
        t = STAMP - 1.0 + i / HEAD_RATE_HZ
        robot.head.history.add(t, {"head_pan": 0.5 * (t - STAMP), "head_tilt": TILT})
    u, v, z = project_arm_point(applied, POINT_ARM, PAN, TILT)
    robot.depth_frame = depth_frame(object_depth(u, v, z))
    with pytest.raises(ToolError, match=r"head moved .* deg within \+-0\.1 s of the depth frame"):
        pixel_call(server, u, v)


def test_head_pose_missing_at_the_stamp_is_refused(tmp_path: Path) -> None:
    server, robot, _, applied = make(tmp_path)
    robot.depth_frame = depth_frame(object_depth(300.0, 200.0, 0.5), stamp_s=STAMP + 30.0)
    with pytest.raises(ToolError, match="head_joint_states|joint_states"):
        pixel_call(server, 300.0, 200.0)


@pytest.mark.parametrize(
    ("depth_m", "message"),
    [(0.10, "nearer than 0.15 m"), (1.4, "farther than 1 m")],
)
def test_depth_outside_the_trusted_range_is_refused(tmp_path: Path, depth_m: float, message: str) -> None:
    server, robot, _, _ = make(tmp_path)
    robot.depth_frame = depth_frame(flat_depth(depth_m))
    with pytest.raises(ToolError, match=message):
        pixel_call(server, 300.0, 200.0)


def test_pixel_on_a_depth_edge_is_refused(tmp_path: Path) -> None:
    server, robot, _, _ = make(tmp_path)
    depth = flat_depth(0.4)
    depth[:, 300:] = 700
    robot.depth_frame = depth_frame(depth)
    with pytest.raises(ToolError, match="depth edge"):
        pixel_call(server, 300.0, 200.0)


def test_hole_in_the_depth_is_refused_as_no_depth(tmp_path: Path) -> None:
    server, robot, _, _ = make(tmp_path)
    depth = flat_depth(0.4)
    depth[190:210, 290:310] = 0
    robot.depth_frame = depth_frame(depth)
    with pytest.raises(ToolError, match="no valid depth"):
        pixel_call(server, 300.0, 200.0)


def test_window_px_is_validated(tmp_path: Path) -> None:
    server, robot, _, _ = make(tmp_path)
    robot.depth_frame = depth_frame(flat_depth(0.4))
    with pytest.raises(ToolError, match="odd"):
        pixel_call(server, 300.0, 200.0, window_px=4)
    assert pixel_call(server, 300.0, 200.0, window_px=9)["depth"]["window_px"] == 9


def test_depth_image_of_another_size_than_the_colour_image_is_refused(tmp_path: Path) -> None:
    server, robot, _, _ = make(tmp_path)
    robot.depth_frame = depth_frame(np.full((180, 320), 400, dtype=np.uint16))
    with pytest.raises(ToolError, match=r"320x180.*640x360"):
        pixel_call(server, 300.0, 200.0)


def test_depth_stream_not_registered_to_colour_is_refused(tmp_path: Path) -> None:
    server, robot, _, _ = make(tmp_path)
    other = INFO.__class__(**{**INFO.__dict__, "k": (380.0, 0.0, 320.0, 0.0, 380.0, 180.0, 0.0, 0.0, 1.0)})
    robot.depth_frame = depth_frame(flat_depth(0.4), info=other)
    with pytest.raises(ToolError, match="not registered"):
        pixel_call(server, 300.0, 200.0)


def test_no_depth_frame_error_reaches_the_agent(tmp_path: Path) -> None:
    server, robot, _, _ = make(tmp_path)
    robot.depth_error = "colour and aligned depth stamps differ by 0.100 s"
    with pytest.raises(ToolError, match="differ by 0.100 s"):
        pixel_call(server, 300.0, 200.0)


def test_without_an_arm_mount_the_arm_point_is_null_and_noted(tmp_path: Path) -> None:
    cfg = depth_config()
    cfg.arm.base_in_base_link = None
    applied = write_applied(tmp_path / "applied", RESIDUAL)
    robot = DepthRobot(tmp_path)
    install_head(robot, cfg, applied, PAN, TILT)
    server = build_mcp_server(robot, cfg, TOKEN)
    robot.depth_frame = depth_frame(flat_depth(0.4))
    data = pixel_call(server, 300.0, 200.0)
    assert data["arm"] is None and data["grasp_ready"] is False
    assert any("arm.base_in_base_link" in note for note in data["notes"])


def test_outside_the_image_is_refused(tmp_path: Path) -> None:
    server, robot, _, _ = make(tmp_path)
    robot.depth_frame = depth_frame(flat_depth(0.4))
    with pytest.raises(ToolError, match="outside"):
        pixel_call(server, 700.0, 200.0)


# camera_point_to_arm ----------------------------------------------------------------------------------------------


def point_call(server: Any, frame: str, x: float, y: float, z: float, **extra: Any) -> dict[str, Any]:
    return call(server, "camera_point_to_arm", {"frame": frame, "x": x, "y": y, "z": z, **extra}).structured_content


@pytest.mark.parametrize("frame", [CAMERA_FRAME, DEPTH_FRAME])
def test_camera_point_to_arm_matches_the_calibration_chain(tmp_path: Path, frame: str) -> None:
    server, robot, _, applied = make(tmp_path)
    chain = calibration_chain(applied)
    point = np.array([0.03, -0.02, 0.42])
    t_base_cam = chain.transform(frame, {"head_pan": PAN, "head_tilt": TILT})
    want_base = (t_base_cam @ np.append(point, 1.0))[:3]
    want_arm = (_arm_in_base() @ np.append(want_base, 1.0))[:3]
    data = point_call(server, frame, *point, stamp_s=STAMP)
    assert data["camera_optical"] == {"frame": frame, "x": 0.03, "y": -0.02, "z": 0.42}
    assert arm_xyz(data["base_link"]) == pytest.approx(want_base, abs=1e-3)  # reported to 1 mm
    assert arm_xyz(data["arm"]) == pytest.approx(want_arm, abs=1e-3)
    assert data["base_link"]["frame"] == "base_link" and data["arm"]["frame"] == "arm"
    assert data["calibration"]["residual_mm"] == 2.9 and data["grasp_ready"] is True
    assert data["head_pose_at_capture"]["stamp_s"] == STAMP and data["head_moving"] is False


def test_camera_point_to_arm_agrees_with_pixel_to_point(tmp_path: Path) -> None:
    server, robot, _, applied = make(tmp_path)
    u, v, z = project_arm_point(applied, POINT_ARM, PAN, TILT)
    robot.depth_frame = depth_frame(object_depth(u, v, z))
    via_pixel = pixel_call(server, u, v)
    optical = via_pixel["camera_optical"]
    via_point = point_call(server, CAMERA_FRAME, optical["x"], optical["y"], optical["z"], stamp_s=STAMP)
    assert arm_xyz(via_point["arm"]) == pytest.approx(arm_xyz(via_pixel["arm"]), abs=1.5e-3)


def test_camera_point_to_arm_without_a_stamp_uses_the_newest_head_pose(tmp_path: Path) -> None:
    server, robot, _, _ = make(tmp_path)
    data = point_call(server, CAMERA_FRAME, 0.0, 0.0, 0.4)
    assert data["head_pose_at_capture"]["pan_deg"] == pytest.approx(math.degrees(PAN), abs=1e-3)
    assert any("no stamp_s" in note for note in data["notes"])


def test_camera_point_to_arm_follows_the_head_pose_at_the_given_stamp(tmp_path: Path) -> None:
    server, robot, cfg, _ = make(tmp_path)
    feed_history(robot.head, -0.6, 0.1, around=STAMP + 5.0)  # a different pose 5 s later (history keeps 10 s)
    here = point_call(server, CAMERA_FRAME, 0.0, 0.0, 0.4, stamp_s=STAMP)
    there = point_call(server, CAMERA_FRAME, 0.0, 0.0, 0.4, stamp_s=STAMP + 5.0)
    assert arm_xyz(here["arm"]) != pytest.approx(arm_xyz(there["arm"]), abs=0.01)
    assert there["head_pose_at_capture"]["pan_deg"] == pytest.approx(math.degrees(-0.6), abs=1e-3)


def test_camera_point_to_arm_refuses_a_moving_head(tmp_path: Path) -> None:
    server, robot, _, _ = make(tmp_path)
    for i in range(int(2 * HEAD_RATE_HZ) + 1):
        t = STAMP + 5.0 + i / HEAD_RATE_HZ
        robot.head.history.add(t, {"head_pan": 0.5 * (t - STAMP - 5.0), "head_tilt": TILT})  # a 0.5 rad/s sweep
    with pytest.raises(ToolError, match="head moved"):
        point_call(server, CAMERA_FRAME, 0.0, 0.0, 0.4, stamp_s=STAMP + 6.0)


def test_camera_point_to_arm_rejects_other_frames_and_bad_depth(tmp_path: Path) -> None:
    server, _, _, _ = make(tmp_path)
    with pytest.raises(ToolError):
        point_call(server, "base_link", 0.0, 0.0, 0.4, stamp_s=STAMP)
    with pytest.raises(ToolError, match="z must be positive"):
        point_call(server, CAMERA_FRAME, 0.0, 0.0, -0.1, stamp_s=STAMP)


def test_camera_point_to_arm_without_a_head_history_is_refused(tmp_path: Path) -> None:
    server, robot, cfg, applied = make(tmp_path)
    robot.head.history.stamps.clear(), robot.head.history.pans.clear(), robot.head.history.tilts.clear()
    with pytest.raises(ToolError, match="no .*samples"):
        point_call(server, CAMERA_FRAME, 0.0, 0.0, 0.4)


# pixel_to_ground gains the arm frame and the head pose --------------------------------------------------------------


def test_pixel_to_ground_reports_the_arm_frame_and_the_head_pose(tmp_path: Path) -> None:
    server, robot, _, _ = make(tmp_path)
    data = call(server, "pixel_to_ground", {"camera": "head", "u": 320.0, "v": 300.0}).structured_content
    base = arm_xyz(data["ground_base_link"])
    expect_arm = (_arm_in_base() @ np.append(base, 1.0))[:3]
    assert arm_xyz(data["ground_arm"]) == pytest.approx(expect_arm, abs=2e-3)
    assert data["ground_arm"]["frame"] == "arm"
    assert data["head_pose_at_capture"]["pan_deg"] == pytest.approx(math.degrees(PAN), abs=1e-3)
    assert data["calibration"]["residual_mm"] == 2.9


def test_pixel_to_ground_with_the_picture_stamp_uses_the_head_pose_at_that_stamp(tmp_path: Path) -> None:
    server, robot, cfg, applied = make(tmp_path)
    # The picture was taken at STAMP with pan 0.2; the head has turned since: the live pose is pan -0.6.
    robot.head_backend.positions.update({"head_pan": -0.6, "head_tilt": TILT})
    stale = call(server, "pixel_to_ground", {"camera": "head", "u": 320.0, "v": 300.0}).structured_content
    exact = call(
        server, "pixel_to_ground", {"camera": "head", "u": 320.0, "v": 300.0, "image_stamp_s": STAMP}
    ).structured_content
    base_exact = arm_xyz(exact["ground_base_link"])
    chain = calibration_chain(applied)
    t = chain.transform(CAMERA_FRAME, {"head_pan": PAN, "head_tilt": TILT})
    ray = np.array([(320.0 - CX) / FX, (300.0 - CY) / FY, 1.0])
    direction = t[:3, :3] @ ray
    s = (0.0 - t[2, 3]) / direction[2]
    expect = t[:3, 3] + s * direction
    assert base_exact[:2] == pytest.approx(expect[:2], abs=2e-3)
    assert arm_xyz(stale["ground_base_link"])[:2] != pytest.approx(base_exact[:2], abs=0.05)
    assert exact["head_pose_at_capture"]["stamp_s"] == STAMP and exact["head_moving"] is False
    assert exact["head_pose_at_capture"]["pan_deg"] == pytest.approx(math.degrees(PAN), abs=1e-3)


def test_pixel_to_ground_refuses_a_picture_stamp_outside_the_head_history(tmp_path: Path) -> None:
    server, _, _, _ = make(tmp_path)
    with pytest.raises(ToolError, match="older than the oldest"):
        call(server, "pixel_to_ground", {"camera": "head", "u": 320.0, "v": 300.0, "image_stamp_s": 1.0})


def test_pixel_to_ground_refuses_while_the_head_moves(tmp_path: Path) -> None:
    server, robot, _, _ = make(tmp_path)
    robot.head.history.stamps.clear(), robot.head.history.pans.clear(), robot.head.history.tilts.clear()
    for i in range(int(2 * HEAD_RATE_HZ) + 1):
        t = STAMP - 1.0 + i / HEAD_RATE_HZ
        robot.head.history.add(t, {"head_pan": 0.5 * (t - STAMP), "head_tilt": TILT})
    with pytest.raises(ToolError, match="head is moving"):
        call(server, "pixel_to_ground", {"camera": "head", "u": 320.0, "v": 300.0})


def test_mark_candidate_points_places_the_dots_with_the_head_pose_at_the_frame_stamp(tmp_path: Path) -> None:
    server, robot, _, _ = make(tmp_path)
    robot.head_backend.positions.update({"head_pan": -0.6, "head_tilt": TILT})  # live pose differs from the frame's
    res = call(server, "mark_candidate_points", {"camera": "head", "spacing_px": 80, "max_points": 200})
    table = next(json.loads(c.text) for c in res.content if c.type == "text" and "set_id" in c.text)
    placed = [p for p in table["points"] if p["ground_base_link"] is not None]
    assert placed
    for point in placed[:5]:
        expect = call(
            server,
            "pixel_to_ground",
            {"camera": "head", "u": point["u"], "v": point["v"], "image_stamp_s": STAMP},
        ).structured_content
        assert arm_xyz(point["ground_base_link"]) == pytest.approx(arm_xyz(expect["ground_base_link"]), abs=0.002)
    live = call(server, "pixel_to_ground", {"camera": "head", "u": placed[0]["u"], "v": placed[0]["v"]})
    assert arm_xyz(live.structured_content["ground_base_link"])[:2] != pytest.approx(
        arm_xyz(placed[0]["ground_base_link"])[:2], abs=0.05
    )  # the live pose (pan -0.6) would have put the dot elsewhere
