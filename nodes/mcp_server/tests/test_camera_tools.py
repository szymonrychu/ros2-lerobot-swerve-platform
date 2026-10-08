"""MCP camera tools end to end on a fake robot with synthetic cameras: pixel->ground, overlays, candidates, calibration."""

import base64
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import pytest
import yaml
from mcp.server.mcpserver.exceptions import ToolError
from ros2_common.camera_geometry import CameraIntrinsics, MountPose, optical_from_mount, project_raw

from mcp_server.camera_tools import TOOL_NAMES
from mcp_server.config import McpServerConfig
from mcp_server.models import BasePose, CameraFrame, RobotState, ScanPoints
from mcp_server.tools import build_mcp_server

from .camera_helpers import FRONT_MOUNT, HEIGHT, WIDTH, camera_config, front_ground_of_center, jpeg_blank
from .test_tools import TOKEN, FakeRobot, call

NOT_CALIBRATED = "camera {} not calibrated: set intrinsics and mount in mcp_server config (see README calibration)"
INTR = CameraIntrinsics.from_hfov(WIDTH, HEIGHT, 90.0)


class CameraRobot(FakeRobot):
    """FakeRobot with 640x480 frames, a movable map pose and a lidar scan."""

    def __init__(self, tmp_path: Path) -> None:
        super().__init__(tmp_path)
        self.pose = BasePose(frame="map", x=1.0, y=2.0, yaw=math.pi / 2, age_s=0.05)
        self.pose_missing = False
        self.scan: ScanPoints | None = ScanPoints(
            frame="base_link", age_s=0.1, points=[(1.0, 0.0, 0.2, 1.0), (0.9, 0.3, 0.2, 0.95), (1.2, -0.2, 0.2, 1.2)]
        )

    def camera_image(self, camera: str, max_px: int) -> CameraFrame:
        self.calls.append(("camera_image", (camera, max_px)))
        ok, jpg = cv2.imencode(".jpg", jpeg_blank())
        assert ok
        return CameraFrame(
            camera=camera, topic="/x", jpeg=jpg.tobytes(), width=WIDTH, height=HEIGHT, stamp_s=50.0, age_s=0.1
        )

    def robot_state(self) -> RobotState:
        self.calls.append(("robot_state", ()))
        return RobotState(pose=None if self.pose_missing else self.pose, arm=self.arm.state())

    def scan_points(self) -> ScanPoints | None:
        self.calls.append(("scan_points", ()))
        return self.scan


def make(tmp_path: Path, cfg: McpServerConfig | None = None) -> tuple[Any, CameraRobot, McpServerConfig]:
    cfg = cfg or camera_config()
    cfg.cameras.calibration_dir = tmp_path / "calib"
    robot = CameraRobot(tmp_path)
    return build_mcp_server(robot, cfg, TOKEN), robot, cfg


def text_json(res: Any) -> dict[str, Any]:
    blocks = [c.text for c in res.content if c.type == "text"]
    for block in blocks:
        data = json.loads(block)
        if "robot_events_since_last_call" not in data:
            return data
    raise AssertionError("no payload text block")


def decode_image(res: Any) -> np.ndarray:
    img = next(c for c in res.content if c.type == "image")
    assert img.mime_type == "image/jpeg"
    arr = cv2.imdecode(np.frombuffer(base64.b64decode(img.data), np.uint8), cv2.IMREAD_COLOR)
    assert arr is not None
    return arr


def front_pixel(ground: tuple[float, float, float]) -> tuple[float, float]:
    t = optical_from_mount(MountPose(**FRONT_MOUNT).to_matrix())
    u, v, _ = project_raw(INTR, t, np.array(ground))
    return u, v


def test_tool_names_are_the_contract() -> None:
    assert TOOL_NAMES == (
        "pixel_to_ground",
        "get_annotated_camera_image",
        "mark_candidate_points",
        "resolve_candidate",
        "capture_calibration_sample",
        "solve_camera_calibration",
        "clear_calibration_samples",
    )


def test_tools_are_registered_with_descriptions(tmp_path: Path) -> None:
    import anyio

    server, _, _ = make(tmp_path)
    tools = {t.name: t for t in anyio.run(server.list_tools)}
    for name in TOOL_NAMES:
        assert name in tools and len(tools[name].description) > 60, name


@pytest.mark.parametrize(
    ("name", "args"),
    [
        ("pixel_to_ground", {"camera": "front", "u": 100.0, "v": 100.0}),
        ("get_annotated_camera_image", {"camera": "front"}),
        ("mark_candidate_points", {"camera": "front"}),
    ],
)
def test_uncalibrated_camera_tools_report_the_documented_error(tmp_path: Path, name: str, args: dict) -> None:
    server, _, _ = make(tmp_path, McpServerConfig())
    with pytest.raises(ToolError) as exc:
        call(server, name, args)
    assert NOT_CALIBRATED.format("front") in str(exc.value)


def test_intrinsics_without_mount_is_still_not_calibrated(tmp_path: Path) -> None:
    cfg = McpServerConfig.model_validate(
        {"cameras": {"front": {"intrinsics": {"hfov_deg": 66.0, "width": 640, "height": 480}}}}
    )
    server, _, _ = make(tmp_path, cfg)
    with pytest.raises(ToolError, match="not calibrated"):
        call(server, "pixel_to_ground", {"camera": "front", "u": 1.0, "v": 1.0})


def test_pixel_to_ground_front(tmp_path: Path) -> None:
    server, _, _ = make(tmp_path)
    res = call(server, "pixel_to_ground", {"camera": "front", "u": WIDTH / 2, "v": HEIGHT / 2})
    data = res.structured_content
    ex, _ = front_ground_of_center()
    assert data["camera"] == "front"
    assert data["ground_base_link"]["x"] == pytest.approx(ex, abs=1e-3)
    assert data["ground_base_link"]["z"] == 0.0
    assert data["ground_map"]["y"] == pytest.approx(2.0 + ex, abs=1e-3)
    assert data["distance_from_base_m"] == pytest.approx(ex, abs=1e-3)
    assert "approximate intrinsics" in data["uncertainty_note"]


def test_pixel_to_ground_without_map_pose_omits_ground_map(tmp_path: Path) -> None:
    server, robot, _ = make(tmp_path)
    robot.pose_missing = True
    data = call(server, "pixel_to_ground", {"camera": "front", "u": 320.0, "v": 240.0}).structured_content
    assert "ground_map" not in data and "ground_base_link" in data


def test_pixel_to_ground_gripper_uses_current_joints_and_arm_frame(tmp_path: Path) -> None:
    server, robot, cfg = make(tmp_path, camera_config(front=False))
    robot.arm_backend.positions.update({"shoulder_lift": 1.0, "elbow_flex": 0.9, "wrist_flex": 1.0})
    hit = None
    for v in range(HEIGHT - 10, 0, -40):
        try:
            hit = call(
                server, "pixel_to_ground", {"camera": "gripper", "u": WIDTH / 2, "v": float(v)}
            ).structured_content
            break
        except ToolError:
            continue
    assert hit is not None, "no pixel of the synthetic gripper camera sees the floor"
    assert hit["ground_arm_base"]["z"] == pytest.approx(cfg.arm.floor_z_m)
    assert "ground_base_link" not in hit


def test_pixel_to_ground_gripper_adds_base_link_with_arm_offset(tmp_path: Path) -> None:
    server, robot, _ = make(tmp_path, camera_config(front=False, arm_offset=True))
    robot.arm_backend.positions.update({"shoulder_lift": 1.0, "elbow_flex": 0.9, "wrist_flex": 1.0})
    for v in range(HEIGHT - 10, 0, -40):
        try:
            data = call(
                server, "pixel_to_ground", {"camera": "gripper", "u": WIDTH / 2, "v": float(v)}
            ).structured_content
        except ToolError:
            continue
        assert data["ground_base_link"]["z"] == pytest.approx(0.0, abs=1e-6)
        assert "ground_map" in data
        return
    pytest.fail("no pixel sees the floor")


def test_pixel_to_ground_gripper_needs_fresh_joints(tmp_path: Path) -> None:
    server, robot, _ = make(tmp_path, camera_config(front=False))
    robot.arm_backend.no_samples = True
    with pytest.raises(ToolError, match="joint_states"):
        call(server, "pixel_to_ground", {"camera": "gripper", "u": 320.0, "v": 400.0})


def test_pixel_to_ground_rejects_sky_and_off_image(tmp_path: Path) -> None:
    cfg = camera_config()
    cfg.cameras.front.mount.pitch = 0.1
    server, _, _ = make(tmp_path, cfg)
    with pytest.raises(ToolError, match="floor"):
        call(server, "pixel_to_ground", {"camera": "front", "u": 320.0, "v": 5.0})
    with pytest.raises(ToolError, match="outside the image"):
        call(server, "pixel_to_ground", {"camera": "front", "u": 9999.0, "v": 5.0})


def test_annotated_image_default_grid(tmp_path: Path) -> None:
    server, robot, _ = make(tmp_path)
    res = call(server, "get_annotated_camera_image", {"camera": "front"})
    img = decode_image(res)
    assert img.shape == (HEIGHT, WIDTH, 3)
    assert (np.abs(img.astype(int) - 90) > 20).any()
    meta = text_json(res)
    assert meta["camera"] == "front" and meta["overlays"] == ["grid"] and meta["frame"] == "base_link"
    assert meta["grid_step_m"] == 0.1 and meta["approximate_intrinsics"] is True
    assert ("camera_image", ("front", 640)) in robot.calls


def test_annotated_image_all_overlays_front(tmp_path: Path) -> None:
    server, _, _ = make(tmp_path, camera_config(arm_offset=True))
    res = call(
        server,
        "get_annotated_camera_image",
        {
            "camera": "front",
            "overlays": ["grid", "reach", "gripper", "planned_gripper", "lidar"],
            "planned_gripper": {"x": 0.2, "y": 0.0, "z": 0.0},
            "grid_step_m": 0.25,
        },
    )
    meta = text_json(res)
    assert meta["overlays"] == ["grid", "reach", "gripper", "planned_gripper", "lidar"]
    assert meta["lidar_points_shown"] == 3
    assert meta["grid_step_m"] == 0.25
    decode_image(res)


def test_annotated_image_notes_unavailable_overlays(tmp_path: Path) -> None:
    server, robot, _ = make(tmp_path)  # no arm offset configured: arm overlays cannot be placed in base_link
    robot.scan = None
    res = call(
        server,
        "get_annotated_camera_image",
        {
            "camera": "front",
            "overlays": ["reach", "gripper", "planned_gripper", "lidar"],
            "planned_gripper": {"x": 0.2, "y": 0.0, "z": 0.0},
        },
    )
    notes = " ".join(text_json(res)["notes"])
    for word in ("reach", "gripper", "planned_gripper", "lidar"):
        assert word in notes


def test_planned_gripper_overlay_requires_the_point(tmp_path: Path) -> None:
    server, _, _ = make(tmp_path)
    with pytest.raises(ToolError, match="planned_gripper"):
        call(server, "get_annotated_camera_image", {"camera": "front", "overlays": ["planned_gripper"]})


def test_annotated_image_rejects_unknown_overlay_and_bad_step(tmp_path: Path) -> None:
    server, _, _ = make(tmp_path)
    with pytest.raises(ToolError):
        call(server, "get_annotated_camera_image", {"camera": "front", "overlays": ["sparkles"]})
    with pytest.raises(ToolError):
        call(server, "get_annotated_camera_image", {"camera": "front", "grid_step_m": 0.001})


def test_annotated_gripper_image_draws_in_the_arm_frame(tmp_path: Path) -> None:
    server, robot, _ = make(tmp_path, camera_config(front=False))
    robot.arm_backend.positions.update({"shoulder_lift": 1.0, "elbow_flex": 0.9, "wrist_flex": 1.0})
    res = call(server, "get_annotated_camera_image", {"camera": "gripper", "overlays": ["grid", "reach", "gripper"]})
    meta = text_json(res)
    assert meta["frame"] == "arm_base"
    assert "tool_point" in meta


def test_mark_candidate_points_returns_image_and_table(tmp_path: Path) -> None:
    server, _, _ = make(tmp_path)
    res = call(server, "mark_candidate_points", {"camera": "front", "spacing_px": 80, "max_points": 30})
    table = text_json(res)
    decode_image(res)
    assert table["set_id"] and table["camera"] == "front"
    pts = table["points"]
    assert 0 < len(pts) <= 30
    assert [p["n"] for p in pts] == list(range(1, len(pts) + 1))
    for p in pts:
        assert p["ground_map"] is not None
        back = front_pixel((p["ground_base_link"]["x"], p["ground_base_link"]["y"], 0.0))
        assert back == pytest.approx((p["u"], p["v"]), abs=0.5)  # ground rounded to 1 mm


def test_mark_candidate_points_skips_sky_pixels(tmp_path: Path) -> None:
    cfg = camera_config()
    cfg.cameras.front.mount.pitch = 0.3  # horizon inside the image: the top rows see sky
    server, _, _ = make(tmp_path, cfg)
    table = text_json(call(server, "mark_candidate_points", {"camera": "front", "spacing_px": 40, "max_points": 200}))
    assert table["points"] and min(p["v"] for p in table["points"]) > 100
    assert table["skipped_no_ground"] > 0


def test_mark_candidate_points_region(tmp_path: Path) -> None:
    server, _, _ = make(tmp_path)
    table = text_json(
        call(
            server,
            "mark_candidate_points",
            {"camera": "front", "region": {"u0": 100, "v0": 200, "u1": 300, "v1": 300}, "spacing_px": 50},
        )
    )
    assert all(100 <= p["u"] <= 300 and 200 <= p["v"] <= 300 for p in table["points"])
    with pytest.raises(ToolError, match="region"):
        call(server, "mark_candidate_points", {"camera": "front", "region": {"u0": 300, "v0": 0, "u1": 100, "v1": 50}})


def test_ground_map_is_null_without_map_pose(tmp_path: Path) -> None:
    server, robot, _ = make(tmp_path)
    robot.pose_missing = True
    table = text_json(call(server, "mark_candidate_points", {"camera": "front", "spacing_px": 120}))
    assert all(p["ground_map"] is None for p in table["points"])


def test_resolve_candidate_returns_stored_values_with_age_and_motion_flags(tmp_path: Path) -> None:
    server, robot, _ = make(tmp_path)
    table = text_json(call(server, "mark_candidate_points", {"camera": "front", "spacing_px": 120}))
    stored = table["points"][1]
    res = call(server, "resolve_candidate", {"set_id": table["set_id"], "n": 2})
    data = res.structured_content
    assert data["point"] == stored and data["camera"] == "front"
    assert data["age_s"] >= 0.0
    assert data["robot_moved_since"] is False and data["arm_moved_since"] is False
    robot.pose = BasePose(frame="map", x=1.5, y=2.0, yaw=math.pi / 2, age_s=0.05)
    robot.arm_backend.positions["elbow_flex"] = 0.5
    moved = call(server, "resolve_candidate", {"set_id": table["set_id"], "n": 2}).structured_content
    assert moved["point"] == stored  # stored values, not recomputed
    assert moved["robot_moved_since"] is True and moved["arm_moved_since"] is True
    assert "moved" in moved["note"]


def test_resolve_candidate_errors(tmp_path: Path) -> None:
    server, _, _ = make(tmp_path)
    table = text_json(call(server, "mark_candidate_points", {"camera": "front", "spacing_px": 160}))
    with pytest.raises(ToolError, match="unknown candidate set"):
        call(server, "resolve_candidate", {"set_id": "nope", "n": 1})
    with pytest.raises(ToolError, match="no point 99"):
        call(server, "resolve_candidate", {"set_id": table["set_id"], "n": 99})


def test_only_the_last_ten_candidate_sets_are_kept(tmp_path: Path) -> None:
    server, _, _ = make(tmp_path)
    ids = [
        text_json(call(server, "mark_candidate_points", {"camera": "front", "spacing_px": 200}))["set_id"]
        for _ in range(11)
    ]
    assert len(set(ids)) == 11
    with pytest.raises(ToolError, match="unknown candidate set"):
        call(server, "resolve_candidate", {"set_id": ids[0], "n": 1})
    call(server, "resolve_candidate", {"set_id": ids[1], "n": 1})


def capture_all(server: Any, grounds: list[tuple[float, float, float]]) -> Any:
    res = None
    for g in grounds:
        u, v = front_pixel(g)
        res = call(
            server,
            "capture_calibration_sample",
            {"camera": "front", "u": u, "v": v, "ground_x": g[0], "ground_y": g[1], "ground_z": g[2]},
        )
    return res


GROUNDS = [(0.7, -0.3, 0.0), (0.9, 0.3, 0.0), (1.2, 0.0, 0.0), (0.6, 0.1, 0.0), (1.0, -0.2, 0.0), (0.8, 0.25, 0.0)]


def test_capture_counts_samples_and_writes_file(tmp_path: Path) -> None:
    server, _, cfg = make(tmp_path, McpServerConfig())  # capture works before the camera is calibrated
    res = capture_all(server, GROUNDS[:2])
    assert res.structured_content["samples"] == 2
    data = json.loads((cfg.cameras.calibration_dir / "front.json").read_text())
    assert len(data["samples"]) == 2 and data["parent_frame"] == "base_link"


def test_capture_gripper_stores_the_current_arm_pose(tmp_path: Path) -> None:
    server, robot, cfg = make(tmp_path, McpServerConfig())
    robot.arm_backend.positions.update({"shoulder_lift": 0.7})
    call(
        server,
        "capture_calibration_sample",
        {"camera": "gripper", "u": 100.0, "v": 200.0, "ground_x": 0.2, "ground_y": 0.0, "ground_z": -0.165},
    )
    sample = json.loads((cfg.cameras.calibration_dir / "gripper.json").read_text())["samples"][0]
    t = np.array(sample["t_frame_parent"])
    assert t.shape == (4, 4) and not np.allclose(t, np.eye(4))
    assert sample["ground"] == [0.2, 0.0, -0.165]
    robot.arm_backend.no_samples = True
    with pytest.raises(ToolError, match="joint_states"):
        call(
            server,
            "capture_calibration_sample",
            {"camera": "gripper", "u": 1.0, "v": 1.0, "ground_x": 0.2, "ground_y": 0.0},
        )


def test_solve_returns_yaml_and_rms_and_never_edits_config(tmp_path: Path) -> None:
    cfg = camera_config(gripper=False)
    cfg.cameras.front.mount = None  # solving works from an initial guess, not only the configured mount
    server, _, cfg = make(tmp_path, cfg)
    capture_all(server, GROUNDS)
    guess = {"x": 0.2, "y": 0.0, "z": 0.7, "roll": 0.0, "pitch": 0.8, "yaw": 0.0}
    res = call(server, "solve_camera_calibration", {"camera": "front", "initial": guess})
    data = res.structured_content
    assert data["rms_px"] < 0.1 and data["samples"] == 6
    mount = yaml.safe_load(data["yaml"])["cameras"]["front"]["mount"]
    assert mount["z"] == pytest.approx(0.8, abs=1e-3) and mount["pitch"] == pytest.approx(1.0, abs=1e-3)
    assert "approximate intrinsics" in data["note"]
    assert cfg.cameras.front.mount is None


def test_solve_defaults_to_the_configured_mount_as_initial(tmp_path: Path) -> None:
    server, _, _ = make(tmp_path, camera_config(gripper=False))
    capture_all(server, GROUNDS)
    data = call(server, "solve_camera_calibration", {"camera": "front"}).structured_content
    assert data["rms_px"] < 0.1


def test_solve_errors(tmp_path: Path) -> None:
    server, _, _ = make(tmp_path, McpServerConfig())
    capture_all(server, GROUNDS[:2])
    with pytest.raises(ToolError, match="not calibrated"):  # intrinsics missing
        call(server, "solve_camera_calibration", {"camera": "front"})
    cfg = McpServerConfig.model_validate(
        {"cameras": {"front": {"intrinsics": {"hfov_deg": 90.0, "width": 640, "height": 480}}}}
    )
    server2, _, _ = make(tmp_path, cfg)
    with pytest.raises(ToolError, match="initial"):  # no mount to start from
        call(server2, "solve_camera_calibration", {"camera": "front"})
    with pytest.raises(ToolError, match="at least 3"):
        call(
            server2,
            "solve_camera_calibration",
            {"camera": "front", "initial": {"x": 0.2, "y": 0.0, "z": 0.7, "roll": 0.0, "pitch": 0.8, "yaw": 0.0}},
        )


def test_clear_calibration_samples(tmp_path: Path) -> None:
    server, _, cfg = make(tmp_path)
    capture_all(server, GROUNDS[:3])
    res = call(server, "clear_calibration_samples", {"camera": "front"})
    assert res.structured_content["removed"] == 3
    assert not (cfg.cameras.calibration_dir / "front.json").exists()
    assert call(server, "clear_calibration_samples", {"camera": "front"}).structured_content["removed"] == 0


def test_camera_argument_is_validated(tmp_path: Path) -> None:
    server, _, _ = make(tmp_path)
    with pytest.raises(ToolError):
        call(server, "pixel_to_ground", {"camera": "rear", "u": 1.0, "v": 1.0})


def test_capture_gripper_stores_raw_joints_and_offset_corrected_parent_pose(tmp_path: Path) -> None:
    from mcp_server.ik import ArmKinematics

    cfg = McpServerConfig()
    cfg.cameras.calibration_dir = tmp_path / "calib"
    robot = CameraRobot(tmp_path)
    offsets = {"shoulder_lift": -0.12, "elbow_flex": 0.09, "wrist_flex": 0.2}
    robot.arm.kin = ArmKinematics(cfg.arm.urdf_path, margin=cfg.limits.arm_limit_margin_rad, joint_offsets=offsets)
    measured = {"shoulder_lift": 0.7, "elbow_flex": 0.3, "wrist_flex": 0.1}
    robot.arm_backend.positions.update(measured)
    server = build_mcp_server(robot, cfg, TOKEN)
    call(
        server,
        "capture_calibration_sample",
        {"camera": "gripper", "u": 100.0, "v": 200.0, "ground_x": 0.2, "ground_y": 0.0, "ground_z": -0.165},
    )
    sample = json.loads((cfg.cameras.calibration_dir / "gripper.json").read_text())["samples"][0]
    assert sample["joints"] == robot.arm_backend.positions
    plain = ArmKinematics(cfg.arm.urdf_path, margin=cfg.limits.arm_limit_margin_rad)
    corrected = {j: robot.arm_backend.positions[j] + offsets.get(j, 0.0) for j in robot.arm_backend.positions}
    expected = plain.link_frame(corrected, "gripper_link")
    np.testing.assert_allclose(np.array(sample["t_frame_parent"]), expected, atol=1e-9)


def test_pixel_to_ground_surface_height_hits_the_raised_plane(tmp_path: Path) -> None:
    server, _, _ = make(tmp_path)
    u, v = front_pixel((0.9, 0.1, 0.03))
    flat = call(server, "pixel_to_ground", {"camera": "front", "u": u, "v": v}).structured_content
    raised = call(
        server, "pixel_to_ground", {"camera": "front", "u": u, "v": v, "surface_height_m": 0.03}
    ).structured_content
    assert flat["surface_height_m"] == 0.0 and flat["ground_base_link"]["z"] == 0.0
    assert raised["surface_height_m"] == 0.03
    assert raised["ground_base_link"] == pytest.approx({"x": 0.9, "y": 0.1, "z": 0.03}, abs=2e-3)
    assert raised["ground_base_link"]["x"] != pytest.approx(flat["ground_base_link"]["x"], abs=1e-3)


def test_pixel_to_ground_surface_height_is_validated(tmp_path: Path) -> None:
    server, _, _ = make(tmp_path)
    with pytest.raises(ToolError):
        call(server, "pixel_to_ground", {"camera": "front", "u": 320.0, "v": 240.0, "surface_height_m": 5.0})


def test_mark_candidate_points_surface_height_is_stored_and_resolved(tmp_path: Path) -> None:
    server, _, _ = make(tmp_path)
    table = text_json(call(server, "mark_candidate_points", {"camera": "front", "spacing_px": 120, "surface_height_m": 0.04}))
    assert table["surface_height_m"] == 0.04
    for p in table["points"]:
        assert p["ground_base_link"]["z"] == 0.04
        back = front_pixel((p["ground_base_link"]["x"], p["ground_base_link"]["y"], 0.04))
        assert back == pytest.approx((p["u"], p["v"]), abs=0.5)
    resolved = call(server, "resolve_candidate", {"set_id": table["set_id"], "n": 1}).structured_content
    assert resolved["point"] == table["points"][0] and resolved["surface_height_m"] == 0.04
    flat = text_json(call(server, "mark_candidate_points", {"camera": "front", "spacing_px": 120}))
    assert flat["surface_height_m"] == 0.0
