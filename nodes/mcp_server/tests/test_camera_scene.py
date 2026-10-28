"""Pixel <-> ground scene logic with synthetic cameras of known mounts."""

import math
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from mcp_server.camera_scene import (
    NOT_CALIBRATED,
    CameraInfoData,
    CameraNotCalibratedError,
    PixelError,
    base_to_map,
    build_scene,
    camera_setup,
    ground_report,
    load_intrinsics,
    parent_transform,
)
from mcp_server.config import McpServerConfig
from mcp_server.ik import ArmKinematics, load_joint_limits
from mcp_server.models import BasePose

from .camera_helpers import FRONT_MOUNT, HEIGHT, WIDTH, camera_config, fresh_head, front_ground_of_center
from .test_tools import FakeRobot

POSE = BasePose(frame="map", x=1.0, y=2.0, yaw=math.pi / 2)
D435_INFO_TOPIC = "/head/d435/color/camera_info"
# A D435 colour CameraInfo at 640x360 as the driver publishes it (factory calibration, plumb_bob, zero distortion).
D435_INFO = CameraInfoData(
    width=640,
    height=360,
    k=(455.2, 0.0, 321.7, 0.0, 455.4, 180.9, 0.0, 0.0, 1.0),
    d=(0.0, 0.0, 0.0, 0.0, 0.0),
    distortion_model="plumb_bob",
    stamp=12.5,
)


def camera_info_config(**intrinsics: object) -> McpServerConfig:
    """Config whose front camera takes its intrinsics from the D435 colour camera_info topic."""
    return McpServerConfig.model_validate(
        {
            "cameras": {
                "front": {"intrinsics": {"camera_info_topic": D435_INFO_TOPIC, **intrinsics}, "mount": FRONT_MOUNT}
            }
        }
    )


@pytest.fixture(scope="module")
def kin() -> ArmKinematics:
    cfg = McpServerConfig()
    assert load_joint_limits(cfg.arm.urdf_path)
    return ArmKinematics(cfg.arm.urdf_path, margin=0.05)


def test_load_intrinsics_none_and_hfov() -> None:
    assert load_intrinsics(None) is None
    cfg = camera_config()
    loaded = load_intrinsics(cfg.cameras.front.intrinsics)
    assert loaded is not None
    intr, approximate = loaded
    assert approximate is True
    assert intr.fx == pytest.approx((WIDTH / 2) / math.tan(math.radians(45.0)))


def test_load_intrinsics_from_calibration_file(tmp_path) -> None:
    path = tmp_path / "cam.yaml"
    path.write_text(
        "image_width: 640\nimage_height: 480\ncamera_matrix: {data: [500, 0, 320, 0, 505, 240, 0, 0, 1]}\n"
        "distortion_coefficients: {data: [0.1, 0, 0, 0, 0]}\n"
    )
    cfg = McpServerConfig.model_validate({"cameras": {"front": {"intrinsics": {"calibration_file": str(path)}}}})
    loaded = load_intrinsics(cfg.cameras.front.intrinsics)
    assert loaded is not None and loaded[1] is False and loaded[0].fy == 505.0


def test_intrinsics_from_camera_info(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    robot.camera_infos["front"] = D435_INFO
    cfg = camera_info_config()
    setup = camera_setup(cfg, "front", source=robot)
    intr = setup.intrinsics
    assert (intr.width, intr.height) == (640, 360)
    assert (intr.fx, intr.fy, intr.cx, intr.cy) == (455.2, 455.4, 321.7, 180.9)
    assert intr.distortion == [0.0] * 5
    assert setup.approximate is False and setup.mount is not None
    assert robot.camera_info_calls == ["front"]
    assert load_intrinsics(cfg.cameras.front.intrinsics, D435_INFO) == (intr, False)
    # The latest CameraInfo is used at each call.
    robot.camera_infos["front"] = replace(D435_INFO, k=(460.0, 0.0, 320.0, 0.0, 461.0, 180.0, 0.0, 0.0, 1.0))
    assert camera_setup(cfg, "front", source=robot).intrinsics.fx == 460.0
    # image_width / image_height still scale the factory intrinsics to the size of the tools' images.
    scaled = camera_setup(camera_info_config(image_width=1280, image_height=720), "front", source=robot).intrinsics
    assert (scaled.width, scaled.height, scaled.fx, scaled.cy) == (1280, 720, 920.0, 360.0)


def test_no_camera_info_is_not_calibrated(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)  # no CameraInfo received yet
    cfg = camera_info_config()
    settings = cfg.cameras.front.intrinsics
    message = NOT_CALIBRATED.format(camera="front")
    with pytest.raises(CameraNotCalibratedError) as exc:
        camera_setup(cfg, "front", source=robot)
    assert str(exc.value) == message
    with pytest.raises(CameraNotCalibratedError):
        camera_setup(cfg, "front")  # no CameraInfo source at all
    assert load_intrinsics(settings, None) is None
    invalid = (
        replace(D435_INFO, k=(0.0,) * 9),  # a zero K: the driver has no calibration
        replace(D435_INFO, distortion_model="rational_polynomial"),
        replace(D435_INFO, width=0, height=0),
    )
    for info in invalid:
        robot.camera_infos["front"] = info
        with pytest.raises(CameraNotCalibratedError) as exc:
            camera_setup(cfg, "front", source=robot)
        assert str(exc.value) == message
        assert load_intrinsics(settings, info) is None
    # A CameraInfo of another aspect ratio than the configured image size is not calibrated either.
    robot.camera_infos["front"] = D435_INFO
    with pytest.raises(CameraNotCalibratedError):
        camera_setup(camera_info_config(image_width=640, image_height=480), "front", source=robot)


def test_not_calibrated_errors_use_the_documented_message() -> None:
    for cfg in (McpServerConfig(), camera_config(front=False)):
        with pytest.raises(CameraNotCalibratedError) as exc:
            camera_setup(cfg, "front")
        assert str(exc.value) == NOT_CALIBRATED.format(camera="front")
    assert "set intrinsics and mount in mcp_server config (see README calibration)" in NOT_CALIBRATED


def test_mount_is_optional_when_not_needed() -> None:
    cfg = McpServerConfig.model_validate(
        {"cameras": {"gripper": {"intrinsics": {"hfov_deg": 60.0, "width": 640, "height": 480}}}}
    )
    setup = camera_setup(cfg, "gripper", need_mount=False)
    assert setup.mount is None and setup.approximate
    with pytest.raises(CameraNotCalibratedError):
        camera_setup(cfg, "gripper")


def test_front_scene_centre_pixel_hits_analytic_floor_point(kin: ArmKinematics) -> None:
    cfg = camera_config()
    setup = camera_setup(cfg, "front")
    scene = build_scene(setup, cfg, parent_transform(cfg, "front", kin, None, head=fresh_head(cfg)))
    assert scene.frame_name == "base_link" and scene.ground_z == 0.0
    hit = scene.ground_point(WIDTH / 2, HEIGHT / 2)
    ex, ey = front_ground_of_center()
    assert hit is not None and hit == pytest.approx([ex, ey, 0.0], abs=1e-6)


def test_front_scene_project_and_ground_round_trip(kin: ArmKinematics) -> None:
    cfg = camera_config()
    scene = build_scene(
        camera_setup(cfg, "front"), cfg, parent_transform(cfg, "front", kin, None, head=fresh_head(cfg))
    )
    for point in ([0.9, 0.2, 0.0], [0.6, -0.3, 0.0], [1.2, 0.0, 0.0]):
        uv = scene.project(np.array(point))
        assert uv is not None
        back = scene.ground_point(*uv)
        assert back is not None and back == pytest.approx(point, abs=1e-6)


def test_above_horizon_pixel_has_no_ground(kin: ArmKinematics) -> None:
    cfg = camera_config()
    scene = build_scene(
        camera_setup(cfg, "front"), cfg, parent_transform(cfg, "front", kin, None, head=fresh_head(cfg))
    )
    assert scene.ground_point(WIDTH / 2, 0.0) is not None  # pitch 1.0 rad, vfov/2 ~ 36.9 deg: top edge hits the floor
    steep = camera_config(front=True)
    steep.cameras.front.mount.pitch = 0.2
    scene2 = build_scene(
        camera_setup(steep, "front"), steep, parent_transform(steep, "front", kin, None, head=fresh_head(steep))
    )
    assert scene2.ground_point(WIDTH / 2, 0.0) is None


def test_gripper_scene_uses_the_current_joint_pose(kin: ArmKinematics) -> None:
    cfg = camera_config(front=False)
    joints = {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow_flex": 0.0, "wrist_flex": 0.0, "wrist_roll": 0.0}
    t0 = parent_transform(cfg, "gripper", kin, joints)
    moved = parent_transform(cfg, "gripper", kin, {**joints, "shoulder_lift": 0.8, "elbow_flex": 0.8})
    assert not np.allclose(t0, moved)
    scene = build_scene(camera_setup(cfg, "gripper"), cfg, moved)
    assert scene.frame_name == "arm_base" and scene.ground_z == pytest.approx(cfg.arm.floor_z_m)
    # A floor point the camera sees projects and comes back at floor height.
    hit = scene.ground_point(WIDTH / 2, HEIGHT - 5)
    if hit is not None:
        assert hit[2] == pytest.approx(cfg.arm.floor_z_m)


def test_gripper_parent_transform_requires_joints(kin: ArmKinematics) -> None:
    cfg = camera_config(front=False)
    with pytest.raises(ValueError, match="joint"):
        parent_transform(cfg, "gripper", kin, None)


def test_front_parent_on_base_link_is_identity_at_any_head_pose(kin: ArmKinematics) -> None:
    cfg = camera_config()  # synthetic mount on base_link, the head URDF root
    assert np.allclose(parent_transform(cfg, "front", kin, None, head=fresh_head(cfg, 0.5, 0.3)), np.eye(4))


def test_frame_conversions_with_and_without_arm_offset(kin: ArmKinematics) -> None:
    plain = camera_config()
    scene = build_scene(camera_setup(plain, "front"), plain, np.eye(4))
    assert scene.ref_to_base(np.array([1.0, 2.0, 0.0])) == pytest.approx([1.0, 2.0, 0.0])
    assert scene.arm_to_ref(np.array([0.1, 0.0, 0.0])) is None  # arm offset not configured
    offset = camera_config(arm_offset=True)
    scene2 = build_scene(camera_setup(offset, "front"), offset, np.eye(4))
    assert scene2.arm_to_ref(np.array([0.1, 0.2, 0.0])) == pytest.approx([0.2, 0.2, 0.165])
    assert scene2.ref_to_arm(np.array([0.2, 0.2, 0.165])) == pytest.approx([0.1, 0.2, 0.0])
    grip = build_scene(camera_setup(offset, "gripper"), offset, np.eye(4))
    assert grip.ref_to_base(np.array([0.0, 0.0, -0.165])) == pytest.approx([0.1, 0.0, 0.0])
    assert grip.base_to_ref(np.array([0.1, 0.0, 0.0])) == pytest.approx([0.0, 0.0, -0.165])


def test_base_to_map_rotates_by_the_robot_yaw() -> None:
    assert base_to_map(POSE, 1.0, 0.0) == pytest.approx((1.0, 3.0))
    assert base_to_map(POSE, 0.0, 1.0) == pytest.approx((0.0, 2.0))


def test_ground_report_front_view(kin: ArmKinematics) -> None:
    cfg = camera_config()
    scene = build_scene(camera_setup(cfg, "front"), cfg, np.eye(4))
    ex, _ = front_ground_of_center()
    rep = ground_report(scene, "front", WIDTH / 2, HEIGHT / 2, POSE)
    assert rep["camera"] == "front" and rep["pixel"] == {"u": WIDTH / 2, "v": HEIGHT / 2}
    g = rep["ground_base_link"]
    assert (g["x"], g["y"], g["z"]) == pytest.approx((ex, 0.0, 0.0), abs=1e-3)
    assert rep["distance_from_base_m"] == pytest.approx(ex, abs=2e-3)
    assert rep["bearing_deg"] == pytest.approx(0.0, abs=1e-3)
    assert rep["ground_map"] == pytest.approx({"x": 1.0, "y": 2.0 + ex}, abs=2e-3)
    assert rep["method"] and "approximate intrinsics" in rep["uncertainty_note"]


def test_ground_report_without_pose_omits_map(kin: ArmKinematics) -> None:
    cfg = camera_config()
    scene = build_scene(camera_setup(cfg, "front"), cfg, np.eye(4))
    assert "ground_map" not in ground_report(scene, "front", WIDTH / 2, HEIGHT / 2, None)


def test_ground_report_gripper_reports_arm_frame_and_base_when_offset(kin: ArmKinematics) -> None:
    joints = {"shoulder_pan": 0.0, "shoulder_lift": 0.9, "elbow_flex": 0.9, "wrist_flex": 1.2, "wrist_roll": 0.0}
    for offset in (False, True):
        cfg = camera_config(front=False, arm_offset=offset)
        scene = build_scene(camera_setup(cfg, "gripper"), cfg, parent_transform(cfg, "gripper", kin, joints))
        uv = scene.project(np.array([0.2, 0.0, cfg.arm.floor_z_m]))
        if uv is None:
            pytest.skip("synthetic pose does not see the point")
        rep = ground_report(scene, "gripper", uv[0], uv[1], POSE)
        assert rep["ground_arm_base"]["z"] == pytest.approx(cfg.arm.floor_z_m)
        assert rep["ground_arm_base"]["x"] == pytest.approx(0.2, abs=2e-3)
        assert ("ground_base_link" in rep) is offset
        assert ("ground_map" in rep) is offset


def test_ground_report_rejects_off_image_and_sky(kin: ArmKinematics) -> None:
    cfg = camera_config()
    scene = build_scene(camera_setup(cfg, "front"), cfg, np.eye(4))
    with pytest.raises(PixelError, match="outside the image"):
        ground_report(scene, "front", WIDTH + 5, 10, None)
    flat = camera_config()
    flat.cameras.front.mount.pitch = 0.1
    scene2 = build_scene(camera_setup(flat, "front"), flat, np.eye(4))
    with pytest.raises(PixelError, match="floor"):
        ground_report(scene2, "front", WIDTH / 2, 5, None)


def test_ground_point_intersects_the_plane_at_the_surface_height(kin: ArmKinematics) -> None:
    cfg = camera_config()
    scene = build_scene(camera_setup(cfg, "front"), cfg, np.eye(4))
    box_top = np.array([0.8, 0.1, 0.03])
    uv = scene.project(box_top)
    assert uv is not None
    on_box = scene.ground_point(*uv, surface_height_m=0.03)
    assert on_box is not None and on_box == pytest.approx(box_top, abs=1e-6)
    on_floor = scene.ground_point(*uv)
    assert on_floor is not None and on_floor[2] == 0.0
    assert not np.allclose(on_floor[:2], on_box[:2], atol=1e-3)


def test_ground_point_below_the_floor_and_gripper_frame_offset(kin: ArmKinematics) -> None:
    joints = {"shoulder_pan": 0.0, "shoulder_lift": 0.9, "elbow_flex": 0.9, "wrist_flex": 1.2, "wrist_roll": 0.0}
    cfg = camera_config(front=False)
    scene = build_scene(camera_setup(cfg, "gripper"), cfg, parent_transform(cfg, "gripper", kin, joints))
    target = np.array([0.2, 0.0, cfg.arm.floor_z_m - 0.10])
    uv = scene.project(target)
    if uv is None:
        pytest.skip("synthetic pose does not see the point")
    hit = scene.ground_point(*uv, surface_height_m=-0.10)
    assert hit is not None and hit == pytest.approx(target, abs=1e-6)


def test_ground_report_reports_the_surface_height_used(kin: ArmKinematics) -> None:
    cfg = camera_config()
    scene = build_scene(camera_setup(cfg, "front"), cfg, np.eye(4))
    uv = scene.project(np.array([0.8, 0.0, 0.05]))
    assert uv is not None
    rep = ground_report(scene, "front", uv[0], uv[1], None, surface_height_m=0.05)
    assert rep["surface_height_m"] == 0.05
    assert rep["ground_base_link"] == pytest.approx({"x": 0.8, "y": 0.0, "z": 0.05}, abs=2e-3)
    assert ground_report(scene, "front", WIDTH / 2, HEIGHT / 2, None)["surface_height_m"] == 0.0
