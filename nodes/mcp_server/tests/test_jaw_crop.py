"""Jaw crop: the region between the jaws in the head D435 colour (front) image, projected from the arm model through
the head camera scene at the head pan/tilt that aims the camera at the FK tool point (design D8)."""

import dataclasses
import math

import cv2
import numpy as np
import pytest
from ros2_common.camera_geometry import CameraIntrinsics, project_point_to_pixel
from ros2_common.head_rgbd_calibration.arm_pose_check import D435_MIN_Z_M

from mcp_server.camera_scene import Scene, build_scene, camera_setup, head_parent_transform, offset_matrix
from mcp_server.config import GraspCheckSettings, McpServerConfig
from mcp_server.floor_guard import GRIPPER_LINK, TOOL_FRAME_LINK, JawModel
from mcp_server.head_kinematics import CAMERA_FRAME, HeadKinematics
from mcp_server.ik import ArmKinematics
from mcp_server.jaw_crop import JawCropError, crop_jpeg, jaw_crop, jaw_points, tool_point_in_base

from .camera_helpers import fresh_head
from .test_camera_scene_head import InfoSource

WIDTH, HEIGHT = 640, 480
JOINTS = {"shoulder_pan": 0.1, "elbow_flex": -0.4, "gripper": 0.5}  # tool point about 0.43 m ahead, 0.44 m up


@pytest.fixture(scope="module")
def cfg() -> McpServerConfig:
    return McpServerConfig()


@pytest.fixture(scope="module")
def kin(cfg: McpServerConfig) -> ArmKinematics:
    return ArmKinematics(
        cfg.arm.urdf_path,
        margin=0.05,
        joint_offsets=cfg.arm.joint_offsets_rad.model_dump(),
        tool_offset=tuple(cfg.arm.tool_offset_m.model_dump().values()),
    )


@pytest.fixture(scope="module")
def jaw(cfg: McpServerConfig, kin: ArmKinematics) -> JawModel:
    return JawModel(kin, cfg.arm.jaw_open_axis, cfg.arm.gripper_closed_rad)


@pytest.fixture(scope="module")
def head(cfg: McpServerConfig) -> HeadKinematics:
    return HeadKinematics(cfg.head.urdf_path)


def to_base(cfg: McpServerConfig, point: np.ndarray) -> np.ndarray:
    """Arm-base point in base_link through the configured arm mount."""
    assert cfg.arm.base_in_base_link is not None
    return (offset_matrix(cfg.arm.base_in_base_link) @ np.append(point, 1.0))[:3]


def aim(cfg: McpServerConfig, head: HeadKinematics, kin: ArmKinematics, jaw: JawModel) -> tuple[float, float]:
    """Head angles that aim the D435 colour axis at the FK tool point of JOINTS."""
    tool = jaw_points(kin, jaw, kin.link_frames(JOINTS), JOINTS["gripper"])["tool_point"]
    return head.look_at(to_base(cfg, tool))


def front_scene(cfg: McpServerConfig, pan: float, tilt: float) -> Scene:
    """Head camera scene (default D435 colour setup, CameraInfo intrinsics) at measured head angles."""
    setup = camera_setup(cfg, "head", source=InfoSource())
    return build_scene(setup, cfg, head_parent_transform(cfg, fresh_head(cfg, pan, tilt)))


@pytest.fixture(scope="module")
def scene(cfg: McpServerConfig, head: HeadKinematics, kin: ArmKinematics, jaw: JawModel) -> Scene:
    return front_scene(cfg, *aim(cfg, head, kin, jaw))


def pixels_of(
    cfg: McpServerConfig, head: HeadKinematics, scene: Scene, angles: tuple[float, float], points: np.ndarray
) -> list[tuple[float, float]]:
    """Independent projection of arm-base points: the head URDF optical frame, no image bounds check."""
    t_optical = head.link_pose(*angles, CAMERA_FRAME)
    intr = scene.intr
    big = CameraIntrinsics(
        width=10**6, height=10**6, fx=intr.fx, fy=intr.fy, cx=intr.cx, cy=intr.cy, distortion=intr.distortion
    )
    out: list[tuple[float, float]] = []
    for p in points:
        pixel = project_point_to_pixel(big, t_optical, to_base(cfg, p))
        assert pixel is not None
        out.append(pixel)
    return out


def settings(**kw: float) -> GraspCheckSettings:
    return GraspCheckSettings(**kw)


def run(scene: Scene, kin: ArmKinematics, jaw: JawModel, s: GraspCheckSettings) -> tuple[int, int, int, int]:
    return jaw_crop(scene, kin, jaw, JOINTS, s)


def test_jaw_points_match_the_self_collision_definition(kin: ArmKinematics, jaw: JawModel) -> None:
    frames = kin.link_frames(JOINTS)
    pts = jaw_points(kin, jaw, frames, JOINTS["gripper"])
    tool = frames[TOOL_FRAME_LINK]
    assert set(pts) == {"jaw_tip", "tool_point", "moving_jaw_tip"}
    np.testing.assert_allclose(pts["jaw_tip"], tool[:3, 3])
    np.testing.assert_allclose(pts["tool_point"], tool[:3, :3] @ kin.tool_offset + tool[:3, 3])
    np.testing.assert_allclose(pts["moving_jaw_tip"], jaw.moving_tip(frames[GRIPPER_LINK], JOINTS["gripper"]))


def test_tool_point_in_base_goes_through_the_arm_mount(cfg: McpServerConfig, kin: ArmKinematics, jaw: JawModel) -> None:
    assert cfg.arm.base_in_base_link is not None
    expected = to_base(cfg, jaw_points(kin, jaw, kin.link_frames(JOINTS), JOINTS["gripper"])["tool_point"])
    np.testing.assert_allclose(tool_point_in_base(kin, jaw, JOINTS, cfg.arm.base_in_base_link), expected)


def test_jaw_crop_from_the_front_camera(
    cfg: McpServerConfig, head: HeadKinematics, kin: ArmKinematics, jaw: JawModel, scene: Scene
) -> None:
    """The box comes from the head camera scene at the aimed head pose: it holds the tool point (at the image centre,
    the head aims there) and both jaw tips as projected through the head URDF optical frame."""
    angles = aim(cfg, head, kin, jaw)
    box = run(scene, kin, jaw, settings(crop_margin_px=0, crop_depth_m=0.0, crop_min_px=1))
    x0, y0, x1, y1 = box
    assert 0 <= x0 < x1 <= scene.intr.width and 0 <= y0 < y1 <= scene.intr.height
    pts = jaw_points(kin, jaw, kin.link_frames(JOINTS), JOINTS["gripper"])
    pixels = pixels_of(cfg, head, scene, angles, np.array(list(pts.values())))
    for u, v in pixels:
        assert x0 - 1 <= u <= x1 + 1 and y0 - 1 <= v <= y1 + 1
    tool_u, tool_v = pixels[list(pts).index("tool_point")]
    assert tool_u == pytest.approx(scene.intr.cx, abs=1.0) and tool_v == pytest.approx(scene.intr.cy, abs=1.0)


def test_box_includes_the_depth_points(
    cfg: McpServerConfig, head: HeadKinematics, kin: ArmKinematics, jaw: JawModel, scene: Scene
) -> None:
    box = run(scene, kin, jaw, settings(crop_margin_px=0, crop_depth_m=0.05, crop_min_px=1))
    shallow = run(scene, kin, jaw, settings(crop_margin_px=0, crop_depth_m=0.0, crop_min_px=1))
    assert (box[2] - box[0]) * (box[3] - box[1]) > (shallow[2] - shallow[0]) * (shallow[3] - shallow[1])
    depth_axis = kin.link_frame(JOINTS, GRIPPER_LINK)[:3, :3] @ jaw.depth_axis
    back = np.array(
        [p + 0.05 * depth_axis for p in jaw_points(kin, jaw, kin.link_frames(JOINTS), JOINTS["gripper"]).values()]
    )
    for u, v in pixels_of(cfg, head, scene, aim(cfg, head, kin, jaw), back):
        assert box[0] - 1 <= u <= box[2] + 1 and box[1] - 1 <= v <= box[3] + 1


def test_margin_is_added_on_every_side(kin: ArmKinematics, jaw: JawModel, scene: Scene) -> None:
    bare = run(scene, kin, jaw, settings(crop_margin_px=0, crop_min_px=1))
    padded = run(scene, kin, jaw, settings(crop_margin_px=10, crop_min_px=1))
    assert padded[0] == pytest.approx(bare[0] - 10, abs=1) and padded[1] == pytest.approx(bare[1] - 10, abs=1)
    assert padded[2] == pytest.approx(bare[2] + 10, abs=1) and padded[3] == pytest.approx(bare[3] + 10, abs=1)


def test_box_is_clamped_to_the_frame(kin: ArmKinematics, jaw: JawModel, scene: Scene) -> None:
    box = run(scene, kin, jaw, settings(crop_margin_px=512, crop_min_px=1))
    assert box == (0, 0, scene.intr.width, scene.intr.height)


def test_jaw_outside_the_image_is_refused(cfg: McpServerConfig, kin: ArmKinematics, jaw: JawModel) -> None:
    """The head looking away (pan 90 deg left, or behind the camera at pan 180 deg) does not see the jaws."""
    for pan in (math.pi / 2, math.pi):
        with pytest.raises(JawCropError, match="outside the head camera image"):
            run(front_scene(cfg, pan, 0.0), kin, jaw, settings(crop_min_px=1))


def test_jaw_inside_the_d435_min_z_is_refused(kin: ArmKinematics, jaw: JawModel, scene: Scene) -> None:
    """The same aimed view with the camera moved along its axis to 0.10 m from the tool point."""
    tool = scene.arm_to_ref(jaw_points(kin, jaw, kin.link_frames(JOINTS), JOINTS["gripper"])["tool_point"])
    assert tool is not None
    near = scene.t_ref_optical.copy()
    near[:3, 3] = tool - 0.10 * near[:3, 2]
    with pytest.raises(JawCropError, match=f"inside the D435 min-Z \\({D435_MIN_Z_M:g} m\\)"):
        run(dataclasses.replace(scene, t_ref_optical=near), kin, jaw, settings(crop_min_px=1))


def test_too_small_a_box_is_refused(kin: ArmKinematics, jaw: JawModel, scene: Scene) -> None:
    with pytest.raises(JawCropError, match="crop_min_px 500"):
        run(scene, kin, jaw, settings(crop_margin_px=0, crop_min_px=500))


def test_unknown_arm_mount_is_refused(kin: ArmKinematics, jaw: JawModel, scene: Scene) -> None:
    with pytest.raises(JawCropError, match="base_in_base_link"):
        run(dataclasses.replace(scene, t_arm_ref=None), kin, jaw, settings(crop_min_px=1))


def encode(w: int, h: int) -> bytes:
    img = np.zeros((h, w, 3), np.uint8)
    img[:, : w // 2] = (0, 0, 255)
    ok, buf = cv2.imencode(".jpg", img)
    assert ok
    return buf.tobytes()


def test_crop_jpeg_crops_and_resizes_to_the_longest_side() -> None:
    out = crop_jpeg(encode(WIDTH, HEIGHT), (100, 50, 500, 250), 200)
    img = cv2.imdecode(np.frombuffer(out, np.uint8), cv2.IMREAD_COLOR)
    assert out[:2] == b"\xff\xd8"
    assert img.shape[:2] == (100, 200)  # 400x200 box -> longest side 200


def test_crop_jpeg_does_not_upscale_a_small_box() -> None:
    out = crop_jpeg(encode(WIDTH, HEIGHT), (10, 10, 110, 60), 640)
    img = cv2.imdecode(np.frombuffer(out, np.uint8), cv2.IMREAD_COLOR)
    assert img.shape[:2] == (50, 100)


def test_crop_jpeg_content_comes_from_the_box() -> None:
    out = crop_jpeg(encode(WIDTH, HEIGHT), (400, 0, 600, 100), 200)
    img = cv2.imdecode(np.frombuffer(out, np.uint8), cv2.IMREAD_COLOR)
    assert img.mean() < 20  # right half of the source is black


def test_crop_jpeg_rejects_undecodable_input() -> None:
    with pytest.raises(ValueError):
        crop_jpeg(b"not a jpeg", (0, 0, 10, 10), 100)


def test_self_collision_fixed_points_reuse_jaw_points(
    cfg: McpServerConfig, kin: ArmKinematics, jaw: JawModel, monkeypatch: pytest.MonkeyPatch
) -> None:
    """SelfCollisionChecker.fixed_points takes its jaw points from jaw_crop.jaw_points (one definition)."""
    from mcp_server import self_collision

    marker = {name: np.full(3, float(i)) for i, name in enumerate(("jaw_tip", "tool_point", "moving_jaw_tip"))}
    monkeypatch.setattr(self_collision, "jaw_points", lambda *args, **kwargs: marker)
    checker = self_collision.SelfCollisionChecker.from_config(kin, cfg, jaw)
    out = checker.fixed_points(kin.link_frames(JOINTS), JOINTS)
    for name, point in marker.items():
        np.testing.assert_allclose(out[name], checker.to_base_link(point[None, :]))
