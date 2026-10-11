"""Jaw crop: the gripper camera region between the jaws, projected from the arm model."""

import math
from pathlib import Path

import cv2
import numpy as np
import pytest
from ros2_common.camera_geometry import CameraIntrinsics, MountPose, optical_from_mount, project_point_to_pixel

from mcp_server.camera_scene import CameraNotCalibratedError, CameraSetup, camera_setup
from mcp_server.config import GraspCheckSettings, McpServerConfig
from mcp_server.floor_guard import GRIPPER_LINK, TOOL_FRAME_LINK, JawModel
from mcp_server.ik import ArmKinematics
from mcp_server.jaw_crop import crop_jpeg, jaw_crop, jaw_points

CALIBRATION = Path(__file__).resolve().parent.parent / "calibration" / "gripper_camera.yaml"
WIDTH, HEIGHT = 640, 480
JOINTS = {"shoulder_pan": 0.1, "elbow_flex": -0.4, "gripper": 0.5}
CAMERA_OFFSET_IN_GRIPPER_LINK = np.array([0.02, -0.10, 0.10])  # metres, camera position relative to the tool point


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


def look_at_mount(position: np.ndarray, target: np.ndarray, extra_yaw: float = 0.0) -> MountPose:
    """Mount in gripper_link whose body x axis (forward) points from position at target."""
    d = target - position
    yaw = math.atan2(d[1], d[0]) + extra_yaw
    pitch = math.atan2(-d[2], math.hypot(d[0], d[1]))
    return MountPose(
        parent_frame=GRIPPER_LINK, x=position[0], y=position[1], z=position[2], roll=0.0, pitch=pitch, yaw=yaw
    )


def make_setup(kin: ArmKinematics, jaw: JawModel, extra_yaw: float = 0.0) -> CameraSetup:
    intr = CameraIntrinsics.from_calibration_yaml(str(CALIBRATION))
    target = jaw.tip_in_gripper_link(jaw.closed_rad)
    mount = look_at_mount(target + CAMERA_OFFSET_IN_GRIPPER_LINK, target, extra_yaw)
    return CameraSetup("gripper", intr, False, GRIPPER_LINK, mount)


def pixels_of(setup: CameraSetup, kin: ArmKinematics, points: np.ndarray) -> list[tuple[float, float]]:
    """Independent projection of arm-base points with the calibration's own intrinsics (no image bounds check)."""
    assert setup.mount is not None
    t_optical = optical_from_mount(kin.link_frame(JOINTS, GRIPPER_LINK) @ setup.mount.to_matrix())
    big = CameraIntrinsics(
        width=10**6,
        height=10**6,
        fx=setup.intrinsics.fx,
        fy=setup.intrinsics.fy,
        cx=setup.intrinsics.cx,
        cy=setup.intrinsics.cy,
        distortion=setup.intrinsics.distortion,
    )
    out = [project_point_to_pixel(big, t_optical, p) for p in points]
    assert all(o is not None for o in out)
    return out  # type: ignore[return-value]


def settings(**kw: float) -> GraspCheckSettings:
    return GraspCheckSettings(**kw)


def run(setup, kin, jaw, s, w=WIDTH, h=HEIGHT, joints=None):
    return jaw_crop(setup, kin, jaw, JOINTS if joints is None else joints, w, h, s)


def test_jaw_points_match_the_self_collision_definition(kin: ArmKinematics, jaw: JawModel) -> None:
    frames = kin.link_frames(JOINTS)
    pts = jaw_points(kin, jaw, frames, JOINTS["gripper"])
    tool = frames[TOOL_FRAME_LINK]
    assert set(pts) == {"jaw_tip", "tool_point", "moving_jaw_tip"}
    np.testing.assert_allclose(pts["jaw_tip"], tool[:3, 3])
    np.testing.assert_allclose(pts["tool_point"], tool[:3, :3] @ kin.tool_offset + tool[:3, 3])
    np.testing.assert_allclose(pts["moving_jaw_tip"], jaw.moving_tip(frames[GRIPPER_LINK], JOINTS["gripper"]))


def test_box_contains_tool_point_and_both_jaw_tips(kin: ArmKinematics, jaw: JawModel) -> None:
    setup = make_setup(kin, jaw)
    box = run(setup, kin, jaw, settings(crop_margin_px=0, crop_depth_m=0.0, crop_min_px=1))
    assert box is not None
    x0, y0, x1, y1 = box
    pts = jaw_points(kin, jaw, kin.link_frames(JOINTS), JOINTS["gripper"])
    for u, v in pixels_of(setup, kin, np.array(list(pts.values()))):
        assert x0 <= u <= x1 and y0 <= v <= y1


def test_box_includes_the_depth_points(kin: ArmKinematics, jaw: JawModel) -> None:
    setup = make_setup(kin, jaw)
    s = settings(crop_margin_px=0, crop_depth_m=0.05, crop_min_px=1)
    box = run(setup, kin, jaw, s)
    shallow = run(setup, kin, jaw, settings(crop_margin_px=0, crop_depth_m=0.0, crop_min_px=1))
    assert box is not None and shallow is not None
    assert (box[2] - box[0]) * (box[3] - box[1]) > (shallow[2] - shallow[0]) * (shallow[3] - shallow[1])
    depth_axis = kin.link_frame(JOINTS, GRIPPER_LINK)[:3, :3] @ jaw.depth_axis
    back = np.array([p + 0.05 * depth_axis for p in jaw_points(kin, jaw, kin.link_frames(JOINTS), JOINTS["gripper"]).values()])
    for u, v in pixels_of(setup, kin, back):
        assert box[0] <= u <= box[2] and box[1] <= v <= box[3]


def test_margin_is_added_on_every_side(kin: ArmKinematics, jaw: JawModel) -> None:
    setup = make_setup(kin, jaw)
    bare = run(setup, kin, jaw, settings(crop_margin_px=0, crop_min_px=1))
    padded = run(setup, kin, jaw, settings(crop_margin_px=10, crop_min_px=1))
    assert bare is not None and padded is not None
    assert padded[0] == pytest.approx(bare[0] - 10, abs=1) and padded[1] == pytest.approx(bare[1] - 10, abs=1)
    assert padded[2] == pytest.approx(bare[2] + 10, abs=1) and padded[3] == pytest.approx(bare[3] + 10, abs=1)


def test_box_is_clamped_to_the_frame(kin: ArmKinematics, jaw: JawModel) -> None:
    setup = make_setup(kin, jaw)
    box = run(setup, kin, jaw, settings(crop_margin_px=512, crop_min_px=1))
    assert box == (0, 0, WIDTH, HEIGHT)
    # the unclamped box is about (319, 123, 511, 264): a 400x200 frame cuts its right and bottom edges
    small = run(setup, kin, jaw, settings(crop_margin_px=0, crop_min_px=1), w=400, h=200)
    assert small is not None
    assert small[2] == 400 and small[3] == 200 and 0 <= small[0] < 400 and 0 <= small[1] < 200


def test_none_when_uncalibrated(kin: ArmKinematics, jaw: JawModel, cfg: McpServerConfig) -> None:
    with pytest.raises(CameraNotCalibratedError):
        camera_setup(cfg, "gripper")
    assert run(None, kin, jaw, settings()) is None
    setup = make_setup(kin, jaw)
    no_mount = CameraSetup(setup.name, setup.intrinsics, setup.approximate, setup.parent_frame, None)
    assert run(no_mount, kin, jaw, settings()) is None


def test_none_when_nothing_is_in_front_of_the_camera(kin: ArmKinematics, jaw: JawModel) -> None:
    setup = make_setup(kin, jaw, extra_yaw=math.pi)
    assert run(setup, kin, jaw, settings(crop_min_px=1)) is None


def test_none_when_the_box_is_too_small(kin: ArmKinematics, jaw: JawModel) -> None:
    setup = make_setup(kin, jaw)
    assert run(setup, kin, jaw, settings(crop_margin_px=0, crop_min_px=500)) is None


def test_none_when_the_box_is_off_the_frame(kin: ArmKinematics, jaw: JawModel) -> None:
    setup = make_setup(kin, jaw)
    box = run(setup, kin, jaw, settings(crop_margin_px=0, crop_min_px=1), w=10, h=10)
    assert box is None or (box[2] - box[0] >= 1 and box[3] - box[1] >= 1)


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


def test_self_collision_fixed_points_reuse_jaw_points(cfg: McpServerConfig, kin: ArmKinematics, jaw: JawModel, monkeypatch: pytest.MonkeyPatch) -> None:
    """SelfCollisionChecker.fixed_points takes its jaw points from jaw_crop.jaw_points (one definition)."""
    from mcp_server import self_collision

    marker = {name: np.full(3, float(i)) for i, name in enumerate(("jaw_tip", "tool_point", "moving_jaw_tip"))}
    monkeypatch.setattr(self_collision, "jaw_points", lambda *args, **kwargs: marker)
    checker = self_collision.SelfCollisionChecker.from_config(kin, cfg, jaw)
    out = checker.fixed_points(kin.link_frames(JOINTS), JOINTS)
    for name, point in marker.items():
        np.testing.assert_allclose(out[name], checker.to_base_link(point[None, :]))
