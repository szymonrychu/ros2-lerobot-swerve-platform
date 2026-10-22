"""The front (head RGB) camera scene follows the measured pan/tilt head pose (design D11)."""

import json
import math
from pathlib import Path

import numpy as np
import pytest
import yaml
from mcp.server.mcpserver.exceptions import ToolError
from pydantic import ValidationError
from ros2_common.head_rgbd_calibration.urdf_text import Origin, head_chain_json

from mcp_server.camera_scene import Scene, build_scene, camera_setup, parent_transform
from mcp_server.config import REPO_ROOT, McpServerConfig
from mcp_server.head import AppliedHeadKinematics, HeadController
from mcp_server.head_kinematics import CAMERA_FRAME, HeadKinematics
from mcp_server.ik import ArmKinematics

from .camera_helpers import HEIGHT, HFOV, WIDTH, fresh_head
from .fakes import FakeHeadBackend
from .test_camera_tools import make
from .test_tools import call

# URDF head_link -> head_camera_link origin (head.urdf head_camera_joint): the deployed front camera mount.
HEAD_MOUNT = {"parent_frame": "head_link", "x": 0.02, "y": -0.025, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
# head.urdf: head_mount_joint z 0.31 + head_tilt origin z 0.04 = tilt axis height above the floor.
TILT_AXIS_Z = 0.35
TILT_AXIS_X = 0.15
CAMERA_FORWARD = 0.02
URDF_ATOL = 1e-4  # the URDF optical rotation uses 1.5708 for pi/2


def head_config(**front: object) -> McpServerConfig:
    """Config whose front camera sits on head_link with the URDF mount and hfov intrinsics."""
    cam: dict[str, object] = {"intrinsics": HFOV, "mount": HEAD_MOUNT, **front}
    return McpServerConfig.model_validate({"cameras": {"front": cam}})


def head_at(cfg: McpServerConfig, pan_deg: float, tilt_deg: float) -> tuple[HeadController, FakeHeadBackend]:
    """A head controller whose fresh /head/joint_states report the given angles."""
    head = fresh_head(cfg, math.radians(pan_deg), math.radians(tilt_deg))
    assert isinstance(head.backend, FakeHeadBackend)
    return head, head.backend


@pytest.fixture(scope="module")
def kin() -> ArmKinematics:
    return ArmKinematics(McpServerConfig().arm.urdf_path, margin=0.05)


def front_scene(cfg: McpServerConfig, kin: ArmKinematics, head: HeadController) -> Scene:
    return build_scene(camera_setup(cfg, "front"), cfg, parent_transform(cfg, "front", kin, None, head=head))


def test_front_scene_at_pan_0_tilt_0_equals_the_urdf_camera_pose(kin: ArmKinematics) -> None:
    cfg = head_config()
    head, _ = head_at(cfg, 0.0, 0.0)
    expected = HeadKinematics(cfg.head.urdf_path).link_pose(0.0, 0.0, CAMERA_FRAME)
    assert np.allclose(front_scene(cfg, kin, head).t_ref_optical, expected, atol=URDF_ATOL)


def test_front_scene_follows_the_head_at_any_pose(kin: ArmKinematics) -> None:
    cfg = head_config()
    head, _ = head_at(cfg, -40.0, 20.0)
    expected = HeadKinematics(cfg.head.urdf_path).link_pose(math.radians(-40.0), math.radians(20.0), CAMERA_FRAME)
    assert np.allclose(front_scene(cfg, kin, head).t_ref_optical, expected, atol=URDF_ATOL)


def test_pan_plus_30_yaws_the_optical_axis_left_30_deg(kin: ArmKinematics) -> None:
    cfg = head_config()
    head, _ = head_at(cfg, 30.0, 0.0)
    axis = front_scene(cfg, kin, head).t_ref_optical[:3, 2]  # optical +z in base_link
    assert math.degrees(math.atan2(axis[1], axis[0])) == pytest.approx(30.0, abs=0.01)
    assert axis[2] == pytest.approx(0.0, abs=1e-4)


def test_tilt_plus_30_centre_pixel_hits_the_floor_about_0_6_m_ahead(kin: ArmKinematics) -> None:
    cfg = head_config()
    head, _ = head_at(cfg, 0.0, 30.0)
    scene = front_scene(cfg, kin, head)
    ground = scene.ground_point(WIDTH / 2, HEIGHT / 2)
    assert ground is not None
    tilt = math.radians(30.0)
    cam_x = TILT_AXIS_X + CAMERA_FORWARD * math.cos(tilt)
    cam_z = TILT_AXIS_Z - CAMERA_FORWARD * math.sin(tilt)
    camera = scene.t_ref_optical[:3, 3]
    assert camera[0] == pytest.approx(cam_x, abs=1e-6) and camera[2] == pytest.approx(cam_z, abs=1e-6)
    ahead = float(ground[0] - camera[0])
    assert ahead == pytest.approx(0.6, abs=0.02)  # 0.34 m / tan(30 deg) = 0.589 m in front of the camera
    assert ground[0] == pytest.approx(cam_x + cam_z / math.tan(tilt), abs=1e-3)
    assert ground[1] == pytest.approx(camera[1], abs=1e-3) and ground[2] == 0.0


def test_stale_head_joints_raise_naming_the_head_joint_states(kin: ArmKinematics) -> None:
    cfg = head_config()
    head, backend = head_at(cfg, 0.0, 0.0)
    backend.stale_after = backend.t - 2 * cfg.head.state_max_age_s
    with pytest.raises(ValueError, match="head joint states") as err:
        parent_transform(cfg, "front", kin, None, head=head)
    assert cfg.head.state_topic in str(err.value)


def test_missing_head_joints_raise(kin: ArmKinematics) -> None:
    cfg = head_config()
    head, backend = head_at(cfg, 0.0, 0.0)
    backend.no_samples = True
    with pytest.raises(ValueError, match="head joint states"):
        parent_transform(cfg, "front", kin, None, head=head)


def test_no_head_raises_instead_of_an_identity_pose(kin: ArmKinematics) -> None:
    with pytest.raises(ValueError, match="head joint states"):
        parent_transform(head_config(), "front", kin, None)


def test_front_parent_defaults_to_head_link() -> None:
    assert McpServerConfig().cameras.front.parent_frame == "head_link"


def test_config_rejects_a_front_parent_that_is_not_a_head_urdf_link() -> None:
    with pytest.raises(ValidationError, match="head URDF"):
        McpServerConfig.model_validate({"cameras": {"front": {"parent_frame": "gripper_link"}}})
    with pytest.raises(ValidationError, match="head URDF"):
        McpServerConfig.model_validate({"cameras": {"front": {"mount": {**HEAD_MOUNT, "parent_frame": "camera"}}}})


def test_config_rejects_a_mount_that_disagrees_with_the_front_parent() -> None:
    with pytest.raises(ValidationError, match="differs from parent_frame"):
        head_config(parent_frame="head_camera_link")


def test_pixel_to_ground_uses_the_measured_head_pan(tmp_path: Path) -> None:
    server, robot, _ = make(tmp_path, head_config())
    robot.head_backend.positions.update({"head_pan": math.radians(30.0), "head_tilt": math.radians(30.0)})
    data = call(server, "pixel_to_ground", {"camera": "front", "u": WIDTH / 2, "v": HEIGHT / 2}).structured_content
    bearing_from_pan_axis = math.degrees(
        math.atan2(data["ground_base_link"]["y"] - 0.10, data["ground_base_link"]["x"] - TILT_AXIS_X)
    )
    assert bearing_from_pan_axis == pytest.approx(30.0, abs=2.5)  # the camera sits 2.5 cm right of the pan axis


def test_pixel_to_ground_refuses_stale_head_joints(tmp_path: Path) -> None:
    server, robot, _ = make(tmp_path, head_config())
    robot.head_backend.stale_after = robot.head_backend.t - 10.0
    with pytest.raises(ToolError, match="head joint states"):
        call(server, "pixel_to_ground", {"camera": "front", "u": WIDTH / 2, "v": HEIGHT / 2})


def test_calibration_sample_stores_the_head_parent_pose(tmp_path: Path) -> None:
    cfg = head_config()
    server, robot, cfg = make(tmp_path, cfg)
    robot.head_backend.positions.update({"head_pan": math.radians(15.0), "head_tilt": math.radians(25.0)})
    call(
        server,
        "capture_calibration_sample",
        {"camera": "front", "u": 320.0, "v": 300.0, "ground_x": 0.8, "ground_y": 0.0, "ground_z": 0.0},
    )
    data = json.loads((cfg.cameras.calibration_dir / "front.json").read_text())
    assert data["parent_frame"] == "head_link"
    expected = HeadKinematics(cfg.head.urdf_path).link_pose(math.radians(15.0), math.radians(25.0), "head_link")
    assert np.allclose(np.array(data["samples"][0]["t_frame_parent"]), expected)


def deployed_config() -> McpServerConfig:
    """The mcp_server config of ansible/group_vars/client.yml, validated."""
    nodes = yaml.safe_load((REPO_ROOT / "ansible" / "group_vars" / "client.yml").read_text())["ros2_nodes"]
    return McpServerConfig.model_validate(yaml.safe_load(next(n for n in nodes if n["name"] == "mcp_server")["config"]))


def test_deployed_head_section_equals_the_code_defaults() -> None:
    assert deployed_config().head == McpServerConfig().head


def test_deployed_front_camera_is_the_applied_head_camera_joint_with_applied_intrinsics() -> None:
    front = deployed_config().cameras.front
    assert front.parent_frame == "head_camera_link" and front.mount is not None
    assert front.mount.parent_frame == "head_camera_link"
    assert np.allclose(front.mount.to_matrix(), np.eye(4))
    assert front.intrinsics is not None
    assert str(front.intrinsics.calibration_file) == (
        "/var/lib/ros2/head_rgbd_calibration/applied/head_camera_calibration.yaml"
    )


def test_zero_mount_on_head_camera_link_equals_the_old_head_link_mount(kin: ArmKinematics) -> None:
    """The deployed zero mount on head_camera_link poses the camera exactly as the old head_link mount did."""
    new = head_config(
        parent_frame="head_camera_link", mount={**HEAD_MOUNT, "parent_frame": "head_camera_link", "x": 0.0, "y": 0.0}
    )
    old = head_config()
    head, _ = head_at(new, 15.0, 25.0)
    assert np.allclose(
        front_scene(new, kin, head).t_ref_optical, front_scene(old, kin, head).t_ref_optical, atol=URDF_ATOL
    )


def test_front_pose_follows_applied_head_camera_joint(kin: ArmKinematics, tmp_path: Path) -> None:
    cfg = head_config(
        parent_frame="head_camera_link", mount={**HEAD_MOUNT, "parent_frame": "head_camera_link", "x": 0.0, "y": 0.0}
    )
    provider = AppliedHeadKinematics(cfg.head.urdf_path, tmp_path)
    backend = FakeHeadBackend({"head_pan": 0.0, "head_tilt": 0.0})
    head = HeadController(backend, provider, cfg.head)
    before = front_scene(cfg, kin, head).t_ref_optical.copy()
    origins = {"head_camera_joint": Origin((0.02, -0.025, 0.03), (0.0, 0.0, 0.0))}
    tmp_path.mkdir(exist_ok=True)
    (tmp_path / "head_chain.json").write_text(head_chain_json(origins, "test", "s2", "ok"))
    (tmp_path / "applied.json").write_text(json.dumps({"session": "s2"}))
    after = front_scene(cfg, kin, head).t_ref_optical
    assert (after[:3, 3] - before[:3, 3])[2] == pytest.approx(0.03, abs=1e-6)
