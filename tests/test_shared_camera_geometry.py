"""Tests for shared/ros2_common/camera_geometry.py (projection, ground intersection, mount solver)."""

import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

_repo_root = Path(__file__).resolve().parent.parent
if (_repo_root / "shared" / "ros2_common").exists():
    sys.path.insert(0, str(_repo_root))

from shared.ros2_common.camera_geometry import (  # noqa: E402
    CameraIntrinsics,
    CameraModel,
    MountPose,
    camera_ray_in_frame,
    intersect_ground,
    optical_from_mount,
    pixel_to_ground,
    pixel_to_ray_camera,
    project_point_to_pixel,
    solve_mount_pose,
    undistort_pixel,
)

WIDTH = 640
HEIGHT = 480
HFOV_DEG = 90.0
# Camera 0.5 m up, looking forward and pitched down 30 degrees.
KNOWN_MOUNT = MountPose(parent_frame="base_link", x=0.1, y=0.02, z=0.5, roll=0.01, pitch=math.radians(30), yaw=0.03)
IDENTITY = np.eye(4)


def make_intr(distortion: list[float] | None = None) -> CameraIntrinsics:
    """Build intrinsics from a 90 degree hfov."""
    intr = CameraIntrinsics.from_hfov(WIDTH, HEIGHT, HFOV_DEG)
    if distortion:
        intr = intr.model_copy(update={"distortion": distortion})
    return intr


def test_from_hfov_fx() -> None:
    """fx = (w/2)/tan(hfov/2); square pixels; principal point at the centre."""
    intr = make_intr()
    assert intr.fx == pytest.approx(WIDTH / 2 / math.tan(math.radians(45)))
    assert intr.fy == pytest.approx(intr.fx)
    assert (intr.cx, intr.cy) == (WIDTH / 2, HEIGHT / 2)


def test_mount_to_matrix_identity_and_translation() -> None:
    """Zero RPY gives identity rotation; translation is copied."""
    m = MountPose(parent_frame="p", x=1, y=2, z=3, roll=0, pitch=0, yaw=0).to_matrix()
    assert m.shape == (4, 4)
    np.testing.assert_allclose(m[:3, :3], np.eye(3), atol=1e-12)
    np.testing.assert_allclose(m[:3, 3], [1, 2, 3])


def test_mount_yaw_rotates_x_to_y() -> None:
    """Fixed-axis RPY: yaw pi/2 maps body x to parent y."""
    m = MountPose(parent_frame="p", x=0, y=0, z=0, roll=0, pitch=0, yaw=math.pi / 2).to_matrix()
    np.testing.assert_allclose(m[:3, :3] @ [1, 0, 0], [0, 1, 0], atol=1e-12)


def test_optical_from_mount_convention() -> None:
    """Optical z is the body x (forward), optical x is body -y (right), optical y is body -z (down)."""
    t = optical_from_mount(IDENTITY)
    np.testing.assert_allclose(t[:3, :3] @ [0, 0, 1], [1, 0, 0], atol=1e-12)
    np.testing.assert_allclose(t[:3, :3] @ [1, 0, 0], [0, -1, 0], atol=1e-12)
    np.testing.assert_allclose(t[:3, :3] @ [0, 1, 0], [0, 0, -1], atol=1e-12)


def test_camera_model_calibrated() -> None:
    """calibrated needs both intrinsics and mount."""
    assert not CameraModel(name="c", intrinsics=None, mount=None).calibrated
    assert not CameraModel(name="c", intrinsics=make_intr(), mount=None).calibrated
    assert CameraModel(name="c", intrinsics=make_intr(), mount=KNOWN_MOUNT).calibrated


def test_center_pixel_ray_is_optical_z() -> None:
    """The principal point looks along +z of the optical frame."""
    intr = make_intr()
    np.testing.assert_allclose(pixel_to_ray_camera(intr, intr.cx, intr.cy), [0, 0, 1], atol=1e-12)


def test_ray_is_unit_and_right_down() -> None:
    """A pixel right of and below centre has positive x and y."""
    intr = make_intr()
    ray = pixel_to_ray_camera(intr, intr.cx + 100, intr.cy + 50)
    assert np.linalg.norm(ray) == pytest.approx(1.0)
    assert ray[0] > 0 and ray[1] > 0


def test_undistort_without_distortion() -> None:
    """No distortion: normalized coordinates are (u-cx)/fx, (v-cy)/fy."""
    intr = make_intr()
    x, y = undistort_pixel(intr, intr.cx + intr.fx * 0.25, intr.cy - intr.fy * 0.1)
    assert (x, y) == pytest.approx((0.25, -0.1))


def test_undistort_inverts_plumb_bob() -> None:
    """Distorting a known normalized point then undistorting recovers it."""
    k1, k2, p1, p2, k3 = -0.2, 0.05, 0.001, -0.002, 0.0
    intr = make_intr([k1, k2, p1, p2, k3])
    xn, yn = 0.3, -0.2
    r2 = xn * xn + yn * yn
    radial = 1 + k1 * r2 + k2 * r2**2 + k3 * r2**3
    xd = xn * radial + 2 * p1 * xn * yn + p2 * (r2 + 2 * xn * xn)
    yd = yn * radial + p1 * (r2 + 2 * yn * yn) + 2 * p2 * xn * yn
    u, v = intr.fx * xd + intr.cx, intr.fy * yd + intr.cy
    assert undistort_pixel(intr, u, v) == pytest.approx((xn, yn), abs=1e-6)


def test_intersect_ground_basic() -> None:
    """A ray from 1 m up pointing 45 degrees down hits the floor 1 m ahead."""
    p = intersect_ground(np.array([0, 0, 1.0]), np.array([1, 0, -1.0]), 0.0)
    np.testing.assert_allclose(p, [1, 0, 0], atol=1e-12)


def test_intersect_ground_parallel_and_behind() -> None:
    """Parallel rays and rays pointing away from the plane give None."""
    assert intersect_ground(np.array([0, 0, 1.0]), np.array([1, 0, 0.0]), 0.0) is None
    assert intersect_ground(np.array([0, 0, 1.0]), np.array([1, 0, 1.0]), 0.0) is None


def test_camera_ray_in_frame_origin() -> None:
    """The ray origin is the camera translation in the frame."""
    t = KNOWN_MOUNT.to_matrix()
    origin, direction = camera_ray_in_frame(optical_from_mount(t), np.array([0, 0, 1.0]))
    np.testing.assert_allclose(origin, [0.1, 0.02, 0.5])
    assert np.linalg.norm(direction) == pytest.approx(1.0)


@pytest.mark.parametrize("distortion", [[], [-0.15, 0.03, 0.0005, -0.0005, 0.0]])
def test_pixel_ground_roundtrip(distortion: list[float]) -> None:
    """pixel -> ground -> pixel returns to the start within 1e-6 px."""
    intr = make_intr(distortion)
    t_opt = optical_from_mount(KNOWN_MOUNT.to_matrix())
    for u, v in [(320, 400), (100, 380), (500, 300), (320, 260)]:
        ground = pixel_to_ground(intr, t_opt, u, v, 0.0)
        assert ground is not None
        back = project_point_to_pixel(intr, t_opt, np.array([ground[0], ground[1], 0.0]))
        assert back is not None
        assert back == pytest.approx((u, v), abs=1e-6)


def test_pixel_above_horizon_gives_none() -> None:
    """A pixel whose ray points up never reaches the ground."""
    intr = make_intr()
    t_opt = optical_from_mount(KNOWN_MOUNT.to_matrix())
    assert pixel_to_ground(intr, t_opt, 320, 5, 0.0) is None


def test_project_behind_camera_and_outside_image() -> None:
    """Points behind the camera or off-image project to None."""
    intr = make_intr()
    t_opt = optical_from_mount(IDENTITY)
    assert project_point_to_pixel(intr, t_opt, np.array([-1.0, 0, 0])) is None
    assert project_point_to_pixel(intr, t_opt, np.array([1.0, 5.0, 0])) is None
    assert project_point_to_pixel(intr, t_opt, np.array([1.0, 0, 0])) == pytest.approx((WIDTH / 2, HEIGHT / 2))


def test_from_calibration_yaml(tmp_path: Path) -> None:
    """Loads a camera_calibration camera_info yaml."""
    data = {
        "image_width": 640,
        "image_height": 480,
        "camera_name": "cam",
        "camera_matrix": {"rows": 3, "cols": 3, "data": [500.0, 0, 321.0, 0, 510.0, 241.0, 0, 0, 1]},
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {"rows": 1, "cols": 5, "data": [-0.1, 0.02, 0.001, 0.002, 0.0]},
    }
    path = tmp_path / "cam.yaml"
    path.write_text(yaml.safe_dump(data))
    intr = CameraIntrinsics.from_calibration_yaml(path)
    assert (intr.width, intr.height, intr.fx, intr.fy, intr.cx, intr.cy) == (640, 480, 500.0, 510.0, 321.0, 241.0)
    assert intr.distortion == [-0.1, 0.02, 0.001, 0.002, 0.0]


def make_samples(intr: CameraIntrinsics, mount: MountPose, noise_px: float, seed: int = 0) -> list:
    """Synthesize (T_frame_parent, pixel, ground) samples over several parent poses and floor points."""
    rng = np.random.default_rng(seed)
    samples = []
    for yaw in (-0.3, 0.0, 0.3):
        for dx in (0.0, 0.1):
            t_parent = np.eye(4)
            c, s = math.cos(yaw), math.sin(yaw)
            t_parent[:2, :2] = [[c, -s], [s, c]]
            t_parent[:3, 3] = [dx, 0.05 * yaw, 0.0]
            t_opt = t_parent @ optical_from_mount(mount.to_matrix())
            for gx, gy in [(1.0, -0.4), (1.0, 0.4), (1.5, 0.0), (0.8, 0.0), (1.3, 0.3), (1.2, -0.3)]:
                px = project_point_to_pixel(intr, t_opt, np.array([gx, gy, 0.0]))
                if px is None:
                    continue
                uv = (px[0] + rng.normal(0, noise_px), px[1] + rng.normal(0, noise_px))
                samples.append((t_parent, uv, (gx, gy, 0.0)))
    return samples


def test_solver_recovers_known_mount() -> None:
    """Noisy synthetic samples recover the mount to < 5 mm / 0.5 deg with rms < 0.5 px."""
    intr = make_intr()
    samples = make_samples(intr, KNOWN_MOUNT, noise_px=0.3)
    assert len(samples) >= 20
    initial = MountPose(parent_frame="base_link", x=0.0, y=0.0, z=0.4, roll=0.0, pitch=math.radians(20), yaw=0.0)
    solved, rms = solve_mount_pose(samples, intr, initial)
    assert rms < 0.5
    assert solved.parent_frame == "base_link"
    for name in ("x", "y", "z"):
        assert abs(getattr(solved, name) - getattr(KNOWN_MOUNT, name)) < 0.005
    for name in ("roll", "pitch", "yaw"):
        assert abs(math.degrees(getattr(solved, name) - getattr(KNOWN_MOUNT, name))) < 0.5


def test_solver_needs_three_samples() -> None:
    """Fewer than 3 samples raises ValueError."""
    intr = make_intr()
    samples = make_samples(intr, KNOWN_MOUNT, noise_px=0.0)[:2]
    with pytest.raises(ValueError):
        solve_mount_pose(samples, intr, KNOWN_MOUNT)


def test_solve_camera_mount_script(tmp_path: Path) -> None:
    """The CLI script loads a samples JSON and prints the solved mount as YAML with the rms."""
    intr = make_intr()
    samples = make_samples(intr, KNOWN_MOUNT, noise_px=0.0)
    payload = {
        "camera": "gripper",
        "intrinsics": intr.model_dump(),
        "initial": {"parent_frame": "base_link", "x": 0, "y": 0, "z": 0.4, "roll": 0, "pitch": 0.4, "yaw": 0},
        "samples": [{"T_frame_parent": t.tolist(), "pixel": list(uv), "ground": list(g)} for t, uv, g in samples],
    }
    path = tmp_path / "samples.json"
    path.write_text(json.dumps(payload))
    out = subprocess.run(
        [sys.executable, str(_repo_root / "scripts" / "solve_camera_mount.py"), str(path), "--camera", "gripper"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    solved = yaml.safe_load(out)["gripper"]["mount"]
    assert solved["parent_frame"] == "base_link"
    assert solved["z"] == pytest.approx(KNOWN_MOUNT.z, abs=1e-3)
    assert "# rms_px:" in out
