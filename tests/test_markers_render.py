"""Tests for scripts/markers/views.py and scripts/markers_render.py: the eight view directions, the camera, the grip fit
scene, the render config and (only where pyvista and the OSMesa libs are installed) the nine PNGs."""

import json
from pathlib import Path
from typing import Any

import numpy as np
import pytest
import trimesh
import yaml
from pydantic import ValidationError

from scripts.markers.jaw_fit import OPEN_AXIS, TOOL_AXIS, fixed_jaw_face_mm, load_gripper, moving_jaw
from scripts.markers.views import VIEW_DIRECTIONS, camera_for, grip_scene
from scripts.markers_render import load_render_config, render_all

REPO_ROOT = Path(__file__).resolve().parents[1]
SHIPPED_RENDER_YAML = REPO_ROOT / "scripts" / "markers_render.yaml"
ARM_URDF = REPO_ROOT / "nodes" / "web_ui" / "urdf" / "so101_arm.urdf"
MARKERS_DIR = REPO_ROOT / "markers"
ARM_NAME = "charuco-arm-6x5-28mm"
GRIPPER = load_gripper(ARM_URDF)
META: dict[str, Any] = json.loads((MARKERS_DIR / f"{ARM_NAME}.json").read_text())
ARM_BODIES = (
    trimesh.load(MARKERS_DIR / f"{ARM_NAME}-base.stl"),
    trimesh.load(MARKERS_DIR / f"{ARM_NAME}-modules.stl"),
)
MIN_PNG_BYTES = 1000
VIEW_COUNT = 8
PNG_COUNT = 9
POST_GRIP_MM = 30.0  # grip length behind the shoulder face (scripts/markers_generate.yaml handle length_mm)
FACE_TOL_MM = 0.5


def test_eight_views_cover_axes_and_two_isometric() -> None:
    assert len(VIEW_DIRECTIONS) == VIEW_COUNT
    assert {k for k in VIEW_DIRECTIONS if not k.startswith("iso")} == {"+x", "-x", "+y", "-y", "+z", "-z"}


def test_camera_looks_at_the_bounds_centre_from_the_direction() -> None:
    position, focal, _ = camera_for((0.0, 0.0, 1.0), np.array([[0, 0, 0], [10, 20, 4]], dtype=float))
    assert focal == pytest.approx((5.0, 10.0, 2.0)) and position[2] > 4.0


def test_camera_view_up_is_not_parallel_to_the_viewing_direction() -> None:
    for direction in VIEW_DIRECTIONS.values():
        position, focal, up = camera_for(direction, np.array([[0, 0, 0], [10, 20, 4]], dtype=float))
        look = np.asarray(focal) - np.asarray(position)
        assert np.linalg.norm(np.cross(look, up)) > 1e-6 * np.linalg.norm(look)


def test_grip_scene_poses_both_jaws_at_the_grip_angle() -> None:
    scene = grip_scene(GRIPPER, META, *ARM_BODIES)
    assert [name for _, name in scene] == ["base", "modules", "fixed_jaw", "moving_jaw"]
    moving = scene[3][0]
    expected = moving_jaw(GRIPPER, META["grip"]["grip_rad"])
    assert moving.vertices == pytest.approx(expected.vertices)
    assert scene[2][0].vertices == pytest.approx(GRIPPER.fixed.vertices)


def test_grip_scene_puts_the_post_between_the_jaws_and_the_plate_past_the_tips() -> None:
    """The right-angle post (decision 296): the plate lies past both jaw tips along TOOL_AXIS, and the post material
    behind the tips stands on the fixed jaw side (OPEN_AXIS from about the fixed jaw face up)."""
    scene = grip_scene(GRIPPER, META, *ARM_BODIES)
    base, moving = scene[0][0], scene[3][0]
    tool, opening = np.asarray(TOOL_AXIS), np.asarray(OPEN_AXIS)
    tips = max(float((GRIPPER.fixed.vertices @ tool).max()), float((moving.vertices @ tool).max()))
    depth = base.vertices @ tool
    assert depth.max() > tips
    post = base.vertices[depth < tips - 1.0]
    assert len(post) > 0
    assert float((post @ opening).min()) > fixed_jaw_face_mm(GRIPPER, (0.0, POST_GRIP_MM)) - FACE_TOL_MM


def test_render_config_rejects_unknown_keys(tmp_path: Path) -> None:
    config = yaml.safe_load(SHIPPED_RENDER_YAML.read_text())
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump({**config, "bogus": 1}))
    with pytest.raises(ValidationError):
        load_render_config(path)


def test_shipped_render_config_loads() -> None:
    config = load_render_config(SHIPPED_RENDER_YAML)
    assert config.board == ARM_NAME and config.size == (1200, 900)


def test_render_writes_nine_pngs(tmp_path: Path) -> None:
    pytest.importorskip("pyvista")  # only in the render group with the OSMesa libs
    paths = render_all(load_render_config(SHIPPED_RENDER_YAML).model_copy(update={"out_dir": str(tmp_path)}))
    assert len(paths) == PNG_COUNT and all(p.stat().st_size > MIN_PNG_BYTES for p in paths)
