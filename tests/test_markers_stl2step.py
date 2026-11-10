"""Tests for scripts/markers/stl2step.py (skipped where OpenCascade cannot load, e.g. without libGL)."""

from pathlib import Path

import pytest
import trimesh

pytest.importorskip("OCP.OCP", reason="cadquery-ocp not installed or libGL missing")

from scripts.markers.stl2step import stl_to_step  # noqa: E402

CUBE_MM = 10.0
CUBE_FACES = 6


def test_cube_triangles_merge_into_six_step_faces(tmp_path: Path) -> None:
    """A triangulated cube becomes a STEP solid with one planar face per side."""
    stl = tmp_path / "cube.stl"
    step = tmp_path / "cube.step"
    trimesh.creation.box(extents=(CUBE_MM, CUBE_MM, CUBE_MM)).export(stl)
    before, after = stl_to_step(str(stl), str(step))
    assert before == 12
    assert after == CUBE_FACES
    assert step.read_text().startswith("ISO-10303-21")
