"""Tests for URDF directory scanner."""

from __future__ import annotations

from pathlib import Path

from web_ui.urdf_scanner import scan_urdf_directory


def test_scan_finds_robot_urdf(urdf_dir: Path) -> None:
    results = scan_urdf_directory(urdf_dir)
    names = [r.name for r in results]
    assert "robot.urdf" in names


def test_scan_parses_links_and_joints(tmp_path: Path) -> None:
    d = tmp_path / "urdf"
    d.mkdir()
    (d / "test.urdf").write_text(
        """<?xml version="1.0"?>
<robot name="r">
  <link name="base_link"/>
  <link name="wheel"/>
  <joint name="j1" type="continuous">
    <parent link="base_link"/>
    <child link="wheel"/>
  </joint>
</robot>"""
    )
    results = scan_urdf_directory(d)
    assert len(results) == 1
    r = results[0]
    assert r.link_count == 2
    assert r.joint_count == 1
    assert r.status == "ok"


def test_scan_detects_missing_mesh(tmp_path: Path) -> None:
    d = tmp_path / "urdf"
    d.mkdir()
    (d / "arm.urdf").write_text(
        """<?xml version="1.0"?>
<robot name="arm">
  <link name="base">
    <visual><geometry><mesh filename="meshes/arm/base.stl"/></geometry></visual>
  </link>
</robot>"""
    )
    results = scan_urdf_directory(d)
    r = results[0]
    assert r.has_meshes is True
    assert "meshes/arm/base.stl" in r.mesh_status
    assert r.mesh_status["meshes/arm/base.stl"] == "missing"


def test_scan_empty_directory(tmp_path: Path) -> None:
    d = tmp_path / "urdf"
    d.mkdir()
    assert scan_urdf_directory(d) == []
