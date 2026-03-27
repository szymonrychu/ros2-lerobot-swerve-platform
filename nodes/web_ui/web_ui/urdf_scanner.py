"""Scan URDF directory and parse link/joint/mesh metadata for /api/urdf/status."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Literal

import structlog
from pydantic import BaseModel

log = structlog.get_logger(__name__)


class UrdfFileStatus(BaseModel):
    """Status of a single URDF file in the URDF directory."""

    name: str
    path: str
    status: Literal["ok", "parse_error", "missing"]
    link_count: int = 0
    joint_count: int = 0
    has_meshes: bool = False
    mesh_status: dict[str, Literal["ok", "missing"]] = {}
    error: str | None = None


def scan_urdf_directory(urdf_dir: Path) -> list[UrdfFileStatus]:
    """Scan *urdf_dir* for URDF files and return their status.

    Parses each .urdf file with ElementTree to count links/joints and check
    referenced mesh files. Non-parseable files are reported with status='parse_error'.

    Args:
        urdf_dir: Directory containing .urdf files and mesh subdirectories.

    Returns:
        list[UrdfFileStatus]: One entry per .urdf file found.
    """
    results: list[UrdfFileStatus] = []
    for urdf_path in sorted(urdf_dir.glob("*.urdf")):
        results.append(_parse_urdf(urdf_path, urdf_dir))
    log.info("urdf_scan_complete", files_found=len(results))
    return results


def _parse_urdf(urdf_path: Path, base_dir: Path) -> UrdfFileStatus:
    try:
        tree = ET.parse(urdf_path)
        root = tree.getroot()
    except ET.ParseError as exc:
        return UrdfFileStatus(
            name=urdf_path.name,
            path=urdf_path.name,
            status="parse_error",
            error=str(exc),
        )

    links = root.findall("link")
    joints = root.findall("joint")

    mesh_refs: list[str] = []
    for mesh_elem in root.iter("mesh"):
        filename = mesh_elem.get("filename", "")
        if filename:
            mesh_refs.append(filename)

    mesh_status: dict[str, Literal["ok", "missing"]] = {}
    for ref in mesh_refs:
        full = base_dir / ref
        mesh_status[ref] = "ok" if full.exists() else "missing"

    log.debug(
        "urdf_parsed",
        name=urdf_path.name,
        links=len(links),
        joints=len(joints),
        meshes=len(mesh_refs),
    )

    return UrdfFileStatus(
        name=urdf_path.name,
        path=urdf_path.name,
        status="ok",
        link_count=len(links),
        joint_count=len(joints),
        has_meshes=bool(mesh_refs),
        mesh_status=mesh_status,
    )
