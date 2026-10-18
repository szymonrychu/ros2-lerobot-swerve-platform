"""Atomic on-disk store of named map bundles (no ROS imports).

A bundle directory holds the slam_toolbox serialized pose graph (``map.posegraph``, ``map.data``), the occupancy
grid (``occupancy.npz``) and ``bundle.json`` (:class:`BundleMeta`).
"""

from __future__ import annotations

import shutil
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

import numpy as np
from pydantic import BaseModel, ValidationError
from ros2_common.maps import MapSummary, Pose2D, validate_map_name
from ros2_common.scan_agreement import GridMap

POSEGRAPH_FILE = "map.posegraph"
POSEGRAPH_DATA_FILE = "map.data"
POSEGRAPH_BASE = "map"
OCCUPANCY_FILE = "occupancy.npz"
META_FILE = "bundle.json"
PHOTOS_DIR = "photos"
PHOTO_KIND = "photo"
TEMP_PREFIX = ".tmp-"
OLD_PREFIX = ".old-"
REQUIRED_SLAM_FILES = (POSEGRAPH_FILE, POSEGRAPH_DATA_FILE)


class BundleExists(Exception):
    """The map name is taken and overwrite was not requested."""


class BundleNotFound(Exception):
    """No bundle with that name."""


class BundleInvalid(Exception):
    """A bundle file is missing or unreadable; the message names the file."""


class MapInfo(BaseModel):
    """Grid geometry stored in ``bundle.json``."""

    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    origin_yaw: float


class BundleMeta(BaseModel):
    """Content of ``bundle.json``."""

    version: Literal[1]
    name: str
    saved_at: float
    note: str
    last_pose: Pose2D
    pois: list[dict[str, Any]]
    poi_revision: int
    map_info: MapInfo
    laser_frame: str


@dataclass(frozen=True)
class LoadedBundle:
    """A bundle read from disk.

    Attributes:
        meta: The bundle metadata.
        grid: The occupancy grid.
        posegraph_base: Path without extension to hand to slam_toolbox deserialize.
        photos_dir: The bundle's ``photos`` directory; None when the bundle has none.
    """

    meta: BundleMeta
    grid: GridMap
    posegraph_base: Path
    photos_dir: Path | None = None


def summarize(meta: BundleMeta) -> MapSummary:
    """Build the summary of a valid bundle.

    Args:
        meta: Bundle metadata (BundleMeta).

    Returns:
        MapSummary: Summary with ``valid=True``.
    """
    photos = sum(1 for entry in meta.pois if entry.get("kind") == PHOTO_KIND)
    return MapSummary(
        name=meta.name,
        saved_at=meta.saved_at,
        note=meta.note,
        poi_count=len(meta.pois) - photos,
        photo_count=photos,
        last_pose=meta.last_pose,
        valid=True,
    )


def read_meta(directory: Path) -> BundleMeta:
    """Read and validate ``bundle.json`` of a bundle directory.

    Args:
        directory: Bundle directory (Path).

    Returns:
        BundleMeta: The parsed metadata.

    Raises:
        BundleInvalid: The file is missing, unreadable or invalid.
    """
    try:
        return BundleMeta.model_validate_json((directory / META_FILE).read_bytes())
    except (OSError, ValidationError, ValueError) as exc:
        raise BundleInvalid(f"{META_FILE} of {directory.name}: {exc}") from exc


class BundleStore:
    """Named map bundles under one root directory.

    Args:
        root: Directory holding the bundles (Path); created on first use.
    """

    def __init__(self, root: Path) -> None:
        self.root = Path(root)

    def path_of(self, name: str) -> Path:
        """Resolve the bundle directory of a name.

        Args:
            name: Map name (str).

        Returns:
            Path: ``root/<name>``.

        Raises:
            ValueError: Invalid name or the path would leave the root.
        """
        validate_map_name(name)
        path = self.root / name
        if path.resolve().parent != self.root.resolve():
            raise ValueError(f"map name {name!r} resolves outside the maps directory")
        return path

    def begin(self, name: str) -> Path:
        """Create a fresh temp directory for a save.

        Args:
            name: Map name (str).

        Returns:
            Path: ``root/.tmp-<name>-<unix ms>``; slam_toolbox serializes to ``<tmp>/map``.
        """
        validate_map_name(name)
        self.root.mkdir(parents=True, exist_ok=True)
        tmp = self.root / f"{TEMP_PREFIX}{name}-{time.time_ns() // 1_000_000}"
        tmp.mkdir()
        return tmp

    def commit(self, tmp: Path, name: str, bundle: BundleMeta, grid: GridMap, overwrite: bool) -> MapSummary:
        """Complete the temp directory and move it to ``root/<name>``.

        Args:
            tmp: Directory from :meth:`begin` holding the serialized pose graph and, when exported, ``photos/`` (Path).
            name: Map name (str).
            bundle: Metadata to write (BundleMeta).
            grid: Occupancy grid to write (GridMap).
            overwrite: Replace an existing bundle (bool).

        Returns:
            MapSummary: Summary of the stored map.

        Raises:
            BundleExists: Name taken and ``overwrite`` is False.
            BundleInvalid: A pose graph file is missing.
        """
        aside: Path | None = None
        try:
            final = self.path_of(name)
            if final.exists() and not overwrite:
                raise BundleExists(f"map {name!r} already exists")
            for filename in REQUIRED_SLAM_FILES:
                if not (tmp / filename).is_file():
                    raise BundleInvalid(f"{filename} missing: slam_toolbox serialize did not write it")
            np.savez(
                tmp / OCCUPANCY_FILE,
                cells=grid.cells.astype(np.int8),
                resolution=grid.resolution,
                origin=np.array([grid.origin_x, grid.origin_y, grid.origin_yaw]),
            )
            (tmp / META_FILE).write_text(bundle.model_copy(update={"name": name}).model_dump_json(indent=2))
            if final.exists():
                aside = self.root / f"{OLD_PREFIX}{name}-{time.time_ns() // 1_000_000}"
                final.rename(aside)
            try:
                tmp.rename(final)
            except OSError:
                if aside is not None:
                    aside.rename(final)
                    aside = None
                raise
        except BaseException:
            shutil.rmtree(tmp, ignore_errors=True)
            raise
        if aside is not None:
            shutil.rmtree(aside, ignore_errors=True)
        return summarize(read_meta(final))

    def load(self, name: str) -> LoadedBundle:
        """Read a bundle.

        Args:
            name: Map name (str).

        Returns:
            LoadedBundle: Metadata, grid and pose graph base path.

        Raises:
            BundleNotFound: No such bundle.
            BundleInvalid: A file is missing or invalid (message names it).
        """
        directory = self.path_of(name)
        if not directory.is_dir():
            raise BundleNotFound(f"map {name!r} not found")
        meta = read_meta(directory)
        for filename in REQUIRED_SLAM_FILES:
            if not (directory / filename).is_file():
                raise BundleInvalid(f"{filename} missing in map {name!r}")
        try:
            with np.load(directory / OCCUPANCY_FILE, allow_pickle=False) as data:
                origin = data["origin"]
                grid = GridMap(
                    resolution=float(data["resolution"]),
                    origin_x=float(origin[0]),
                    origin_y=float(origin[1]),
                    origin_yaw=float(origin[2]),
                    cells=data["cells"].astype(np.int8),
                )
        except (OSError, ValueError, KeyError) as exc:
            raise BundleInvalid(f"{OCCUPANCY_FILE} of {name}: {exc}") from exc
        photos = directory / PHOTOS_DIR
        return LoadedBundle(
            meta=meta,
            grid=grid,
            posegraph_base=directory / POSEGRAPH_BASE,
            photos_dir=photos if photos.is_dir() else None,
        )

    def list(self) -> list[MapSummary]:
        """List all bundles sorted by name; temp directories are skipped.

        Returns:
            list[MapSummary]: Invalid bundles appear with ``valid=False`` and the error.
        """
        if not self.root.is_dir():
            return []
        summaries: list[MapSummary] = []
        for directory in sorted(self.root.iterdir(), key=lambda p: p.name):
            if not directory.is_dir() or directory.name.startswith("."):
                continue
            try:
                summaries.append(summarize(read_meta(directory)))
            except BundleInvalid as exc:
                summaries.append(
                    MapSummary(
                        name=directory.name,
                        saved_at=0.0,
                        note="",
                        poi_count=0,
                        last_pose=Pose2D(x=0.0, y=0.0, yaw=0.0),
                        valid=False,
                        error=str(exc),
                    )
                )
        return summaries

    def delete(self, name: str) -> None:
        """Remove a bundle directory with its photos.

        Args:
            name: Map name (str).

        Raises:
            BundleNotFound: No such bundle.
        """
        directory = self.path_of(name)
        if not directory.is_dir():
            raise BundleNotFound(f"map {name!r} not found")
        shutil.rmtree(directory)

    def cleanup_temp(self) -> None:
        """Remove leftover ``.tmp-*`` and ``.old-*`` directories (node start)."""
        if not self.root.is_dir():
            return
        for directory in self.root.iterdir():
            if directory.is_dir() and directory.name.startswith((TEMP_PREFIX, OLD_PREFIX)):
                shutil.rmtree(directory, ignore_errors=True)
