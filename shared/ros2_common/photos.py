"""Photo POI contract shared by poi_store, mcp_server and web_ui.

A photo is a camera image plus where the camera was: PhotoView (camera pose in the map), camera_view (computes it from
the base pose and the camera mount), effective_view (stored view, or one derived from the stored sensor data of a
legacy line), photo_as_poi (the POI form), PhotoAdd (validated add payload), PhotoSelector (bulk delete selector) and
write_atomic (crash-safe file write used for the JPEGs and the index).
"""

import math
import os
import re
import tempfile
from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal, Self

from pydantic import BaseModel, Field, field_validator, model_validator
from scipy.spatial.transform import Rotation

if TYPE_CHECKING:
    from .lookups import LookupRecord

FILE_MODE = 0o644
PHOTO_ID_RE = re.compile(r"^[0-9a-f]{32}$")
MAX_NAME_LEN = 60
MAX_SELECTOR_IDS = 500
PHOTO_KIND = "photo"
PhotoStatus = Literal["open", "done", "cancelled"]
PhotoCreator = Literal["agent", "user"]
ViewSource = Literal["camera_mount", "base_heading"]


class PlanarPose(BaseModel):
    """Robot pose in the map frame when the image was taken."""

    x: float
    y: float
    yaw: float


class CameraPose(BaseModel):
    """Camera body pose in base_link (fixed-axis roll/pitch/yaw, rad)."""

    xyz: list[float] = Field(min_length=3, max_length=3)
    rpy: list[float] = Field(min_length=3, max_length=3)


class PhotoView(BaseModel):
    """Camera pose in the map at capture time (all numbers finite).

    x, y, z: camera position (m, map; z None without a known mount). yaw: heading of the camera forward axis (rad,
    counter-clockwise from +x). pitch: rad, positive means looking down (None without a mount). hfov: horizontal
    field of view (rad, None when unknown or the image is not the full frame). source: camera_mount (base pose
    composed with the mount) or base_heading (base position and yaw only).
    """

    x: float = Field(allow_inf_nan=False)
    y: float = Field(allow_inf_nan=False)
    z: float | None = Field(default=None, allow_inf_nan=False)
    yaw: float = Field(allow_inf_nan=False)
    pitch: float | None = Field(default=None, allow_inf_nan=False)
    hfov: float | None = Field(default=None, allow_inf_nan=False)
    source: ViewSource


def camera_view(
    base_pose_map: PlanarPose | None, camera_pose_base: CameraPose | None, hfov: float | None
) -> PhotoView | None:
    """Camera pose in the map from the base pose and the camera mount.

    Map yaw is the base yaw plus the heading of the body x axis of the mount rotation (fixed-axis xyz rpy); pitch is
    positive when that axis points down; position is the base pose composed with the mount xyz.

    Args:
        base_pose_map (PlanarPose | None): Robot pose in the map, None when unknown.
        camera_pose_base (CameraPose | None): Camera mount in base_link, None when unknown.
        hfov (float | None): Horizontal field of view (rad) of the image, None when unknown.

    Returns:
        PhotoView | None: None without base_pose_map; base_heading view (z, pitch, hfov None) without a mount.
    """
    if base_pose_map is None:
        return None
    if camera_pose_base is None:
        return PhotoView(x=base_pose_map.x, y=base_pose_map.y, yaw=base_pose_map.yaw, source="base_heading")
    forward = Rotation.from_euler("xyz", camera_pose_base.rpy).as_matrix()[:, 0]
    cx, cy, cz = camera_pose_base.xyz
    map_yaw = base_pose_map.yaw + math.atan2(forward[1], forward[0])
    cos_yaw, sin_yaw = math.cos(base_pose_map.yaw), math.sin(base_pose_map.yaw)
    return PhotoView(
        x=base_pose_map.x + cos_yaw * cx - sin_yaw * cy,
        y=base_pose_map.y + sin_yaw * cx + cos_yaw * cy,
        z=cz,
        yaw=math.remainder(map_yaw, math.tau),
        pitch=math.asin(max(-1.0, min(1.0, -float(forward[2])))),
        hfov=hfov,
        source="camera_mount",
    )


def effective_view(rec: "LookupRecord") -> PhotoView | None:
    """The stored view, or for legacy lines the view derived from the stored base pose and mount.

    Args:
        rec (LookupRecord): Index record.

    Returns:
        PhotoView | None: None when the record has no pose at all.
    """
    if rec.view is not None:
        return rec.view
    return camera_view(rec.base_pose_map, rec.camera_pose_base, None)


def photo_as_poi(rec: "LookupRecord") -> dict[str, Any]:
    """The POI form of a photo; x and y are None (never 0) when the photo has no pose.

    Args:
        rec (LookupRecord): Index record.

    Returns:
        dict[str, Any]: JSON-ready POI dict with the capture data under "photo".
    """
    view = effective_view(rec)
    return {
        "id": rec.id,
        "kind": PHOTO_KIND,
        "frame": "map",
        "x": None if view is None else view.x,
        "y": None if view is None else view.y,
        "name": rec.name,
        "note": rec.note,
        "status": rec.status,
        "created_by": rec.created_by,
        "created_at": rec.ts,
        "updated_at": rec.updated_at,
        "poi_ids": list(rec.poi_ids),
        "photo": {
            "file": rec.file,
            "bytes": rec.bytes,
            "width": rec.width,
            "height": rec.height,
            "camera": rec.camera,
            "tool": rec.tool,
            "mcp_session": rec.mcp_session,
            "base_pose_map": None if rec.base_pose_map is None else rec.base_pose_map.model_dump(),
            "camera_pose_base": None if rec.camera_pose_base is None else rec.camera_pose_base.model_dump(),
            "arm_joints": rec.arm_joints,
            "view": None if view is None else view.model_dump(),
        },
    }


class PhotoAdd(BaseModel):
    """Add payload for a photo: the record minus server-owned fields (bytes, status, updated_at, kind).

    The client chooses the id; the file must be '<id>.jpg'. Unknown keys (such as bytes) are rejected.
    """

    model_config = {"extra": "forbid"}

    id: str
    ts: float = Field(ge=0.0, allow_inf_nan=False)
    tool: str
    camera: str
    width: int = Field(gt=0)
    height: int = Field(gt=0)
    file: str
    name: str = Field(default="", max_length=MAX_NAME_LEN)
    created_by: PhotoCreator = "agent"
    base_pose_map: PlanarPose | None = None
    arm_joints: dict[str, float] | None = None
    camera_pose_base: CameraPose | None = None
    mcp_session: str | None = None
    note: str = ""
    poi_ids: list[str] = Field(default_factory=list)
    view: PhotoView | None = None

    @field_validator("id")
    @classmethod
    def id_is_32_hex(cls, value: str) -> str:
        """Require a 32-character lowercase hex id.

        Args:
            value (str): Candidate id.

        Returns:
            str: The id.
        """
        if not PHOTO_ID_RE.match(value):
            raise ValueError("id must be 32 lowercase hex characters")
        return value

    @model_validator(mode="after")
    def file_matches_id(self) -> Self:
        """Require file == '<id>.jpg'.

        Returns:
            Self: The validated payload.
        """
        if self.file != f"{self.id}.jpg":
            raise ValueError("file must equal '<id>.jpg'")
        return self


class PhotoSelector(BaseModel):
    """Bulk photo selector: exactly one of ids, older_than_s or keep_newest; created_by and camera narrow it."""

    ids: list[str] | None = Field(default=None, min_length=1, max_length=MAX_SELECTOR_IDS)
    older_than_s: float | None = Field(default=None, gt=0.0, allow_inf_nan=False)
    keep_newest: int | None = Field(default=None, ge=0)
    created_by: PhotoCreator | None = None
    camera: str | None = None
    dry_run: bool = False

    @model_validator(mode="after")
    def exactly_one_criterion(self) -> Self:
        """Require exactly one of ids, older_than_s, keep_newest.

        Returns:
            Self: The validated selector.
        """
        given = [v for v in (self.ids, self.older_than_s, self.keep_newest) if v is not None]
        if len(given) != 1:
            raise ValueError("exactly one of ids, older_than_s, keep_newest is required")
        return self


def write_atomic(path: Path, data: bytes) -> None:
    """Write a file atomically: fsynced temp file in the same directory (mode 0644), then rename.

    Args:
        path (Path): Target file.
        data (bytes): Content.
    """
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=f".{path.name}-")
    try:
        with os.fdopen(fd, "wb") as fh:
            os.fchmod(fh.fileno(), FILE_MODE)
            fh.write(data)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, path)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise
