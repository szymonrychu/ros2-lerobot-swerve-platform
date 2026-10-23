"""On-disk session store of the calibration node (design D3): only selected poses, safe ids, atomic writes.

Layout under state_dir: sessions/<sid>/session.json, sessions/<sid>/poses/<pid>/{rgb.jpg, amplitude.png, depth.png,
pose.json, rgb_thumb.jpg, tof_thumb.jpg} and sessions/<sid>/results/<rid>/result.json (plus the files the compute job
writes there).
"""

import json
import logging
import os
import re
import shutil
import threading
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Literal

import cv2
import numpy as np
from ros2_common.head_rgbd_calibration.calibrate import PoseView
from ros2_common.head_rgbd_calibration.charuco import Detection
from ros2_common.head_rgbd_calibration.frames import CameraInfoData
from ros2_common.head_rgbd_calibration.hints import capture_hints
from ros2_common.head_rgbd_calibration.poses import CapturedPose
from ros2_common.head_rgbd_calibration.views import ARM_JOINTS, HEAD_JOINTS, ArmBoardView

from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.images import rgb_thumbnail, tof_thumbnail

LOGGER = logging.getLogger(__name__)

SESSION_ID_RE = re.compile(r"^calib-(\d{8}-\d{6})(?:-(\d+))?$", re.ASCII)
POSE_ID_RE = re.compile(r"^\d{3,}$", re.ASCII)
RESULT_ID_RE = re.compile(r"^r-\d{8}-\d{6}$", re.ASCII)
SESSION_PREFIX = "calib-"
RESULT_PREFIX = "r-"
ID_TIME_FORMAT = "%Y%m%d-%H%M%S"
POSE_ID_DIGITS = 3
SESSIONS_DIR = "sessions"
POSES_DIR = "poses"
RESULTS_DIR = "results"
SESSION_FILE = "session.json"
POSE_FILE = "pose.json"
RESULT_FILE = "result.json"
RGB_FILE = "rgb.jpg"
AMPLITUDE_FILE = "amplitude.png"
DEPTH_FILE = "depth.png"
THUMBNAIL_FILES = {"rgb": "rgb_thumb.jpg", "tof": "tof_thumb.jpg"}
TEMP_PREFIX = ".tmp-"
RGB_JPEG_QUALITY = 95
KIND_RGBD = "rgbd"
KIND_HEAD_ARM = "head_arm"
STILL_TIERS = (0, 1, 2)


class InvalidId(ValueError):
    """An id does not match its pattern or resolves outside the state dir (maps to 404)."""


class NotFound(LookupError):
    """The session, pose, result or file does not exist (maps to 404)."""


@dataclass(frozen=True)
class SessionViews:
    """A session's poses as the compute job needs them.

    Attributes:
        views: One PoseView per stored pose, in pose id order.
        rgb_size: RGB (width, height), None while the session has no poses.
        tof_size: ToF (width, height), None while the session has no poses.
        tof_info: SDK ToF camera_info recorded for the session, None if none arrived.
        lens_position: Lens position the session was captured with.
        pose_ids: Pose ids matching views.
    """

    views: list[PoseView]
    rgb_size: tuple[int, int] | None
    tof_size: tuple[int, int] | None
    tof_info: CameraInfoData | None
    lens_position: float
    pose_ids: list[str]


def write_bytes_atomic(path: Path, data: bytes) -> None:
    """Write data to path through a temporary file and os.replace."""
    tmp = path.with_name(f"{TEMP_PREFIX}{path.name}")
    tmp.write_bytes(data)
    os.replace(tmp, path)


def write_json_atomic(path: Path, data: Any) -> None:
    """Write data as JSON to path through a temporary file and os.replace."""
    write_bytes_atomic(path, json.dumps(data, indent=2).encode())


def encode_png(image: np.ndarray) -> bytes:
    """PNG bytes of an image (uint16 stays uint16)."""
    ok, buffer = cv2.imencode(".png", image)
    if not ok:
        raise ValueError("PNG encoding failed")
    return buffer.tobytes()


def detection_json(stamp_ns: int, detection: Detection) -> dict[str, Any]:
    """The pose.json block of one camera."""
    return {
        "stamp_ns": int(stamp_ns),
        "ids": [int(i) for i in detection.ids],
        "points": [[float(x), float(y)] for x, y in detection.points],
    }


def detection_from_json(block: dict[str, Any]) -> Detection:
    """Rebuild a Detection from a pose.json block."""
    return Detection(
        ids=np.array(block["ids"], dtype=np.int32), points=np.array(block["points"], dtype=np.float32).reshape(-1, 2)
    )


def session_sort_key(sid: str) -> tuple[str, int]:
    """Chronological key of a session id (the same-second suffix counts from 1)."""
    match = SESSION_ID_RE.match(sid)
    return (match.group(1), int(match.group(2) or 1)) if match else ("", 0)


class SessionStore:
    """Calibration sessions on disk; mutations are serialised by one lock."""

    def __init__(self, state_dir: Path, cfg: NodeConfig | None = None) -> None:
        """Create a store.

        Args:
            state_dir: Root directory of the node state.
            cfg: Node config supplying the coverage grid, thumbnail width and amplitude percentiles (defaults).
        """
        self.state_dir = Path(state_dir)
        self.cfg = cfg or NodeConfig()
        self.lock = threading.Lock()

    # paths

    def checked(self, path: Path) -> Path:
        """The path itself when it resolves under state_dir, else InvalidId."""
        if not path.resolve().is_relative_to(self.state_dir.resolve()):
            raise InvalidId(f"path escapes the state dir: {path.name}")
        return path

    def session_dir(self, sid: str) -> Path:
        """Directory of a session (InvalidId for a bad id; it need not exist)."""
        if not SESSION_ID_RE.fullmatch(sid):
            raise InvalidId(f"invalid session id: {sid!r}")
        return self.checked(self.state_dir / SESSIONS_DIR / sid)

    def existing_session_dir(self, sid: str) -> Path:
        """Directory of an existing session (NotFound if it has no session.json)."""
        path = self.session_dir(sid)
        if not (path / SESSION_FILE).is_file():
            raise NotFound(f"no such session: {sid}")
        return path

    def pose_dir(self, sid: str, pid: str) -> Path:
        """Directory of a pose (InvalidId for a bad id; it need not exist)."""
        if not POSE_ID_RE.fullmatch(pid):
            raise InvalidId(f"invalid pose id: {pid!r}")
        return self.checked(self.existing_session_dir(sid) / POSES_DIR / pid)

    def result_dir(self, sid: str, rid: str) -> Path:
        """Directory of a result (InvalidId for a bad id; it need not exist, the compute job creates it)."""
        if not RESULT_ID_RE.fullmatch(rid):
            raise InvalidId(f"invalid result id: {rid!r}")
        return self.checked(self.existing_session_dir(sid) / RESULTS_DIR / rid)

    def thumbnail_path(self, sid: str, pid: str, kind: str) -> Path:
        """Path of a pose thumbnail.

        Args:
            sid: Session id.
            pid: Pose id.
            kind: "rgb" or "tof".

        Returns:
            The existing thumbnail file; InvalidId for another kind, NotFound if the file is missing.
        """
        if kind not in THUMBNAIL_FILES:
            raise InvalidId(f"invalid thumbnail kind: {kind!r}")
        path = self.pose_dir(sid, pid) / THUMBNAIL_FILES[kind]
        if not path.is_file():
            raise NotFound(f"no {kind} thumbnail for pose {pid}")
        return path

    # sessions

    def read_session(self, sid: str) -> dict[str, Any]:
        """The raw session.json content."""
        return json.loads((self.existing_session_dir(sid) / SESSION_FILE).read_text())

    def pose_ids(self, sid: str) -> list[str]:
        """Stored pose ids of a session, ascending."""
        poses = self.existing_session_dir(sid) / POSES_DIR
        if not poses.is_dir():
            return []
        return sorted(p.name for p in poses.iterdir() if POSE_ID_RE.fullmatch(p.name) and (p / POSE_FILE).is_file())

    def result_ids(self, sid: str) -> list[str]:
        """Ids of the results that hold a result.json, newest first."""
        results = self.existing_session_dir(sid) / RESULTS_DIR
        if not results.is_dir():
            return []
        found = [p.name for p in results.iterdir() if RESULT_ID_RE.fullmatch(p.name) and (p / RESULT_FILE).is_file()]
        return sorted(found, reverse=True)

    def kind(self, sid: str) -> str:
        """The session kind, "rgbd" for a session.json without the key."""
        return self.read_session(sid).get("kind", KIND_RGBD)

    def require_kind(self, sid: str, wanted: str, action: str) -> None:
        """Raise ValueError unless the session is of the wanted kind."""
        kind = self.kind(sid)
        if kind != wanted:
            raise ValueError(f"{action} needs a {wanted} session, {sid} is {kind}")

    def summary(self, sid: str) -> dict[str, Any]:
        """session.json plus kind, pose_ids, pose_count and result_count."""
        data = self.read_session(sid)
        data["kind"] = data.get("kind", KIND_RGBD)
        ids = self.pose_ids(sid)
        return {**data, "pose_ids": ids, "pose_count": len(ids), "result_count": len(self.result_ids(sid))}

    def create_session(self, now: datetime, lens_position: float, kind: Literal["rgbd", "head_arm"] = KIND_RGBD) -> str:
        """Create a session.

        Args:
            now: Creation time; it names the session.
            lens_position: Lens position the head camera is locked at.
            kind: "rgbd" (RGB+ToF poses) or "head_arm" (still RGB poses with measured joints).

        Returns:
            The id calib-YYYYmmdd-HHMMSS, with -N appended when that second is taken.
        """
        base = f"{SESSION_PREFIX}{now.strftime(ID_TIME_FORMAT)}"
        with self.lock:
            sessions = self.state_dir / SESSIONS_DIR
            sessions.mkdir(parents=True, exist_ok=True)
            sid, attempt = base, 1
            while True:
                try:
                    (sessions / sid).mkdir()
                    break
                except FileExistsError:
                    attempt += 1
                    sid = f"{base}-{attempt}"
            path = self.session_dir(sid)
            write_json_atomic(
                path / SESSION_FILE,
                {
                    "id": sid,
                    "kind": kind,
                    "created": now.isoformat(),
                    "lens_position": lens_position,
                    "rgb_size": None,
                    "tof_size": None,
                    "tof_info": None,
                    "next_pose": 1,
                },
            )
        return sid

    def list_sessions(self) -> list[dict[str, Any]]:
        """Every session newest first: session.json fields plus pose_ids, pose_count and result_count."""
        sessions = self.state_dir / SESSIONS_DIR
        if not sessions.is_dir():
            return []
        out = []
        for path in sessions.iterdir():
            if SESSION_ID_RE.fullmatch(path.name) and (path / SESSION_FILE).is_file():
                try:
                    out.append(self.summary(path.name))
                except (OSError, ValueError):
                    LOGGER.exception("unreadable session %s skipped", path.name)
        return sorted(out, key=lambda s: session_sort_key(s["id"]), reverse=True)

    def load_session(self, sid: str) -> dict[str, Any]:
        """One session: session.json fields plus pose_ids, pose_count and result_count."""
        return self.summary(sid)

    def set_tof_info(self, sid: str, info: CameraInfoData) -> None:
        """Record the SDK ToF camera_info (k, p, d, width, height, distortion_model) in session.json."""
        block = {
            "k": [float(v) for v in info.k],
            "p": [float(v) for v in info.p],
            "d": [float(v) for v in info.d],
            "width": int(info.width),
            "height": int(info.height),
            "distortion_model": info.distortion_model,
        }
        with self.lock:
            data = self.read_session(sid)
            data["tof_info"] = block
            write_json_atomic(self.session_dir(sid) / SESSION_FILE, data)

    def set_image_sizes(self, sid: str, rgb_size: tuple[int, int], tof_size: tuple[int, int]) -> None:
        """Record the image sizes (width, height); the first call wins, a different size raises ValueError."""
        with self.lock:
            data = self.read_session(sid)
            self.apply_sizes(data, rgb_size, tof_size)
            write_json_atomic(self.session_dir(sid) / SESSION_FILE, data)

    @staticmethod
    def apply_sizes(data: dict[str, Any], rgb_size: tuple[int, int], tof_size: tuple[int, int]) -> None:
        """Set the sizes in a session dict, or raise ValueError when they differ from the recorded ones."""
        wanted = {"rgb_size": list(rgb_size), "tof_size": list(tof_size)}
        for key, value in wanted.items():
            if data[key] is not None and data[key] != value:
                raise ValueError(f"{key} {value} differs from the session's {data[key]}: image size changed")
        data.update(wanted)

    # poses

    def add_pose(self, sid: str, pose: CapturedPose) -> str:
        """Store a captured pose.

        Args:
            sid: Session id.
            pose: The pose with its detections, as the PoseCounter produced it.

        Returns:
            The new pose id, zero-padded from the session's next_pose counter and never reused after a delete.

        Raises:
            ValueError: the session is a head_arm one, the amplitude frame is empty (no thumbnail possible,
                nothing is stored) or an image has another size than the session's first pose.
        """
        self.require_kind(sid, KIND_RGBD, "add_pose")
        tof_thumb = tof_thumbnail(pose.amplitude.image, pose.tof_detection, self.cfg.selection.amplitude_percentiles)
        if tof_thumb is None:
            raise ValueError("empty amplitude frame: pose not stored")
        rgb_jpeg = pose.rgb_raw
        if rgb_jpeg is None:
            ok, buffer = cv2.imencode(".jpg", pose.rgb.image, [cv2.IMWRITE_JPEG_QUALITY, RGB_JPEG_QUALITY])
            if not ok:
                raise ValueError("JPEG encoding failed")
            rgb_jpeg = buffer.tobytes()
        files = {
            RGB_FILE: rgb_jpeg,
            AMPLITUDE_FILE: encode_png(pose.amplitude.image),
            THUMBNAIL_FILES["rgb"]: rgb_thumbnail(pose.rgb.image, pose.rgb_detection, self.cfg.thumbnail_width_px),
            THUMBNAIL_FILES["tof"]: tof_thumb,
        }
        if pose.depth is not None:
            files[DEPTH_FILE] = encode_png(pose.depth.image)
        meta = {
            "rgb": detection_json(pose.rgb.stamp_ns, pose.rgb_detection),
            "tof": detection_json(pose.amplitude.stamp_ns, pose.tof_detection),
            "depth_stamp_ns": None if pose.depth is None else int(pose.depth.stamp_ns),
            "hints": capture_hints(pose.rgb_detection, pose.tof_detection, self.cfg.board),
        }
        rgb_h, rgb_w = pose.rgb.image.shape[:2]
        tof_h, tof_w = pose.amplitude.image.shape[:2]
        with self.lock:
            data = self.read_session(sid)
            self.apply_sizes(data, (rgb_w, rgb_h), (tof_w, tof_h))
            pid = self.write_pose_dir(sid, data, files, meta)
            write_json_atomic(self.session_dir(sid) / SESSION_FILE, data)
        return pid

    def add_still_pose(
        self,
        sid: str,
        rgb_jpeg: bytes,
        rgb_image: np.ndarray,
        detection: Detection,
        joints: dict[str, float],
        stamps: dict[str, int],
        arm_pose: int,
        tier: int,
        head_offset: int,
    ) -> str:
        """Store a still RGB view of the gripper board with the measured joints (head_arm sessions).

        Args:
            sid: Session id.
            rgb_jpeg: The camera JPEG, stored as received.
            rgb_image: The decoded BGR frame (size check and thumbnail).
            detection: Board corners found in rgb_image.
            joints: Measured rad of all seven joints (HEAD_JOINTS and ARM_JOINTS).
            stamps: Stamps in ns of the sources, e.g. {"rgb": ..., "joints": ...}.
            arm_pose: Index of the scripted arm pose, 0 or more.
            tier: Capture tier 0, 1 or 2.
            head_offset: Index of the head offset, 0 or more.

        Returns:
            The new pose id.

        Raises:
            ValueError: wrong session kind, a joint missing, tier not 0/1/2, a negative arm_pose or head_offset,
                or an image size other than the session's.
        """
        self.require_kind(sid, KIND_HEAD_ARM, "add_still_pose")
        missing = [name for name in (*HEAD_JOINTS, *ARM_JOINTS) if name not in joints]
        if missing:
            raise ValueError(f"missing joints: {', '.join(missing)}")
        if tier not in STILL_TIERS:
            raise ValueError(f"tier must be 0, 1 or 2, got {tier}")
        if arm_pose < 0 or head_offset < 0:
            raise ValueError(f"arm_pose and head_offset must not be negative, got {arm_pose} and {head_offset}")
        files = {
            RGB_FILE: bytes(rgb_jpeg),
            THUMBNAIL_FILES["rgb"]: rgb_thumbnail(rgb_image, detection, self.cfg.thumbnail_width_px),
        }
        meta = {
            "stamps": {key: int(value) for key, value in stamps.items()},
            "rgb": {
                "ids": [int(i) for i in detection.ids],
                "points": [[float(x), float(y)] for x, y in detection.points],
            },
            "joints": {name: float(value) for name, value in joints.items()},
            "arm_pose": int(arm_pose),
            "tier": int(tier),
            "head_offset": int(head_offset),
        }
        rgb_h, rgb_w = rgb_image.shape[:2]
        with self.lock:
            data = self.read_session(sid)
            wanted = [rgb_w, rgb_h]
            if data["rgb_size"] is not None and data["rgb_size"] != wanted:
                raise ValueError(f"rgb_size {wanted} differs from the session's {data['rgb_size']}: image size changed")
            data["rgb_size"] = wanted
            pid = self.write_pose_dir(sid, data, files, meta)
            write_json_atomic(self.session_dir(sid) / SESSION_FILE, data)
        return pid

    def write_pose_dir(self, sid: str, data: dict[str, Any], files: dict[str, bytes], meta: dict[str, Any]) -> str:
        """Write a pose directory atomically and advance data["next_pose"]; the caller holds the lock."""
        pid = f"{data['next_pose']:0{POSE_ID_DIGITS}d}"
        poses = self.session_dir(sid) / POSES_DIR
        poses.mkdir(exist_ok=True)
        tmp = poses / f"{TEMP_PREFIX}{pid}"
        shutil.rmtree(tmp, ignore_errors=True)
        tmp.mkdir()
        try:
            for name, content in files.items():
                (tmp / name).write_bytes(content)
            (tmp / POSE_FILE).write_text(json.dumps(meta, indent=2))
            os.replace(tmp, poses / pid)
        except BaseException:
            shutil.rmtree(tmp, ignore_errors=True)
            raise
        data["next_pose"] += 1
        return pid

    def load_arm_board_views(self, sid: str) -> list[ArmBoardView]:
        """The still poses of a head_arm session as fit views (view_id = pose id), in pose order."""
        self.require_kind(sid, KIND_HEAD_ARM, "load_arm_board_views")
        views = []
        for pid in self.pose_ids(sid):
            meta = json.loads((self.pose_dir(sid, pid) / POSE_FILE).read_text())
            views.append(
                ArmBoardView(
                    view_id=pid,
                    arm_pose=meta["arm_pose"],
                    tier=meta["tier"],
                    joints=dict(meta["joints"]),
                    detection=detection_from_json(meta["rgb"]),
                    head_offset=meta["head_offset"],
                )
            )
        return views

    def captured_arm_poses(self, sid: str) -> set[int]:
        """The arm pose indices that have at least one stored still pose."""
        self.require_kind(sid, KIND_HEAD_ARM, "captured_arm_poses")
        return {view.arm_pose for view in self.load_arm_board_views(sid)}

    def pose_hints(self, sid: str, pid: str) -> list[str]:
        """Capture hints of a stored pose.

        Args:
            sid: Session id.
            pid: Pose id.

        Returns:
            The pose.json hints, or hints computed from the stored detections when the pose predates them
            (nothing is written back).
        """
        meta = json.loads((self.pose_dir(sid, pid) / POSE_FILE).read_text())
        if self.kind(sid) == KIND_HEAD_ARM:
            return []
        if "hints" in meta:
            return list(meta["hints"])
        return capture_hints(detection_from_json(meta["rgb"]), detection_from_json(meta["tof"]), self.cfg.board)

    def delete_pose(self, sid: str, pid: str) -> None:
        """Delete one pose directory (its id is not reused); NotFound when it does not exist."""
        with self.lock:
            path = self.pose_dir(sid, pid)
            if not path.is_dir():
                raise NotFound(f"no such pose: {pid}")
            shutil.rmtree(path)

    def load_views(self, sid: str) -> SessionViews:
        """Rebuild the compute inputs of a session from pose.json and depth.png."""
        data = self.read_session(sid)
        ids = self.pose_ids(sid)
        views = []
        for pid in ids:
            path = self.pose_dir(sid, pid)
            meta = json.loads((path / POSE_FILE).read_text())
            depth = cv2.imread(str(path / DEPTH_FILE), cv2.IMREAD_UNCHANGED) if (path / DEPTH_FILE).is_file() else None
            views.append(PoseView(detection_from_json(meta["rgb"]), detection_from_json(meta["tof"]), depth))
        info = data["tof_info"]
        tof_info = (
            None
            if info is None
            else CameraInfoData(
                width=info["width"],
                height=info["height"],
                distortion_model=info["distortion_model"],
                d=np.array(info["d"], dtype=np.float64),
                k=np.array(info["k"], dtype=np.float64),
                p=np.array(info["p"], dtype=np.float64),
            )
        )
        return SessionViews(
            views=views,
            rgb_size=None if data["rgb_size"] is None else tuple(data["rgb_size"]),
            tof_size=None if data["tof_size"] is None else tuple(data["tof_size"]),
            tof_info=tof_info,
            lens_position=data["lens_position"],
            pose_ids=ids,
        )

    def coverage(self, sid: str) -> dict[str, dict[str, Any]]:
        """Corner counts per image grid cell over every stored pose.

        Returns:
            {rgb: {cols, rows, counts}, tof: {...}} with counts[row][col]; all zero without poses. Only stored
            corners are counted.
        """
        data = self.read_session(sid)
        cols, rows = self.cfg.coverage.cols, self.cfg.coverage.rows
        out: dict[str, dict[str, Any]] = {}
        cameras = ("rgb",) if data.get("kind", KIND_RGBD) == KIND_HEAD_ARM else ("rgb", "tof")
        for camera in cameras:
            counts = np.zeros((rows, cols), dtype=int)
            size = data[f"{camera}_size"]
            if size is not None:
                for pid in self.pose_ids(sid):
                    meta = json.loads((self.pose_dir(sid, pid) / POSE_FILE).read_text())
                    for x, y in meta[camera]["points"]:
                        col = min(cols - 1, max(0, int(x / size[0] * cols)))
                        row = min(rows - 1, max(0, int(y / size[1] * rows)))
                        counts[row, col] += 1
            out[camera] = {"cols": cols, "rows": rows, "counts": counts.tolist()}
        return out

    # results

    @staticmethod
    def new_result_id(now: datetime) -> str:
        """Result id r-YYYYmmdd-HHMMSS for the given time."""
        return f"{RESULT_PREFIX}{now.strftime(ID_TIME_FORMAT)}"

    def list_results(self, sid: str) -> list[dict[str, Any]]:
        """The result.json of every result of the session, newest first."""
        return [json.loads((self.result_dir(sid, rid) / RESULT_FILE).read_text()) for rid in self.result_ids(sid)]
