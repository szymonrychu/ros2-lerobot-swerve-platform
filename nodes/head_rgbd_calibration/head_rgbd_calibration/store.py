"""On-disk session store of the calibration node (design D3): only selected poses, safe ids, atomic writes.

Layout under state_dir: sessions/<sid>/session.json, sessions/<sid>/poses/<pid>/{rgb.jpg, depth.png, pose.json,
rgb_thumb.jpg, depth_thumb.jpg} for a D435 intrinsics check session (rgb.jpg, pose.json and rgb_thumb.jpg for a
head_arm one) and sessions/<sid>/results/<rid>/result.json (plus the files the compute job writes there).
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
from ros2_common.head_rgbd_calibration.charuco import Detection
from ros2_common.head_rgbd_calibration.config import BoardConfig
from ros2_common.head_rgbd_calibration.frames import CameraInfoData, StampedImage
from ros2_common.head_rgbd_calibration.hints import FEW_CORNERS_FRACTION
from ros2_common.head_rgbd_calibration.views import ARM_JOINTS, HEAD_JOINTS, ArmBoardView

from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.images import depth_thumbnail, rgb_thumbnail

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
DEPTH_FILE = "depth.png"
THUMBNAIL_FILES = {"rgb": "rgb_thumb.jpg", "depth": "depth_thumb.jpg"}
TEMP_PREFIX = ".tmp-"
RGB_JPEG_QUALITY = 95
KIND_INTRINSICS = "intrinsics"
KIND_HEAD_ARM = "head_arm"
# Kind a session.json without the key reads as: a session of the retired RGB + ToF calibration, which nothing computes.
KIND_RGBD = "rgbd"
STILL_TIERS = (0, 1, 2)


class InvalidId(ValueError):
    """An id does not match its pattern or resolves outside the state dir (maps to 404)."""


class NotFound(LookupError):
    """The session, pose, result or file does not exist (maps to 404)."""


@dataclass(frozen=True)
class ColourDepthStill:
    """A captured D435 still: colour and aligned depth of one stamp, the colour detection and the factory intrinsics.

    Attributes:
        number: Pose number, continuing from the poses already stored.
        rgb: Decoded BGR colour frame.
        rgb_raw: Compressed bytes the colour frame was decoded from, None when not passed (then encoded at store).
        depth: Depth registered to the colour frame, uint16 mm (0 is no depth), the colour frame's stamp.
        detection: Board detection in the colour image.
        camera_info: The factory colour camera_info at capture time.
    """

    number: int
    rgb: StampedImage
    rgb_raw: bytes | None
    depth: StampedImage
    detection: Detection
    camera_info: CameraInfoData


@dataclass(frozen=True)
class StoredStill:
    """One stored still as the compute job reads it back.

    Attributes:
        pose_id: Pose id.
        rgb: The stored colour frame, decoded (BGR).
        depth: The stored aligned depth, uint16 mm.
        camera_info: The factory colour camera_info stored with it, None when pose.json carries none.
    """

    pose_id: str
    rgb: np.ndarray
    depth: np.ndarray
    camera_info: CameraInfoData | None


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


def camera_info_json(info: CameraInfoData) -> dict[str, Any]:
    """The pose.json block of a camera_info (width, height, distortion_model, d, k, p)."""
    return {
        "width": int(info.width),
        "height": int(info.height),
        "distortion_model": info.distortion_model,
        "d": [float(v) for v in info.d],
        "k": [float(v) for v in info.k],
        "p": [float(v) for v in info.p],
    }


def camera_info_from_json(block: dict[str, Any]) -> CameraInfoData:
    """Rebuild a CameraInfoData from a pose.json block."""
    return CameraInfoData(
        width=int(block["width"]),
        height=int(block["height"]),
        distortion_model=str(block["distortion_model"]),
        d=np.array(block["d"], dtype=np.float64),
        k=np.array(block["k"], dtype=np.float64),
        p=np.array(block["p"], dtype=np.float64),
    )


def colour_hints(detection: Detection, board: BoardConfig) -> list[str]:
    """Hints for an operator on how to improve a captured still.

    Args:
        detection: ChArUco detection in the colour image.
        board: The calibration board (its inner corner count sets the few-corners threshold).

    Returns:
        list[str]: The few-corners hint when the colour camera sees under FEW_CORNERS_FRACTION of the inner corners;
        [] for a good still.
    """
    inner = (board.squares_x - 1) * (board.squares_y - 1)
    count = len(detection.ids)
    if count < FEW_CORNERS_FRACTION * inner:
        return [f"few corners: {count} RGB - move the board closer / fill the frame"]
    return []


def board_depth_samples(depth_mm: np.ndarray, detection: Detection) -> int:
    """Valid aligned depth pixels inside the convex hull of the detected corners (the plane bias samples).

    Args:
        depth_mm: uint16 depth registered to the colour frame, mm (0 is no depth).
        detection: Board corners in colour pixels.

    Returns:
        int: Pixel count; 0 for fewer than three corners.
    """
    if len(detection.points) < 3:
        return 0
    mask = np.zeros(depth_mm.shape, dtype=np.uint8)
    hull = cv2.convexHull(np.rint(detection.points).astype(np.int32).reshape(-1, 1, 2))
    cv2.fillConvexPoly(mask, hull, 1)
    return int(np.count_nonzero((mask > 0) & (depth_mm > 0)))


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
            cfg: Node config supplying the coverage grid, thumbnail width and board (defaults).
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
            kind: "rgb" or "depth".

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
        """The session kind, "rgbd" (the retired RGB + ToF calibration) for a session.json without the key."""
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

    def create_session(self, now: datetime, kind: Literal["intrinsics", "head_arm"] = KIND_INTRINSICS) -> str:
        """Create a session.

        Args:
            now: Creation time; it names the session.
            kind: "intrinsics" (D435 colour + aligned depth stills) or "head_arm" (still RGB poses with joints).

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
                    "rgb_size": None,
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

    @staticmethod
    def apply_rgb_size(data: dict[str, Any], image: np.ndarray) -> None:
        """Record the colour (width, height) in a session dict: the first pose sets it, another size is refused."""
        height, width = image.shape[:2]
        wanted = [width, height]
        if data["rgb_size"] is not None and data["rgb_size"] != wanted:
            raise ValueError(f"rgb_size {wanted} differs from the session's {data['rgb_size']}: image size changed")
        data["rgb_size"] = wanted

    # poses

    def add_pose(self, sid: str, still: ColourDepthStill) -> str:
        """Store a captured D435 still: colour JPEG, aligned depth PNG (16-bit), thumbnails and pose.json.

        Args:
            sid: Session id.
            still: The still with its detection and factory camera_info, as the capture judged it.

        Returns:
            The new pose id, zero-padded from the session's next_pose counter and never reused after a delete.

        Raises:
            ValueError: the session is not an intrinsics session, the depth is not the colour size (not aligned) or
                holds no valid pixel, the camera_info is for another image size, or the colour frame has another size
                than the session's first pose. Nothing is stored then.
        """
        self.require_kind(sid, KIND_INTRINSICS, "add_pose")
        rgb_h, rgb_w = still.rgb.image.shape[:2]
        if still.depth.image.shape[:2] != (rgb_h, rgb_w):
            raise ValueError(
                f"aligned depth {still.depth.image.shape[1]}x{still.depth.image.shape[0]} is not the colour size "
                f"{rgb_w}x{rgb_h}: pose not stored"
            )
        info = still.camera_info
        if (info.width, info.height) != (rgb_w, rgb_h):
            raise ValueError(f"camera_info {info.width}x{info.height} is not the colour size {rgb_w}x{rgb_h}")
        depth_thumb = depth_thumbnail(still.depth.image, still.detection, self.cfg.thumbnail_width_px)
        if depth_thumb is None:
            raise ValueError("aligned depth has no valid pixel: pose not stored")
        rgb_jpeg = still.rgb_raw
        if rgb_jpeg is None:
            ok, buffer = cv2.imencode(".jpg", still.rgb.image, [cv2.IMWRITE_JPEG_QUALITY, RGB_JPEG_QUALITY])
            if not ok:
                raise ValueError("JPEG encoding failed")
            rgb_jpeg = buffer.tobytes()
        files = {
            RGB_FILE: rgb_jpeg,
            DEPTH_FILE: encode_png(still.depth.image),
            THUMBNAIL_FILES["rgb"]: rgb_thumbnail(still.rgb.image, still.detection, self.cfg.thumbnail_width_px),
            THUMBNAIL_FILES["depth"]: depth_thumb,
        }
        meta = {
            "rgb": detection_json(still.rgb.stamp_ns, still.detection),
            "depth_stamp_ns": int(still.depth.stamp_ns),
            "camera_info": camera_info_json(info),
            "plane_bias_samples": board_depth_samples(still.depth.image, still.detection),
            "hints": colour_hints(still.detection, self.cfg.board),
        }
        with self.lock:
            data = self.read_session(sid)
            self.apply_rgb_size(data, still.rgb.image)
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
        camera_info: CameraInfoData,
    ) -> str:
        """Store a still RGB view of the gripper board with the measured joints and the factory colour camera_info
        (head_arm sessions; the head-from-arm fit takes its intrinsics from it).

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
            camera_info: The factory colour camera_info current when the frame was captured.

        Returns:
            The new pose id.

        Raises:
            ValueError: wrong session kind, a joint missing, tier not 0/1/2, a negative arm_pose or head_offset,
                a camera_info for another image size, or an image size other than the session's.
        """
        self.require_kind(sid, KIND_HEAD_ARM, "add_still_pose")
        missing = [name for name in (*HEAD_JOINTS, *ARM_JOINTS) if name not in joints]
        if missing:
            raise ValueError(f"missing joints: {', '.join(missing)}")
        if tier not in STILL_TIERS:
            raise ValueError(f"tier must be 0, 1 or 2, got {tier}")
        if arm_pose < 0 or head_offset < 0:
            raise ValueError(f"arm_pose and head_offset must not be negative, got {arm_pose} and {head_offset}")
        rgb_h, rgb_w = rgb_image.shape[:2]
        if (camera_info.width, camera_info.height) != (rgb_w, rgb_h):
            raise ValueError(
                f"camera_info {camera_info.width}x{camera_info.height} is not the colour size {rgb_w}x{rgb_h}"
            )
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
            "camera_info": camera_info_json(camera_info),
        }
        with self.lock:
            data = self.read_session(sid)
            self.apply_rgb_size(data, rgb_image)
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

    def load_arm_camera_infos(self, sid: str) -> dict[str, CameraInfoData | None]:
        """The colour camera_info stored with each still pose of a head_arm session, by pose id in pose order.

        Args:
            sid: Session id.

        Returns:
            dict[str, CameraInfoData | None]: The camera_info per pose id; None for a pose.json without one.

        Raises:
            ValueError: the session is not a head_arm session.
        """
        self.require_kind(sid, KIND_HEAD_ARM, "load_arm_camera_infos")
        infos: dict[str, CameraInfoData | None] = {}
        for pid in self.pose_ids(sid):
            info = json.loads((self.pose_dir(sid, pid) / POSE_FILE).read_text()).get("camera_info")
            infos[pid] = None if info is None else camera_info_from_json(info)
        return infos

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
            The pose.json hints; [] for a head_arm pose.
        """
        meta = json.loads((self.pose_dir(sid, pid) / POSE_FILE).read_text())
        return list(meta.get("hints", []))

    def delete_pose(self, sid: str, pid: str) -> None:
        """Delete one pose directory (its id is not reused); NotFound when it does not exist."""
        with self.lock:
            path = self.pose_dir(sid, pid)
            if not path.is_dir():
                raise NotFound(f"no such pose: {pid}")
            shutil.rmtree(path)

    def load_stills(self, sid: str) -> list[StoredStill]:
        """The stored stills of an intrinsics session, in pose id order: colour, aligned depth and camera_info.

        Raises:
            ValueError: the session is not an intrinsics session, or a stored image does not decode.
        """
        self.require_kind(sid, KIND_INTRINSICS, "load_stills")
        stills = []
        for pid in self.pose_ids(sid):
            path = self.pose_dir(sid, pid)
            meta = json.loads((path / POSE_FILE).read_text())
            rgb = cv2.imread(str(path / RGB_FILE), cv2.IMREAD_COLOR)
            depth = cv2.imread(str(path / DEPTH_FILE), cv2.IMREAD_UNCHANGED)
            if rgb is None or depth is None:
                raise ValueError(f"pose {pid}: {RGB_FILE} or {DEPTH_FILE} is missing or does not decode")
            info = meta.get("camera_info")
            stills.append(StoredStill(pid, rgb, depth, None if info is None else camera_info_from_json(info)))
        return stills

    def coverage(self, sid: str) -> dict[str, dict[str, Any]]:
        """Corner counts per image grid cell over every stored pose.

        Returns:
            {rgb: {cols, rows, counts}} with counts[row][col] over the colour image; all zero without poses. Only
            stored corners are counted.
        """
        data = self.read_session(sid)
        cols, rows = self.cfg.coverage.cols, self.cfg.coverage.rows
        counts = np.zeros((rows, cols), dtype=int)
        size = data["rgb_size"]
        if size is not None:
            for pid in self.pose_ids(sid):
                meta = json.loads((self.pose_dir(sid, pid) / POSE_FILE).read_text())
                for x, y in meta["rgb"]["points"]:
                    col = min(cols - 1, max(0, int(x / size[0] * cols)))
                    row = min(rows - 1, max(0, int(y / size[1] * rows)))
                    counts[row, col] += 1
        return {"rgb": {"cols": cols, "rows": rows, "counts": counts.tolist()}}

    # results

    @staticmethod
    def new_result_id(now: datetime) -> str:
        """Result id r-YYYYmmdd-HHMMSS for the given time."""
        return f"{RESULT_PREFIX}{now.strftime(ID_TIME_FORMAT)}"

    def list_results(self, sid: str) -> list[dict[str, Any]]:
        """The result.json of every result of the session, newest first."""
        return [json.loads((self.result_dir(sid, rid) / RESULT_FILE).read_text()) for rid in self.result_ids(sid)]
