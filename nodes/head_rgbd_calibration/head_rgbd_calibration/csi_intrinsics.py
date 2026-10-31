"""Intrinsics calibration of the CSI cameras (front and back Camera Module 3): capture, compute and apply.

Layout under state_dir: csi/<camera>/<sid>/session.json, csi/<camera>/<sid>/views/<NNN>/{image.jpg, view.json} and
csi/<camera>/<sid>/results/<rid>/result.json. A capture subscribes `csi_cameras.<camera>.image_topic` only while it
runs and keeps a view when the ChArUco board is detected and the view moved at least `csi_min_view_shift_px` from
the last kept one. Compute fits K and plumb_bob D (shared core intrinsics.calibrate, `limits.min_views`). Apply
writes applied/<camera>_camera_calibration.yaml (ROS camera_calibration format, head_camera_cpp's camera_info_file)
and then the `csi_intrinsics.<camera>` record of applied/applied.json, which starts the apply path unit; its script
try-restarts the camera's unit only when that file changed.

Threads: the image callback runs on the ROS executor thread, the routes on HTTP threads. `lock` guards the run (the
applied.json read-modify-write goes through apply.update_applied); `feed_lock` serialises detection and view writes.
"""

import json
import re
import subprocess
import threading
from collections.abc import Callable
from dataclasses import dataclass, field
from datetime import datetime, timedelta
from functools import partial
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from fastapi import APIRouter
from pydantic import BaseModel
from ros2_common.head_rgbd_calibration.charuco import MIN_CORNERS, Detection, detect, make_board
from ros2_common.head_rgbd_calibration.frames import decode_compressed, stamp_ns
from ros2_common.head_rgbd_calibration.intrinsics import calibrate
from sensor_msgs.msg import CompressedImage

from head_rgbd_calibration.app import HTTP_CONFLICT, RunFn, error_response
from head_rgbd_calibration.apply import APPLIED_DIR, APPLIED_MARKER, unit_state, update_applied
from head_rgbd_calibration.apply import CSI_APPLIED_KEY as APPLIED_KEY
from head_rgbd_calibration.capture import Busy, rgb_qos
from head_rgbd_calibration.config import CsiCameraConfig, NodeConfig
from head_rgbd_calibration.store import (
    ID_TIME_FORMAT,
    RESULT_FILE,
    InvalidId,
    NotFound,
    write_bytes_atomic,
    write_json_atomic,
)

CSI_DIR = "csi"
SESSION_FILE = "session.json"
VIEWS_DIR = "views"
VIEW_FILE = "view.json"
IMAGE_FILE = "image.jpg"
RESULTS_DIR = "results"
KIND_CSI = "csi_intrinsics"
SESSION_PREFIX = "csi-"
RESULT_PREFIX = "r-"
CSI_SESSION_ID_RE = re.compile(r"^csi-\d{8}-\d{6}(?:-\d+)?$", re.ASCII)
CSI_RESULT_ID_RE = re.compile(r"^r-\d{8}-\d{6}(?:-\d+)?$", re.ASCII)
VIEW_ID_RE = re.compile(r"^\d{3,}$", re.ASCII)
VIEW_ID_DIGITS = 3
JPEG_QUALITY = 95
IDLE = "idle"
STOPPED = "stopped"
WAITING = "waiting for frames"
NO_BOARD = "no board in the image"
SAME_VIEW = "board found - move it to a new place or angle"
OTHER_SIZE = "frame size differs from the session's"
DROP_LOG_THROTTLE_S = 5.0
DISTORTION_MODEL = "plumb_bob"
CAMERA_NAME_SUFFIX = "_camera"


def applied_file_name(camera: str) -> str:
    """Name of a camera's applied calibration file in applied/ (head_camera_cpp's camera_info_file).

    Args:
        camera: CSI camera name ("front", "back").

    Returns:
        str: "<camera>_camera_calibration.yaml".
    """
    return f"{camera}{CAMERA_NAME_SUFFIX}_calibration.yaml"


def kept_status(count: int) -> str:
    """Status line after a kept view.

    Args:
        count: Views kept in the session.

    Returns:
        str: "view N kept - move the board".
    """
    return f"view {count} kept - move the board"


def view_differs(previous: Detection | None, detection: Detection, min_shift_px: float) -> bool:
    """Whether a detection is a new view compared with the last kept one.

    Args:
        previous: The last kept detection, None when nothing is kept yet.
        detection: The new detection.
        min_shift_px: Mean shift of the shared corners (px) a new view needs.

    Returns:
        bool: True without a previous view, when fewer than MIN_CORNERS corners are shared, or when the shared corners
        moved at least min_shift_px on average.
    """
    if previous is None:
        return True
    shared, now_index, before_index = np.intersect1d(detection.ids, previous.ids, return_indices=True)
    if len(shared) < MIN_CORNERS:
        return True
    shift = np.linalg.norm(detection.points[now_index] - previous.points[before_index], axis=1)
    return float(np.mean(shift)) >= min_shift_px


def matrix_block(matrix: np.ndarray) -> dict[str, Any]:
    """ROS camera_calibration matrix mapping.

    Args:
        matrix: 1-D or 2-D array.

    Returns:
        dict[str, Any]: {rows, cols, data} with data row-major floats.
    """
    matrix = np.atleast_2d(np.asarray(matrix, dtype=np.float64))
    return {"rows": matrix.shape[0], "cols": matrix.shape[1], "data": [float(v) for v in matrix.reshape(-1)]}


def calibration_yaml(result: dict[str, Any], camera: str) -> str:
    """The ROS camera_calibration YAML of a CSI result, as head_camera_cpp's load_camera_info reads it.

    Args:
        result: The result.json content (k 3x3, d 5, image_size [w, h]).
        camera: CSI camera name.

    Returns:
        str: YAML with image_width, image_height, camera_name, camera_matrix, distortion_model (plumb_bob),
        distortion_coefficients, rectification_matrix (identity, monocular) and projection_matrix (K | 0).
    """
    k = np.asarray(result["k"], dtype=np.float64).reshape(3, 3)
    doc = {
        "image_width": int(result["image_size"][0]),
        "image_height": int(result["image_size"][1]),
        "camera_name": f"{camera}{CAMERA_NAME_SUFFIX}",
        "camera_matrix": matrix_block(k),
        "distortion_model": DISTORTION_MODEL,
        "distortion_coefficients": matrix_block(np.asarray(result["d"], dtype=np.float64)),
        "rectification_matrix": matrix_block(np.eye(3)),
        "projection_matrix": matrix_block(np.hstack([k, np.zeros((3, 1))])),
    }
    return yaml.safe_dump(doc, sort_keys=False)


@dataclass
class CsiRun:
    """One capture run of a CSI camera.

    Attributes:
        camera: Camera name.
        session_id: Session the views are stored in.
        started_at: Start time.
        stops_at: Time the timer stops the capture.
        view_count: Views kept in the session.
        last: The last kept detection (the next view must differ from it).
        image_size: (width, height) of the session's views, None before the first.
        status: Status line shown to the operator.
        running: False once stopped.
        subscriptions: The image subscription.
        timer: The max_capture_s timer.
    """

    camera: str
    session_id: str
    started_at: datetime
    stops_at: datetime
    view_count: int
    last: Detection | None
    image_size: tuple[int, int] | None
    status: str = WAITING
    running: bool = True
    subscriptions: list[Any] = field(default_factory=list)
    timer: Any = None


class CsiCalibration:
    """Capture, compute and apply of the CSI camera intrinsics; one capture at a time."""

    def __init__(self, node: Any, cfg: NodeConfig, clock: Callable[[], datetime]) -> None:
        """Create an idle controller (nothing is subscribed until start).

        Args:
            node: The rclpy node the subscription and the timer are created on.
            cfg: Node config (state_dir, csi_cameras, csi_min_view_shift_px, board, limits, coverage, max_capture_s).
            clock: Returns the current time; it names sessions and results.
        """
        self.node = node
        self.cfg = cfg
        self.clock = clock
        self.logger = node.get_logger()
        self.board = make_board(cfg.board)
        self.root = Path(cfg.state_dir) / CSI_DIR
        self.lock = threading.Lock()
        self.feed_lock = threading.Lock()
        self.run: CsiRun | None = None

    # paths

    def camera_config(self, camera: str) -> CsiCameraConfig:
        """The config of a camera; NotFound for a camera not in csi_cameras."""
        found = self.cfg.csi_cameras.get(camera)
        if found is None:
            raise NotFound(f"no such CSI camera: {camera} (configured: {', '.join(self.cfg.csi_cameras)})")
        return found

    def session_dir(self, camera: str, sid: str) -> Path:
        """Directory of an existing session of a camera (NotFound, InvalidId)."""
        self.camera_config(camera)
        if not CSI_SESSION_ID_RE.fullmatch(sid):
            raise InvalidId(f"invalid CSI session id: {sid!r}")
        path = self.root / camera / sid
        if not (path / SESSION_FILE).is_file():
            raise NotFound(f"no such {camera} session: {sid}")
        return path

    def result_dir(self, camera: str, sid: str, rid: str) -> Path:
        """Directory of a result (InvalidId for a bad id; it need not exist)."""
        if not CSI_RESULT_ID_RE.fullmatch(rid):
            raise InvalidId(f"invalid result id: {rid!r}")
        return self.session_dir(camera, sid) / RESULTS_DIR / rid

    def view_dirs(self, camera: str, sid: str) -> list[Path]:
        """Stored view directories of a session, ascending."""
        views = self.session_dir(camera, sid) / VIEWS_DIR
        if not views.is_dir():
            return []
        return sorted(p for p in views.iterdir() if VIEW_ID_RE.fullmatch(p.name) and (p / VIEW_FILE).is_file())

    def detections(self, camera: str, sid: str) -> list[Detection]:
        """The stored detections of a session, in view order."""
        found = []
        for path in self.view_dirs(camera, sid):
            meta = json.loads((path / VIEW_FILE).read_text())
            found.append(
                Detection(
                    ids=np.asarray(meta["ids"], dtype=np.int32), points=np.asarray(meta["points"], dtype=np.float32)
                )
            )
        return found

    def session_ids(self, camera: str) -> list[str]:
        """Session ids of a camera, newest first."""
        path = self.root / camera
        if not path.is_dir():
            return []
        found = [p for p in path.iterdir() if CSI_SESSION_ID_RE.fullmatch(p.name) and (p / SESSION_FILE).is_file()]
        created = {p.name: json.loads((p / SESSION_FILE).read_text())["created"] for p in found}
        return sorted(created, key=lambda sid: (created[sid], sid), reverse=True)

    def create_session(self, camera: str, now: datetime) -> str:
        """Create a session of a camera named csi-YYYYmmdd-HHMMSS (-N appended when that second is taken)."""
        base = f"{SESSION_PREFIX}{now.strftime(ID_TIME_FORMAT)}"
        parent = self.root / camera
        parent.mkdir(parents=True, exist_ok=True)
        sid, attempt = base, 1
        while True:
            try:
                (parent / sid).mkdir()
                break
            except FileExistsError:
                attempt += 1
                sid = f"{base}-{attempt}"
        doc = {"id": sid, "camera": camera, "created": now.isoformat(), "image_size": None}
        write_json_atomic(parent / sid / SESSION_FILE, doc)
        return sid

    # state

    def coverage(self, camera: str, sid: str, image_size: list[int] | None) -> dict[str, Any]:
        """Corner counts per image grid cell over the session's views ({cols, rows, counts}, all zero without)."""
        cols, rows = self.cfg.coverage.cols, self.cfg.coverage.rows
        counts = np.zeros((rows, cols), dtype=int)
        if image_size is not None:
            for detection in self.detections(camera, sid):
                for x, y in detection.points:
                    col = min(cols - 1, max(0, int(x / image_size[0] * cols)))
                    row = min(rows - 1, max(0, int(y / image_size[1] * rows)))
                    counts[row, col] += 1
        return {"cols": cols, "rows": rows, "counts": counts.tolist()}

    def session_state(self, camera: str, sid: str) -> dict[str, Any]:
        """{id, created, view_count, image_size, coverage, results} of a session."""
        doc = json.loads((self.session_dir(camera, sid) / SESSION_FILE).read_text())
        return {
            "id": sid,
            "created": doc["created"],
            "view_count": len(self.view_dirs(camera, sid)),
            "image_size": doc["image_size"],
            "coverage": self.coverage(camera, sid, doc["image_size"]),
            "results": self.results(camera, sid),
        }

    def capture_state(self) -> dict[str, Any]:
        """{running, camera, session_id, status, view_count, started_at, stops_at} of the latest capture."""
        with self.lock:
            run = self.run
            if run is None:
                return {
                    "running": False,
                    "camera": None,
                    "session_id": None,
                    "status": IDLE,
                    "view_count": 0,
                    "started_at": None,
                    "stops_at": None,
                }
            return {
                "running": run.running,
                "camera": run.camera,
                "session_id": run.session_id,
                "status": run.status,
                "view_count": run.view_count,
                "started_at": run.started_at.isoformat(),
                "stops_at": run.stops_at.isoformat() if run.running else None,
            }

    def state(self, camera: str) -> dict[str, Any]:
        """The state of one camera for the HTTP API.

        Args:
            camera: CSI camera name.

        Returns:
            {camera, capture, session, applied}: capture is the latest capture (of any camera); session is the
            capture's session when it is this camera's, else the camera's newest session, else None; applied is the
            camera's applied.json record or None.

        Raises:
            NotFound: the camera is not configured.
        """
        self.camera_config(camera)
        capture = self.capture_state()
        sid = capture["session_id"] if capture["camera"] == camera else None
        if sid is None:
            sid = next(iter(self.session_ids(camera)), None)
        return {
            "camera": camera,
            "capture": capture,
            "session": None if sid is None else self.session_state(camera, sid),
            "applied": self.applied_record(camera),
        }

    # capture

    def start(self, camera: str, session_id: str | None) -> dict[str, Any]:
        """Start a capture on a new session of the camera, or on an existing one to add views.

        Args:
            camera: CSI camera name.
            session_id: Session to reopen, None for a new one.

        Returns:
            The state(camera) after the start.

        Raises:
            Busy: a capture is running.
            NotFound, InvalidId: unknown camera or session.
        """
        settings = self.camera_config(camera)
        with self.lock:
            if self.run is not None and self.run.running:
                raise Busy(f"a {self.run.camera} capture is already running")
            last, size, count = None, None, 0
            if session_id is not None:
                kept = self.detections(camera, session_id)
                doc = json.loads((self.session_dir(camera, session_id) / SESSION_FILE).read_text())
                last, count = (kept[-1] if kept else None), len(kept)
                size = (int(doc["image_size"][0]), int(doc["image_size"][1])) if doc["image_size"] else None
            started = self.clock()
            sid = session_id or self.create_session(camera, started)
            run = CsiRun(
                camera=camera,
                session_id=sid,
                started_at=started,
                stops_at=started + timedelta(seconds=self.cfg.max_capture_s),
                view_count=count,
                last=last,
                image_size=size,
            )
            try:
                topic = settings.image_topic
                callback = partial(self.on_image, run, topic)
                run.subscriptions.append(
                    self.node.create_subscription(CompressedImage, topic, callback, rgb_qos(self.cfg.qos.rgb))
                )
                run.timer = self.node.create_timer(self.cfg.max_capture_s, partial(self.expire, run))
            except BaseException:
                run.running = False
                self.release(run)
                raise
            self.run = run
        self.logger.info(f"{camera} intrinsics capture started on session {sid}")
        return self.state(camera)

    def stop(self, camera: str | None = None) -> dict[str, Any] | None:
        """Stop the running capture.

        Args:
            camera: Stop only when the capture is this camera's (its state is returned); None stops any capture.

        Returns:
            The state(camera) after the stop, None when camera is None.
        """
        if camera is not None:
            self.camera_config(camera)
        with self.lock:
            run = self.run
        if run is not None and (camera is None or run.camera == camera):
            self.finish(run, STOPPED)
        return None if camera is None else self.state(camera)

    def expire(self, run: CsiRun) -> None:
        """Timer callback: stop this capture after max_capture_s."""
        self.finish(run, f"{STOPPED}: max_capture_s ({self.cfg.max_capture_s:g} s) reached")

    def finish(self, run: CsiRun, reason: str) -> None:
        """Mark the run stopped with reason and destroy its subscription and timer (once)."""
        with self.lock:
            if not run.running:
                return
            run.running = False
            run.status = reason
        self.release(run)
        self.logger.info(f"{run.camera} intrinsics capture on session {run.session_id} {reason}")

    def release(self, run: CsiRun) -> None:
        """Destroy the subscription and the timer of a run; a failing destroy does not stop the other."""
        handles, run.subscriptions = run.subscriptions, []
        timer, run.timer = run.timer, None
        for sub in handles:
            try:
                self.node.destroy_subscription(sub)
            except Exception as exc:  # keep destroying the rest
                self.logger.error(f"destroying a CSI capture subscription failed: {exc}")
        if timer is not None:
            try:
                timer.cancel()
                self.node.destroy_timer(timer)
            except Exception as exc:  # logged, the subscription is gone already
                self.logger.error(f"destroying the CSI capture timer failed: {exc}")

    def on_image(self, run: CsiRun, topic: str, msg: Any) -> None:
        """Image callback (executor thread): keep the frame as a view when the board is found and the view is new."""
        if not run.running:
            return
        raw = bytes(msg.data)
        try:
            image = decode_compressed(msg, topic)
        except ValueError as exc:
            self.logger.warning(f"dropped a frame on {topic}: {exc}", throttle_duration_sec=DROP_LOG_THROTTLE_S)
            return
        stamp = stamp_ns(msg)
        with self.feed_lock:
            if not run.running:
                return
            try:
                status = self.judge(run, image, raw, stamp)
            except Exception as exc:  # an executor callback must not raise; the operator sees the reason
                status = f"view not stored: {exc}"
                self.logger.error(status)
            with self.lock:
                if run.running:
                    run.status = status

    def judge(self, run: CsiRun, image: np.ndarray, raw: bytes, stamp: int) -> str:
        """Detect the board and store the view when it is new; returns the status line."""
        size = (int(image.shape[1]), int(image.shape[0]))
        if run.image_size is not None and size != run.image_size:
            return OTHER_SIZE
        detection = detect(image, self.board)
        if detection is None:
            return NO_BOARD
        if not view_differs(run.last, detection, self.cfg.csi_min_view_shift_px):
            return SAME_VIEW
        session = self.session_dir(run.camera, run.session_id)
        if run.image_size is None:
            doc = json.loads((session / SESSION_FILE).read_text())
            write_json_atomic(session / SESSION_FILE, {**doc, "image_size": list(size)})
            run.image_size = size
        number = run.view_count + 1
        view = session / VIEWS_DIR / f"{number:0{VIEW_ID_DIGITS}d}"
        view.mkdir(parents=True, exist_ok=True)
        write_bytes_atomic(view / IMAGE_FILE, raw)
        meta = {
            "stamp_ns": stamp,
            "ids": [int(i) for i in detection.ids],
            "points": [[float(x), float(y)] for x, y in detection.points],
        }
        write_json_atomic(view / VIEW_FILE, meta)
        run.last = detection
        with self.lock:
            run.view_count = number
        self.logger.info(f"{run.camera} session {run.session_id}: view {number} kept")
        return kept_status(number)

    # compute

    def compute(self, camera: str, sid: str) -> dict[str, Any]:
        """Fit K and plumb_bob D over the session's views and store result.json.

        Args:
            camera: CSI camera name.
            sid: Session id.

        Returns:
            The result: {kind: "csi_intrinsics", camera, session, result, created, image_size [w, h], k (3x3), d (5),
            rms, per_view_errors, view_count}.

        Raises:
            Busy: the session is capturing.
            ValueError: fewer than limits.min_views views (the message names the count).
            NotFound, InvalidId: unknown camera or session.
        """
        path = self.session_dir(camera, sid)
        with self.lock:
            if self.run is not None and self.run.running and (self.run.camera, self.run.session_id) == (camera, sid):
                raise Busy("the session is capturing; stop it before computing")
        doc = json.loads((path / SESSION_FILE).read_text())
        detections = self.detections(camera, sid)
        if doc["image_size"] is None or not detections:
            raise ValueError(f"calibration needs at least {self.cfg.limits.min_views} views, got 0 views")
        fitted = calibrate(detections, self.board, tuple(doc["image_size"]), self.cfg.limits)
        now = self.clock()
        base = f"{RESULT_PREFIX}{now.strftime(ID_TIME_FORMAT)}"
        rid, attempt = base, 1
        while (path / RESULTS_DIR / rid).exists():
            attempt += 1
            rid = f"{base}-{attempt}"
        result = {
            "kind": KIND_CSI,
            "camera": camera,
            "session": sid,
            "result": rid,
            "created": now.isoformat(),
            "image_size": list(doc["image_size"]),
            "k": np.asarray(fitted.k, dtype=np.float64).tolist(),
            "d": [float(v) for v in fitted.d],
            "rms": float(fitted.rms),
            "per_view_errors": [float(v) for v in fitted.per_view_errors],
            "view_count": fitted.view_count,
        }
        target = path / RESULTS_DIR / rid
        target.mkdir(parents=True)
        write_json_atomic(target / RESULT_FILE, result)
        self.logger.info(f"{camera} session {sid}: result {rid} rms {fitted.rms:.3f} px")
        return result

    def results(self, camera: str, sid: str) -> list[dict[str, Any]]:
        """The results of a session, newest first."""
        results = self.session_dir(camera, sid) / RESULTS_DIR
        if not results.is_dir():
            return []
        found = [p for p in results.iterdir() if CSI_RESULT_ID_RE.fullmatch(p.name) and (p / RESULT_FILE).is_file()]
        docs = [json.loads((p / RESULT_FILE).read_text()) for p in found]
        return sorted(docs, key=lambda d: (d["created"], d["result"]), reverse=True)

    def result(self, camera: str, sid: str, rid: str) -> dict[str, Any]:
        """One result.json (NotFound when absent)."""
        path = self.result_dir(camera, sid, rid) / RESULT_FILE
        if not path.is_file():
            raise NotFound(f"no such result: {rid}")
        return json.loads(path.read_text())

    # apply

    def applied_record(self, camera: str) -> dict[str, Any] | None:
        """The camera's record in applied.json, None when it has none."""
        marker = Path(self.cfg.state_dir) / APPLIED_DIR / APPLIED_MARKER
        if not marker.is_file():
            return None
        records = json.loads(marker.read_text()).get(APPLIED_KEY) or {}
        return records.get(camera)

    def apply(self, camera: str, sid: str, rid: str, run: RunFn) -> dict[str, Any]:
        """Write applied/<camera>_camera_calibration.yaml, then record it in applied.json (both atomically).

        Args:
            camera: CSI camera name.
            sid: Session id.
            rid: Result id.
            run: subprocess.run-compatible callable for the unit state.

        Returns:
            {applied: {kind: "csi_intrinsics", camera, session, result, applied_at}, units: [{name, active_state,
            restarted}]} with the camera's unit.

        Raises:
            NotFound, InvalidId: unknown camera, session or result.
        """
        settings = self.camera_config(camera)
        result = self.result(camera, sid, rid)
        now = self.clock()
        record = {"kind": KIND_CSI, "camera": camera, "session": sid, "result": rid, "applied_at": now.isoformat()}
        target = Path(self.cfg.state_dir) / APPLIED_DIR

        def add_record(kept: dict[str, Any]) -> dict[str, Any]:
            return {**kept, APPLIED_KEY: {**(kept.get(APPLIED_KEY) or {}), camera: record}}

        update_applied(target, add_record, [(applied_file_name(camera), calibration_yaml(result, camera).encode())])
        self.logger.info(f"{camera} intrinsics {sid}/{rid} applied")
        return {"applied": record, "units": [unit_state(settings.unit, now.timestamp(), run)]}


class CsiStartRequest(BaseModel):
    """Body of POST /api/csi/{camera}/start."""

    session_id: str | None = None


def csi_router(csi: CsiCalibration, run: RunFn = subprocess.run) -> APIRouter:
    """Routes of the CSI intrinsics calibration under /api/csi/{camera} (web_ui proxies /api/calibration/rgbd/*).

    Errors use the common {ok: false, message} body of build_app: an unknown camera, session or result is 404, a
    running capture is 409, too few views 409 naming the count.

    Args:
        csi: The controller.
        run: subprocess.run-compatible callable for the unit state after an apply.

    Returns:
        APIRouter: state, start, stop, compute, results and apply routes.
    """
    router = APIRouter(prefix="/api/csi/{camera}")

    @router.get("/state")
    def state(camera: str) -> dict[str, Any]:
        return csi.state(camera)

    @router.post("/start")
    def start(camera: str, body: CsiStartRequest | None = None) -> dict[str, Any]:
        return csi.start(camera, body.session_id if body else None)

    @router.post("/stop")
    def stop(camera: str) -> dict[str, Any] | None:
        return csi.stop(camera)

    @router.post("/sessions/{sid}/compute")
    def compute(camera: str, sid: str) -> Any:
        try:
            return csi.compute(camera, sid)
        except ValueError as exc:
            if isinstance(exc, InvalidId):
                raise
            return error_response(HTTP_CONFLICT, str(exc))

    @router.get("/sessions/{sid}/results")
    def results(camera: str, sid: str) -> list[dict[str, Any]]:
        return csi.results(camera, sid)

    @router.get("/sessions/{sid}/results/{rid}")
    def result(camera: str, sid: str, rid: str) -> dict[str, Any]:
        return csi.result(camera, sid, rid)

    @router.post("/sessions/{sid}/results/{rid}/apply")
    def apply(camera: str, sid: str, rid: str) -> dict[str, Any]:
        return csi.apply(camera, sid, rid, run)

    return router
