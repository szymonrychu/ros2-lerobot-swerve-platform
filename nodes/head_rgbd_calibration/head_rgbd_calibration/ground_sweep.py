"""Ground board sweep (user decision 312): the FULL ChArUco board lies flat on the floor, close and fully visible at the
homed head; the head sweeps a pan/tilt grid around it and the node stores the board corners, the colour frames, the
aligned depth stats and the depth floor planes at each pose, for 1 to `max_placements` board placements.

Per placement: the head goes home (`fit.home_pan_deg`, `fit.home_tilt_deg`); the board must show at least
`home_min_corners` corners there; it is located on the floor from those corners (rays through the applied head chain
meet the board plane, a 2D rigid fit: no PnP, so no planar flip); the head aims at it (`aim_head`) and visits the
offset grid around the aim (`plan_grid`: tilt clamped, poses whose predicted board leaves the image dropped before the
head moves). At each pose, once the head is reached and still and `floor.settle_s` has passed, up to
`frames_per_pose` colour frames are kept that show at least `min_corners` corners (detection retried at
`detect_upscale` times the size for the small oblique markers), each with the measured head angles nearest its stamp,
the received JPEG, and the aligned depth bias against the board plane (PnP seeded from the located board, so no flip;
`calibrate.board_plane_bias_mm`); then the depth floor planes of `floor.clouds_per_pose` clouds as the floor sweep
fits them (the cross-check). A pose with fewer than `min_frames_per_pose` frames is skipped with its reasons. After the
placement the head returns home and the sweep awaits the next placement (`next_placement`, after the user moved the
board) or the end (`stop`, or `placement_wait_s` without the next one). Every exit commands the head back to where it
was at start, writes observations.json and releases the subscriptions and the publisher.

Layout: ground_board/<sid>/observations.json and ground_board/<sid>/frames/<view_id>.jpg.

The head moves only through the floor sweep's command path (FloorSweep.publish on `floor.command_topic`, the motion
and stillness settings of `floor`); nothing else is published.

Telemetry: calib_event lines of flow `ground_sweep` with the session id as run id: the start (config, start pan/tilt)
or the refused start, per placement its start, the located board (home corners, upscale, aim, grid, dropped poses) or
its failure, each pose (observed: corners, upscale and depth bias per frame, planes, reached angles; skipped at
WARNING with the reasons), the awaiting and the next placement, the restore and the end (done, stopped or error with
the counts).
"""

import logging
import math
import threading
from collections import Counter, deque
from collections.abc import Callable
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from ros2_common.head_rgbd_calibration.calibrate import board_plane_bias_mm
from ros2_common.head_rgbd_calibration.chain import UrdfChain
from ros2_common.head_rgbd_calibration.charuco import Detection, board_corners_3d, board_outline, detect, make_board
from ros2_common.head_rgbd_calibration.floor_fit import PAN_JOINT, TILT_JOINT
from ros2_common.head_rgbd_calibration.frames import (
    CameraInfoData,
    decode_camera_info,
    decode_compressed,
    decode_mono16,
    stamp_ns,
)
from ros2_common.head_rgbd_calibration.ground_board_fit import board_on_floor, locate_board
from ros2_common.head_rgbd_calibration.head_arm_fit import aim_head, board_pnp, camera_pose
from ros2_common.head_rgbd_calibration.intrinsics import IntrinsicsResult
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, JointState, PointCloud2

from head_rgbd_calibration.capture import reliable_qos, rgb_qos
from head_rgbd_calibration.config import MAX_FLOOR_PAN_DEG, GroundBoardConfig, NodeConfig
from head_rgbd_calibration.floor_sweep import (
    DONE,
    ERROR,
    NS_PER_S,
    POLL_S,
    PUBLISHER_DEPTH,
    STOPPED,
    SUBSCRIPTION_DEPTH,
    Busy,
    FloorSweep,
    JointSample,
    Run,
    Stopped,
    SweepError,
)
from head_rgbd_calibration.store import write_bytes_atomic, write_json_atomic
from head_rgbd_calibration.telemetry import calib_event

LOG = logging.getLogger(__name__)

GROUND_DIR = "ground_board"
FRAMES_DIR = "frames"
OBSERVATIONS_FILE = "observations.json"
SESSION_FORMAT = "ground-%Y%m%d-%H%M%S"
FLOW = "ground_sweep"
SWEEP_STEP = "sweep"
STARTING = "starting"
HOMING = "homing"
LOCATING = "locating"
SWEEPING = "sweeping"
AWAITING = "awaiting_placement"
COLOUR_QUEUE = 8
DEPTH_HISTORY = 16
AWAIT_POLL_S = 0.05
GRID_DECIMALS = 3
REASON_NO_FRAME = "no colour frame within frame_timeout_s"
REASON_NO_JOINT = "no joint state within max_stamp_dt_s"
REASON_OFF_TARGET = "head not at the target at the frame stamp"
REASON_UNDECODABLE = "colour frame does not decode"
REASON_FEW_CORNERS = "fewer than {corners} corners"
REASON_NO_INFO = "no colour camera_info yet"
HOME_FAILED = "board not detected at home with {corners}+ corners: lay the FULL board flat, close and fully in view"
HOME_UNREACHED = "the head did not reach home within {timeout:g} s"
NOT_LOCATED = "the board could not be located on the floor from the home view"
NOT_AWAITED = "no placement is awaited: start a session or wait for the placement to finish"
MSG_AWAITING = "move the board to a new place on the floor (close, fully visible at home), then press Next placement"


@dataclass
class GroundRun(Run):
    """One ground board session: the floor sweep run plus the board views, placements and the camera_info.

    Attributes:
        views: Kept board views (observations.json `views`).
        placements: Completed placements (located board, aim, grid, counts).
        failures: Placements that failed before any view (reason).
        camera_info: The colour camera_info of the session (the first one received).
        next_pose: Next pose index (unique over the session).
    """

    views: list[dict[str, Any]] = field(default_factory=list)
    placements: list[dict[str, Any]] = field(default_factory=list)
    failures: list[dict[str, Any]] = field(default_factory=list)
    camera_info: CameraInfoData | None = None
    next_pose: int = 0


def detect_frame(
    image: np.ndarray, board: cv2.aruco.CharucoBoard, min_corners: int, upscale: int
) -> tuple[Detection | None, int]:
    """Detect the board, retrying at `upscale` times the size when too few corners come back.

    Args:
        image: BGR colour frame.
        board: The ChArUco board (make_board).
        min_corners: Corners a detection needs.
        upscale: Enlargement of the retry (1 disables it).

    Returns:
        tuple[Detection | None, int]: The detection with the most corners if it has at least `min_corners` (else
        None), and the upscale of the last attempt.
    """
    best = detect(image, board, 1)
    used = 1
    if (best is None or len(best.ids) < min_corners) and upscale > 1:
        used = upscale
        retry = detect(image, board, upscale)
        if retry is not None and (best is None or len(retry.ids) > len(best.ids)):
            best = retry
    if best is None or len(best.ids) < min_corners:
        return None, used
    return best, used


def plan_grid(
    chain: UrdfChain,
    board_pose: np.ndarray,
    aim: tuple[float, float],
    info: Any,
    cfg: GroundBoardConfig,
    camera_frame: str,
) -> tuple[list[tuple[float, float]], int]:
    """The pan/tilt poses around the aim that keep the whole board in view (predicted through the applied chain).

    Args:
        chain: Head URDF chain (the applied chain).
        board_pose: 4x4 located board pose in base_link (`board_on_floor`).
        aim: (pan, tilt) rad that put the board centre on the optical axis.
        info: Colour camera info (width, height, k row-major, d).
        cfg: Ground board config (offsets, tilt clamp, image margin).
        camera_frame: Colour optical frame.

    Returns:
        tuple[list[tuple[float, float]], int]: (pan, tilt) deg in serpentine order (one tilt row at a time, the pan
        direction alternating), duplicates after the clamp removed; and the number of poses dropped.
    """
    k = np.asarray(info.k, dtype=float).reshape(3, 3)
    d = np.asarray(info.d, dtype=float)
    corners = board_corners_3d(cfg.fit.board)
    margin = cfg.image_margin_px
    grid: list[tuple[float, float]] = []
    dropped = 0
    for row, d_tilt in enumerate(cfg.tilt_offsets_deg):
        pans = cfg.pan_offsets_deg if row % 2 == 0 else cfg.pan_offsets_deg[::-1]
        for d_pan in pans:
            pan = float(np.clip(math.degrees(aim[0]) + d_pan, -MAX_FLOOR_PAN_DEG, MAX_FLOOR_PAN_DEG))
            tilt = float(np.clip(math.degrees(aim[1]) + d_tilt, cfg.min_tilt_deg, cfg.max_tilt_deg))
            pose = (round(pan, GRID_DECIMALS), round(tilt, GRID_DECIMALS))
            if pose in grid:
                continue
            joints = {PAN_JOINT: math.radians(pose[0]), TILT_JOINT: math.radians(pose[1])}
            in_camera = np.linalg.solve(camera_pose(chain, joints, None, camera_frame), board_pose)
            depth = (in_camera[:3, :3] @ corners.T).T[:, 2] + in_camera[2, 3]
            points = cv2.projectPoints(corners, cv2.Rodrigues(in_camera[:3, :3])[0], in_camera[:3, 3], k, d)[0]
            points = points.reshape(-1, 2)
            inside = (
                (depth > 0).all()
                and (points[:, 0] >= margin).all()
                and (points[:, 0] <= info.width - margin).all()
                and (points[:, 1] >= margin).all()
                and (points[:, 1] <= info.height - margin).all()
            )
            if inside:
                grid.append(pose)
            else:
                dropped += 1
    return grid, dropped


def info_json(info: CameraInfoData) -> dict[str, Any]:
    """The camera_info fields the fit needs, plain JSON."""
    return {
        "width": int(info.width),
        "height": int(info.height),
        "distortion_model": info.distortion_model,
        "k": [float(v) for v in np.asarray(info.k).reshape(-1)],
        "d": [float(v) for v in np.asarray(info.d).reshape(-1)],
    }


class GroundSweep(FloorSweep):
    """Runs one ground board session at a time on a worker thread (FloorSweep provides the head motion)."""

    flow = FLOW

    def __init__(
        self,
        node: Any,
        cfg: NodeConfig,
        urdf_text: Callable[[], str],
        clock: Callable[[], float],
        sleep: Callable[[float], None],
        now: Callable[[], datetime] = datetime.now,
        blocked: Callable[[], str | None] | None = None,
    ) -> None:
        """Create an idle sweep (nothing is subscribed or published until start).

        Args:
            node: The rclpy node the subscriptions and the publisher are created on.
            cfg: Node config (state_dir, topics, camera_frame, floor motion and plane settings, ground_board).
            urdf_text: Returns the applied head URDF text (locating, aiming, the grid and the plane prior).
            clock: Monotonic seconds; all waits are measured on it.
            sleep: Sleeps seconds (the injected clock's sleep in tests).
            now: Wall time, names the session.
            blocked: Returns a reason while something else forbids a session, else None.
        """
        super().__init__(node, cfg, urdf_text, clock, sleep, now, blocked)
        self.ground = cfg.ground_board
        self.charuco = make_board(self.ground.fit.board)
        self.colour: deque[tuple[int, int, Any]] = deque(maxlen=COLOUR_QUEUE)
        self.colour_seq = 0
        self.depth: deque[tuple[int, Any]] = deque(maxlen=DEPTH_HISTORY)
        self.camera_info: CameraInfoData | None = None
        self.wake = threading.Event()
        self.placement = 0
        self.placements_done = 0
        self.view_count = 0
        self.plane_count = 0
        self.message: str | None = None
        self.board: dict[str, float] | None = None

    def state(self) -> dict[str, Any]:
        """The session state for the HTTP API.

        Returns:
            {running, status, session_id, placement, placements_done, max_placements, awaiting_placement, pose, total,
            views, planes, skipped, board, message, error}: placement is the 0-based placement being captured (the
            next one while awaiting), pose the 1-based grid pose of it and total its grid size, board the located
            board {x_m, y_m, yaw_deg} of the current placement (None before), message an operator hint.
        """
        with self.lock:
            return {
                "running": self.running,
                "status": self.status,
                "session_id": self.session_id,
                "placement": self.placement,
                "placements_done": self.placements_done,
                "max_placements": self.ground.max_placements,
                "awaiting_placement": self.status == AWAITING,
                "pose": self.pose,
                "total": self.total,
                "views": self.view_count,
                "planes": self.plane_count,
                "skipped": self.skipped_count,
                "board": self.board,
                "message": self.message,
                "error": self.error,
            }

    def stop(self) -> dict[str, Any]:
        """End the session: a running placement stops (its views so far are kept), an awaited one is not taken.

        Returns:
            The state() after the request.
        """
        self.stop_event.set()
        self.wake.set()
        return self.state()

    def next_placement(self) -> dict[str, Any]:
        """Capture the next placement (the user moved the board).

        Returns:
            The state() after the request.

        Raises:
            Busy: no placement is awaited.
        """
        with self.lock:
            if not self.running or self.status != AWAITING:
                raise Busy(NOT_AWAITED)
            self.status = STARTING
            self.message = None
        self.wake.set()
        return self.state()

    def start(self) -> str:
        """Start a session and its first placement.

        Returns:
            The session id.

        Raises:
            Busy: a session runs, or `blocked` names a reason.
            SweepError: the head URDF is unusable or no head joint state arrives within move_timeout_s (nothing moved).
        """
        with self.lock:
            if self.running:
                raise Busy("a ground board session is already running")
            reason = self.blocked() if self.blocked is not None else None
            if reason:
                raise Busy(reason)
            self.running = True
            self.status = STARTING
            self.error = self.message = self.session_id = self.board = None
            self.pose = self.total = self.skipped_count = self.view_count = self.plane_count = 0
            self.placement = self.placements_done = 0
        handles: list[Any] = []
        publisher: Any = None
        try:
            try:
                chain = UrdfChain.from_text(self.urdf_text())
            except (ValueError, KeyError, OSError) as exc:
                raise SweepError(f"cannot read the head URDF: {exc}") from exc
            self.stop_event.clear()
            self.wake.clear()
            self.samples.clear()
            self.clouds.clear()
            self.colour.clear()
            self.depth.clear()
            self.camera_info = None
            topics = self.cfg.topics
            qos = self.cfg.qos
            handles += [
                self.node.create_subscription(
                    JointState, self.floor.joint_states_topic, self.on_joint_state, rgb_qos(SUBSCRIPTION_DEPTH)
                ),
                self.node.create_subscription(
                    PointCloud2, self.floor.cloud_topic, self.on_cloud, rgb_qos(SUBSCRIPTION_DEPTH)
                ),
                self.node.create_subscription(CompressedImage, topics.rgb, self.on_colour, rgb_qos(qos.rgb)),
                self.node.create_subscription(Image, topics.depth, self.on_depth, reliable_qos(qos.depth)),
                self.node.create_subscription(CameraInfo, topics.rgb_info, self.on_info, reliable_qos(1)),
            ]
            publisher = self.node.create_publisher(JointState, self.floor.command_topic, PUBLISHER_DEPTH)
            start_sample = self.wait_first_sample()
            run = self.open_ground_run(start_sample, handles, publisher)
        except BaseException as exc:
            self.release(handles, publisher, None)
            with self.lock:
                self.running = False
                self.status = ERROR
                self.error = str(exc)
            calib_event(LOG, FLOW, None, SWEEP_STEP, ERROR, logging.ERROR, str(exc), error_type=type(exc).__name__)
            raise
        with self.lock:
            self.session_id = run.session_id
        calib_event(
            LOG,
            FLOW,
            run.session_id,
            SWEEP_STEP,
            "start",
            start_pan_deg=math.degrees(run.start_pan),
            start_tilt_deg=math.degrees(run.start_tilt),
            config=self.ground.model_dump(exclude={"fit", "verify"}),
            home={"pan_deg": self.ground.fit.home_pan_deg, "tilt_deg": self.ground.fit.home_tilt_deg},
            board=self.ground.fit.board.name,
            motion=self.floor.model_dump(exclude={"fit", "pan_deg", "tilt_deg"}),
        )
        self.thread = threading.Thread(target=self.work, args=(run, chain), name="ground-sweep", daemon=True)
        self.thread.start()
        return run.session_id

    def open_ground_run(self, start: JointSample, handles: list[Any], publisher: Any) -> GroundRun:
        """Create the session directory (a taken second gets a -N suffix) and the run record."""
        created = self.now()
        base = created.strftime(SESSION_FORMAT)
        root = Path(self.cfg.state_dir) / GROUND_DIR
        root.mkdir(parents=True, exist_ok=True)
        session_id, number = base, 1
        while True:
            try:
                directory = root / session_id
                directory.mkdir()
                break
            except FileExistsError:
                number += 1
                session_id = f"{base}-{number}"
        (directory / FRAMES_DIR).mkdir()
        return GroundRun(
            session_id=session_id,
            directory=directory,
            created=created,
            grid=[],
            start_pan=start.pan,
            start_tilt=start.tilt,
            subscriptions=handles,
            publisher=publisher,
            observations=[],
            skipped=[],
            dropped=Counter(),
        )

    # executor-thread callbacks

    def on_colour(self, msg: Any) -> None:
        """Queue a colour frame; decoding happens on the worker thread."""
        with self.lock:
            self.colour_seq += 1
            self.colour.append((self.colour_seq, stamp_ns(msg), msg))

    def on_depth(self, msg: Any) -> None:
        """Keep recent aligned depth frames by stamp (decoded on the worker thread)."""
        with self.lock:
            self.depth.append((stamp_ns(msg), msg))

    def on_info(self, msg: Any) -> None:
        """Keep the latest colour camera_info."""
        info = decode_camera_info(msg)
        with self.lock:
            self.camera_info = info

    # worker thread

    def work(self, run: GroundRun, chain: UrdfChain) -> None:
        """Worker: capture placements until the last one, a stop or the placement wait; then always restore the head,
        write observations.json and release the handles."""
        final, error = DONE, None
        try:
            self.session(run, chain)
        except Stopped:
            final = STOPPED
        except SweepError as exc:
            final, error = ERROR, str(exc)
        except Exception as exc:  # reported as the session error, the finally block still restores the head
            final, error = ERROR, f"{type(exc).__name__}: {exc}"
            LOG.exception("ground sweep %s raised", run.session_id)
        finally:
            self.restore(run)
            try:
                self.write(run)
            except OSError as exc:
                final, error = ERROR, f"cannot write {OBSERVATIONS_FILE}: {exc}"
            self.release(run.subscriptions, run.publisher, run.session_id)
            with self.lock:
                self.running = False
                self.status = final
                self.error = error
                self.message = None
        calib_event(
            LOG,
            FLOW,
            run.session_id,
            SWEEP_STEP,
            final,
            logging.ERROR if error else (logging.WARNING if final == STOPPED else logging.INFO),
            error,
            placements=len(run.placements),
            placement_failures=len(run.failures),
            views=len(run.views),
            planes=len(run.observations),
            skipped=len(run.skipped),
            skip_reasons=Counter(skip["reason"] for skip in run.skipped),
            dropped=run.dropped,
            directory=run.directory,
        )

    def session(self, run: GroundRun, chain: UrdfChain) -> None:
        """Capture placements; between them await `next_placement` (or the end)."""
        while True:
            self.capture_placement(run, chain)
            self.write(run)
            if len(run.placements) >= self.ground.max_placements:
                return
            if not self.await_placement(run):
                return

    def await_placement(self, run: GroundRun) -> bool:
        """Wait for `next_placement`; False on stop or after `placement_wait_s`.

        Args:
            run (GroundRun): The session.

        Returns:
            bool: True when the next placement was asked for.
        """
        with self.lock:
            self.wake.clear()  # under the lock, before AWAITING: a next_placement cannot land between them and be lost
            self.status = AWAITING
            self.placement = len(run.placements)
            if self.message is None:
                self.message = MSG_AWAITING
        calib_event(LOG, FLOW, run.session_id, "placement", "awaiting", placements=len(run.placements))
        began = self.clock()
        while True:
            if self.stop_event.is_set():
                return False
            if self.wake.wait(AWAIT_POLL_S):
                if self.stop_event.is_set():
                    return False
                calib_event(LOG, FLOW, run.session_id, "placement", "next", placement=len(run.placements))
                return True
            if self.clock() - began >= self.ground.placement_wait_s:
                calib_event(
                    LOG,
                    FLOW,
                    run.session_id,
                    "placement",
                    "wait_ended",
                    logging.WARNING,
                    f"no next placement within {self.ground.placement_wait_s:g} s",
                )
                return False

    def capture_placement(self, run: GroundRun, chain: UrdfChain) -> None:
        """One placement: home, locate the board, plan the grid, visit it, return home."""
        index = len(run.placements)
        home = (self.ground.fit.home_pan_deg, self.ground.fit.home_tilt_deg)
        with self.lock:
            self.placement = index
            self.status = HOMING
            self.pose = self.total = 0
            self.board = None
            self.message = None
        calib_event(LOG, FLOW, run.session_id, "placement", "start", placement=index, home=home)
        still = self.move_to(run, *home)
        if still is None:
            self.fail_placement(run, index, HOME_UNREACHED.format(timeout=self.floor.move_timeout_s))
            return
        with self.lock:
            self.status = LOCATING
        info = self.wait_camera_info()
        if info is None:
            self.fail_placement(run, index, REASON_NO_INFO)
            return
        if run.camera_info is None:
            run.camera_info = info
        pose = run.next_pose
        home_views, reasons = self.capture_frames(
            run, chain, index, pose, home, still, self.ground.home_min_corners, None
        )
        if not home_views:
            detail = ", ".join(f"{count} x {reason}" for reason, count in reasons.items())
            self.fail_placement(
                run, index, f"{HOME_FAILED.format(corners=self.ground.home_min_corners)} ({detail})", reasons=reasons
            )
            return
        first = home_views[0]
        k = np.asarray(info.k, dtype=float).reshape(3, 3)
        d = np.asarray(info.d, dtype=float)
        joints = {PAN_JOINT: first["pan_rad"], TILT_JOINT: first["tilt_rad"]}
        camera = camera_pose(chain, joints, None, self.cfg.camera_frame)
        located = locate_board(camera, self.view_detection(first), k, d, self.ground.fit.board)
        if located is None:
            self.fail_placement(run, index, NOT_LOCATED)
            return
        board_pose = board_on_floor(self.ground.fit.board, *located)
        centre = (board_pose @ np.append(board_corners_3d(self.ground.fit.board).mean(axis=0), 1.0))[:3]
        aim = aim_head(chain, centre, self.cfg.camera_frame)
        grid, dropped = plan_grid(chain, board_pose, aim, info, self.ground, self.cfg.camera_frame)
        located_json = {"x_m": located[0], "y_m": located[1], "yaw_deg": math.degrees(located[2])}
        aim_json = {"pan_deg": math.degrees(aim[0]), "tilt_deg": math.degrees(aim[1])}
        with self.lock:
            self.board = dict(located_json)
            self.status = SWEEPING
            self.total = len(grid) + 1
            self.pose = 1
        calib_event(
            LOG,
            FLOW,
            run.session_id,
            "placement",
            "located",
            placement=index,
            located=located_json,
            aim=aim_json,
            grid=grid,
            dropped_poses=dropped,
            home_corners=len(first["ids"]),
            upscale=first["upscale"],
        )
        record = {
            "placement": index,
            "located": located_json,
            "aim": aim_json,
            "grid": [{"pan_deg": pan, "tilt_deg": tilt} for pan, tilt in grid],
            "dropped_poses": dropped,
            "poses": 0,
            "skipped": 0,
            "views": 0,
        }
        run.placements.append(record)
        self.keep_pose(run, record, chain, pose, home, still, home_views)
        for number, (pan_deg, tilt_deg) in enumerate(grid):
            if self.stop_event.is_set():
                raise Stopped
            with self.lock:
                self.pose = number + 2
            pose = run.next_pose
            began = self.clock()
            reached = self.move_to(run, pan_deg, tilt_deg)
            if reached is None:
                self.skip_pose(
                    run,
                    record,
                    pose,
                    pan_deg,
                    tilt_deg,
                    f"did not reach pan/tilt within {self.floor.move_timeout_s:g} s",
                    last_measured=self.measured_deg(self.latest()),
                    waited_s=self.clock() - began,
                )
                continue
            views, reasons = self.capture_frames(
                run, chain, index, pose, (pan_deg, tilt_deg), reached, self.ground.min_corners, board_pose
            )
            if len(views) < self.ground.min_frames_per_pose:
                detail = ", ".join(f"{count} x {reason}" for reason, count in reasons.items())
                self.skip_pose(
                    run,
                    record,
                    pose,
                    pan_deg,
                    tilt_deg,
                    f"{len(views)} of {self.ground.frames_per_pose} frames kept: {detail}",
                    reached=self.measured_deg(reached),
                    frame_reasons=reasons,
                )
                continue
            self.keep_pose(run, record, chain, pose, (pan_deg, tilt_deg), reached, views, reasons)
        if self.move_to(run, *home) is None:
            calib_event(
                LOG, FLOW, run.session_id, "placement", "home_unconfirmed", logging.WARNING, "head not back home"
            )
        with self.lock:
            self.placements_done = len(run.placements)
        calib_event(
            LOG,
            FLOW,
            run.session_id,
            "placement",
            "done",
            placement=index,
            poses=record["poses"],
            skipped=record["skipped"],
            views=record["views"],
        )

    def fail_placement(self, run: GroundRun, index: int, reason: str, **data: Any) -> None:
        """Record a placement that got no view (the user can fix the board and ask for the next placement)."""
        run.failures.append({"placement": index, "reason": reason})
        with self.lock:
            self.message = reason
        calib_event(LOG, FLOW, run.session_id, "placement", "failed", logging.WARNING, reason, placement=index, **data)

    def wait_camera_info(self) -> CameraInfoData | None:
        """The latest colour camera_info, waiting up to frame_timeout_s for the first one."""
        began = self.clock()
        while True:
            with self.lock:
                info = self.camera_info
            if info is not None:
                return info
            self.check_alive()
            if self.clock() - began >= self.ground.frame_timeout_s:
                return None
            self.sleep(POLL_S)

    def next_colour(self, last_seq: int, threshold_ns: int) -> tuple[int, int, Any] | None:
        """Oldest queued colour frame newer than last_seq and stamped after threshold_ns, or None."""
        with self.lock:
            for item in self.colour:
                if item[0] > last_seq and item[1] > threshold_ns:
                    return item
        return None

    def depth_at(self, stamp: int) -> Any:
        """The aligned depth message of exactly this stamp, None when it has not arrived."""
        with self.lock:
            for depth_stamp, msg in self.depth:
                if depth_stamp == stamp:
                    return msg
        return None

    def capture_frames(
        self,
        run: GroundRun,
        chain: UrdfChain,
        placement: int,
        pose: int,
        target: tuple[float, float],
        still: JointSample,
        min_corners: int,
        board_pose: np.ndarray | None,
    ) -> tuple[list[dict[str, Any]], Counter[str]]:
        """Keep up to frames_per_pose colour frames stamped after the still time plus settle_s.

        Args:
            run (GroundRun): The session.
            chain (UrdfChain): Applied head chain (the PnP seed of the depth stats).
            placement (int): Placement index.
            pose (int): Pose index.
            target (tuple[float, float]): Commanded (pan, tilt) deg.
            still (JointSample): The sample at which the head was reached and still.
            min_corners (int): Corners a frame needs.
            board_pose (np.ndarray | None): Located board in base_link (the PnP seed); None skips the depth stats.

        Returns:
            tuple: (view dicts, dropped frames by reason).
        """
        threshold = still.stamp_ns + int(self.floor.settle_s * NS_PER_S)
        views: list[dict[str, Any]] = []
        reasons: Counter[str] = Counter()
        with self.lock:
            last_seq = self.colour_seq
        attempts = 0
        while len(views) < self.ground.frames_per_pose and attempts < 2 * self.ground.frames_per_pose:
            attempts += 1
            began = self.clock()
            item = self.next_colour(last_seq, threshold)
            while item is None:
                self.check_alive()
                if self.clock() - began >= self.ground.frame_timeout_s:
                    break
                self.sleep(POLL_S)
                item = self.next_colour(last_seq, threshold)
            if item is None:
                reasons[REASON_NO_FRAME] += 1
                break
            last_seq = item[0]
            outcome = self.frame_view(run, chain, placement, pose, len(views), target, item, min_corners, board_pose)
            if isinstance(outcome, str):
                reasons[outcome] += 1
            else:
                views.append(outcome)
        return views, reasons

    def frame_view(
        self,
        run: GroundRun,
        chain: UrdfChain,
        placement: int,
        pose: int,
        frame: int,
        target: tuple[float, float],
        item: tuple[int, int, Any],
        min_corners: int,
        board_pose: np.ndarray | None,
    ) -> dict[str, Any] | str:
        """One view of a colour frame (its JPEG written to frames/), or the reason it was dropped."""
        _seq, stamp, msg = item
        sample = self.nearest_sample(stamp)
        if sample is None:
            return REASON_NO_JOINT
        tol = self.floor.reach_tol_deg
        if abs(math.degrees(sample.pan) - target[0]) > tol or abs(math.degrees(sample.tilt) - target[1]) > tol:
            return REASON_OFF_TARGET
        try:
            image = decode_compressed(msg, self.cfg.topics.rgb)
        except ValueError:
            return REASON_UNDECODABLE
        detection, upscale = detect_frame(image, self.charuco, min_corners, self.ground.detect_upscale)
        if detection is None:
            return REASON_FEW_CORNERS.format(corners=min_corners)
        view_id = f"p{placement}-{pose:03d}-{frame}"
        bias = self.depth_bias(chain, sample, stamp, detection, board_pose)
        image_path = f"{FRAMES_DIR}/{view_id}.jpg"
        write_bytes_atomic(run.directory / image_path, bytes(msg.data))
        return {
            "view_id": view_id,
            "placement": placement,
            "pose": pose,
            "frame": frame,
            "pan_rad": sample.pan,
            "tilt_rad": sample.tilt,
            "stamp_ns": stamp,
            "joint_stamp_ns": sample.stamp_ns,
            "ids": [int(v) for v in detection.ids],
            "points": [[float(x), float(y)] for x, y in np.asarray(detection.points).reshape(-1, 2)],
            "upscale": upscale,
            "image": image_path,
            "depth_bias_mm": None if bias is None else bias[0],
            "depth_spread_mm": None if bias is None else bias[1],
        }

    def depth_bias(
        self,
        chain: UrdfChain,
        sample: JointSample,
        stamp: int,
        detection: Detection,
        board_pose: np.ndarray | None,
    ) -> tuple[float, float] | None:
        """Aligned depth minus the board plane over the board (median, spread mm), None without depth or a seed."""
        msg = self.depth_at(stamp)
        with self.lock:
            info = self.camera_info
        if msg is None or board_pose is None or info is None:
            return None
        try:
            depth = decode_mono16(msg)
        except ValueError:
            return None
        k = np.asarray(info.k, dtype=float).reshape(3, 3)
        d = np.asarray(info.d, dtype=float)
        board = self.ground.fit.board
        camera = camera_pose(chain, {PAN_JOINT: sample.pan, TILT_JOINT: sample.tilt}, None, self.cfg.camera_frame)
        objects = board_corners_3d(board)[np.asarray(detection.ids, dtype=int)]
        found = board_pnp(objects, np.asarray(detection.points, dtype=float), k, d, np.linalg.solve(camera, board_pose))
        if found is None:
            return None
        rvec = cv2.Rodrigues(found[:3, :3])[0]
        outline = cv2.projectPoints(board_outline(board, 0.0), rvec, found[:3, 3], k, d)[0].reshape(-1, 2)
        intrinsics = IntrinsicsResult(k, d, math.nan, np.zeros(0), 0, (int(info.width), int(info.height)))
        return board_plane_bias_mm(depth, outline, (found[:3, :3], found[:3, 3]), intrinsics)

    def view_detection(self, view: dict[str, Any]) -> Detection:
        """The Detection of a stored view dict."""
        return Detection(
            ids=np.asarray(view["ids"], dtype=np.int32), points=np.asarray(view["points"], dtype=np.float32)
        )

    def keep_pose(
        self,
        run: GroundRun,
        record: dict[str, Any],
        chain: UrdfChain,
        pose: int,
        target: tuple[float, float],
        still: JointSample,
        views: list[dict[str, Any]],
        reasons: Counter[str] | None = None,
    ) -> None:
        """Keep a pose's views, fit its depth floor planes (the cross-check) and log it."""
        rng = np.random.default_rng(self.floor.fit.plane.seed + pose)
        planes, cloud_reasons = self.capture_pose(chain, rng, pose, still, target)
        for plane in planes:
            plane["placement"] = record["placement"]
        run.views.extend(views)
        run.observations.extend(planes)
        run.dropped.update(cloud_reasons)
        run.next_pose = pose + 1
        record["poses"] += 1
        record["views"] += len(views)
        with self.lock:
            self.view_count = len(run.views)
            self.plane_count = len(run.observations)
        calib_event(
            LOG,
            FLOW,
            run.session_id,
            "pose",
            "observed",
            placement=record["placement"],
            pose=pose,
            target={"pan_deg": target[0], "tilt_deg": target[1]},
            reached=self.measured_deg(still),
            corners=[len(v["ids"]) for v in views],
            upscale=[v["upscale"] for v in views],
            depth_bias_mm=[v["depth_bias_mm"] for v in views],
            dropped_frames=reasons or {},
            planes=len(planes),
            plane_height_m=[p["height_m"] for p in planes],
            dropped_clouds=cloud_reasons,
        )

    def skip_pose(
        self,
        run: GroundRun,
        record: dict[str, Any],
        pose: int,
        pan_deg: float,
        tilt_deg: float,
        reason: str,
        **data: Any,
    ) -> None:
        """Record and log a skipped pose (the pose index is used up so indices stay unique)."""
        run.skipped.append(
            {"placement": record["placement"], "pose": pose, "pan_deg": pan_deg, "tilt_deg": tilt_deg, "reason": reason}
        )
        run.next_pose = pose + 1
        record["skipped"] += 1
        with self.lock:
            self.skipped_count = len(run.skipped)
        calib_event(
            LOG,
            FLOW,
            run.session_id,
            "pose",
            "skipped",
            logging.WARNING,
            reason,
            placement=record["placement"],
            pose=pose,
            target={"pan_deg": pan_deg, "tilt_deg": tilt_deg},
            **data,
        )

    def write(self, run: GroundRun) -> None:  # type: ignore[override]
        """Write observations.json atomically (after each placement and at the end)."""
        write_json_atomic(
            run.directory / OBSERVATIONS_FILE,
            {
                "session": run.session_id,
                "created": run.created.isoformat(),
                "board": self.ground.fit.board.name,
                "camera_frame": self.cfg.camera_frame,
                "sensor_frame": self.floor.sensor_frame,
                "camera_info": None if run.camera_info is None else info_json(run.camera_info),
                "home": {"pan_deg": self.ground.fit.home_pan_deg, "tilt_deg": self.ground.fit.home_tilt_deg},
                "placements": run.placements,
                "placement_failures": run.failures,
                "views": run.views,
                "observations": run.observations,
                "skipped": run.skipped,
                "dropped": dict(run.dropped),
            },
        )
