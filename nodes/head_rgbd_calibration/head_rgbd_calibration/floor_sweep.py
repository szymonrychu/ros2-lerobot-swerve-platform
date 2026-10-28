"""Guided head sweep (design D5, D11): command a pan/tilt grid, wait for stillness, fit one D435 floor plane per cloud.

Threads: `start`, `stop` and `state` run on HTTP threads; one worker thread runs the sweep; the subscription callbacks
run on the ROS executor thread. `lock` guards the shared fields and is only held for short updates; the worker
waits through the injected `sleep`. The only thing published is the head command (JointState on the command topic,
radians), and every exit of the worker (done, stop, error, exception) commands the head back to the pan/tilt it had at
start and destroys the two subscriptions and the publisher. A pose without a valid plane, or with fewer than
min_clouds_per_pose fitted clouds (partly blocked: an object in view), is skipped with its reason and none of its clouds
are kept; nothing is filled in.
"""

import math
import threading
from collections import Counter, deque
from collections.abc import Callable
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
from ros2_common.head_rgbd_calibration.chain import UrdfChain
from ros2_common.head_rgbd_calibration.floor_fit import PAN_JOINT, TILT_JOINT, fit_plane, predicted_floor
from ros2_common.head_rgbd_calibration.frames import stamp_ns
from sensor_msgs.msg import JointState, PointCloud2
from sensor_msgs_py import point_cloud2

from head_rgbd_calibration.capture import rgb_qos
from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.store import write_json_atomic

FLOOR_DIR = "floor_fit"
OBSERVATIONS_FILE = "observations.json"
SESSION_PREFIX = "floor-"
SESSION_FORMAT = "floor-%Y%m%d-%H%M%S"
POLL_S = 0.05
SAMPLE_HISTORY = 4000
CLOUD_QUEUE = 8
SUBSCRIPTION_DEPTH = 10
PUBLISHER_DEPTH = 10
NS_PER_S = 1_000_000_000
IDLE = "idle"
DONE = "done"
STOPPED = "stopped"
ERROR = "error"
REASON_NO_JOINT = "no joint state within max_stamp_dt_s"
REASON_OFF_TARGET = "head not at the target at the cloud stamp"
REASON_NO_PLANE = "no floor plane in the cloud"
REASON_WRONG_FRAME = "cloud not in {frame}"
REASON_NO_CLOUD = "no cloud within cloud_timeout_s"
REASON_PARTLY_BLOCKED = "partly blocked: {found} of {clouds} clouds"
POINT_FIELDS = ("x", "y", "z")


class SweepError(RuntimeError):
    """The sweep cannot start or has to abort (the reason is the message)."""


class Busy(SweepError):
    """A sweep is already running, or an RGB-D capture or compute is (maps to 409)."""


class Stopped(Exception):
    """Raised inside the worker when stop() was called."""


@dataclass(frozen=True)
class JointSample:
    """A measured head pan/tilt: header stamp (ns), angles (rad) and the local clock time it arrived."""

    stamp_ns: int
    pan: float
    tilt: float
    received_s: float


@dataclass
class Run:
    """One sweep: session, grid and the ROS handles it owns.

    Attributes:
        session_id: Session id, also the directory name under floor_fit/.
        directory: Where observations.json is written.
        created: Creation time.
        grid: (pan_deg, tilt_deg) per pose in serpentine order.
        start_pan: Pan (rad) the head had at start; restored on every exit.
        start_tilt: Tilt (rad) the head had at start.
        subscriptions: Subscriptions created by start.
        publisher: The head command publisher.
        observations: Fitted planes so far.
        skipped: Skipped poses so far.
        dropped: Dropped cloud counts by reason.
    """

    session_id: str
    directory: Path
    created: datetime
    grid: list[tuple[float, float]]
    start_pan: float
    start_tilt: float
    subscriptions: list[Any]
    publisher: Any
    observations: list[dict[str, Any]]
    skipped: list[dict[str, Any]]
    dropped: Counter[str]


def serpentine_grid(pan_deg: list[float], tilt_deg: list[float]) -> list[tuple[float, float]]:
    """(pan, tilt) poses, one tilt row at a time, the pan direction alternating per row.

    Args:
        pan_deg: Pan values (deg).
        tilt_deg: Tilt values (deg), + is down.

    Returns:
        list[tuple[float, float]]: Poses in visiting order.
    """
    grid = []
    for row, tilt in enumerate(tilt_deg):
        pans = pan_deg if row % 2 == 0 else pan_deg[::-1]
        grid.extend((pan, tilt) for pan in pans)
    return grid


class FloorSweep:
    """Runs one floor sweep at a time on a worker thread."""

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
            cfg: Node config (state_dir and the floor block).
            urdf_text: Returns the head URDF text the plane prior is predicted from.
            clock: Monotonic seconds; all waits are measured on it.
            sleep: Sleeps seconds (the injected clock's sleep in tests).
            now: Wall time, names the session.
            blocked: Returns a reason while something else (an RGB-D capture or compute) forbids a sweep, else None.
        """
        self.node = node
        self.cfg = cfg
        self.floor = cfg.floor
        self.urdf_text = urdf_text
        self.clock = clock
        self.sleep = sleep
        self.now = now
        self.blocked = blocked
        self.logger = node.get_logger()
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.samples: deque[JointSample] = deque(maxlen=SAMPLE_HISTORY)
        self.clouds: deque[tuple[int, int, Any]] = deque(maxlen=CLOUD_QUEUE)
        self.cloud_seq = 0
        self.thread: threading.Thread | None = None
        self.running = False
        self.session_id: str | None = None
        self.pose = 0
        self.total = 0
        self.observation_count = 0
        self.skipped_count = 0
        self.status = IDLE
        self.error: str | None = None

    def state(self) -> dict[str, Any]:
        """The sweep state for the HTTP API.

        Returns:
            {running, session_id, pose, total, observations, skipped, status, error}: pose is the 1-based pose being
            worked on (the last one after the sweep), observations and skipped are counts.
        """
        with self.lock:
            return {
                "running": self.running,
                "session_id": self.session_id,
                "pose": self.pose,
                "total": self.total,
                "observations": self.observation_count,
                "skipped": self.skipped_count,
                "status": self.status,
                "error": self.error,
            }

    def wait(self, timeout: float | None = None) -> None:
        """Block until the worker thread has finished (no-op when none runs).

        Args:
            timeout: Longest wait in seconds, None waits without limit.
        """
        thread = self.thread
        if thread is not None:
            thread.join(timeout)

    def stop(self) -> dict[str, Any]:
        """Ask the running sweep to stop; it restores the head and writes the poses done so far.

        Returns:
            The state() after the request.
        """
        self.stop_event.set()
        return self.state()

    def start(self) -> str:
        """Start a sweep on a new session.

        Returns:
            The session id.

        Raises:
            Busy: a sweep runs, or `blocked` names a reason.
            SweepError: the head URDF is unusable or no head joint state arrives within move_timeout_s (nothing moved).
        """
        with self.lock:
            if self.running:
                raise Busy("a floor sweep is already running")
            reason = self.blocked() if self.blocked is not None else None
            if reason:
                raise Busy(reason)
            self.running = True
            self.status = "starting"
            self.error = None
            self.session_id = None
            self.pose = self.total = self.observation_count = self.skipped_count = 0
        handles: list[Any] = []
        publisher: Any = None
        try:
            try:
                chain = UrdfChain.from_text(self.urdf_text())
            except (ValueError, KeyError, OSError) as exc:
                raise SweepError(f"cannot read the head URDF: {exc}") from exc
            self.stop_event.clear()
            self.samples.clear()
            self.clouds.clear()
            handles.append(
                self.node.create_subscription(
                    JointState, self.floor.joint_states_topic, self.on_joint_state, rgb_qos(SUBSCRIPTION_DEPTH)
                )
            )
            handles.append(
                self.node.create_subscription(
                    PointCloud2, self.floor.cloud_topic, self.on_cloud, rgb_qos(SUBSCRIPTION_DEPTH)
                )
            )
            publisher = self.node.create_publisher(JointState, self.floor.command_topic, PUBLISHER_DEPTH)
            start_sample = self.wait_first_sample()
            run = self.open_run(start_sample, handles, publisher)
        except BaseException as exc:
            self.release(handles, publisher)
            with self.lock:
                self.running = False
                self.status = ERROR
                self.error = str(exc)
            raise
        with self.lock:
            self.session_id = run.session_id
            self.total = len(run.grid)
            self.status = "running"
        self.thread = threading.Thread(target=self.work, args=(run, chain), name="floor-sweep", daemon=True)
        self.thread.start()
        self.logger.info(f"floor sweep {run.session_id} started: {len(run.grid)} poses")
        return run.session_id

    def wait_first_sample(self) -> JointSample:
        """Latest head joint state, waiting up to move_timeout_s for the first one."""
        timeout = self.floor.move_timeout_s
        began = self.clock()
        while True:
            sample = self.latest()
            if sample is not None:
                return sample
            if self.clock() - began >= timeout:
                raise SweepError(
                    f"no head joint state on {self.floor.joint_states_topic} within {timeout:g} s; nothing was moved"
                )
            self.sleep(POLL_S)

    def open_run(self, start: JointSample, handles: list[Any], publisher: Any) -> Run:
        """Create the session directory (a taken second gets a -N suffix) and the run record."""
        created = self.now()
        base = created.strftime(SESSION_FORMAT)
        root = Path(self.cfg.state_dir) / FLOOR_DIR
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
        return Run(
            session_id=session_id,
            directory=directory,
            created=created,
            grid=serpentine_grid(self.floor.pan_deg, self.floor.tilt_deg),
            start_pan=start.pan,
            start_tilt=start.tilt,
            subscriptions=handles,
            publisher=publisher,
            observations=[],
            skipped=[],
            dropped=Counter(),
        )

    # executor-thread callbacks

    def on_joint_state(self, msg: Any) -> None:
        """Keep the head pan/tilt with its header stamp; messages without both head joints are ignored."""
        try:
            pan = msg.position[list(msg.name).index(PAN_JOINT)]
            tilt = msg.position[list(msg.name).index(TILT_JOINT)]
        except (ValueError, IndexError):
            return
        sample = JointSample(stamp_ns(msg), float(pan), float(tilt), self.clock())
        with self.lock:
            self.samples.append(sample)

    def on_cloud(self, msg: Any) -> None:
        """Queue a D435 cloud; decoding happens on the worker thread."""
        with self.lock:
            self.cloud_seq += 1
            self.clouds.append((self.cloud_seq, stamp_ns(msg), msg))

    # worker thread

    def latest(self) -> JointSample | None:
        with self.lock:
            return self.samples[-1] if self.samples else None

    def publish(self, run: Run, pan_rad: float, tilt_rad: float) -> None:
        """Publish one head command."""
        msg = JointState()
        msg.header.frame_id = self.floor.command_frame_id
        msg.name = [PAN_JOINT, TILT_JOINT]
        msg.position = [pan_rad, tilt_rad]
        run.publisher.publish(msg)

    def check_alive(self, limit_stop: bool = True) -> None:
        """Raise Stopped after stop(), SweepError when no joint state arrived for move_timeout_s."""
        if limit_stop and self.stop_event.is_set():
            raise Stopped
        sample = self.latest()
        if sample is not None and self.clock() - sample.received_s > self.floor.move_timeout_s:
            raise SweepError(
                f"head joint states stopped arriving on {self.floor.joint_states_topic} "
                f"(none for {self.floor.move_timeout_s:g} s)"
            )

    def reached_and_still(self, target_pan: float, target_tilt: float, since_s: float) -> JointSample | None:
        """The newest sample when the head is at the target and still over still_window_s, else None."""
        floor = self.floor
        now = self.clock()
        if now - since_s < floor.still_window_s:
            return None
        with self.lock:
            window = [s for s in self.samples if s.received_s >= now - floor.still_window_s]
        if len(window) < 2:
            return None
        pans = np.degrees([s.pan for s in window])
        tilts = np.degrees([s.tilt for s in window])
        newest = window[-1]
        at_target = (
            abs(math.degrees(newest.pan) - target_pan) <= floor.reach_tol_deg
            and abs(math.degrees(newest.tilt) - target_tilt) <= floor.reach_tol_deg
        )
        still = np.ptp(pans) < floor.still_tol_deg and np.ptp(tilts) < floor.still_tol_deg
        return newest if at_target and still else None

    def move_to(self, run: Run, pan_deg: float, tilt_deg: float) -> JointSample | None:
        """Command a pose and wait for it to be reached and still; None when move_timeout_s passes first."""
        self.publish(run, math.radians(pan_deg), math.radians(tilt_deg))
        began = self.clock()
        while True:
            self.check_alive()
            sample = self.reached_and_still(pan_deg, tilt_deg, began)
            if sample is not None:
                return sample
            if self.clock() - began >= self.floor.move_timeout_s:
                return None
            self.sleep(POLL_S)

    def next_cloud(self, last_seq: int, threshold_ns: int) -> tuple[int, int, Any] | None:
        """Oldest queued cloud newer than last_seq and stamped after threshold_ns, or None."""
        with self.lock:
            for item in self.clouds:
                if item[0] > last_seq and item[1] > threshold_ns:
                    return item
        return None

    def nearest_sample(self, cloud_stamp_ns: int) -> JointSample | None:
        """The joint state nearest the stamp within max_stamp_dt_s, else None."""
        with self.lock:
            samples = list(self.samples)
        if not samples:
            return None
        best = min(samples, key=lambda s: abs(s.stamp_ns - cloud_stamp_ns))
        if abs(best.stamp_ns - cloud_stamp_ns) > self.floor.max_stamp_dt_s * NS_PER_S:
            return None
        return best

    def fit_cloud(
        self,
        chain: UrdfChain,
        rng: np.random.Generator,
        item: tuple[int, int, Any],
        target: tuple[float, float],
        pose: int,
    ) -> dict[str, Any] | str:
        """One observation of a cloud, or the reason it was dropped."""
        _seq, cloud_stamp, msg = item
        sample = self.nearest_sample(cloud_stamp)
        if sample is None:
            return REASON_NO_JOINT
        tol = self.floor.reach_tol_deg
        if abs(math.degrees(sample.pan) - target[0]) > tol or abs(math.degrees(sample.tilt) - target[1]) > tol:
            return REASON_OFF_TARGET
        if msg.header.frame_id != self.floor.sensor_frame:
            return REASON_WRONG_FRAME.format(frame=self.floor.sensor_frame)
        points = np.asarray(point_cloud2.read_points_numpy(msg, field_names=POINT_FIELDS, skip_nans=True), dtype=float)
        prior = predicted_floor(chain, sample.pan, sample.tilt, None, self.floor.sensor_frame)
        plane = fit_plane(points, prior, self.floor.fit.plane, rng)
        if plane is None:
            return REASON_NO_PLANE
        return {
            "pose": pose,
            "pan_rad": sample.pan,
            "tilt_rad": sample.tilt,
            "normal": [float(v) for v in plane.normal],
            "height_m": float(plane.height_m),
            "inliers": plane.inliers,
            "candidates": plane.candidates,
            "rms_m": float(plane.rms_m),
            "stamp_ns": cloud_stamp,
        }

    def capture_pose(
        self,
        chain: UrdfChain,
        rng: np.random.Generator,
        pose: int,
        still: JointSample,
        target: tuple[float, float],
    ) -> tuple[list[dict[str, Any]], Counter[str]]:
        """Fit the next clouds_per_pose clouds stamped after the still time plus settle_s."""
        floor = self.floor
        threshold = still.stamp_ns + int(floor.settle_s * NS_PER_S)
        found: list[dict[str, Any]] = []
        reasons: Counter[str] = Counter()
        with self.lock:
            last_seq = self.cloud_seq
        for _ in range(floor.clouds_per_pose):
            began = self.clock()
            item = self.next_cloud(last_seq, threshold)
            while item is None:
                self.check_alive()
                if self.clock() - began >= floor.cloud_timeout_s:
                    break
                self.sleep(POLL_S)
                item = self.next_cloud(last_seq, threshold)
            if item is None:
                reasons[REASON_NO_CLOUD] += 1
                break
            last_seq = item[0]
            outcome = self.fit_cloud(chain, rng, item, target, pose)
            if isinstance(outcome, str):
                reasons[outcome] += 1
            else:
                found.append(outcome)
        return found, reasons

    def work(self, run: Run, chain: UrdfChain) -> None:
        """Worker: sweep the grid, then always restore the head, write observations.json and release the handles."""
        final, error = DONE, None
        try:
            self.sweep(run, chain)
        except Stopped:
            final = STOPPED
        except SweepError as exc:
            final, error = ERROR, str(exc)
        except Exception as exc:  # reported as the sweep error, the finally block still restores the head
            final, error = ERROR, f"{type(exc).__name__}: {exc}"
            self.logger.error(f"floor sweep {run.session_id} failed: {error}")
        finally:
            self.restore(run)
            try:
                self.write(run)
            except OSError as exc:
                final, error = ERROR, f"cannot write {OBSERVATIONS_FILE}: {exc}"
                self.logger.error(error)
            self.release(run.subscriptions, run.publisher)
            with self.lock:
                self.running = False
                self.status = final
                self.error = error
        self.logger.info(f"floor sweep {run.session_id} {final}")

    def sweep(self, run: Run, chain: UrdfChain) -> None:
        """Visit every pose; a pose that is not reached, has no valid plane or fewer than min_clouds_per_pose fitted
        clouds is skipped with its reason."""
        rng = np.random.default_rng(self.floor.fit.plane.seed)
        for pose, (pan_deg, tilt_deg) in enumerate(run.grid):
            if self.stop_event.is_set():
                raise Stopped
            with self.lock:
                self.pose = pose + 1
            still = self.move_to(run, pan_deg, tilt_deg)
            if still is None:
                self.record_skip(
                    run, pose, pan_deg, tilt_deg, f"did not reach pan/tilt within {self.floor.move_timeout_s:g} s"
                )
                continue
            found, reasons = self.capture_pose(chain, rng, pose, still, (pan_deg, tilt_deg))
            run.dropped.update(reasons)
            if len(found) >= self.floor.min_clouds_per_pose:
                run.observations.extend(found)
                with self.lock:
                    self.observation_count = len(run.observations)
            elif found:
                reason = REASON_PARTLY_BLOCKED.format(found=len(found), clouds=self.floor.clouds_per_pose)
                self.record_skip(run, pose, pan_deg, tilt_deg, reason)
            else:
                detail = ", ".join(f"{count} x {reason}" for reason, count in reasons.items())
                self.record_skip(run, pose, pan_deg, tilt_deg, f"no valid plane: {detail}")

    def record_skip(self, run: Run, pose: int, pan_deg: float, tilt_deg: float, reason: str) -> None:
        run.skipped.append({"pose": pose, "pan_deg": pan_deg, "tilt_deg": tilt_deg, "reason": reason})
        with self.lock:
            self.skipped_count = len(run.skipped)
        self.logger.warning(f"floor sweep pose {pose} ({pan_deg:g}, {tilt_deg:g}) skipped: {reason}")

    def restore(self, run: Run) -> None:
        """Command the start pan/tilt and wait (up to move_timeout_s) for it; failures are logged, never raised."""
        try:
            self.publish(run, run.start_pan, run.start_tilt)
            began = self.clock()
            target = (math.degrees(run.start_pan), math.degrees(run.start_tilt))
            while self.reached_and_still(target[0], target[1], began) is None:
                if self.clock() - began >= self.floor.move_timeout_s:
                    self.logger.warning("the head did not confirm the restored pan/tilt")
                    return
                self.sleep(POLL_S)
        except Exception as exc:  # the handles are still released by the caller
            self.logger.error(f"restoring the head failed: {exc}")

    def write(self, run: Run) -> None:
        """Write observations.json atomically (also after a stop or an error: the poses done so far)."""
        write_json_atomic(
            run.directory / OBSERVATIONS_FILE,
            {
                "session": run.session_id,
                "created": run.created.isoformat(),
                "sensor_frame": self.floor.sensor_frame,
                "grid": [{"pose": i, "pan_deg": pan, "tilt_deg": tilt} for i, (pan, tilt) in enumerate(run.grid)],
                "observations": run.observations,
                "skipped": run.skipped,
                "dropped": dict(run.dropped),
            },
        )

    def release(self, subscriptions: list[Any], publisher: Any) -> None:
        """Destroy the subscriptions and the publisher; a failing destroy does not stop the others."""
        for sub in subscriptions:
            try:
                self.node.destroy_subscription(sub)
            except Exception as exc:  # keep destroying the rest
                self.logger.error(f"destroying a floor sweep subscription failed: {exc}")
        if publisher is not None:
            try:
                self.node.destroy_publisher(publisher)
            except Exception as exc:  # logged, the subscriptions are gone already
                self.logger.error(f"destroying the floor sweep publisher failed: {exc}")
