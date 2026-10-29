"""Pan/tilt head controller: validated joint moves with arrival detection from real /head/joint_states only."""

import bisect
import json
import logging
import math
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, Protocol

from pydantic import BaseModel
from ros2_common.head_rgbd_calibration.urdf_text import APPLIED_MARKER_FILE, apply_overrides

from .arm import JointSample
from .config import HeadSettings
from .head_kinematics import HeadKinematics
from .models import HeadAngles

LOGGER = logging.getLogger(__name__)
HEAD_FRAME_ID = "autonomy"
POLL_INTERVAL_S = 0.05
MoveStatus = Literal["converged", "timeout", "no_feedback"]


class HeadError(RuntimeError):
    """A head request was refused (angle outside the URDF limits)."""


class HeadHistoryError(ValueError):
    """The head pose at an image stamp cannot be told (no samples, stamp outside the history, a gap in the samples)."""


@dataclass(frozen=True)
class HeadPoseAtStamp:
    """Pan and tilt at one stamp, interpolated from /head/joint_states, with the head motion around that stamp.

    Attributes:
        pan (float): head_pan at the stamp (rad, + left).
        tilt (float): head_tilt at the stamp (rad, + down).
        stamp_s (float): The requested stamp (ROS time, s).
        nearest_gap_s (float): Distance from the stamp to the nearest joint sample (s).
        window_motion_rad (float): Largest pan or tilt excursion of the samples within +- stamp_window_s (rad).
        moving (bool): window_motion_rad exceeds the configured threshold (the pose at the stamp is not reliable).
    """

    pan: float
    tilt: float
    stamp_s: float
    nearest_gap_s: float
    window_motion_rad: float
    moving: bool


class HeadHistory:
    """Ring buffer of /head/joint_states samples keyed by their header stamp (ROS time), thread safe.

    The camera frames carry their capture stamp; the head pose that placed a frame is the interpolated pan/tilt at
    that stamp, not the latest measurement.
    """

    def __init__(self, settings: HeadSettings) -> None:
        """Create an empty history.

        Args:
            settings (HeadSettings): joint_names, history_s, stamp_window_s, max_motion_rad, max_sample_gap_s, state_topic.
        """
        self.settings = settings
        self.lock = threading.Lock()
        self.stamps: list[float] = []
        self.pans: list[float] = []
        self.tilts: list[float] = []

    def __len__(self) -> int:
        """Number of samples kept.

        Returns:
            int: Sample count.
        """
        with self.lock:
            return len(self.stamps)

    def oldest_stamp(self) -> float | None:
        """Stamp of the oldest sample.

        Returns:
            float | None: Seconds, None when empty.
        """
        with self.lock:
            return self.stamps[0] if self.stamps else None

    def newest_stamp(self) -> float | None:
        """Stamp of the newest sample.

        Returns:
            float | None: Seconds, None when empty.
        """
        with self.lock:
            return self.stamps[-1] if self.stamps else None

    def add(self, stamp_s: float, positions: dict[str, float]) -> None:
        """Insert a sample in stamp order and drop samples older than history_s before the newest.

        A sample lacking a head joint or with a non-finite position or stamp is ignored.

        Args:
            stamp_s (float): Header stamp (ROS time, s).
            positions (dict[str, float]): Joint name -> rad.
        """
        pan_name, tilt_name = self.settings.joint_names
        if pan_name not in positions or tilt_name not in positions:
            return
        pan, tilt = positions[pan_name], positions[tilt_name]
        if not all(math.isfinite(v) for v in (stamp_s, pan, tilt)):
            return
        with self.lock:
            index = bisect.bisect_right(self.stamps, stamp_s)
            self.stamps.insert(index, stamp_s)
            self.pans.insert(index, pan)
            self.tilts.insert(index, tilt)
            keep_from = bisect.bisect_left(self.stamps, self.stamps[-1] - self.settings.history_s)
            if keep_from:
                del self.stamps[:keep_from], self.pans[:keep_from], self.tilts[:keep_from]

    def pose_at(self, stamp_s: float) -> HeadPoseAtStamp:
        """Pan and tilt at a stamp by linear interpolation, with the head motion within +- stamp_window_s.

        A stamp just after the newest sample (within max_sample_gap_s) holds the newest sample; the motion check
        then decides whether that is trustworthy.

        Args:
            stamp_s (float): Image header stamp (ROS time, s).

        Returns:
            HeadPoseAtStamp: The pose; `moving` is True when the head moved more than max_motion_rad in the window.

        Raises:
            HeadHistoryError: No samples, the stamp is older than the oldest sample or more than max_sample_gap_s
                after the newest, or the samples around it are further apart than max_sample_gap_s.
        """
        cfg = self.settings
        with self.lock:
            stamps, pans, tilts = list(self.stamps), list(self.pans), list(self.tilts)
        if not stamps:
            raise HeadHistoryError(f"no {cfg.state_topic} samples received yet: the head pose at the image is unknown")
        if stamp_s < stamps[0]:
            raise HeadHistoryError(
                f"image stamp {stamp_s:.3f} is older than the oldest {cfg.state_topic} sample {stamps[0]:.3f} "
                f"(history {cfg.history_s:g} s): the head pose at the image is unknown"
            )
        right = bisect.bisect_left(stamps, stamp_s)
        if right == len(stamps):
            if stamp_s - stamps[-1] > cfg.max_sample_gap_s:
                raise HeadHistoryError(
                    f"image stamp {stamp_s:.3f} is {stamp_s - stamps[-1]:.3f} s after the newest {cfg.state_topic} "
                    f"sample (limit {cfg.max_sample_gap_s:g} s): the head pose at the image is unknown"
                )
            pan, tilt, gap = pans[-1], tilts[-1], stamp_s - stamps[-1]
        elif stamps[right] == stamp_s or right == 0:
            pan, tilt, gap = pans[right], tilts[right], abs(stamps[right] - stamp_s)
        else:
            left = right - 1
            span = stamps[right] - stamps[left]
            if span > cfg.max_sample_gap_s:
                raise HeadHistoryError(
                    f"gap of {span:.3f} s in the {cfg.state_topic} samples around the image stamp {stamp_s:.3f} "
                    f"(limit {cfg.max_sample_gap_s:g} s): the head pose at the image is unknown"
                )
            weight = (stamp_s - stamps[left]) / span
            pan = pans[left] + weight * (pans[right] - pans[left])
            tilt = tilts[left] + weight * (tilts[right] - tilts[left])
            gap = min(stamp_s - stamps[left], stamps[right] - stamp_s)
        lo = bisect.bisect_left(stamps, stamp_s - cfg.stamp_window_s)
        hi = bisect.bisect_right(stamps, stamp_s + cfg.stamp_window_s)
        in_window_pan = [*pans[lo:hi], pan]
        in_window_tilt = [*tilts[lo:hi], tilt]
        motion = max(max(in_window_pan) - min(in_window_pan), max(in_window_tilt) - min(in_window_tilt))
        return HeadPoseAtStamp(pan, tilt, stamp_s, gap, motion, motion > cfg.max_motion_rad)


class AppliedHeadKinematics:
    """Head kinematics on the repo URDF with the applied calibration overrides, rebuilt when the marker changes.

    The marker is ``applied_dir/applied.json``: a new, changed or removed marker (mtime_ns) triggers a rebuild on the
    next ``current()`` call, so an apply reaches the running server without a restart.
    """

    def __init__(self, urdf_path: Path, applied_dir: Path | None) -> None:
        """Build the first kinematics (the repo URDF plus whatever is applied now).

        Args:
            urdf_path (Path): Repo head URDF.
            applied_dir (Path | None): Applied calibration directory; None disables the override.

        Raises:
            ValueError: When the first build fails (there is no previous kinematics to keep).
        """
        self.urdf_path = urdf_path
        self.applied_dir = applied_dir
        self.lock = threading.Lock()
        self.stamp = self.marker_stamp()
        self.kinematics = self.build()

    def marker_stamp(self) -> int | None:
        """Modification time of the applied marker.

        Returns:
            int | None: mtime_ns, or None when there is no applied_dir or no marker.
        """
        if self.applied_dir is None:
            return None
        try:
            return (self.applied_dir / APPLIED_MARKER_FILE).stat().st_mtime_ns
        except OSError:
            return None

    def marker_fields(self) -> tuple[str, str]:
        """Session and result named by the applied marker, for the reload log line.

        Returns:
            tuple[str, str]: (session, result); "none" for a field that is absent, or both when unreadable.
        """
        if self.applied_dir is None:
            return "none", "none"
        try:
            data = json.loads((self.applied_dir / APPLIED_MARKER_FILE).read_text())
            record = data.get("head_chain")
            if not isinstance(record, dict):
                record = data
            return str(record.get("session", "none")), str(record.get("result", "none"))
        except (OSError, ValueError, AttributeError):
            return "none", "none"

    def build(self) -> HeadKinematics:
        """Parse the repo URDF with the applied overrides, logging every override warning.

        Returns:
            HeadKinematics: The new kinematics.
        """
        text = self.urdf_path.read_text()
        source = str(self.urdf_path)
        if self.applied_dir is not None:
            text, warnings = apply_overrides(text, self.applied_dir)
            for warning in warnings:
                LOGGER.warning("ignoring head override %s", warning)
            source = f"{source} + {self.applied_dir}"
        return HeadKinematics.from_text(text, source)

    def current(self) -> HeadKinematics:
        """Kinematics of the applied chain, rebuilt first when the applied marker changed.

        A failed rebuild is logged and the previous kinematics stay in use.

        Returns:
            HeadKinematics: Current kinematics.
        """
        with self.lock:
            stamp = self.marker_stamp()
            if stamp == self.stamp:
                return self.kinematics
            self.stamp = stamp
            try:
                self.kinematics = self.build()
            except Exception as error:
                LOGGER.exception("head kinematics reload failed (%s), keeping the previous chain", error)
            else:
                session, result = self.marker_fields()
                LOGGER.info("head kinematics reloaded: applied session %s, result %s", session, result)
            return self.kinematics


class HeadBackend(Protocol):
    """What HeadController needs from the robot (RosRobot implements it, FakeHeadBackend fakes it)."""

    def publish_head_command(self, positions: dict[str, float]) -> None:
        """Publish a head joint command (frame_id autonomy).

        Args:
            positions (dict[str, float]): Joint name -> rad.
        """
        ...

    def head_sample(self) -> JointSample | None:
        """Latest /head/joint_states sample, or None before the first message.

        Returns:
            JointSample | None: Sample with its monotonic receive time.
        """
        ...

    def now(self) -> float:
        """Monotonic time.

        Returns:
            float: Seconds.
        """
        ...

    def sleep(self, seconds: float) -> None:
        """Sleep the calling thread.

        Args:
            seconds (float): Duration.
        """
        ...


class HeadMoveResult(BaseModel):
    """Outcome of a head move: converged | timeout | no_feedback, with the measured angles when known (rad)."""

    status: MoveStatus
    target: dict[str, float]
    measured: dict[str, float] | None


class HeadController:
    """Commands the head joints and waits for the measured angles to arrive."""

    def __init__(
        self,
        backend: HeadBackend,
        kinematics: AppliedHeadKinematics,
        settings: HeadSettings,
        history: HeadHistory | None = None,
    ) -> None:
        """Bind the backend, the head kinematics provider, the settings and the joint-state history.

        Args:
            backend (HeadBackend): Robot access.
            kinematics (AppliedHeadKinematics): Provider of the applied head chain (limits, look_at, link_pose).
            settings (HeadSettings): Head configuration.
            history (HeadHistory | None): Joint-state history fed by the robot's /head/joint_states subscription; a new
                empty one when omitted.
        """
        self.backend = backend
        self.provider = kinematics
        self.settings = settings
        self.history = history if history is not None else HeadHistory(settings)

    def pose_at(self, stamp_s: float) -> HeadPoseAtStamp:
        """Head pan/tilt at an image stamp from the joint-state history.

        Args:
            stamp_s (float): Image header stamp (ROS time, s).

        Returns:
            HeadPoseAtStamp: Interpolated pose with the motion flag.

        Raises:
            HeadHistoryError: As HeadHistory.pose_at.
        """
        return self.history.pose_at(stamp_s)

    @property
    def kinematics(self) -> HeadKinematics:
        """Head kinematics of the applied chain, re-read at each access.

        Returns:
            HeadKinematics: The provider's current kinematics.
        """
        return self.provider.current()

    def measured(self) -> dict[str, float] | None:
        """Measured head angles from a fresh sample; never zero-filled.

        Returns:
            dict[str, float] | None: Joint name -> rad, or None when missing, stale or lacking a head joint.
        """
        sample = self.backend.head_sample()
        if sample is None or self.backend.now() - sample.stamp > self.settings.state_max_age_s:
            return None
        names = self.settings.joint_names
        if any(name not in sample.positions for name in names):
            return None
        return {name: sample.positions[name] for name in names}

    def angles_deg(self) -> HeadAngles | None:
        """Measured head angles in degrees from a fresh sample.

        Returns:
            HeadAngles | None: Pan/tilt in degrees, None when measured() has no fresh sample.
        """
        measured = self.measured()
        if measured is None:
            return None
        pan, tilt = (measured[name] for name in self.settings.joint_names)
        return HeadAngles(pan_deg=math.degrees(pan), tilt_deg=math.degrees(tilt))

    def check(self, pan: float, tilt: float) -> None:
        """Refuse angles outside the URDF limits (no clamping).

        Args:
            pan (float): head_pan (rad, + left).
            tilt (float): head_tilt (rad, + down).

        Raises:
            HeadError: Naming the offending joint and the limits in degrees.
        """
        for joint, value in zip(self.settings.joint_names, (pan, tilt), strict=True):
            lower, upper = self.kinematics.limits[joint]
            if not (math.isfinite(value) and lower <= value <= upper):
                raise HeadError(
                    f"{joint} {math.degrees(value):.1f} deg is outside the limits "
                    f"[{math.degrees(lower):.1f}, {math.degrees(upper):.1f}] deg"
                )

    def move(self, pan: float, tilt: float) -> HeadMoveResult:
        """Command the head once and wait until both joints are within tolerance or the timeout passes.

        Args:
            pan (float): Target head_pan (rad).
            tilt (float): Target head_tilt (rad).

        Returns:
            HeadMoveResult: converged, timeout (still moving or stuck) or no_feedback (no fresh joint states).

        Raises:
            HeadError: When an angle is outside the limits (nothing is published).
        """
        self.check(pan, tilt)
        target = dict(zip(self.settings.joint_names, (pan, tilt), strict=True))
        self.backend.publish_head_command(target)
        deadline = self.backend.now() + self.settings.move_timeout_s
        seen = False
        measured = self.measured()
        while True:
            if measured is not None:
                seen = True
                if all(abs(measured[j] - v) <= self.settings.tolerance_rad for j, v in target.items()):
                    return HeadMoveResult(status="converged", target=target, measured=measured)
            if self.backend.now() >= deadline:
                break
            self.backend.sleep(POLL_INTERVAL_S)
            measured = self.measured()
        return HeadMoveResult(status="timeout" if seen else "no_feedback", target=target, measured=measured)

    def home(self) -> HeadMoveResult:
        """Move to the configured home angles.

        Returns:
            HeadMoveResult: As move.
        """
        return self.move(self.settings.home_pan_rad, self.settings.home_tilt_rad)
