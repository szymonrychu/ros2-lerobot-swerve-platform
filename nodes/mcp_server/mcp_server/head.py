"""Pan/tilt head controller: validated joint moves with arrival detection from real /head/joint_states only."""

import json
import logging
import math
import threading
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
            return str(data.get("session", "none")), str(data.get("result", "none"))
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

    def __init__(self, backend: HeadBackend, kinematics: AppliedHeadKinematics, settings: HeadSettings) -> None:
        """Bind the backend, the head kinematics provider and the settings.

        Args:
            backend (HeadBackend): Robot access.
            kinematics (AppliedHeadKinematics): Provider of the applied head chain (limits, look_at, link_pose).
            settings (HeadSettings): Head configuration.
        """
        self.backend = backend
        self.provider = kinematics
        self.settings = settings

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
