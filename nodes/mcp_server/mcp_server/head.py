"""Pan/tilt head controller: validated joint moves with arrival detection from real /head/joint_states only."""

import math
from typing import Literal, Protocol

from pydantic import BaseModel

from .arm import JointSample
from .config import HeadSettings
from .head_kinematics import HeadKinematics
from .models import HeadAngles

HEAD_FRAME_ID = "autonomy"
POLL_INTERVAL_S = 0.05
MoveStatus = Literal["converged", "timeout", "no_feedback"]


class HeadError(RuntimeError):
    """A head request was refused (angle outside the URDF limits)."""


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

    def __init__(self, backend: HeadBackend, kinematics: HeadKinematics, settings: HeadSettings) -> None:
        """Bind the backend, the URDF limits and the settings.

        Args:
            backend (HeadBackend): Robot access.
            kinematics (HeadKinematics): Head URDF (limits).
            settings (HeadSettings): Head configuration.
        """
        self.backend = backend
        self.kinematics = kinematics
        self.settings = settings

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
