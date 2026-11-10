"""The accepted arm calibration in the self-collision guard: an Accept on the Calibrate page reaches the teleop body
check without a restart.

head_rgbd_calibration's arm to camera accept writes applied/arm_calibration.json and the `arm_calibration` record of
applied/applied.json. The reload state is the shared ros2_common `applied_arm_calibration.AppliedArmCalibration`, the
same one mcp_server builds on: the accepted calibration overrides exactly the joint offsets, the arm base pose and the
mount height (= base z) of `self_collision_guard.arm`; the configured client.yml values are the baseline, used while
nothing is accepted, after the record is removed, or when the calibration cannot be used. The tool point stays
configured (it is not fitted).

`startup` loads it before the guard is built. `tick` runs on a CALIBRATION_POLL_S node timer and re-reads only when the
applied.json identity (mtime_ns, size, inode) changed. filter_node runs every callback and its main loop on one thread
(single_threaded or EventsExecutor), so a timer callback runs between two guard checks, never during one: that is the
idle point, and the swap (`swap_checker_calibration`) is applied at once. An unreadable or invalid calibration, or a
swap that raises, logs an error and keeps the values in use until applied.json changes again.
"""

from collections.abc import Callable
from pathlib import Path
from typing import Any, Protocol

from arm_collision.self_collision import SelfCollisionChecker
from arm_collision.settings import ArmBaseOffset, ArmModelSettings
from pydantic import ValidationError
from ros2_common.head_rgbd_calibration.applied_arm_calibration import AppliedArmCalibration
from ros2_common.head_rgbd_calibration.arm_calibration_file import ArmCalibrationError, ArmCalibrationFile

# Period (s) of the applied.json check; a stat per tick, the file is read only after a change.
CALIBRATION_POLL_S = 1.0
CONFIG_SOURCE = "the configured arm values (client.yml self_collision_guard.arm)"

# Swaps the running checker to new arm settings (swap_checker_calibration bound to the guard's checker).
ApplyFn = Callable[[ArmModelSettings], None]


class Log(Protocol):
    """The node logger methods used here (rclpy RcutilsLogger, or a logging.Logger in tests): message -> log."""

    info: Callable[[str], Any]
    error: Callable[[str], Any]


def source_text(calibration: ArmCalibrationFile | None) -> str:
    """Log name of a calibration source.

    Args:
        calibration (ArmCalibrationFile | None): Accepted calibration, None for the configured values.

    Returns:
        str: "accepted arm calibration <session>/<result> (accepted <time>)" or CONFIG_SOURCE.
    """
    if calibration is None:
        return CONFIG_SOURCE
    accepted = calibration.accepted_at.isoformat()
    return f"accepted arm calibration {calibration.session}/{calibration.result} (accepted {accepted})"


def values_text(arm: ArmModelSettings) -> str:
    """The calibrated arm values for the log line.

    Args:
        arm (ArmModelSettings): Arm settings.

    Returns:
        str: Joint offsets, base pose and mount height.
    """
    return (
        f"joint_offsets_rad {arm.joint_offsets_rad.model_dump()}, base_in_base_link "
        f"{None if arm.base_in_base_link is None else arm.base_in_base_link.model_dump()}, "
        f"arm_base_height_m {arm.arm_base_height_m}"
    )


def swap_checker_calibration(checker: SelfCollisionChecker, arm: ArmModelSettings) -> None:
    """Swap the arm mount and the joint offsets of a running checker in place (its kinematics and jaw follow).

    The mount goes first: set_mount builds the new C++ body kernel before it changes anything, so a failed build keeps
    the old calibration whole. No mount means straight above base_link at the mount height (as from_settings).

    Args:
        checker (SelfCollisionChecker): The guard's checker.
        arm (ArmModelSettings): Validated settings carrying the new calibration.
    """
    checker.set_mount(arm.base_in_base_link or ArmBaseOffset(z=arm.arm_base_height_m))
    checker.kin.set_offsets(arm.joint_offsets_rad.model_dump())


class GuardCalibration(AppliedArmCalibration[ArmModelSettings]):
    """The guard's arm calibration: the accepted one over the configured arm (see the module docstring)."""

    def __init__(self, configured: ArmModelSettings, applied_dir: Path | None, log: Log) -> None:
        """Snapshot the configured arm (the baseline).

        Args:
            configured (ArmModelSettings): self_collision_guard.arm.
            applied_dir (Path | None): self_collision_guard.applied_dir; None keeps the configured arm.
            log (Log): Node logger.
        """
        super().__init__(configured, applied_dir)
        self.log = log

    def read_logged(self) -> tuple[ArmModelSettings, ArmCalibrationFile | None] | None:
        """Read and merge the accepted calibration; a failure is logged and gives None.

        Returns:
            tuple[ArmModelSettings, ArmCalibrationFile | None] | None: (merged settings, calibration or None for the
                configured values), None when the calibration cannot be used.
        """
        try:
            return self.read()
        except (ArmCalibrationError, ValidationError) as exc:
            self.log.error(
                f"Self-collision guard: accepted arm calibration in {self.applied_dir} not applied, keeping "
                f"{source_text(self.loaded)}: {exc}"
            )
            return None

    def startup(self) -> ArmModelSettings:
        """Load the accepted calibration before the guard is built.

        Returns:
            ArmModelSettings: The arm settings to build the guard with.
        """
        self.changed()
        loaded = self.read_logged()
        if loaded is not None:
            self.use(*loaded)
        self.log.info(f"Self-collision guard arm calibration: {source_text(self.loaded)}: {values_text(self.current)}")
        return self.current

    def tick(self, apply: ApplyFn) -> bool:
        """Re-read after an applied.json change and apply a changed calibration. Never raises.

        Args:
            apply (ApplyFn): Swaps the running checker (swap_checker_calibration bound to it).

        Returns:
            bool: True when a new calibration was applied.
        """
        if not self.changed():
            return False
        loaded = self.read_logged()
        if loaded is None or self.same(loaded[1]):
            return False
        settings, calibration = loaded
        try:
            apply(settings)
        except Exception as exc:  # the node timer must keep running: a failed swap keeps the values in use
            self.log.error(
                f"Self-collision guard: swapping to {source_text(calibration)} failed, keeping "
                f"{source_text(self.loaded)}: {exc}"
            )
            return False
        old = self.use(settings, calibration)
        self.log.info(
            f"Self-collision guard arm calibration now {source_text(calibration)}: {values_text(settings)} "
            f"(was {values_text(old)})"
        )
        return True
