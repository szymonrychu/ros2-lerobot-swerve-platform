"""Simple smooth joint interpolation for goal_position commands."""

from dataclasses import dataclass


def smoothstep01(progress: float) -> float:
    """Return smoothstep interpolation factor for progress in [0, 1].

    Args:
        progress: Interpolation progress in [0.0, 1.0].

    Returns:
        float: Smoothstep value in [0.0, 1.0] with ease-in/ease-out curve.
    """
    p = min(1.0, max(0.0, float(progress)))
    return p * p * (3.0 - 2.0 * p)


@dataclass
class JointInterpolator:
    """Track and sample smooth transitions between joint command targets.

    Attributes:
        current_position: Last sampled position (servo goal units).
        start_position: Position at interpolation start.
        target_position: Current target position.
        start_time_s: Monotonic time when current interpolation started.
        duration_s: Interpolation duration in seconds.
    """

    current_position: float
    start_position: float
    target_position: float
    start_time_s: float
    duration_s: float


def make_joint_interpolator(initial_position: float, now_s: float) -> JointInterpolator:
    """Create interpolator initialized at the provided position/time.

    Args:
        initial_position: Initial joint position in servo goal units.
        now_s: Current monotonic time in seconds.

    Returns:
        JointInterpolator: Initialized interpolator with no pending motion.
    """
    initial = float(initial_position)
    return JointInterpolator(
        current_position=initial,
        start_position=initial,
        target_position=initial,
        start_time_s=float(now_s),
        duration_s=0.0,
    )


def set_joint_target(interpolator: JointInterpolator, target_position: float, now_s: float, duration_s: float) -> None:
    """Set a new target and start a smooth transition from current sampled position.

    Args:
        interpolator: Joint interpolator state to update.
        target_position: New target in servo goal units.
        now_s: Current monotonic time in seconds.
        duration_s: Interpolation duration in seconds.
    """
    current = sample_joint(interpolator, now_s)
    interpolator.start_position = current
    interpolator.current_position = current
    interpolator.target_position = float(target_position)
    interpolator.start_time_s = float(now_s)
    interpolator.duration_s = max(0.0, float(duration_s))


def sample_joint(interpolator: JointInterpolator, now_s: float) -> float:
    """Sample interpolated joint position for current time.

    Args:
        interpolator: Joint interpolator state.
        now_s: Current monotonic time in seconds.

    Returns:
        float: Interpolated position in servo goal units.
    """
    if interpolator.duration_s <= 0.0:
        interpolator.current_position = interpolator.target_position
        return interpolator.current_position
    elapsed = max(0.0, float(now_s) - interpolator.start_time_s)
    progress = min(1.0, elapsed / interpolator.duration_s)
    alpha = smoothstep01(progress)
    interpolator.current_position = (
        interpolator.start_position + (interpolator.target_position - interpolator.start_position) * alpha
    )
    return interpolator.current_position
