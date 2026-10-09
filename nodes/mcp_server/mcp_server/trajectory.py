"""Joint-space trajectory interpolation (synchronised quintic), limit clamping and tracking error."""

import math

# Peak velocity of the quintic 10s^3 - 15s^4 + 6s^5 is 15/8 * distance / duration.
QUINTIC_PEAK_VELOCITY_FACTOR = 15.0 / 8.0

JointLimits = dict[str, tuple[float, float]]


def quintic(s: float) -> float:
    """Minimum-jerk blend: 0 at s<=0, 1 at s>=1, zero velocity and acceleration at both ends.

    Args:
        s (float): Normalised time.

    Returns:
        float: Normalised position in [0, 1].
    """
    s = min(max(s, 0.0), 1.0)
    return s * s * s * (10.0 + s * (-15.0 + 6.0 * s))


def clamp_to_limits(
    targets: dict[str, float], limits: JointLimits, margin: float, overrides: dict[str, float] | None = None
) -> dict[str, float]:
    """Clamp joint targets into [lower + margin, upper - margin].

    Args:
        targets (dict[str, float]): Joint name -> target (rad).
        limits (JointLimits): Joint name -> (lower, upper) from the URDF.
        margin (float): Safety margin kept away from each limit (rad).
        overrides (dict[str, float] | None): Per-joint margins replacing `margin` (rad).

    Returns:
        dict[str, float]: Clamped targets.

    Raises:
        KeyError: For a joint without limits.
    """
    out: dict[str, float] = {}
    for name, value in targets.items():
        lo, hi = limits[name]
        m = (overrides or {}).get(name, margin)
        out[name] = min(max(value, lo + m), hi - m)
    return out


def trajectory_duration(
    start: dict[str, float], goal: dict[str, float], max_velocity: float, min_duration: float
) -> float:
    """Duration so that no joint of a synchronised quintic exceeds max_velocity.

    Args:
        start (dict[str, float]): Start positions.
        goal (dict[str, float]): Goal positions (same joints).
        max_velocity (float): Per-joint velocity cap (rad/s).
        min_duration (float): Lower bound on the duration (s).

    Returns:
        float: Duration in seconds.
    """
    if max_velocity <= 0.0:
        raise ValueError("max_velocity must be positive")
    distance = max((abs(goal[j] - start[j]) for j in goal), default=0.0)
    return max(QUINTIC_PEAK_VELOCITY_FACTOR * distance / max_velocity, min_duration)


def plan_trajectory(
    start: dict[str, float],
    goal: dict[str, float],
    max_velocity: float,
    rate_hz: float,
    min_duration: float = 0.0,
) -> list[dict[str, float]]:
    """Sample a synchronised quintic from start to goal at rate_hz (the start itself is not included).

    Args:
        start (dict[str, float]): Start positions.
        goal (dict[str, float]): Goal positions; must name the same joints as start.
        max_velocity (float): Per-joint velocity cap (rad/s).
        rate_hz (float): Setpoint rate.
        min_duration (float): Lower bound on the duration (s).

    Returns:
        list[dict[str, float]]: Setpoints; the last one equals goal exactly.
    """
    if set(start) != set(goal):
        raise ValueError(f"start and goal joints differ: {sorted(start)} vs {sorted(goal)}")
    duration = trajectory_duration(start, goal, max_velocity, min_duration)
    steps = max(1, math.ceil(duration * rate_hz))
    points: list[dict[str, float]] = []
    for i in range(1, steps):
        blend = quintic(i / steps)
        points.append({j: start[j] + (goal[j] - start[j]) * blend for j in goal})
    points.append(dict(goal))
    return points


def max_abs_error(a: dict[str, float], b: dict[str, float], joints: list[str]) -> float:
    """Largest absolute difference between two joint maps over the given joints.

    Args:
        a (dict[str, float]): First positions.
        b (dict[str, float]): Second positions.
        joints (list[str]): Joints to compare.

    Returns:
        float: Max |a - b| (0.0 for no joints).
    """
    return max((abs(a[j] - b[j]) for j in joints), default=0.0)


def path_trajectory(
    start: dict[str, float], path: list[dict[str, float]], max_velocity: float, rate_hz: float
) -> list[dict[str, float]]:
    """Sample a joint-space polyline (start, then path) with one quintic time profile over its whole length.

    Progress along the polyline is measured with the largest joint change of each leg, so no joint exceeds
    max_velocity and the motion starts and stops smoothly without halting at the intermediate samples.

    Args:
        start (dict[str, float]): Pose before the first path sample.
        path (list[dict[str, float]]): Samples (same joints as start); the last one is the goal.
        max_velocity (float): Per-joint velocity cap (rad/s).
        rate_hz (float): Setpoint rate.

    Returns:
        list[dict[str, float]]: Setpoints (start excluded); the last one equals path[-1] exactly.
    """
    if max_velocity <= 0.0:
        raise ValueError("max_velocity must be positive")
    nodes = [start, *path]
    legs = [max(abs(b[j] - a[j]) for j in b) for a, b in zip(nodes, nodes[1:], strict=False)]
    total = sum(legs)
    if total <= 0.0:
        return [dict(path[-1])]
    steps = max(1, math.ceil(QUINTIC_PEAK_VELOCITY_FACTOR * total / max_velocity * rate_hz))
    points: list[dict[str, float]] = []
    leg, covered = 0, 0.0
    for i in range(1, steps):
        distance = quintic(i / steps) * total
        while leg < len(legs) - 1 and covered + legs[leg] < distance:
            covered += legs[leg]
            leg += 1
        a, b = nodes[leg], nodes[leg + 1]
        f = 0.0 if legs[leg] <= 0.0 else min(1.0, (distance - covered) / legs[leg])
        points.append({j: a[j] + (b[j] - a[j]) * f for j in b})
    points.append(dict(path[-1]))
    return points
