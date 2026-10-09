"""Joint-space trajectory interpolation (synchronised quintic), limit clamping and tracking error."""

import math
from dataclasses import dataclass

# Peak velocity of the quintic 10s^3 - 15s^4 + 6s^5 is 15/8 * distance / duration.
QUINTIC_PEAK_VELOCITY_FACTOR = 15.0 / 8.0

# Peak acceleration of the rest-to-rest quintic is 10 / sqrt(3) * distance / duration^2.
QUINTIC_PEAK_ACCEL_FACTOR = 10.0 / math.sqrt(3.0)
# Samples per segment used to check the velocity and acceleration peaks of a blended segment.
BLEND_CHECK_SAMPLES = 64
# Growth of a segment duration per limit-check iteration beyond the measured overshoot.
BLEND_GROWTH = 1.02
BLEND_MAX_ITERATIONS = 200

JointLimits = dict[str, tuple[float, float]]


@dataclass(frozen=True)
class BlendPlan:
    """A continuous trajectory through via points (see blend_trajectory).

    Attributes:
        points (list[dict[str, float]]): Setpoints at the streaming rate (start excluded, the last one is the final via).
        via_indices (list[int]): Index into points at which each via is reached (the last is len(points) - 1).
        durations_s (list[float]): Duration of each segment (start -> via 0, via 0 -> via 1, ...).
        via_velocities (list[dict[str, float]]): Joint velocity at each via (rad/s; zero at the final one).
    """

    points: list[dict[str, float]]
    via_indices: list[int]
    durations_s: list[float]
    via_velocities: list[dict[str, float]]


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


def hermite_quintic(
    p0: float, p1: float, v0: float, v1: float, duration: float, t: float
) -> tuple[float, float, float]:
    """Quintic segment with given end positions and velocities and zero end accelerations.

    Args:
        p0 (float): Start position.
        p1 (float): End position.
        v0 (float): Start velocity (per second).
        v1 (float): End velocity (per second).
        duration (float): Segment duration (s, > 0).
        t (float): Time into the segment (s, clamped to [0, duration]).

    Returns:
        tuple[float, float, float]: Position, velocity and acceleration at t.
    """
    s = min(max(t / duration, 0.0), 1.0)
    big_v0, big_v1, delta = v0 * duration, v1 * duration, p1 - p0
    c3 = 10.0 * delta - 6.0 * big_v0 - 4.0 * big_v1
    c4 = -15.0 * delta + 8.0 * big_v0 + 7.0 * big_v1
    c5 = 6.0 * delta - 3.0 * big_v0 - 3.0 * big_v1
    pos = p0 + s * (big_v0 + s * s * (c3 + s * (c4 + s * c5)))
    vel = (big_v0 + s * s * (3.0 * c3 + s * (4.0 * c4 + s * 5.0 * c5))) / duration
    acc = s * (6.0 * c3 + s * (12.0 * c4 + s * 20.0 * c5)) / (duration * duration)
    return pos, vel, acc


def via_velocity(slope_in: float, slope_out: float, max_velocity: float) -> float:
    """Velocity at a via point: the mean of the adjacent slopes when they share a sign, else zero (turning point).

    Args:
        slope_in (float): Average velocity of the segment arriving at the via.
        slope_out (float): Average velocity of the segment leaving the via.
        max_velocity (float): Velocity cap.

    Returns:
        float: Via velocity (rad/s), within +-max_velocity.
    """
    if slope_in == 0.0 or slope_out == 0.0 or (slope_in > 0.0) != (slope_out > 0.0):
        return 0.0
    return max(-max_velocity, min(max_velocity, 0.5 * (slope_in + slope_out)))


def blend_velocities(
    nodes: list[dict[str, float]], durations: list[float], max_velocity: float
) -> list[dict[str, float]]:
    """Velocities at every node (start and final at rest) for the given segment durations.

    Args:
        nodes (list[dict[str, float]]): Start then the vias.
        durations (list[float]): Segment durations.
        max_velocity (float): Velocity cap.

    Returns:
        list[dict[str, float]]: One velocity map per node.
    """
    joints = list(nodes[0])
    out = [dict.fromkeys(joints, 0.0)]
    for k in range(1, len(nodes) - 1):
        out.append(
            {
                j: via_velocity(
                    (nodes[k][j] - nodes[k - 1][j]) / durations[k - 1],
                    (nodes[k + 1][j] - nodes[k][j]) / durations[k],
                    max_velocity,
                )
                for j in joints
            }
        )
    out.append(dict.fromkeys(joints, 0.0))
    return out


def segment_peaks(
    a: dict[str, float], b: dict[str, float], va: dict[str, float], vb: dict[str, float], duration: float
) -> tuple[float, float]:
    """Largest |velocity| and |acceleration| of any joint over one blended segment (sampled).

    Args:
        a (dict[str, float]): Segment start.
        b (dict[str, float]): Segment end.
        va (dict[str, float]): Start velocities.
        vb (dict[str, float]): End velocities.
        duration (float): Segment duration (s).

    Returns:
        tuple[float, float]: Peak velocity (rad/s) and peak acceleration (rad/s^2).
    """
    peak_v = peak_a = 0.0
    for i in range(BLEND_CHECK_SAMPLES + 1):
        t = duration * i / BLEND_CHECK_SAMPLES
        for j in b:
            _, vel, acc = hermite_quintic(a[j], b[j], va[j], vb[j], duration, t)
            peak_v, peak_a = max(peak_v, abs(vel)), max(peak_a, abs(acc))
    return peak_v, peak_a


def blend_trajectory(
    start: dict[str, float],
    vias: list[dict[str, float]],
    max_velocity: float,
    max_accel: float,
    rate_hz: float,
) -> BlendPlan:
    """One continuous trajectory from start through every via (no stop at the intermediate vias).

    Each segment is a quintic with zero acceleration at its ends and velocity continuity at the vias: the via velocity
    of a joint is the mean of its adjacent average slopes, or zero where the joint changes direction (no overshoot).
    Segment durations start from the rest-to-rest quintic bounds and grow until the sampled per-joint velocity and
    acceleration peaks stay within max_velocity and max_accel. The trajectory starts and ends at rest.

    Args:
        start (dict[str, float]): Start pose.
        vias (list[dict[str, float]]): Poses to pass through (same joints as start); the last is the goal.
        max_velocity (float): Per-joint velocity cap (rad/s).
        max_accel (float): Per-joint acceleration cap (rad/s^2).
        rate_hz (float): Setpoint rate.

    Returns:
        BlendPlan: Setpoints (start excluded, last = final via exactly), via indices, durations and via velocities.

    Raises:
        ValueError: For no vias, mismatched joints or non-positive limits.
    """
    if not vias:
        raise ValueError("at least one via is required")
    if max_velocity <= 0.0 or max_accel <= 0.0 or rate_hz <= 0.0:
        raise ValueError("max_velocity, max_accel and rate_hz must be positive")
    for via in vias:
        if set(via) != set(start):
            raise ValueError(f"via joints differ from the start: {sorted(via)} vs {sorted(start)}")
    nodes = [start, *vias]
    period = 1.0 / rate_hz
    durations = []
    for a, b in zip(nodes, nodes[1:], strict=False):
        distance = max((abs(b[j] - a[j]) for j in b), default=0.0)
        durations.append(
            max(
                QUINTIC_PEAK_VELOCITY_FACTOR * distance / max_velocity,
                math.sqrt(QUINTIC_PEAK_ACCEL_FACTOR * distance / max_accel),
                period,
            )
        )
    velocities = blend_velocities(nodes, durations, max_velocity)
    for _ in range(BLEND_MAX_ITERATIONS):
        grown = False
        for k in range(len(durations)):
            peak_v, peak_a = segment_peaks(nodes[k], nodes[k + 1], velocities[k], velocities[k + 1], durations[k])
            factor = max(peak_v / max_velocity, math.sqrt(peak_a / max_accel))
            if factor > 1.0 + 1e-9:
                durations[k] *= factor * BLEND_GROWTH
                grown = True
        if not grown:
            break
        velocities = blend_velocities(nodes, durations, max_velocity)
    ends = [sum(durations[: k + 1]) for k in range(len(durations))]
    steps = max(1, math.ceil(ends[-1] * rate_hz - 1e-9))
    points: list[dict[str, float]] = []
    segment = 0
    for i in range(1, steps):
        t = i * period
        while segment < len(durations) - 1 and t > ends[segment]:
            segment += 1
        t0 = ends[segment] - durations[segment]
        a, b = nodes[segment], nodes[segment + 1]
        va, vb = velocities[segment], velocities[segment + 1]
        points.append({j: hermite_quintic(a[j], b[j], va[j], vb[j], durations[segment], t - t0)[0] for j in b})
    points.append(dict(vias[-1]))
    via_indices = [min(steps - 1, max(0, round(end * rate_hz) - 1)) for end in ends]
    via_indices[-1] = steps - 1
    return BlendPlan(
        points=points,
        via_indices=via_indices,
        durations_s=[round(d, 6) for d in durations],
        via_velocities=velocities[1:],
    )
