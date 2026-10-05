"""Forward and inverse kinematics for symmetric 4-wheel swerve drive."""

import math

import numpy as np

# Wheel positions in body frame (x forward, y left): fl, fr, rl, rr.
# Order matches joint_names: fl_drive, fl_steer, fr_drive, fr_steer, rl_drive, rl_steer, rr_drive, rr_steer.
WHEEL_ORDER = ("fl", "fr", "rl", "rr")
# Headings up to this far past the steering limit keep the wheel on its current side (clamped to the limit)
# instead of swinging 180 deg to the other side; avoids flip-flopping during sideways motion.
STEER_LIMIT_HYSTERESIS_RAD = 0.1
# Group-level hysteresis: when both common sides (all wheels unflipped / all flipped) are feasible, the previous
# side is kept unless the other one reduces the summed steering travel of the moving wheels by more than this.
GROUP_FLIP_HYSTERESIS_RAD = 0.2
# A leave-one-wheel-out solution replaces the full one only when its residual is below this fraction of it.
SLIP_DROP_IMPROVEMENT = 0.5
# Baseline odometry twist variance (m/s)^2 and (rad/s)^2 before the slip residual is added.
TWIST_VARIANCE_FLOOR = 0.01


def wheel_positions(lx: float, ly: float) -> np.ndarray:
    """Return 4x2 array of (x, y) wheel positions in body frame for fl, fr, rl, rr.

    Args:
        lx: Half-length (center to front/rear axle), meters.
        ly: Half-width (center to left/right), meters.

    Returns:
        np.ndarray: Shape (4, 2), rows are [x, y] for each wheel (fl, fr, rl, rr).
    """
    return np.array(
        [
            [lx, ly],  # fl
            [lx, -ly],  # fr
            [-lx, ly],  # rl
            [-lx, -ly],  # rr
        ],
        dtype=float,
    )


def inverse_kinematics(
    vx: float,
    vy: float,
    omega: float,
    lx: float,
    ly: float,
    wheel_radius: float,
) -> tuple[list[float], list[float]]:
    """Compute steering angles (rad) and drive angular velocities (rad/s) for each wheel.

    Body frame: x forward, y left, omega positive = counterclockwise (yaw).
    Wheel order: fl, fr, rl, rr.

    Args:
        vx: Forward velocity in body frame, m/s.
        vy: Leftward velocity in body frame, m/s.
        omega: Angular velocity about vertical, rad/s.
        lx: Half-length, m.
        ly: Half-width, m.
        wheel_radius: Wheel radius, m.

    Returns:
        tuple: (steer_angles, drive_angular_velocities), each list of 4 floats (rad, rad/s).
    """
    positions = wheel_positions(lx, ly)
    steer_angles: list[float] = []
    drive_angular: list[float] = []
    for i in range(4):
        x_i, y_i = positions[i, 0], positions[i, 1]
        vx_i = vx - omega * y_i
        vy_i = vy + omega * x_i
        speed = math.sqrt(vx_i * vx_i + vy_i * vy_i)
        if speed < 1e-9:
            steer_angles.append(0.0)
            drive_angular.append(0.0)
            continue
        alpha = math.atan2(vy_i, vx_i)
        steer_angles.append(alpha)
        drive_angular.append(speed / wheel_radius)
    return steer_angles, drive_angular


def fk_system(
    steer_angles: list[float],
    drive_angular_velocities: list[float],
    lx: float,
    ly: float,
    wheel_radius: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Build the 8x3 least-squares system A @ [vx, vy, omega] = b of the wheel states.

    Each wheel i: vx_i = s_i*cos(alpha_i), vy_i = s_i*sin(alpha_i) with s_i = drive_angular_i * R, and
    body: vx_i = vx - omega*y_i, vy_i = vy + omega*x_i. Rows 2i and 2i+1 belong to wheel i.

    Args:
        steer_angles: Steering angle per wheel (rad), order fl, fr, rl, rr.
        drive_angular_velocities: Drive angular velocity per wheel (rad/s), order fl, fr, rl, rr.
        lx: Half-length, m.
        ly: Half-width, m.
        wheel_radius: Wheel radius, m.

    Returns:
        tuple[np.ndarray, np.ndarray]: (A with shape (8, 3), b with shape (8,)).
    """
    positions = wheel_positions(lx, ly)
    A = np.zeros((8, 3))
    b = np.zeros(8)
    for i in range(4):
        x_i, y_i = positions[i, 0], positions[i, 1]
        alpha = steer_angles[i] if i < len(steer_angles) else 0.0
        drive = drive_angular_velocities[i] if i < len(drive_angular_velocities) else 0.0
        s_i = drive * wheel_radius
        A[2 * i, :] = [1, 0, -y_i]
        A[2 * i + 1, :] = [0, 1, x_i]
        b[2 * i] = s_i * math.cos(alpha)
        b[2 * i + 1] = s_i * math.sin(alpha)
    return A, b


def solve_twist(A: np.ndarray, b: np.ndarray) -> tuple[tuple[float, float, float], float]:
    """Least-squares twist of a stacked wheel system and its residual.

    Args:
        A: Coefficient matrix with shape (n, 3), n even (two rows per wheel).
        b: Right-hand side with shape (n,).

    Returns:
        tuple: ((vx, vy, omega), residual) with residual = ||A x - b|| / sqrt(n / 2 + 1), m/s. The divisor is
        sqrt(5) for the full 8-equation system and sqrt(4) for a leave-one-wheel-out 6-equation system.
    """
    x, _residuals, _rank, _s = np.linalg.lstsq(A, b, rcond=None)
    residual = float(np.linalg.norm(A @ x - b) / math.sqrt(len(b) / 2 + 1))
    return (float(x[0]), float(x[1]), float(x[2])), residual


def forward_kinematics_with_residual(
    steer_angles: list[float],
    drive_angular_velocities: list[float],
    lx: float,
    ly: float,
    wheel_radius: float,
) -> tuple[tuple[float, float, float], float]:
    """Compute body twist (vx, vy, omega) from wheel states plus the least-squares residual.

    The residual r = ||A x - b|| / sqrt(5) (m/s; 8 equations, 3 unknowns) is ~0 when the four wheels agree on a
    single rigid-body twist and grows when a wheel slips or stalls.

    Args:
        steer_angles: Steering angle per wheel (rad), order fl, fr, rl, rr.
        drive_angular_velocities: Drive angular velocity per wheel (rad/s), order fl, fr, rl, rr.
        lx: Half-length, m.
        ly: Half-width, m.
        wheel_radius: Wheel radius, m.

    Returns:
        tuple: ((vx, vy, omega) in body frame (m/s, m/s, rad/s), residual in m/s).
    """
    A, b = fk_system(steer_angles, drive_angular_velocities, lx, ly, wheel_radius)
    return solve_twist(A, b)


def forward_kinematics(
    steer_angles: list[float],
    drive_angular_velocities: list[float],
    lx: float,
    ly: float,
    wheel_radius: float,
) -> tuple[float, float, float]:
    """Compute body twist (vx, vy, omega) from wheel states (least squares over all four wheels).

    Args:
        steer_angles: Steering angle per wheel (rad), order fl, fr, rl, rr.
        drive_angular_velocities: Drive angular velocity per wheel (rad/s), order fl, fr, rl, rr.
        lx: Half-length, m.
        ly: Half-width, m.
        wheel_radius: Wheel radius, m.

    Returns:
        tuple: (vx, vy, omega) in body frame (m/s, m/s, rad/s).
    """
    return forward_kinematics_with_residual(steer_angles, drive_angular_velocities, lx, ly, wheel_radius)[0]


def robust_forward_kinematics(
    steer_angles: list[float],
    drive_angular_velocities: list[float],
    lx: float,
    ly: float,
    wheel_radius: float,
    slip_residual_threshold_mps: float,
) -> tuple[tuple[float, float, float], float]:
    """Forward kinematics that drops a single slipping wheel.

    When the full residual exceeds the threshold, the four leave-one-wheel-out 6x3 least-squares problems are
    solved; if the best one has a residual below half the full residual, its twist is used (the odd wheel is
    ignored) and its residual reported. Otherwise (several wheels disagree) the full solution is kept.

    Args:
        steer_angles: Steering angle per wheel (rad), order fl, fr, rl, rr.
        drive_angular_velocities: Drive angular velocity per wheel (rad/s), order fl, fr, rl, rr.
        lx: Half-length, m.
        ly: Half-width, m.
        wheel_radius: Wheel radius, m.
        slip_residual_threshold_mps: Full-system residual above which wheel dropping is attempted, m/s.

    Returns:
        tuple: ((vx, vy, omega) in body frame, residual in m/s of the solution that was used).
    """
    A, b = fk_system(steer_angles, drive_angular_velocities, lx, ly, wheel_radius)
    twist, residual = solve_twist(A, b)
    if residual <= slip_residual_threshold_mps:
        return twist, residual
    best_twist, best_residual = twist, residual
    for dropped in range(4):
        keep = [row for row in range(8) if row // 2 != dropped]
        candidate, candidate_residual = solve_twist(A[keep], b[keep])
        if candidate_residual < best_residual:
            best_twist, best_residual = candidate, candidate_residual
    if best_residual < SLIP_DROP_IMPROVEMENT * residual:
        return best_twist, best_residual
    return twist, residual


def odometry_twist_variances(residual: float, lx: float, ly: float) -> tuple[float, float]:
    """Odometry twist variances that grow with the wheel-consistency residual.

    Args:
        residual: Forward-kinematics residual, m/s.
        lx: Half-length, m.
        ly: Half-width, m.

    Returns:
        tuple[float, float]: (var_xy in (m/s)^2, var_yaw in (rad/s)^2). The yaw term converts the residual to a
        rate through the largest module radius hypot(lx, ly).
    """
    return (
        TWIST_VARIANCE_FLOOR + residual**2,
        TWIST_VARIANCE_FLOOR + (residual / math.hypot(lx, ly)) ** 2,
    )


def steer_angle_difference(current: float, desired: float) -> float:
    """Smallest difference (rad) from current to desired steer angle in [-pi, pi].

    Args:
        current: Current steering angle, rad.
        desired: Desired steering angle, rad.

    Returns:
        float: Difference in rad, in [-pi, pi].
    """
    diff = desired - current
    while diff > math.pi:
        diff -= 2.0 * math.pi
    while diff < -math.pi:
        diff += 2.0 * math.pi
    return diff


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi].

    Args:
        angle: Angle in radians, any value.

    Returns:
        float: Normalized angle in [-pi, pi].
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def fold_to_steer_range(
    steer: float,
    drive: float,
    current_steer: float,
    limit: float,
) -> tuple[float, float]:
    """Map a wheel heading into the steering range [-limit, limit] (limit >= pi/2).

    A wheel at heading a driving at speed s is equivalent to heading a + pi driving at -s. With a
    +-90 deg steering range every heading has at least one reachable equivalent; when both are reachable
    (exactly at the boundary) the one closer to current_steer is used.

    Args:
        steer: Desired wheel heading from IK, rad (any value).
        drive: Desired drive angular velocity, rad/s.
        current_steer: Current measured steering angle, rad.
        limit: Steering limit (absolute), rad.

    Returns:
        tuple[float, float]: (steer_rad within [-limit, limit], drive_rad_per_s).
    """
    candidates = [(normalize_angle(steer), drive), (normalize_angle(steer + math.pi), -drive)]
    reachable = [c for c in candidates if abs(c[0]) <= limit + 1e-9]
    # Hysteresis: a candidate slightly past the limit on the wheel's current side is acceptable (clamped below).
    reachable += [
        c for c in candidates if limit < abs(c[0]) <= limit + STEER_LIMIT_HYSTERESIS_RAD and c[0] * current_steer > 0
    ]
    if not reachable:
        # Only possible when limit < pi/2: clamp the closer candidate (drive kept, direction approximate).
        reachable = [min(candidates, key=lambda c: abs(c[0]))]
        reachable = [(max(-limit, min(limit, reachable[0][0])), reachable[0][1])]
    best = min(reachable, key=lambda c: abs(steer_angle_difference(current_steer, c[0])))
    return (max(-limit, min(limit, best[0])), best[1])


def desaturate_wheel_speeds(drives: list[float], max_speed: float) -> list[float]:
    """Scale all wheel speeds by a common factor so none exceeds max_speed (keeps the motion direction).

    Args:
        drives: Wheel drive angular velocities, rad/s.
        max_speed: Maximum allowed magnitude, rad/s.

    Returns:
        list[float]: Scaled wheel speeds, rad/s.
    """
    peak = max((abs(d) for d in drives), default=0.0)
    if peak <= max_speed or peak == 0.0:
        return list(drives)
    scale = max_speed / peak
    return [d * scale for d in drives]


def flip_candidate(steer: float, drive: float, flip: bool) -> tuple[float, float]:
    """Return one of the two equivalent wheel states: (heading, drive) or (heading + pi, -drive).

    Args:
        steer: Wheel heading from IK, rad (any value).
        drive: Drive angular velocity from IK, rad/s.
        flip: True for the flipped equivalent (heading + pi, reversed drive).

    Returns:
        tuple[float, float]: (heading normalized to [-pi, pi], drive), rad and rad/s.
    """
    if flip:
        return normalize_angle(steer + math.pi), -drive
    return normalize_angle(steer), drive


def candidate_feasible(candidate: float, current_steer: float, limit: float) -> bool:
    """Return True if a candidate heading is reachable: within the limit, or up to STEER_LIMIT_HYSTERESIS_RAD past
    it on the side the wheel is currently on (a centred wheel counts as being on both sides).

    Args:
        candidate: Candidate heading, rad, in [-pi, pi].
        current_steer: Current steering target of the wheel, rad.
        limit: Steering limit (absolute), rad.

    Returns:
        bool: True if the candidate may be used (clamped to the limit).
    """
    if abs(candidate) <= limit + 1e-9:
        return True
    return abs(candidate) <= limit + STEER_LIMIT_HYSTERESIS_RAD and candidate * current_steer >= 0.0


def choose_common_flip(
    ik_steer: list[float],
    moving: list[bool],
    current_steer: list[float],
    limit: float,
    previous_flip: bool | None,
) -> bool | None:
    """Choose one side (all wheels unflipped or all flipped) for the whole set of moving wheels.

    A side is feasible when its candidate heading is reachable for every moving wheel (see candidate_feasible).
    Among feasible sides the one with the smaller summed steering travel from current_steer wins, except that the
    previous side is kept unless the other one saves more than GROUP_FLIP_HYSTERESIS_RAD of travel.

    Args:
        ik_steer: IK heading per wheel, rad.
        moving: Per wheel, True when it has a non-zero drive command (stopped wheels are ignored).
        current_steer: Current steering target per wheel, rad.
        limit: Steering limit (absolute), rad.
        previous_flip: Side chosen in the previous cycle, or None if it used the per-wheel fallback.

    Returns:
        bool | None: False = all unflipped, True = all flipped, None = no common side is feasible. With no moving
        wheel the previous choice is returned unchanged.
    """
    wheels = [i for i in range(len(ik_steer)) if moving[i]]
    if not wheels:
        return previous_flip
    travel: dict[bool, float] = {}
    for flip in (False, True):
        candidates = [flip_candidate(ik_steer[i], 1.0, flip)[0] for i in wheels]
        if all(candidate_feasible(c, current_steer[i], limit) for c, i in zip(candidates, wheels)):
            travel[flip] = sum(abs(max(-limit, min(limit, c)) - current_steer[i]) for c, i in zip(candidates, wheels))
    if not travel:
        return None
    if len(travel) == 1:
        return next(iter(travel))
    best = min(travel, key=lambda flip: (travel[flip], flip))
    if previous_flip is not None and travel[previous_flip] - travel[best] <= GROUP_FLIP_HYSTERESIS_RAD:
        return previous_flip
    return best


def compute_wheel_commands(
    vx: float,
    vy: float,
    omega: float,
    current_steer: list[float],
    lx: float,
    ly: float,
    wheel_radius: float,
    steer_limit: float,
    max_wheel_speed: float,
    previous_flip: bool | None = None,
) -> tuple[list[float], list[float], bool | None]:
    """Full swerve command: IK, common side choice (or per-wheel fold), hold steer when stopped, desaturate.

    All moving wheels use the same equivalent (all unflipped or all flipped, see choose_common_flip) so they stay
    aligned at the +-90 deg limit; only when no common side is feasible (e.g. rotation in place) each wheel is
    folded on its own with fold_to_steer_range.

    Args:
        vx: Forward body velocity, m/s.
        vy: Leftward body velocity, m/s.
        omega: Yaw rate (CCW positive), rad/s.
        current_steer: Current steering angle per wheel (fl, fr, rl, rr), rad.
        lx: Half-length (center to front/rear yaw axis), m.
        ly: Half-width (center to left/right yaw axis), m.
        wheel_radius: Wheel radius, m.
        steer_limit: Steering limit (absolute), rad.
        max_wheel_speed: Maximum drive angular velocity, rad/s.
        previous_flip: Common side chosen in the previous cycle (None if none / per-wheel fallback).

    Returns:
        tuple[list[float], list[float], bool | None]: (steer angles rad, drive angular velocities rad/s, common
        side: False unflipped, True flipped, None per-wheel fallback), wheel order fl, fr, rl, rr.
    """
    ik_steer, ik_drive = inverse_kinematics(vx, vy, omega, lx, ly, wheel_radius)
    moving = [d != 0.0 for d in ik_drive]
    flip = choose_common_flip(ik_steer, moving, current_steer, steer_limit, previous_flip)
    steer_out: list[float] = []
    drive_out: list[float] = []
    for i in range(4):
        if not moving[i]:
            # Stopped wheel: keep its current heading instead of swinging back to centre.
            steer_out.append(max(-steer_limit, min(steer_limit, current_steer[i])))
            drive_out.append(0.0)
            continue
        if flip is None:
            steer, drive = fold_to_steer_range(ik_steer[i], ik_drive[i], current_steer[i], steer_limit)
        else:
            steer, drive = flip_candidate(ik_steer[i], ik_drive[i], flip)
        steer_out.append(max(-steer_limit, min(steer_limit, steer)))
        drive_out.append(drive)
    return steer_out, desaturate_wheel_speeds(drive_out, max_wheel_speed), flip


def wheel_states(
    joint_positions: dict[str, float],
    joint_velocities: dict[str, float],
    steer_joints: list[str],
    drive_joints: list[str],
) -> tuple[list[float], list[float]] | None:
    """Collect measured steer angles and drive velocities; None unless every joint has been reported.

    Args:
        joint_positions: Latest joint name -> position (rad).
        joint_velocities: Latest joint name -> velocity (rad/s).
        steer_joints: Steering joint names, order fl, fr, rl, rr.
        drive_joints: Drive joint names, order fl, fr, rl, rr.

    Returns:
        tuple[list[float], list[float]] | None: (steer angles, drive velocities), or None if any is missing.
    """
    if any(j not in joint_positions for j in steer_joints) or any(j not in joint_velocities for j in drive_joints):
        return None
    return [joint_positions[j] for j in steer_joints], [joint_velocities[j] for j in drive_joints]


def integrate_odometry(
    pose: tuple[float, float, float],
    twist: tuple[float, float, float],
    dt: float,
) -> tuple[float, float, float]:
    """Integrate a body twist into an odom-frame pose using the midpoint heading.

    Args:
        pose: (x m, y m, theta rad) in the odom frame.
        twist: (vx m/s, vy m/s, omega rad/s) in the body frame.
        dt: Time step, s.

    Returns:
        tuple[float, float, float]: New (x, y, theta), theta normalized to [-pi, pi].
    """
    x, y, theta = pose
    vx, vy, omega = twist
    mid = theta + 0.5 * omega * dt
    x += (vx * math.cos(mid) - vy * math.sin(mid)) * dt
    y += (vx * math.sin(mid) + vy * math.cos(mid)) * dt
    return (x, y, normalize_angle(theta + omega * dt))


def should_zero_drive(
    current_steer: float,
    desired_steer: float,
    threshold_rad: float,
) -> bool:
    """Return True if drive should be zeroed to avoid strain (steer error above threshold).

    Args:
        current_steer: Current steering angle, rad.
        desired_steer: Desired steering angle, rad.
        threshold_rad: Max allowed error before zeroing drive, rad.

    Returns:
        bool: True if |error| > threshold.
    """
    return abs(steer_angle_difference(current_steer, desired_steer)) > threshold_rad
