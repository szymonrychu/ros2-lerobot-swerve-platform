"""Constant-velocity Kalman filter helpers for joint target tracking."""

from dataclasses import dataclass

MIN_DT_S = 1e-6


@dataclass
class JointKalmanFilter:
    """Single-joint Kalman state for position+velocity tracking.

    Attributes:
        position: Estimated position in servo steps.
        velocity: Estimated velocity in steps/second.
        p11: Covariance term for position variance.
        p12: Covariance term for position/velocity covariance.
        p21: Covariance term for velocity/position covariance.
        p22: Covariance term for velocity variance.
        last_time_s: Last monotonic update timestamp.
    """

    position: float
    velocity: float
    p11: float
    p12: float
    p21: float
    p22: float
    last_time_s: float


def make_joint_kalman_filter(initial_position: float, now_s: float) -> JointKalmanFilter:
    """Create an initialized joint Kalman filter.

    Args:
        initial_position: Initial servo goal in steps.
        now_s: Current monotonic time in seconds.

    Returns:
        JointKalmanFilter: Initialized filter state.
    """
    return JointKalmanFilter(
        position=float(initial_position),
        velocity=0.0,
        p11=25.0,
        p12=0.0,
        p21=0.0,
        p22=400.0,
        last_time_s=float(now_s),
    )


def predict_joint(
    filter_state: JointKalmanFilter, now_s: float, process_noise_pos: float, process_noise_vel: float
) -> None:
    """Run Kalman predict step with constant-velocity process model.

    Args:
        filter_state: Filter state to update in-place.
        now_s: Current monotonic time in seconds.
        process_noise_pos: Position process noise density (steps^2 / s).
        process_noise_vel: Velocity process noise density ((steps/s)^2 / s).
    """
    dt = max(MIN_DT_S, float(now_s) - filter_state.last_time_s)

    # x = F x
    new_position = filter_state.position + filter_state.velocity * dt
    new_velocity = filter_state.velocity

    # P = F P F^T + Q
    p11 = filter_state.p11 + dt * (filter_state.p21 + filter_state.p12) + dt * dt * filter_state.p22
    p12 = filter_state.p12 + dt * filter_state.p22
    p21 = filter_state.p21 + dt * filter_state.p22
    p22 = filter_state.p22

    q11 = max(0.0, float(process_noise_pos)) * dt
    q22 = max(0.0, float(process_noise_vel)) * dt
    p11 += q11
    p22 += q22

    filter_state.position = new_position
    filter_state.velocity = new_velocity
    filter_state.p11 = p11
    filter_state.p12 = p12
    filter_state.p21 = p21
    filter_state.p22 = p22
    filter_state.last_time_s = float(now_s)


def update_joint(
    filter_state: JointKalmanFilter,
    measurement_position: float,
    now_s: float,
    process_noise_pos: float,
    process_noise_vel: float,
    measurement_noise: float,
) -> None:
    """Run full predict+update cycle for a position measurement.

    Args:
        filter_state: Filter state to update in-place.
        measurement_position: Measured target position in steps.
        now_s: Current monotonic time in seconds.
        process_noise_pos: Position process noise density.
        process_noise_vel: Velocity process noise density.
        measurement_noise: Position measurement variance.
    """
    predict_joint(filter_state, now_s, process_noise_pos, process_noise_vel)

    z = float(measurement_position)
    r = max(1e-9, float(measurement_noise))

    innovation = z - filter_state.position
    s = filter_state.p11 + r
    k1 = filter_state.p11 / s
    k2 = filter_state.p21 / s

    filter_state.position = filter_state.position + k1 * innovation
    filter_state.velocity = filter_state.velocity + k2 * innovation

    p11 = (1.0 - k1) * filter_state.p11
    p12 = (1.0 - k1) * filter_state.p12
    p21 = filter_state.p21 - k2 * filter_state.p11
    p22 = filter_state.p22 - k2 * filter_state.p12

    filter_state.p11 = p11
    filter_state.p12 = p12
    filter_state.p21 = p21
    filter_state.p22 = p22


def predicted_position(filter_state: JointKalmanFilter, lead_time_s: float) -> float:
    """Return short-horizon predicted position from current state.

    Args:
        filter_state: Filter state.
        lead_time_s: Prediction horizon in seconds.

    Returns:
        float: Predicted position in steps.
    """
    horizon = max(0.0, float(lead_time_s))
    return filter_state.position + filter_state.velocity * horizon
