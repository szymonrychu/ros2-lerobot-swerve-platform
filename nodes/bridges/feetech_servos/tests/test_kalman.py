"""Unit tests for constant-velocity joint Kalman tracking."""

from feetech_servos.kalman import make_joint_kalman_filter, predict_joint, predicted_position, update_joint


def test_kalman_tracks_constant_velocity_signal() -> None:
    """Filter converges to roughly correct velocity on linear motion."""
    f = make_joint_kalman_filter(initial_position=0.0, now_s=0.0)
    for i in range(1, 76):
        now = i * 0.02
        measurement = 100.0 * now  # 100 steps/s
        update_joint(
            f,
            measurement_position=measurement,
            now_s=now,
            process_noise_pos=200.0,
            process_noise_vel=1200.0,
            measurement_noise=16.0,
        )
    assert f.velocity > 80.0
    assert abs(f.position - 150.0) < 8.0


def test_kalman_prediction_lead_moves_ahead() -> None:
    """Predicted position should advance with positive velocity."""
    f = make_joint_kalman_filter(initial_position=0.0, now_s=0.0)
    update_joint(
        f,
        measurement_position=10.0,
        now_s=0.1,
        process_noise_pos=200.0,
        process_noise_vel=1200.0,
        measurement_noise=16.0,
    )
    predict_joint(
        f,
        now_s=0.12,
        process_noise_pos=200.0,
        process_noise_vel=1200.0,
    )
    lead = predicted_position(f, 0.05)
    assert lead >= f.position


def test_kalman_velocity_decay_reduces_extrapolated_speed() -> None:
    """Prediction with decay should reduce retained velocity over time."""
    f = make_joint_kalman_filter(initial_position=0.0, now_s=0.0)
    update_joint(
        f,
        measurement_position=20.0,
        now_s=0.1,
        process_noise_pos=200.0,
        process_noise_vel=1200.0,
        measurement_noise=16.0,
    )
    velocity_before = abs(f.velocity)
    predict_joint(
        f,
        now_s=0.3,
        process_noise_pos=200.0,
        process_noise_vel=1200.0,
        velocity_decay_per_s=6.0,
    )
    assert abs(f.velocity) < velocity_before
