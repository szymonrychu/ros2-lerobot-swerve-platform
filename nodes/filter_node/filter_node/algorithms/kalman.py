"""Constant-velocity Kalman filter in radian space for joint command smoothing."""

import math
from dataclasses import dataclass
from typing import Any

from .base import FilterAlgorithm

MIN_DT_S = 1e-6


@dataclass
class _KalmanState:
    """Per-joint Kalman state (position rad, velocity rad/s, covariance, last time)."""

    position: float
    velocity: float
    p11: float
    p12: float
    p21: float
    p22: float
    last_time_s: float


class KalmanFilterAlgorithm(FilterAlgorithm):
    """Constant-velocity Kalman filter; params: process_noise_pos, process_noise_vel, measurement_noise,
    prediction_lead_s, velocity_decay_per_s, max_prediction_time_s.
    """

    def __init__(self, params: dict[str, Any]) -> None:
        self.process_noise_pos = float(params.get("process_noise_pos", 0.0002))
        self.process_noise_vel = float(params.get("process_noise_vel", 0.012))
        self.measurement_noise = float(params.get("measurement_noise", 3.6e-5))
        self.prediction_lead_s = max(0.0, float(params.get("prediction_lead_s", 0.03)))
        self.velocity_decay_per_s = max(0.0, float(params.get("velocity_decay_per_s", 4.0)))
        self.max_prediction_time_s = max(0.0, float(params.get("max_prediction_time_s", 0.12)))

    def create_state(self, joint_name: str, initial_position_rad: float, now_s: float) -> _KalmanState:
        return _KalmanState(
            position=float(initial_position_rad),
            velocity=0.0,
            p11=25.0 * 1e-6,
            p12=0.0,
            p21=0.0,
            p22=400.0 * 1e-6,
            last_time_s=float(now_s),
        )

    def update(
        self,
        state: Any,
        joint_name: str,
        measurement_rad: float,
        now_s: float,
    ) -> None:
        if not isinstance(state, _KalmanState):
            return
        self._predict(state, now_s)
        z = float(measurement_rad)
        r = max(1e-12, self.measurement_noise)
        innovation = z - state.position
        s = state.p11 + r
        k1 = state.p11 / s
        k2 = state.p21 / s
        state.position = state.position + k1 * innovation
        state.velocity = state.velocity + k2 * innovation
        state.p11 = (1.0 - k1) * state.p11
        state.p12 = (1.0 - k1) * state.p12
        state.p21 = state.p21 - k2 * state.p11
        state.p22 = state.p22 - k2 * state.p12
        state.last_time_s = float(now_s)

    def _predict(self, state: _KalmanState, now_s: float) -> None:
        dt = max(MIN_DT_S, float(now_s) - state.last_time_s)
        decay = self.velocity_decay_per_s
        velocity_scale = math.exp(-decay * dt) if decay > 0.0 else 1.0
        velocity = state.velocity * velocity_scale
        state.position = state.position + velocity * dt
        state.velocity = velocity
        p11 = state.p11 + dt * (state.p21 + state.p12) + dt * dt * state.p22
        p12 = state.p12 + dt * state.p22
        p21 = state.p21 + dt * state.p22
        p22 = state.p22
        p11 += max(0.0, self.process_noise_pos) * dt
        p22 += max(0.0, self.process_noise_vel) * dt
        state.p11, state.p12, state.p21, state.p22 = p11, p12, p21, p22
        state.last_time_s = float(now_s)

    def predict(
        self,
        state: Any,
        joint_name: str,
        now_s: float,
        last_measurement_time_s: float | None = None,
    ) -> float:
        if not isinstance(state, _KalmanState):
            return 0.0
        if (
            last_measurement_time_s is not None
            and self.max_prediction_time_s > 0
            and (now_s - last_measurement_time_s) > self.max_prediction_time_s
        ):
            state.velocity = 0.0
        self._predict(state, now_s)
        lead = self.prediction_lead_s
        return state.position + state.velocity * max(0.0, lead)
