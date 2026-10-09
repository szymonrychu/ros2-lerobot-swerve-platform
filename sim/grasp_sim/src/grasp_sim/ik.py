"""Small damped-least-squares IK on the MuJoCo model: tool point position plus approach pitch."""

import mujoco
import numpy as np

from grasp_sim.config import SceneConfig
from grasp_sim.scene import build_model

SOLVED_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")
GRIPPER_BODY = "gripper"
APPROACH_LOCAL = np.array([0.0, 0.0, -1.0])
MAX_ITERATIONS = 200
DAMPING = 1e-3
STEP_LIMIT_RAD = 0.2
FD_EPS = 1e-6
POSITION_TOLERANCE_M = 5e-4
PITCH_TOLERANCE_RAD = 5e-3
PITCH_WEIGHT = 0.1
DEFAULT_SEED = {"shoulder_pan": 0.0, "shoulder_lift": 0.3, "elbow_flex": 0.8, "wrist_flex": 0.5}


class IKError(RuntimeError):
    """Raised when no solution within tolerance is found for the target."""


class ArmIK:
    """IK for the first four SO-101 joints with the wrist roll fixed by the caller (URDF angles)."""

    def __init__(self, scene: SceneConfig) -> None:
        """Compile a model of the scene and cache joint addresses and limits.

        Args:
            scene (SceneConfig): Scene whose joint limit overrides apply.
        """
        self.model = build_model(scene)
        self.data = mujoco.MjData(self.model)
        self.adr = {j: int(self.model.joint(j).qposadr[0]) for j in (*SOLVED_JOINTS, "wrist_roll")}
        self.limits = np.array([self.model.joint(j).range for j in SOLVED_JOINTS])

    def forward(self, q: dict[str, float], local_point: tuple[float, float, float]) -> tuple[np.ndarray, float]:
        """Forward kinematics of a point in the gripper frame.

        Args:
            q (dict[str, float]): URDF joint angles including wrist_roll (rad).
            local_point (tuple[float, float, float]): Point in the gripper_link frame (m).

        Returns:
            tuple[np.ndarray, float]: World point (m) and approach pitch (rad, positive = tool pointing down).
        """
        for joint, adr in self.adr.items():
            self.data.qpos[adr] = q[joint]
        mujoco.mj_kinematics(self.model, self.data)
        body = self.data.body(GRIPPER_BODY)
        rot = body.xmat.reshape(3, 3)
        approach = rot @ APPROACH_LOCAL
        return body.xpos + rot @ np.array(local_point), float(np.arcsin(-approach[2]))

    def residual(
        self, theta: np.ndarray, roll: float, local_point: tuple[float, float, float]
    ) -> tuple[np.ndarray, float]:
        """Forward map of the solved joints to (point, pitch)."""
        q = dict(zip(SOLVED_JOINTS, theta, strict=True))
        q["wrist_roll"] = roll
        return self.forward(q, local_point)

    def solve(
        self,
        target: np.ndarray,
        local_point: tuple[float, float, float],
        pitch_down_rad: float,
        roll: float,
        seed: dict[str, float] | None = None,
    ) -> dict[str, float]:
        """Solve pan, lift, elbow and wrist_flex for a tool point and approach pitch.

        Args:
            target (np.ndarray): Target world point (arm frame, m) for local_point.
            local_point (tuple[float, float, float]): Tool point in the gripper_link frame (m).
            pitch_down_rad (float): Desired approach pitch (rad, positive = tool pointing down).
            roll (float): Fixed wrist_roll in URDF rad (it moves off-axis tool points).
            seed (dict[str, float] | None): Starting joint angles (keeps paths continuous).

        Returns:
            dict[str, float]: URDF joint angles for the four solved joints plus wrist_roll.

        Raises:
            IKError: If the target is not reached within tolerance.
        """
        start = {**DEFAULT_SEED, **(seed or {})}
        theta = np.array([start[j] for j in SOLVED_JOINTS], dtype=float)
        for _ in range(MAX_ITERATIONS):
            pos, pitch = self.residual(theta, roll, local_point)
            err = np.append(target - pos, PITCH_WEIGHT * (pitch_down_rad - pitch))
            if np.linalg.norm(err[:3]) < POSITION_TOLERANCE_M and abs(pitch_down_rad - pitch) < PITCH_TOLERANCE_RAD:
                out = dict(zip(SOLVED_JOINTS, theta.tolist(), strict=True))
                out["wrist_roll"] = roll
                return out
            jac = np.zeros((4, 4))
            for i in range(4):
                bumped = theta.copy()
                bumped[i] += FD_EPS
                p2, pitch2 = self.residual(bumped, roll, local_point)
                jac[:3, i] = (p2 - pos) / FD_EPS
                jac[3, i] = PITCH_WEIGHT * (pitch2 - pitch) / FD_EPS
            step = jac.T @ np.linalg.solve(jac @ jac.T + DAMPING * np.eye(4), err)
            norm = np.linalg.norm(step)
            if norm > STEP_LIMIT_RAD:
                step *= STEP_LIMIT_RAD / norm
            theta = np.clip(theta + step, self.limits[:, 0], self.limits[:, 1])
        raise IKError(f"no IK solution for target {np.round(target, 4).tolist()} pitch {pitch_down_rad:.2f}")
