"""Arm self-collision soft boundary: clearance of the arm links to the robot body (body_model), no ROS.

Checked parts (CHECKED_PARTS), all from one FK pass per joint sample and converted to base_link with the arm mount:
the upper arm (shoulder_lift axis -> elbow), forearm (elbow -> wrist_flex axis) and wrist link (wrist_flex axis ->
gripper_link) as capsules of half the STS3215 width, the gripper body / fixed jaw hull points and the three jaw points
of the floor guard (jaw tip, tool point, moving jaw tip). The arm base and the shoulder link stand on the crossbar and
only turn in place about the pan axis, so they are not checked at all (in particular not against the crossbar).

Path semantics (BodyReport, over the executed joint trajectory samples): a sample whose smallest clearance is below
body_check.soft_margin_m runs at slow_speed_scale; a sample below hard_margin_m (or penetrating) rejects the whole
motion before it starts. A start that is already inside the hard margin of an obstacle may move out: for that obstacle
the limit becomes the start clearance minus ESCAPE_TOLERANCE_M (the clearance must not get worse).
"""

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from .arm_geometry import (
    FOREARM_CAPSULE,
    GRIPPER_BODY,
    GRIPPER_BODY_POINTS,
    JAW_POINTS,
    LINK_ORIGINS,
    UPPER_ARM_CAPSULE,
    WRIST_CAPSULE,
)
from .body_model import BodyModel, Obstacle, capsule_signed_distance
from .config import ArmBaseOffset, BodyCheckSettings, McpServerConfig
from .floor_guard import GRIPPER_LINK, TOOL_FRAME_LINK, JawModel, yaw_matrix
from .ik import ArmKinematics

CAPSULES = (UPPER_ARM_CAPSULE, FOREARM_CAPSULE, WRIST_CAPSULE)
CHECKED_PARTS = (*(c[0] for c in CAPSULES), GRIPPER_BODY, *JAW_POINTS)
NORMAL_SCALE = 1.0
# A start inside the hard margin may move out as long as no clearance drops more than this below its start value (m).
ESCAPE_TOLERANCE_M = 0.002
# Joint-space step (rad) of a straight joint line sampled for a check (planner free moves, home routes).
JOINT_LINE_STEP_RAD = 0.02
ROUND_M = 4


def joint_line(
    start: dict[str, float], goal: dict[str, float], step: float = JOINT_LINE_STEP_RAD
) -> list[dict[str, float]]:
    """Samples of the straight joint-space line start -> goal (the path every quintic joint move follows).

    Args:
        start (dict[str, float]): Start pose (must name every joint of goal).
        goal (dict[str, float]): Goal pose; joints only in start keep their start value.
        step (float): Largest joint change between samples (rad, > 0).

    Returns:
        list[dict[str, float]]: Samples, start and goal included.
    """
    span = max((abs(goal[j] - start[j]) for j in goal), default=0.0)
    n = max(1, math.ceil(span / step - 1e-9))
    return [start | {j: start[j] + (goal[j] - start[j]) * i / n for j in goal} for i in range(n + 1)]


def combine_scales(floor: list[float], body: list[float]) -> list[float]:
    """Per-sample speed scale of the floor guard and the body check together: the slower wins.

    Args:
        floor (list[float]): Floor guard scales.
        body (list[float]): Body check scales (same length).

    Returns:
        list[float]: Combined scales.
    """
    return [min(a, b) for a, b in zip(floor, body, strict=True)]


def obstacle_hint(obstacle: Obstacle, part: str, settings: BodyCheckSettings) -> str:
    """How to get a rejected motion past an obstacle.

    Args:
        obstacle (Obstacle): The limiting obstacle.
        part (str): The arm part that comes too close.
        settings (BodyCheckSettings): Margins.

    Returns:
        str: The hint.
    """
    where = {
        "lidar": "the lidar dome stands front-left of the arm base",
        "wheel": "the wheels steer, so the whole circle around each steer axis is blocked",
        "frame": "the side frames run along both sides above the wheels",
        "crossbar": "the crossbar is the robot deck under and behind the arm base",
    }
    note = next((text for key, text in where.items() if obstacle.name.startswith(key)), "")
    return (
        f"keep the {part} more than {settings.soft_margin_m:g} m from {obstacle.name} (top at z "
        f"{obstacle.top_z:.3f} m in base_link{'; ' + note if note else ''}): raise the arm first (shoulder_lift back, "
        f"elbow up) so it passes above z {obstacle.top_z + settings.soft_margin_m:.2f} m, then lower outside the "
        f"obstacle, or approach from another side (turn shoulder_pan away from {obstacle.name}). get_arm_reach lists "
        "the collision-free envelope and the pitches that work for a point."
    )


@dataclass(frozen=True)
class PoseClearance:
    """Clearance of one joint configuration to every obstacle: name -> (clearance m, closest part)."""

    per_obstacle: dict[str, tuple[float, str]]

    @property
    def obstacle(self) -> str:
        """Obstacle with the smallest clearance.

        Returns:
            str: Obstacle name ('' without obstacles).
        """
        return min(self.per_obstacle, key=lambda n: self.per_obstacle[n][0], default="")

    @property
    def min_clearance(self) -> float:
        """Smallest clearance (m; inf without obstacles).

        Returns:
            float: Clearance.
        """
        return self.per_obstacle[self.obstacle][0] if self.per_obstacle else math.inf

    @property
    def part(self) -> str:
        """Arm part closest to the limiting obstacle.

        Returns:
            str: Part name ('' without obstacles).
        """
        return self.per_obstacle[self.obstacle][1] if self.per_obstacle else ""


@dataclass
class BodyReport:
    """Per-sample body check of a trajectory: speed scales, smallest clearance, its obstacle and part, the verdict."""

    scales: list[float]
    clearances: list[float]
    obstacles: list[str]
    parts: list[str]
    settings: BodyCheckSettings
    enabled: bool = True
    violation: int | None = None  # first sample breaking the hard limit
    violation_obstacle: str = ""
    violation_part: str = ""
    violation_clearance: float = math.inf
    hints: dict[str, str] = field(default_factory=dict)

    @property
    def rejected(self) -> bool:
        """Whether the motion must not run.

        Returns:
            bool: True when a sample breaks the hard limit.
        """
        return self.violation is not None

    @property
    def min_clearance(self) -> float:
        """Smallest clearance over the samples (m; inf for no samples or a disabled check).

        Returns:
            float: Clearance.
        """
        return min(self.clearances, default=math.inf)

    def summary(self) -> dict[str, Any] | None:
        """Slowdown report for motion results; None when no sample was slowed.

        Returns:
            dict[str, Any] | None: obstacle, link, min_clearance_m, speed_scale, soft_margin_m, slowed_samples, samples
                and every obstacle that slowed a sample.
        """
        slowed = [i for i, s in enumerate(self.scales) if s < NORMAL_SCALE]
        if not slowed:
            return None
        worst = min(slowed, key=lambda i: self.clearances[i])
        return {
            "obstacle": self.obstacles[worst],
            "link": self.parts[worst],
            "min_clearance_m": round(self.clearances[worst], ROUND_M),
            "speed_scale": self.settings.slow_speed_scale,
            "soft_margin_m": self.settings.soft_margin_m,
            "slowed_samples": len(slowed),
            "samples": len(self.scales),
            "obstacles": sorted({self.obstacles[i] for i in slowed}),
        }

    def rejection(self) -> dict[str, Any]:
        """Why the motion was rejected (empty when it was not).

        Returns:
            dict[str, Any]: obstacle, link, clearance_m, hard_margin_m, sample, samples and a hint.
        """
        if self.violation is None:
            return {}
        return {
            "obstacle": self.violation_obstacle,
            "link": self.violation_part,
            "clearance_m": round(self.violation_clearance, ROUND_M),
            "hard_margin_m": self.settings.hard_margin_m,
            "sample": self.violation,
            "samples": len(self.scales),
            "hint": self.hints.get(self.violation_obstacle, ""),
        }

    def message(self) -> str:
        """One-line rejection message for tool errors.

        Returns:
            str: The message ('' when not rejected).
        """
        r = self.rejection()
        if not r:
            return ""
        return (
            f"refused (self-collision): the {r['link']} would come {r['clearance_m']:.3f} m from the robot body "
            f"obstacle {r['obstacle']} (hard margin {r['hard_margin_m']:g} m) at trajectory sample {r['sample']} of "
            f"{r['samples']}. Hint: {r['hint']} Nothing moved."
        )


class SelfCollisionChecker:
    """Body clearance of arm joint configurations (see the module docstring)."""

    def __init__(
        self,
        kin: ArmKinematics,
        body: BodyModel,
        settings: BodyCheckSettings,
        mount: ArmBaseOffset,
        jaw: JawModel,
        gripper_joint: str = "gripper",
    ) -> None:
        """Bind the checker to the arm model and the body.

        Args:
            kin (ArmKinematics): Kinematics on the arm URDF.
            body (BodyModel): Body obstacles (base_link).
            settings (BodyCheckSettings): Margins, slow scale, enabled.
            mount (ArmBaseOffset): Arm base pose in base_link.
            jaw (JawModel): Moving jaw model.
            gripper_joint (str): Gripper joint name in the samples (closed when missing).
        """
        self.kin = kin
        self.body = body
        self.settings = settings
        self.mount = mount
        self.jaw = jaw
        self.gripper = gripper_joint
        self.rotation = yaw_matrix(mount.yaw)
        self.offset = np.array([mount.x, mount.y, mount.z])

    @classmethod
    def from_config(cls, kin: ArmKinematics, config: McpServerConfig, jaw: JawModel) -> "SelfCollisionChecker":
        """Checker from the node config (body, body_check, arm mount; no mount = straight above base_link at the mount
        height, as the floor guard assumes).

        Args:
            kin (ArmKinematics): Kinematics.
            config (McpServerConfig): Node configuration.
            jaw (JawModel): Jaw model.

        Returns:
            SelfCollisionChecker: The checker.
        """
        mount = config.arm.base_in_base_link or ArmBaseOffset(z=config.arm.arm_base_height_m)
        body = BodyModel.from_settings(config.body)
        return cls(kin, body, config.body_check, mount, jaw, config.arm.gripper_joint)

    def to_base_link(self, points: np.ndarray) -> np.ndarray:
        """Arm-frame points (..., 3) -> base_link.

        Args:
            points (np.ndarray): Points in the arm base frame (m).

        Returns:
            np.ndarray: Points in base_link (m).
        """
        return np.asarray(points, dtype=np.float64) @ self.rotation.T + self.offset

    def part_points(self, joints: dict[str, float]) -> dict[str, np.ndarray]:
        """Base_link geometry of every checked part for one joint configuration.

        Args:
            joints (dict[str, float]): Joint name -> rad (measured space; gripper optional).

        Returns:
            dict[str, np.ndarray]: capsule parts -> (2, 3) axis ends; 'gripper body' -> (K, 3) hull points; jaw
                points -> (1, 3).
        """
        frames = self.kin.link_frames(joints)
        origins = {key: frames[link][:3, 3] for key, link in LINK_ORIGINS.items()}
        out = {name: self.to_base_link(np.array([origins[a], origins[b]])) for name, a, b, _ in CAPSULES}
        gripper = frames[GRIPPER_LINK]
        out[GRIPPER_BODY] = self.to_base_link(GRIPPER_BODY_POINTS @ gripper[:3, :3].T + gripper[:3, 3])
        tool = frames[TOOL_FRAME_LINK]
        jaw = {
            "jaw_tip": tool[:3, 3],
            "tool_point": tool[:3, :3] @ self.kin.tool_offset + tool[:3, 3],
            "moving_jaw_tip": self.jaw.moving_tip(gripper, joints.get(self.gripper, self.jaw.closed_rad)),
        }
        for name in JAW_POINTS:
            out[name] = self.to_base_link(jaw[name][None, :])
        return out

    def clearance_matrix(self, samples: list[dict[str, float]]) -> tuple[np.ndarray, np.ndarray]:
        """Clearance of every sample to every obstacle and the closest part.

        Args:
            samples (list[dict[str, float]]): Joint configurations.

        Returns:
            tuple[np.ndarray, np.ndarray]: (N, M) clearances (m) and (N, M) indices into CHECKED_PARTS.
        """
        n, m = len(samples), len(self.body.obstacles)
        if n == 0 or m == 0:
            return np.zeros((n, m)), np.zeros((n, m), dtype=int)
        geometry = [self.part_points(s) for s in samples]
        stacked = {name: np.stack([g[name] for g in geometry]) for name in CHECKED_PARTS}
        out = np.empty((n, m, len(CHECKED_PARTS)))
        for k, obstacle in enumerate(self.body.obstacles):
            for p, (name, *_rest, radius) in enumerate(CAPSULES):
                ends = stacked[name]
                out[:, k, p] = capsule_signed_distance(obstacle, ends[:, 0], ends[:, 1], radius)
            for p in range(len(CAPSULES), len(CHECKED_PARTS)):
                out[:, k, p] = obstacle.signed_distance(stacked[CHECKED_PARTS[p]]).min(axis=1)
        return out.min(axis=2), out.argmin(axis=2)

    def pose(self, joints: dict[str, float]) -> PoseClearance:
        """Clearance of one configuration to every obstacle (computed even when the check is disabled).

        Args:
            joints (dict[str, float]): Joint configuration.

        Returns:
            PoseClearance: Per obstacle (clearance, closest part).
        """
        values, parts = self.clearance_matrix([joints])
        return PoseClearance(
            {o.name: (float(values[0, k]), CHECKED_PARTS[int(parts[0, k])]) for k, o in enumerate(self.body.obstacles)}
        )

    def path(self, samples: list[dict[str, float]]) -> BodyReport:
        """Speed scale of every sample and the hard verdict of a trajectory (samples[0] = the start pose).

        Args:
            samples (list[dict[str, float]]): Joint samples the arm will follow, start first.

        Returns:
            BodyReport: Scales, clearances, limiting obstacle/part per sample and the verdict.
        """
        cfg = self.settings
        n = len(samples)
        if not cfg.enabled or n == 0:
            return BodyReport([NORMAL_SCALE] * n, [math.inf] * n, [""] * n, [""] * n, cfg, enabled=False)
        values, parts = self.clearance_matrix(samples)
        names = [o.name for o in self.body.obstacles]
        worst = values.argmin(axis=1)
        clearances = [float(values[i, worst[i]]) for i in range(n)]
        report = BodyReport(
            scales=[cfg.slow_speed_scale if c < cfg.soft_margin_m else NORMAL_SCALE for c in clearances],
            clearances=clearances,
            obstacles=[names[int(worst[i])] for i in range(n)],
            parts=[CHECKED_PARTS[int(parts[i, worst[i]])] for i in range(n)],
            settings=cfg,
            hints={o.name: obstacle_hint(o, "arm", cfg) for o in self.body.obstacles},
        )
        start = values[0]
        limit = np.where(start >= cfg.hard_margin_m, cfg.hard_margin_m, start - ESCAPE_TOLERANCE_M)
        broken = values < limit[None, :]
        if broken.any():
            i = int(np.argmax(broken.any(axis=1)))
            k = int(np.argmin(np.where(broken[i], values[i], np.inf)))
            part = CHECKED_PARTS[int(parts[i, k])]
            report.violation = i
            report.violation_obstacle = names[k]
            report.violation_part = part
            report.violation_clearance = float(values[i, k])
            report.hints[names[k]] = obstacle_hint(self.body.obstacles[k], part, cfg)
        return report
