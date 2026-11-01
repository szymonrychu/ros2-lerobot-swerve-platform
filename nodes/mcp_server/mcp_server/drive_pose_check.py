"""FK check of the drive pose (the arm pose held while navigating) and of every leg the arm takes to and from it.

The drive controller (drive_pose.py) lifts with ArmController.home_motion(drive pose), so the lift follows the first
home_routes route from home whose every leg clears the body at the hard margin (ArmController.plan_home), and the
lower follows the same choice from the drive pose back home. Both chosen routes are sampled leg by leg (joint_line)
and every sample is held to:

- body: clearance of every arm part (self_collision part points and capsules) to the body (robot_body.yaml, with the
  head post and the head envelope about the pan axis) at least margin_m; where the parked home is already closer to an obstacle, its clearance minus ESCAPE_TOLERANCE_M;
- floor: the tool point and both wrist capsule ends at least min_tool_height_m above the floor;
- stretched: never elbow_flex < 0 with shoulder_lift above stretched_shoulder_lift_max_rad;
- footprint: the arm's ground shadow inside the static footprint outline (grown by simplify_m, the test the
  footprint tick uses to keep the static footprint published);

and the pose itself to:

- roll: wrist_roll and gripper equal to home (no roll guard is needed on the way);
- view (floor visibility): with the head tilted VIEW_TILT_DEG down at every pan from 0 to VIEW_PAN_MIN_DEG (right) in
  VIEW_PAN_STEP_DEG steps, no floor point of the region ahead of and beside the right wheels (floor_region: the
  head_aim footprint box, lookahead_m past its front and right edges) that the D435 depth camera has in its frustum
  is hidden by an arm link: the ray from the depth optical frame to the point passes no occluder (occluders: the
  upper arm, forearm and wrist capsules and the wrist end -> tool point, LINK_CAPSULE_RADIUS_M each). The fixed
  shoulder root, the upper arm end cap about the shoulder_lift axis that only shoulder_pan moves, is not counted.
  No pose leaves the region fully visible (the upper arm next to the shoulder axis hides floor at pans -50..-90 for
  every shoulder_lift), so the rule is a regression guard: the hidden area summed over the guarded pans
  (VIEW_GUARD_PAN_MAX_DEG..VIEW_PAN_MIN_DEG) is at most VIEW_MAX_OCCLUDED_M2 and at most VIEW_MAX_HOME_FRACTION of
  the parked home's. Home hides 1.60 m2 over pans 0..-90 and 1.12 m2 over -50..-90; the pinned drive pose 0.165 m2
  over -50..-90 (nothing at 0..-40). A shoulder_pan swing would reach 0.088 m2 (not searched: D4 keeps the pan).

Margins and heights are the shared arm_pose_check defaults (PoseCheckConfig: margin_m 0.03, min_tool_height_m 0.10,
stretched_shoulder_lift_max_rad 1.85). The D435 depth field of view is the datasheet
87 x 58 deg of the 640x360 depth profile (nodes/bridges/realsense_d435/config/realsense_d435.yaml).

`python -m mcp_server.drive_pose_check search <client.yml> <home json>` grids shoulder_lift, elbow_flex and
wrist_flex (shoulder_pan, wrist_roll and gripper as home) and prints every candidate that passes, one JSON line each,
least occluded floor first (floor_occlusion_m2: every visible floor cell within SCORE_RANGE_M, summed over the
checked pans), then lowest static gravity torque on shoulder_lift.
"""

import itertools
import json
import logging
import math
import sys
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml
from arm_collision.arm_geometry import LINK_CAPSULE_RADIUS_M, UPPER_ARM_CAPSULE, WRIST_CAPSULE
from arm_collision.ik import ArmKinematics
from arm_collision.self_collision import ESCAPE_TOLERANCE_M, SelfCollisionChecker, joint_line
from arm_collision.settings import footprint_body
from ros2_common.head_rgbd_calibration.arm_pose_check import (
    PoseCheckConfig,
)

from .arm import home_routes
from .config import REPO_ROOT, McpServerConfig
from .floor_guard import JawModel
from .footprint import arm_shadow
from .footprint_geometry import body_outline, static_outline
from .head_kinematics import DEPTH_FRAME, HeadKinematics
from .sag import GravityModel

LOGGER = logging.getLogger(__name__)
CHECKS = ("body", "floor", "stretched", "roll", "footprint", "view")
SHOULDER_LIFT = "shoulder_lift"
ELBOW_FLEX = "elbow_flex"
WRIST_FLEX = "wrist_flex"
UNCHANGED_JOINTS = ("wrist_roll", "gripper")  # equal to home: no roll on the way, the jaw as parked
SEARCH_JOINTS = (SHOULDER_LIFT, ELBOW_FLEX, WRIST_FLEX)
SEARCH_STEP_RAD = 0.1
SAME_RAD = 1e-9
TOOL_POINT = "tool_point"
D435_DEPTH_HFOV_DEG = 87.0
D435_DEPTH_VFOV_DEG = 58.0
VIEW_TILT_DEG = 30.0
VIEW_PAN_MIN_DEG = -90.0
VIEW_PAN_STEP_DEG = 10.0
VIEW_GUARD_PAN_MAX_DEG = -50.0  # the guarded pans: -50..-90 deg
VIEW_MAX_OCCLUDED_M2 = 0.18
VIEW_MAX_HOME_FRACTION = 0.15
HEAD_AIM_CONFIG = REPO_ROOT / "nodes" / "head_aim_cpp" / "config" / "head_aim.yaml"
REGION_CELL_M = 0.02  # floor region grid
SCORE_CELL_M = 0.04  # ranking grid
SCORE_RANGE_M = 1.5  # ranking grid half size about base_link
PARALLEL_EPS = 1e-12


@dataclass(frozen=True)
class DrivePoseViolation:
    """One failed check.

    Attributes:
        check (str): One of CHECKS.
        detail (str): What failed, where (route, leg sample, pan).
    """

    check: str
    detail: str


@dataclass(frozen=True)
class Candidate:
    """A drive pose candidate that passed every check.

    Attributes:
        joints (dict[str, float]): The pose (measured space, every arm joint).
        occluded_floor_m2 (float): Visible floor hidden by the arm, summed over the checked pans (m^2).
        shoulder_lift_torque_nm (float): Static gravity torque about shoulder_lift (N m).
    """

    joints: dict[str, float]
    occluded_floor_m2: float
    shoulder_lift_torque_nm: float


def in_frustum(points: np.ndarray, half_h: float, half_v: float, radius: float) -> np.ndarray:
    """Whether each point (optical frame: z forward, x right, y down) lies within radius of the camera's pyramid.

    Args:
        points (np.ndarray): (N, 3) points in the optical frame (m).
        half_h (float): Horizontal half field of view (rad).
        half_v (float): Vertical half field of view (rad).
        radius (float): Growth of the pyramid (m), the capsule radius.

    Returns:
        np.ndarray: (N,) booleans.
    """
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    planes = (
        np.abs(x) * math.cos(half_h) - z * math.sin(half_h),
        np.abs(y) * math.cos(half_v) - z * math.sin(half_v),
    )
    return (planes[0] < radius) & (planes[1] < radius) & (z > -radius)


def floor_region(min_x: float, max_x: float, min_y: float, max_y: float, lookahead: float, cell: float) -> np.ndarray:
    """Floor cell centres ahead of and beside the right wheels: up to lookahead past the front and right edges of the
    footprint box, from its rear edge to the centre line.

    Args:
        min_x (float): Rear edge of the footprint box (m, base_link).
        max_x (float): Front edge (m).
        min_y (float): Right edge (m).
        max_y (float): Left edge (m); the region never reaches left of the centre line.
        lookahead (float): Depth of the region past the edges (m).
        cell (float): Grid cell (m).

    Returns:
        np.ndarray: (N, 3) floor points (z = 0).
    """
    xs = np.arange(min_x + cell / 2.0, max_x + lookahead, cell)
    ys = np.arange(min_y - lookahead + cell / 2.0, min(0.0, max_y), cell)
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    keep = (gx >= max_x) | (gy <= min_y)
    return np.column_stack([gx[keep], gy[keep], np.zeros(int(keep.sum()))])


def rays_blocked(
    origin: np.ndarray, targets: np.ndarray, starts: np.ndarray, ends: np.ndarray, radii: np.ndarray
) -> np.ndarray:
    """Whether the segment origin -> target passes within radius of any capsule axis start -> end.

    Args:
        origin (np.ndarray): (3,) ray origin (m).
        targets (np.ndarray): (N, 3) ray ends (m).
        starts (np.ndarray): (M, 3) capsule axis starts (m).
        ends (np.ndarray): (M, 3) capsule axis ends (m, distinct from the starts).
        radii (np.ndarray): (M,) capsule radii (m).

    Returns:
        np.ndarray: (N,) booleans.
    """
    d1 = targets - origin  # (N, 3)
    d2 = ends - starts  # (M, 3)
    r = origin - starts  # (M, 3)
    a = np.einsum("ij,ij->i", d1, d1)[:, None]
    e = np.einsum("ij,ij->i", d2, d2)[None, :]
    f = np.einsum("ij,ij->i", d2, r)[None, :]
    b = d1 @ d2.T
    c = d1 @ r.T
    denom = a * e - b * b
    s = np.where(denom > PARALLEL_EPS, np.clip((b * f - c * e) / np.maximum(denom, PARALLEL_EPS), 0.0, 1.0), 0.0)
    t = (b * s + f) / e
    s = np.where(t < 0.0, np.clip(-c / a, 0.0, 1.0), np.where(t > 1.0, np.clip((b - c) / a, 0.0, 1.0), s))
    t = np.clip(t, 0.0, 1.0)
    gap = origin + d1[:, None, :] * s[..., None] - (starts[None, :, :] + d2[None, :, :] * t[..., None])
    return (np.linalg.norm(gap, axis=-1) < radii[None, :]).any(axis=1)


def arm_kinematics(config: McpServerConfig) -> ArmKinematics:
    """Arm kinematics as the node builds them (offsets, tool point, limit overrides).

    Args:
        config (McpServerConfig): Node configuration.

    Returns:
        ArmKinematics: The kinematics.
    """
    return ArmKinematics(
        config.arm.urdf_path,
        margin=config.limits.arm_limit_margin_rad,
        joint_offsets=config.arm.joint_offsets_rad.model_dump(),
        tool_offset=tuple(config.arm.tool_offset_m.model_dump().values()),
        limit_overrides=config.arm.joint_limit_overrides_rad,
    )


def deployed_config(client_vars: Path) -> McpServerConfig:
    """The mcp_server config of client.yml with this checkout's arm and head URDFs.

    Args:
        client_vars (Path): ansible/group_vars/client.yml.

    Returns:
        McpServerConfig: The validated config.
    """
    nodes = yaml.safe_load(client_vars.read_text())["ros2_nodes"]
    raw = next(n for n in nodes if n["name"] == "mcp_server")["config"]
    data = dict(yaml.safe_load(raw) if isinstance(raw, str) else raw)
    config = McpServerConfig.model_validate(data)
    defaults = McpServerConfig()
    config.arm.urdf_path = defaults.arm.urdf_path
    config.head.urdf_path = defaults.head.urdf_path
    return config


class DrivePoseChecker:
    """The drive pose checks (module docstring) on one configuration."""

    def __init__(self, config: McpServerConfig) -> None:
        """Build the arm model, the body checker, the static outline (floor-level body) and the head kinematics.

        Args:
            config (McpServerConfig): Node configuration (arm, body, body_check, head, nav_footprint).

        Raises:
            ValueError: Without nav_footprint (no static outline to hold the shadow to).
        """
        footprint = config.nav_footprint
        if footprint is None:
            raise ValueError("drive pose check needs nav_footprint (the static footprint outline)")
        self.limits = PoseCheckConfig()
        self.kin = arm_kinematics(config)
        jaw = JawModel(self.kin, config.arm.jaw_open_axis, config.arm.gripper_closed_rad)
        # One body (with the head post and head envelope) for the routes plan_home takes and the drive pose clearance.
        self.clearance = SelfCollisionChecker.from_settings(self.kin, config.body, config.body_check, config.arm, jaw)
        self.obstacles = [o.name for o in self.clearance.body.obstacles]
        floor = footprint_body(config.body)
        outline = body_outline(floor.boxes, floor.cylinders, footprint.circle_segments, footprint.close_gap_m)
        self.static = static_outline(outline, footprint.parked_arm_shadow).buffer(footprint.simplify_m)
        self.segments = footprint.circle_segments
        self.head = HeadKinematics(Path(config.head.urdf_path))
        self.gravity = GravityModel(Path(config.arm.urdf_path))
        self.offsets = config.arm.joint_offsets_rad.model_dump()
        aim = yaml.safe_load(HEAD_AIM_CONFIG.read_text())
        box = (aim["footprint_min_x"], aim["footprint_max_x"], aim["footprint_min_y"], aim["footprint_max_y"])
        self.region = floor_region(*box, aim["lookahead_m"], REGION_CELL_M)
        span = np.arange(-SCORE_RANGE_M + SCORE_CELL_M / 2.0, SCORE_RANGE_M, SCORE_CELL_M)
        gx, gy = np.meshgrid(span, span, indexing="ij")
        outside = ~((gx > box[0]) & (gx < box[1]) & (gy > box[2]) & (gy < box[3]))
        self.score_floor = np.column_stack([gx[outside], gy[outside], np.zeros(int(outside.sum()))])
        pans = np.arange(0.0, VIEW_PAN_MIN_DEG - VIEW_PAN_STEP_DEG / 2.0, -VIEW_PAN_STEP_DEG)
        self.views = [
            (float(pan), self.head.link_pose(math.radians(pan), math.radians(VIEW_TILT_DEG), DEPTH_FRAME))
            for pan in pans
        ]

    def route(self, start: dict[str, float], goal: dict[str, float]) -> tuple[str, list[dict[str, float]]] | None:
        """The route ArmController.plan_home takes from start to goal: the first whose legs clear the body.

        Args:
            start (dict[str, float]): Start pose (every arm joint).
            goal (dict[str, float]): Goal pose (every arm joint).

        Returns:
            tuple[str, list[dict[str, float]]] | None: (description, legs), None when no route clears the body.
        """
        routes = home_routes(self.kin, start, goal)
        if not self.clearance.settings.enabled:
            return routes[0]
        for note, legs in routes:
            prev = start
            for leg in legs:
                if self.clearance.path(joint_line(prev, leg)).rejected:
                    break
                prev = leg
            else:
                return note, legs
        return None

    def route_violations(
        self, home: dict[str, float], start: dict[str, float], goal: dict[str, float], name: str
    ) -> list[DrivePoseViolation]:
        """Body, floor, stretched and footprint checks of every sample of the route start -> goal.

        Args:
            home (dict[str, float]): Parked home (its clearances bound the body check where it is closer).
            start (dict[str, float]): Route start.
            goal (dict[str, float]): Route goal.
            name (str): "lift" or "lower", for the details.

        Returns:
            list[DrivePoseViolation]: The first failing sample of each check.
        """
        chosen = self.route(start, goal)
        if chosen is None:
            return [DrivePoseViolation("body", f"{name}: no home_routes route clears the body")]
        note, legs = chosen
        samples: list[dict[str, float]] = []
        prev = start
        for leg in legs:
            samples += joint_line(prev, leg)[(1 if samples else 0) :]
            prev = leg
        where = f"{name} via {note!r}"
        out: dict[str, DrivePoseViolation] = {}

        def note_once(check: str, detail: str) -> None:
            out.setdefault(check, DrivePoseViolation(check, f"{where}: {detail}"))

        home_clear = self.clearances([home])[0]
        limit = np.minimum(self.limits.margin_m, home_clear - ESCAPE_TOLERANCE_M)
        stacked = self.clearance.stacked_points(samples)
        clear = self.clearance.clearances_of(stacked)[0]
        for i, sample in enumerate(samples):
            bad = np.flatnonzero(clear[i] < limit)
            if bad.size:
                k = int(bad[0])
                note_once("body", f"sample {i} {self.obstacles[k]} clearance {clear[i, k]:.3f} m < {limit[k]:.3f} m")
            low = min(float(stacked[TOOL_POINT][i, 0, 2]), float(stacked[WRIST_CAPSULE[0]][i, :2, 2].min()))
            if low < self.limits.min_tool_height_m:
                note_once("floor", f"sample {i} tool or wrist {low:.3f} m < {self.limits.min_tool_height_m} m")
            if sample[SHOULDER_LIFT] > self.limits.stretched_shoulder_lift_max_rad and sample[ELBOW_FLEX] < 0.0:
                note_once("stretched", f"sample {i} shoulder_lift {sample[SHOULDER_LIFT]:.3f} with elbow_flex < 0")
            parts = {key: points[i] for key, points in stacked.items()}
            if not self.static.contains(arm_shadow(parts, self.segments)):
                note_once("footprint", f"sample {i} arm shadow outside the static footprint")
        return [out[c] for c in CHECKS if c in out]

    def clearances(self, samples: list[dict[str, float]]) -> np.ndarray:
        """Clearance of every sample to every obstacle of the body with the head envelope.

        Args:
            samples (list[dict[str, float]]): Joint configurations.

        Returns:
            np.ndarray: (N, obstacles) clearances (m).
        """
        return self.clearance.clearances_of(self.clearance.stacked_points(samples))[0]

    def occluders(self, pose: Mapping[str, float]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Arm link capsules that can hide floor: the upper arm without its fixed root cap, forearm, wrist link and the
        wrist end -> tool point.

        Args:
            pose (Mapping[str, float]): Arm pose.

        Returns:
            tuple[np.ndarray, np.ndarray, np.ndarray]: (M, 3) starts, (M, 3) ends, (M,) radii.
        """
        parts = self.clearance.part_points(dict(pose))
        shoulder, elbow = parts[UPPER_ARM_CAPSULE[0]][0], parts[UPPER_ARM_CAPSULE[0]][1]
        root = shoulder + (elbow - shoulder) / np.linalg.norm(elbow - shoulder) * LINK_CAPSULE_RADIUS_M
        wrist = parts[WRIST_CAPSULE[0]]
        starts = np.array([root, parts["forearm"][0], wrist[0], wrist[1]])
        ends = np.array([elbow, parts["forearm"][1], wrist[1], parts[TOOL_POINT][0]])
        return starts, ends, np.full(len(starts), LINK_CAPSULE_RADIUS_M)

    def occluded_per_pan(self, pose: Mapping[str, float], floor: np.ndarray, cell: float) -> list[tuple[float, float]]:
        """Floor area in the depth frustum that the arm hides, per checked pan.

        Args:
            pose (Mapping[str, float]): Arm pose.
            floor (np.ndarray): (N, 3) floor cell centres.
            cell (float): Cell size (m).

        Returns:
            list[tuple[float, float]]: (pan deg, hidden area m^2).
        """
        occluders = self.occluders(pose)
        half_h, half_v = math.radians(D435_DEPTH_HFOV_DEG / 2.0), math.radians(D435_DEPTH_VFOV_DEG / 2.0)
        out = []
        for pan, camera in self.views:
            origin = camera[:3, 3]
            visible = floor[in_frustum((floor - origin) @ camera[:3, :3], half_h, half_v, 0.0)]
            hidden = int(rays_blocked(origin, visible, *occluders).sum()) if len(visible) else 0
            out.append((pan, hidden * cell * cell))
        return out

    def region_occlusion_m2(self, pose: Mapping[str, float]) -> float:
        """Hidden floor of the region ahead of and beside the right wheels, summed over the checked pans.

        Args:
            pose (Mapping[str, float]): Arm pose.

        Returns:
            float: Area (m^2); 0 passes the view check.
        """
        return sum(area for _, area in self.occluded_per_pan(pose, self.region, REGION_CELL_M))

    def floor_occlusion_m2(self, pose: Mapping[str, float]) -> float:
        """Hidden visible floor within SCORE_RANGE_M, summed over the checked pans (the search ranking).

        Args:
            pose (Mapping[str, float]): Arm pose.

        Returns:
            float: Area (m^2).
        """
        return sum(area for _, area in self.occluded_per_pan(pose, self.score_floor, SCORE_CELL_M))

    def guarded_occlusion_m2(self, pose: Mapping[str, float]) -> float:
        """Hidden floor of the region summed over the guarded pans (VIEW_GUARD_PAN_MAX_DEG..VIEW_PAN_MIN_DEG).

        Args:
            pose (Mapping[str, float]): Arm pose.

        Returns:
            float: Area (m^2).
        """
        hidden = self.occluded_per_pan(pose, self.region, REGION_CELL_M)
        return sum(area for pan, area in hidden if pan <= VIEW_GUARD_PAN_MAX_DEG)

    def view_violation(self, home: Mapping[str, float], pose: Mapping[str, float]) -> DrivePoseViolation | None:
        """Whether the arm hides too much floor of the region ahead of and beside the right wheels (module docstring).

        Args:
            home (Mapping[str, float]): Parked home (the reference area).
            pose (Mapping[str, float]): Drive pose.

        Returns:
            DrivePoseViolation | None: The guarded area against both limits, None when within them.
        """
        area = self.guarded_occlusion_m2(pose)
        limit = min(VIEW_MAX_OCCLUDED_M2, VIEW_MAX_HOME_FRACTION * self.guarded_occlusion_m2(home))
        if area <= limit:
            return None
        return DrivePoseViolation(
            "view",
            f"arm hides {area:.3f} m2 of floor beside the right wheels over pans {VIEW_GUARD_PAN_MAX_DEG:.0f}.."
            f"{VIEW_PAN_MIN_DEG:.0f} deg at tilt {VIEW_TILT_DEG:.0f} deg > {limit:.3f} m2 (at most "
            f"{VIEW_MAX_OCCLUDED_M2} m2 and {VIEW_MAX_HOME_FRACTION:.0%} of home)",
        )

    def pose_violations(self, home: Mapping[str, float], pose: Mapping[str, float]) -> list[DrivePoseViolation]:
        """The checks of the pose alone: roll and view.

        Args:
            home (Mapping[str, float]): Parked home.
            pose (Mapping[str, float]): Drive pose.

        Returns:
            list[DrivePoseViolation]: Failed checks.
        """
        out = []
        changed = [j for j in UNCHANGED_JOINTS if abs(pose[j] - home[j]) > SAME_RAD]
        if changed:
            out.append(DrivePoseViolation("roll", f"{', '.join(changed)} differ from home"))
        view = self.view_violation(home, pose)
        if view is not None:
            out.append(view)
        return out

    def check(self, home: Mapping[str, float], pose: Mapping[str, float]) -> list[DrivePoseViolation]:
        """Every check of the drive pose from home (module docstring).

        Args:
            home (Mapping[str, float]): Parked home (measured space, every arm joint).
            pose (Mapping[str, float]): Drive pose (measured space, every arm joint).

        Returns:
            list[DrivePoseViolation]: Empty when the pose passes.
        """
        home, pose = dict(home), dict(pose)
        out = self.route_violations(home, home, pose, "lift") + self.route_violations(home, pose, home, "lower")
        out += self.pose_violations(home, pose)
        return sorted(out, key=lambda v: CHECKS.index(v.check))

    def shoulder_lift_torque(self, pose: Mapping[str, float]) -> float:
        """Static gravity torque about shoulder_lift at a measured pose.

        Args:
            pose (Mapping[str, float]): Measured joints.

        Returns:
            float: Torque (N m).
        """
        return self.gravity.torques({j: v + self.offsets.get(j, 0.0) for j, v in pose.items()})[SHOULDER_LIFT]


def check_drive_pose(
    config: McpServerConfig, home: Mapping[str, float], pose: Mapping[str, float]
) -> list[DrivePoseViolation]:
    """Every drive pose check (module docstring) of pose from home.

    Args:
        config (McpServerConfig): Node configuration.
        home (Mapping[str, float]): Parked home (measured space, every arm joint).
        pose (Mapping[str, float]): Drive pose (measured space, every arm joint).

    Returns:
        list[DrivePoseViolation]: Empty = pass.
    """
    return DrivePoseChecker(config).check(home, pose)


def search_grid(kin: ArmKinematics, step: float = SEARCH_STEP_RAD) -> dict[str, list[float]]:
    """Measured-space values of the searched joints, every step inside their limits.

    Args:
        kin (ArmKinematics): Kinematics (URDF limits, margin and offsets).
        step (float): Grid step (rad).

    Returns:
        dict[str, list[float]]: Joint -> values (rad, rounded to 3 decimals).
    """
    grid = {}
    for joint in SEARCH_JOINTS:
        lo, hi = kin.limits[joint]
        offset = kin.offsets.get(joint, 0.0)
        values = np.arange(lo - offset, hi - offset + 1e-9, step)
        grid[joint] = [round(float(v), 3) for v in values]
    return grid


def search(
    checker: DrivePoseChecker,
    home: Mapping[str, float],
    grid: Mapping[str, list[float]],
    progress: Callable[[int, int], None] | None = None,
) -> list[Candidate]:
    """Every grid pose (other joints as home) that passes, least occluded floor first, then lowest |shoulder_lift
    gravity torque|.

    Args:
        checker (DrivePoseChecker): The checks.
        home (Mapping[str, float]): Parked home.
        grid (Mapping[str, list[float]]): Joint -> values of SEARCH_JOINTS.
        progress (Callable[[int, int], None] | None): Called with (done, total) after each candidate.

    Returns:
        list[Candidate]: Passing candidates, ranked.
    """
    combos = list(itertools.product(*(grid[j] for j in SEARCH_JOINTS)))
    found = []
    for done, values in enumerate(combos, start=1):
        pose = dict(home) | dict(zip(SEARCH_JOINTS, values, strict=True))
        # The pose checks are cheap and fail most candidates: the route checks only run for the rest.
        if not checker.pose_violations(home, pose) and not checker.check(home, pose):
            found.append(Candidate(pose, checker.floor_occlusion_m2(pose), checker.shoulder_lift_torque(pose)))
        if progress is not None:
            progress(done, len(combos))
    return sorted(found, key=lambda c: (c.occluded_floor_m2, abs(c.shoulder_lift_torque_nm)))


def main(argv: list[str]) -> int:
    """`search <client.yml> <home json>`: print the passing candidates, one JSON object per line.

    Args:
        argv (list[str]): Command line after the module name.

    Returns:
        int: Exit code (0 with candidates, 1 without, 2 on bad usage).
    """
    if len(argv) != 3 or argv[0] != "search":
        print("usage: python -m mcp_server.drive_pose_check search <client.yml> <home json>", file=sys.stderr)
        return 2
    config = deployed_config(Path(argv[1]))
    home = json.loads(argv[2])
    checker = DrivePoseChecker(config)
    found = search(checker, home, search_grid(checker.kin))
    for rank, cand in enumerate(found, start=1):
        print(
            json.dumps(
                {
                    "rank": rank,
                    "occluded_floor_m2": round(cand.occluded_floor_m2, 4),
                    "shoulder_lift_torque_nm": round(cand.shoulder_lift_torque_nm, 4),
                    "joints": cand.joints,
                }
            )
        )
    return 0 if found else 1


if __name__ == "__main__":
    logging.basicConfig(level=logging.WARNING)
    sys.exit(main(sys.argv[1:]))
