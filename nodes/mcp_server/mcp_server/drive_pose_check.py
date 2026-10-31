"""FK check of the drive pose (the arm pose held while navigating) and of every leg the arm takes to and from it.

The drive controller (drive_pose.py) lifts with ArmController.home_motion(drive pose), so the lift follows the first
home_routes route from home whose every leg clears the body at the hard margin (ArmController.plan_home), and the
lower follows the same choice from the drive pose back home. Both chosen routes are sampled leg by leg (joint_line)
and every sample is held to:

- body: clearance of every arm part (self_collision part points and capsules) to the body and the head envelope
  (ros2_common arm_pose_check.head_envelope: a cylinder about the pan axis containing the head at every pan/tilt) at
  least margin_m; where the parked home is already closer to an obstacle, its clearance minus ESCAPE_TOLERANCE_M;
- floor: the tool point and both wrist capsule ends at least min_tool_height_m above the floor;
- stretched: never elbow_flex < 0 with shoulder_lift above stretched_shoulder_lift_max_rad;
- footprint: the arm's ground shadow inside the static footprint outline (grown by simplify_m, the test the
  footprint tick uses to keep the static footprint published);

and the pose itself to:

- roll: wrist_roll and gripper equal to home (no roll guard is needed on the way);
- view: no arm part (link capsules with their radius, gripper body and jaw points) inside the D435 depth frustum with
  the head tilted VIEW_TILT_DEG down at every pan from 0 to VIEW_PAN_MIN_DEG (right) in VIEW_PAN_STEP_DEG steps.

Margins and heights are the shared arm_pose_check defaults (PoseCheckConfig: margin_m 0.03, min_tool_height_m 0.10,
stretched_shoulder_lift_max_rad 1.85, head envelope 0.075 / 0.09 m). The D435 depth field of view is the datasheet
87 x 58 deg of the 640x360 depth profile (nodes/bridges/realsense_d435/config/realsense_d435.yaml).

`python -m mcp_server.drive_pose_check search <client.yml> <home json>` grids shoulder_lift, elbow_flex and
wrist_flex (shoulder_pan, wrist_roll and gripper as home) and prints every candidate that passes, one JSON line each,
lowest static gravity torque on shoulder_lift first.
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
from ros2_common.head_rgbd_calibration.arm_pose_check import (
    BodyConfig,
    BoxObstacle,
    CylinderObstacle,
    PoseCheckConfig,
    head_envelope,
)

from .arm import home_routes
from .arm_geometry import WRIST_CAPSULE
from .config import BodyCylinder, BodySettings, McpServerConfig
from .floor_guard import JawModel
from .footprint import arm_shadow
from .footprint_geometry import body_outline, static_outline
from .head_kinematics import DEPTH_FRAME, HeadKinematics
from .ik import ArmKinematics
from .sag import GravityModel
from .self_collision import CAPSULES, ESCAPE_TOLERANCE_M, SelfCollisionChecker, joint_line

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
CAPSULE_RADII = {name: radius for name, _start, _end, radius in CAPSULES}
CAPSULE_SAMPLE_M = 0.005  # spacing of the points checked along a capsule axis in the view check


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
        shoulder_lift_torque_nm (float): Static gravity torque about shoulder_lift (N m).
    """

    joints: dict[str, float]
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
        """Build the arm model, the body with the head envelope, the static outline and the head kinematics.

        Args:
            config (McpServerConfig): Node configuration (arm, body, body_check, head, nav_footprint).

        Raises:
            ValueError: Without nav_footprint (no static outline to hold the shadow to).
        """
        footprint = config.nav_footprint
        if footprint is None:
            raise ValueError("drive pose check needs nav_footprint (the static footprint outline)")
        self.limits = PoseCheckConfig(
            body=BodyConfig(
                boxes=[BoxObstacle(name=b.name, center=b.center, size=b.size) for b in config.body.boxes],
                cylinders=[CylinderObstacle(**c.model_dump()) for c in config.body.cylinders],
            )
        )
        self.kin = arm_kinematics(config)
        jaw = JawModel(self.kin, config.arm.jaw_open_axis, config.arm.gripper_closed_rad)
        # The router sees the body plan_home sees; the clearance checker adds the head envelope.
        self.router = SelfCollisionChecker.from_config(self.kin, config, jaw)
        head = head_envelope(Path(config.head.urdf_path).read_text(), self.limits)
        with_head = BodySettings(
            boxes=config.body.boxes, cylinders=[*config.body.cylinders, BodyCylinder(**head.model_dump())]
        )
        self.clearance = SelfCollisionChecker.from_config(self.kin, config.model_copy(update={"body": with_head}), jaw)
        self.obstacles = [o.name for o in self.clearance.body.obstacles]
        outline = body_outline(
            config.body.boxes, config.body.cylinders, footprint.circle_segments, footprint.close_gap_m
        )
        self.static = static_outline(outline, footprint.parked_arm_shadow).buffer(footprint.simplify_m)
        self.segments = footprint.circle_segments
        self.head = HeadKinematics(Path(config.head.urdf_path))
        self.gravity = GravityModel(Path(config.arm.urdf_path))
        self.offsets = config.arm.joint_offsets_rad.model_dump()
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
        if not self.router.settings.enabled:
            return routes[0]
        for note, legs in routes:
            prev = start
            for leg in legs:
                if self.router.path(joint_line(prev, leg)).rejected:
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

    def view_violation(self, pose: dict[str, float]) -> DrivePoseViolation | None:
        """Whether an arm part is inside the D435 depth frustum at any checked head pan.

        Args:
            pose (dict[str, float]): Drive pose.

        Returns:
            DrivePoseViolation | None: The pans that see the arm, None when none does.
        """
        parts = self.clearance.part_points(pose)
        spheres = []
        for name, radius in CAPSULE_RADII.items():
            a, b = parts[name][0], parts[name][1]
            n = max(2, math.ceil(float(np.linalg.norm(b - a)) / CAPSULE_SAMPLE_M) + 1)
            spheres.append((np.linspace(a, b, n), radius))
        other = [points for key, points in parts.items() if key not in CAPSULE_RADII]
        spheres.append((np.vstack(other), 0.0))
        half_h, half_v = math.radians(D435_DEPTH_HFOV_DEG / 2.0), math.radians(D435_DEPTH_VFOV_DEG / 2.0)
        seen = []
        for pan, camera in self.views:
            for points, radius in spheres:
                optical = (points - camera[:3, 3]) @ camera[:3, :3]
                if in_frustum(optical, half_h, half_v, radius).any():
                    seen.append(pan)
                    break
        if not seen:
            return None
        pans = ", ".join(f"{p:.0f}" for p in seen)
        return DrivePoseViolation("view", f"arm in the D435 frustum at tilt {VIEW_TILT_DEG:.0f} deg, pan {pans} deg")

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
        changed = [j for j in UNCHANGED_JOINTS if abs(pose[j] - home[j]) > SAME_RAD]
        if changed:
            out.append(DrivePoseViolation("roll", f"{', '.join(changed)} differ from home"))
        view = self.view_violation(pose)
        if view is not None:
            out.append(view)
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
    """Every grid pose (other joints as home) that passes, lowest |shoulder_lift gravity torque| first.

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
        if not checker.check(home, pose):
            found.append(Candidate(pose, checker.shoulder_lift_torque(pose)))
        if progress is not None:
            progress(done, len(combos))
    return sorted(found, key=lambda c: abs(c.shoulder_lift_torque_nm))


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
                {"rank": rank, "shoulder_lift_torque_nm": round(cand.shoulder_lift_torque_nm, 4), "joints": cand.joints}
            )
        )
    return 0 if found else 1


if __name__ == "__main__":
    logging.basicConfig(level=logging.WARNING)
    sys.exit(main(sys.argv[1:]))
