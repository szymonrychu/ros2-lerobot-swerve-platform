"""Kernel inputs of the mcp_server bench: the deployed client.yml config, recorded grasp cases and synthetic data.

Every ``*_call`` function builds its inputs once (outside the timing) and returns a zero-argument callable that runs
the compute path of one kernel, so bench_kernels times exactly the kernel and not its set-up. No ROS is imported.
Recorded inputs: ansible/group_vars/client.yml (the deployed config), tests/data/grasp_golden.json (grasp scenarios)
and tests/fakes.py is not needed because the kernels are pure. Synthetic inputs use fixed seeds.
"""

import json
import math
import random
from collections.abc import Callable
from functools import cache
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml
from ros2_common.camera_geometry import pixel_to_ground

from mcp_server import arm_geometry, body_model, camera_overlay, camera_scene, footprint_geometry, frontiers, perception
from mcp_server.arm_geometry import JAW_POINTS
from mcp_server.arm_reach import ReachEnvelope, joint_range, point_reach
from mcp_server.camera_candidates import CandidateStore, RobotSnapshot
from mcp_server.config import REPO_ROOT, McpServerConfig, ReachMapSettings
from mcp_server.floor_guard import FloorGuard, JawModel, SurfaceModel, base_link_to_arm
from mcp_server.grasp import GraspPlanner, ObjectSpec, grasp_params, kernel_params
from mcp_server.grasp_check import GripMeasurements, band_verdict, fuse, grip_evidence
from mcp_server.ik import ArmKinematics
from mcp_server.jaw_crop import jaw_crop
from mcp_server.reach_map import build as build_reach_map
from mcp_server.reach_map import checker_from_config
from mcp_server.reference.ik_reach import robot_inputs
from mcp_server.sag import GravityModel, SagCompensator, compensate
from mcp_server.self_collision import SelfCollisionChecker, joint_line
from tests.camera_helpers import FRONT_MOUNT, HFOV

Call = Callable[[], Any]

REPO_PLACEHOLDER = "{{ ros2_repo_dest }}"
CLIENT_GROUP_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
GRASP_GOLDEN = Path(__file__).resolve().parents[1] / "tests" / "data" / "grasp_golden.json"
SEED = {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow_flex": 1.2, "wrist_flex": 0.3, "wrist_roll": 0.0}
# A gripper-camera pose with the arm raised and the jaws half open (joints in measured space, rad).
GRIPPER_VIEW = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -0.6,
    "elbow_flex": 1.2,
    "wrist_flex": 0.5,
    "wrist_roll": 0.0,
    "gripper": 0.4,
}
RNG_SEED = 20261013
SCAN_BEAMS = 1440
MAP_SIDE_CELLS = 800
MAP_RESOLUTION_M = 0.05
# Golden cases of the frontiers kernels (mcp_core_cpp parity): small maps, so the golden JSON stays small.
FRONTIER_CASE_RES_M = 0.025
FRONTIER_CASE_COARSE_RES_M = 0.05
FRONTIER_CASE_PASS_M = 0.13  # 0.2 m footprint / 2 + the deployed 0.03 m margin
FRONTIER_CASE_FOOTPRINT_M = 0.2
IMAGE_W, IMAGE_H = 640, 480
LIDAR_POINTS = 720
PATH_SAMPLES = 40
BODY_POINTS = 4000
# Home pose of the 2026-10-10 calibration (tests/test_self_collision.py HOME).
COLLISION_HOME = {
    "shoulder_pan": -0.0138,
    "shoulder_lift": -1.3622,
    "elbow_flex": 1.5769,
    "wrist_flex": -0.0752,
    "wrist_roll": -1.6245,
}
# Gripper opening (rad) of the collision scenario poses that do not set one.
COLLISION_GRIPPER = 0.3
# Joint step (rad) of the collision golden's joint lines (coarser than the planner's: keeps the golden small).
COLLISION_LINE_STEP_RAD = 0.05
SAG_JOINTS_POSE = {"shoulder_lift": -0.8, "elbow_flex": 1.4, "wrist_flex": 0.2}
SAG_GAINS = {"shoulder_lift": 0.1, "elbow_flex": 0.1, "wrist_flex": 0.1}
SAG_MAX_RAD = 0.12
# Coarse reach map (the tests' COARSE grid: the deployed grid takes minutes, the kernel scales with the sample count).
REACH_MAP = ReachMapSettings(
    heading_step_deg=45.0,
    r_step_m=0.04,
    r_max_m=0.48,
    z_step_m=0.05,
    z_max_m=0.45,
    planar_samples=(24, 24, 14),
    samples_per_cell=1,
)
# ik_reach golden cases (tests/regenerate_core_golden.py): random tool points in a box around the arm, pitched ones,
# and a second, denser reach map grid.
REACH_POINTS = 48
REACH_PITCHED_POINTS = 16
REACH_POINT_BOX = ((-0.05, 0.45), (-0.35, 0.25), (0.0, 0.45))
REACH_PITCH_RANGE = (-0.5, 1.5)
REACH_MAP_DENSE = REACH_MAP.model_copy(
    update={"heading_step_deg": 30.0, "r_step_m": 0.03, "planar_samples": (20, 20, 12), "samples_per_cell": 2}
)


@cache
def config() -> McpServerConfig:
    """The deployed mcp_server config (ansible/group_vars/client.yml) with the URDF path made absolute.

    Returns:
        McpServerConfig: Validated config.
    """
    group_vars = yaml.safe_load(CLIENT_GROUP_VARS.read_text())
    node = next(n for n in group_vars["ros2_nodes"] if n["name"] == "mcp_server")
    data = yaml.safe_load(node["config"]) if isinstance(node["config"], str) else node["config"]
    data["arm"]["urdf_path"] = str(REPO_ROOT / data["arm"]["urdf_path"])
    gripper = data["cameras"]["gripper"]["intrinsics"]
    gripper["calibration_file"] = gripper["calibration_file"].replace(REPO_PLACEHOLDER, str(REPO_ROOT))
    # The front camera is not calibrated on the robot: use the tests' synthetic hfov camera and mount.
    data["cameras"]["front"] = {"parent_frame": "base_link", "intrinsics": HFOV, "mount": FRONT_MOUNT}
    return McpServerConfig.model_validate(data)


@cache
def kinematics() -> ArmKinematics:
    """Arm kinematics of the deployed config.

    Returns:
        ArmKinematics: Kinematics with the deployed offsets, tool offset and limits.
    """
    cfg = config()
    return ArmKinematics(
        cfg.arm.urdf_path,
        margin=cfg.limits.arm_limit_margin_rad,
        joint_offsets=cfg.arm.joint_offsets_rad.model_dump(),
        tool_offset=tuple(cfg.arm.tool_offset_m.model_dump().values()),
        limit_overrides=cfg.arm.joint_limit_overrides_rad,
    )


@cache
def jaw() -> JawModel:
    """Jaw model of the deployed config.

    Returns:
        JawModel: Jaw geometry.
    """
    return JawModel(kinematics(), config().arm.jaw_open_axis, config().arm.gripper_closed_rad)


@cache
def checker() -> SelfCollisionChecker:
    """Self collision checker of the deployed config.

    Returns:
        SelfCollisionChecker: Body checker.
    """
    return SelfCollisionChecker.from_config(kinematics(), config(), jaw())


@cache
def planner() -> GraspPlanner:
    """Grasp planner of the deployed config.

    Returns:
        GraspPlanner: Planner.
    """
    cfg = config()
    assert cfg.arm.base_in_base_link is not None
    return GraspPlanner(
        kinematics(), cfg, FloorGuard(kinematics(), cfg.floor_guard, cfg.arm.base_in_base_link, jaw()), jaw()
    )


@cache
def scene(camera: str) -> camera_scene.Scene:
    """Camera scene of the deployed config (front at base_link, gripper at GRIPPER_VIEW).

    Args:
        camera (str): 'front' or 'gripper'.

    Returns:
        camera_scene.Scene: The scene.
    """
    cfg = config()
    setup = camera_scene.camera_setup(cfg, camera)
    parent = camera_scene.parent_transform(cfg, camera, kinematics(), GRIPPER_VIEW)
    return camera_scene.build_scene(setup, cfg, parent)


def grasp_cases() -> list[dict[str, Any]]:
    """Recorded grasp scenarios of tests/data/grasp_golden.json.

    Returns:
        list[dict[str, Any]]: Cases (x, y, support_z, surface_z, size, gap, strategy, pitch).
    """
    return json.loads(GRASP_GOLDEN.read_text())


def grasp_plan_call(strategy: str | None) -> Call:
    """Cold grasp planning of the first recorded case (the IK cache is cleared per call).

    Args:
        strategy (str | None): Strategy to plan with; None uses the case's recorded strategy.

    Returns:
        Call: Plans the case once.
    """
    cfg, kin, plan = config(), kinematics(), planner()
    mount = cfg.arm.base_in_base_link
    assert mount is not None
    params = grasp_params(cfg.grasp, None)
    cases = grasp_cases()[:1]

    def call() -> None:
        kin.solutions.clear()
        for case in cases:
            obj = ObjectSpec(
                frame="arm",
                x=case["x"],
                y=case["y"],
                support_z=case["support_z"],
                depth_m=case["size"][0],
                width_m=case["size"][1],
                height_m=case["size"][2],
                gap_below_m=case["gap"],
            )
            surface = SurfaceModel(surface_z_m=case["surface_z"] + mount.z, tilt=None, tilt_source="none", mount=mount)
            plan.plan(obj, strategy or case["strategy"], params, surface, dict(SEED), case["pitch"])

    return call


class SolveLineRecorder(GraspPlanner):
    """GraspPlanner that records the keyword arguments of each waypoint IK call (solve_point / line) it makes."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        """Build the planner with an empty record.

        Args:
            *args (Any): GraspPlanner arguments.
            **kwargs (Any): GraspPlanner keyword arguments.
        """
        super().__init__(*args, **kwargs)
        self.cases: list[dict[str, Any]] = []

    def solve_waypoints(
        self,
        label: str,
        start: Any,
        end: Any,
        pitch: float,
        seed: dict[str, float],
        extra: Any,
        params: Any,
        budget: Any,
    ) -> tuple[list[dict[str, float]], str | None]:
        """Record the solve_line case, then solve (see GraspPlanner.solve_waypoints)."""
        self.cases.append(
            {
                "geometry": self.geometry,
                "label": label,
                "start": None if start is None else [float(v) for v in start],
                "end": [float(v) for v in end],
                "pitch": float(pitch),
                "seed": {j: float(seed[j]) for j in self.kin.joint_names},
                "extra": None if extra is None else [float(v) for v in extra],
                "heading_bias": float(self.kin.last_heading_bias),
                "params": kernel_params(params),
                "budget_limit": None if budget is None else budget.limit,
                "budget_used": 0 if budget is None else self.kin.solve_count - budget.start,
            }
        )
        return super().solve_waypoints(label, start, end, pitch, seed, extra, params, budget)


def grasp_solve_line_cases() -> list[dict[str, Any]]:
    """solve_line cases: every waypoint IK call of the recorded grasp scenarios, plus the failure paths.

    The recorded scenarios (tests/data/grasp_golden.json) are planned cold with their strategy; each solve_point /
    line call is one case. The edge cases reuse a recorded line: a target beyond reach, a target out of the joint
    limits, a spent budget, a too small max_joint_jump_rad, a stall threshold every pose violates, and a kinematics
    without tool offset.

    Returns:
        list[dict[str, Any]]: Keyword arguments of mcp_core_cpp.grasp.solve_line (and its reference).
    """
    cfg, kin = config(), kinematics()
    mount = cfg.arm.base_in_base_link
    assert mount is not None
    recorder = SolveLineRecorder(kin, cfg, FloorGuard(kin, cfg.floor_guard, mount, jaw()), jaw())
    params = grasp_params(cfg.grasp, None)
    for case in grasp_cases():
        kin.solutions.clear()
        obj = ObjectSpec(
            frame="arm",
            x=case["x"],
            y=case["y"],
            support_z=case["support_z"],
            depth_m=case["size"][0],
            width_m=case["size"][1],
            height_m=case["size"][2],
            gap_below_m=case["gap"],
        )
        surface = SurfaceModel(surface_z_m=case["surface_z"] + mount.z, tilt=None, tilt_source="none", mount=mount)
        recorder.plan(obj, case["strategy"], params, surface, dict(SEED), case["pitch"])
    cases = recorder.cases
    line = next(c for c in cases if c["start"] is not None)
    no_tool = line["geometry"] | {"tool_offset": [0.0, 0.0, 0.0]}
    edge = [
        line | {"label": "far", "start": None, "end": [1.0, 0.0, 0.1]},
        line | {"label": "behind", "start": None, "end": [-0.1, 0.0, -0.15]},
        line | {"label": "spent", "budget_limit": 10, "budget_used": 10},
        line | {"label": "jumpy", "params": line["params"] | {"max_joint_jump_rad": 1e-4}},
        line
        | {
            "label": "stall",
            "params": line["params"] | {"stall_shoulder_lift_rad": -3.0, "stretched_elbow_max_rad": 3.0},
        },
        line | {"label": "no_tool_offset", "geometry": no_tool, "extra": None},
    ]
    return cases + edge


def band_verdict_call() -> Call:
    """band_verdict, grip_evidence and fuse of a typical object grip.

    Returns:
        Call: One evaluation.
    """
    m = GripMeasurements(
        grasp_id="bench",
        baseline_load=10.0,
        baseline_source="rest",
        contact="load_delta",
        peak_load_delta=120.0,
        hold_load_delta=80.0,
        rest_current_a=0.1,
        hold_current_a=0.5,
        current_delta_a=0.4,
        jaw_gap_rad=0.2,
        in_empty_band=False,
    )

    def call() -> None:
        band_verdict(m, 50.0, 0.2, 90.0, 40.0)
        grip_evidence(m, 50.0, 0.2, 90.0, 40.0)
        fuse("object", "yes")

    return call


def jaw_crop_call() -> Call:
    """jaw_crop of the gripper camera at GRIPPER_VIEW.

    Returns:
        Call: One crop box computation.
    """
    cfg = config()
    setup = camera_scene.camera_setup(cfg, "gripper")

    def call() -> Any:
        return jaw_crop(setup, kinematics(), jaw(), GRIPPER_VIEW, IMAGE_W, IMAGE_H, cfg.grasp_check)

    return call


def camera_overlay_pixels() -> list[tuple[float, float]]:
    """Candidate pixels of a mark_candidate_points call over the whole image.

    Returns:
        list[tuple[float, float]]: Pixels.
    """
    return camera_overlay.candidate_pixels(IMAGE_W, IMAGE_H, None, 20, 400)


def build_scene_call() -> Call:
    """build_scene of the front camera (setup is resolved each time, as the tools do).

    Returns:
        Call: One scene build.
    """
    cfg = config()

    def call() -> Any:
        setup = camera_scene.camera_setup(cfg, "front")
        return camera_scene.build_scene(setup, cfg, camera_scene.parent_transform(cfg, "front", kinematics(), None))

    return call


def gripper_scene_call() -> Call:
    """build_scene of the gripper camera: forward kinematics of the parent link plus the scene.

    Returns:
        Call: One scene build.
    """
    cfg = config()

    def call() -> Any:
        setup = camera_scene.camera_setup(cfg, "gripper")
        return camera_scene.build_scene(
            setup, cfg, camera_scene.parent_transform(cfg, "gripper", kinematics(), GRIPPER_VIEW)
        )

    return call


def candidate_store_call() -> Call:
    """mark_candidate_points compute: the pixel grid, a floor point per pixel and the store insert.

    Returns:
        Call: One candidate set.
    """
    sc = scene("front")
    store = CandidateStore()
    snapshot = RobotSnapshot(pose=(0.0, 0.0, 0.0), joints=dict(SEED))

    def call() -> None:
        points = []
        for n, (u, v) in enumerate(camera_overlay_pixels(), start=1):
            ground = sc.ground_point(u, v)
            if ground is not None:
                points.append({"n": n, "u": u, "v": v, "x": float(ground[0]), "y": float(ground[1])})
        store.add("front", snapshot, points)

    return call


def pixel_to_ground_call() -> Call:
    """pixel_to_ground of a 400 pixel grid with the front camera.

    Returns:
        Call: 400 ray casts.
    """
    sc = scene("front")
    pixels = camera_overlay_pixels()

    def call() -> None:
        for u, v in pixels:
            pixel_to_ground(sc.intr, sc.t_ref_optical, u, v, sc.ground_z)

    return call


def grid_call(draw: bool) -> Call:
    """The floor grid overlay of get_annotated_camera_image (polylines and labels, optionally drawn).

    Args:
        draw (bool): Also draw the grid onto an image.

    Returns:
        Call: One overlay.
    """
    sc = scene("front")
    img = np.full((IMAGE_H, IMAGE_W, 3), 90, np.uint8)

    def call() -> None:
        if draw:
            camera_overlay.draw_grid(img, sc, 0.5)
        else:
            camera_overlay.grid_polylines(sc, 0.5)
            camera_overlay.grid_labels(sc, 0.5)

    return call


def lidar_points() -> np.ndarray:
    """Synthetic lidar returns in base_link.

    Returns:
        np.ndarray: Nx4 rows (x, y, z, range).
    """
    rng = np.random.default_rng(RNG_SEED)
    angles = rng.uniform(-0.8, 0.8, LIDAR_POINTS)
    r = rng.uniform(0.5, 4.0, LIDAR_POINTS)
    return np.column_stack([r * np.cos(angles), r * np.sin(angles), np.full(LIDAR_POINTS, 0.1), r])


def lidar_overlay_call() -> Call:
    """draw_lidar of a full scan onto an image.

    Returns:
        Call: One draw.
    """
    sc = scene("front")
    pts = lidar_points()
    img = np.full((IMAGE_H, IMAGE_W, 3), 90, np.uint8)

    def call() -> None:
        camera_overlay.draw_lidar(img, sc, pts)

    return call


def project_points_call() -> Call:
    """project_points of 720 lidar points.

    Returns:
        Call: One projection.
    """
    sc = scene("front")
    pts = lidar_points()

    def call() -> None:
        camera_overlay.project_points(sc.intr, sc.t_ref_optical, pts[:, :3])

    return call


def circle_runs_call() -> Call:
    """circle_runs of a 1 m reach circle.

    Returns:
        Call: One circle.
    """
    sc = scene("front")

    def call() -> None:
        camera_overlay.circle_runs(sc, (0.5, 0.0), 1.0)

    return call


def scan_ranges() -> list[float]:
    """Synthetic LaserScan ranges with a few invalid returns.

    Returns:
        list[float]: SCAN_BEAMS ranges.
    """
    rng = random.Random(RNG_SEED)
    ranges = [rng.uniform(0.3, 8.0) for _ in range(SCAN_BEAMS)]
    for i in range(0, SCAN_BEAMS, 37):
        ranges[i] = math.inf
    return ranges


def scan_call(summarize: bool) -> Call:
    """scan_points or summarize_scan of a 1440 beam scan.

    Args:
        summarize (bool): summarize_scan (sector obstacles) instead of scan_points.

    Returns:
        Call: One scan conversion.
    """
    ranges = scan_ranges()
    args = (ranges, -math.pi, 2 * math.pi / SCAN_BEAMS, 0.15, 10.0, 0.1, 0.0, 0.0)
    fn = perception.summarize_scan if summarize else perception.scan_points

    def call() -> None:
        fn(*args)

    return call


def occupancy_cells(side: int = MAP_SIDE_CELLS) -> list[int]:
    """A synthetic square occupancy grid (room walls, free interior, unknown outside) as a flat list.

    Args:
        side (int): Cells per side; the room layout scales with it (800 is the bench map).

    Returns:
        list[int]: Row-major cells.
    """

    def at(cell: int) -> int:
        return cell * side // MAP_SIDE_CELLS

    wall = max(1, at(4))
    grid = np.full((side, side), -1, np.int16)
    grid[at(100) : at(700), at(100) : at(700)] = 0
    grid[at(100) : at(100) + wall, at(100) : at(700)] = grid[at(700) - wall : at(700), at(100) : at(700)] = 100
    grid[at(100) : at(700), at(100) : at(100) + wall] = grid[at(100) : at(700), at(700) - wall : at(700)] = 100
    grid[at(300) : at(300) + wall, at(100) : at(500)] = 100
    return grid.ravel().tolist()


def map_call(render: bool) -> Call:
    """map_stats or render_map_crop of the synthetic map as the node receives it (a list of ints).

    Args:
        render: render_map_crop instead of map_stats.

    Returns:
        Call: One call.
    """
    cells = occupancy_cells()

    def call() -> None:
        if render:
            perception.render_map_crop(
                cells, MAP_SIDE_CELLS, MAP_SIDE_CELLS, MAP_RESOLUTION_M, -20.0, -20.0, 0.0, 0.0, 0.5, 4.0, 400
            )
        else:
            perception.map_stats(cells, MAP_SIDE_CELLS, MAP_SIDE_CELLS, MAP_RESOLUTION_M)

    return call


# Golden cases of the ported camera kernels (mcp_server/reference/camera.py). The maps use the bench room layout at
# GOLDEN_MAP_SIDE cells per side (the 800 x 800 bench map would make a golden of several MB) plus edge cases.
GOLDEN_MAP_SIDE = 200
GOLDEN_SMALL_MAP_SIDE = 100
# Lidar points of the distorted-camera goldens (a third of the scan keeps the golden small).
DISTORTED_POINTS = 240
GOLDEN_MAP_ORIGIN_M = -5.0
# Plumb-bob and rational (8 coefficient) distortion for the lidar goldens, on top of the deployed (distortion-free)
# front camera.
PLUMB_BOB = [-0.28, 0.09, 0.0012, -0.0009, -0.012]
RATIONAL = [-0.28, 0.09, 0.0012, -0.0009, -0.012, 0.03, -0.01, 0.002]


def map_edge_cells() -> list[int]:
    """A small fixed-seed grid with every occupancy value class, the thresholds and their neighbours.

    Returns:
        list[int]: 50 x 40 row-major cells in -1..100.
    """
    rng = np.random.default_rng(RNG_SEED)
    cells = rng.integers(-1, 101, 50 * 40)
    cells[:8] = [-1, 0, 25, 26, 64, 65, 100, -1]
    return cells.tolist()


def map_stats_cases() -> list[dict[str, Any]]:
    """map_stats inputs: the bench room (list of ints, as the bench passes it), threshold edges, int8 / int64 arrays
    (rclpy and numpy grids), an empty map and a resolution that rounds.

    Returns:
        list[dict[str, Any]]: Keyword arguments of map_stats.
    """
    room = occupancy_cells(GOLDEN_MAP_SIDE)
    edge = map_edge_cells()
    return [
        {"data": room, "width": GOLDEN_MAP_SIDE, "height": GOLDEN_MAP_SIDE, "resolution": MAP_RESOLUTION_M},
        {"data": edge, "width": 50, "height": 40, "resolution": 0.05},
        {"data": np.array(edge, np.int8), "width": 50, "height": 40, "resolution": 0.0333},
        {"data": np.array(room, np.int64), "width": GOLDEN_MAP_SIDE, "height": GOLDEN_MAP_SIDE, "resolution": 0.025},
        {"data": [], "width": 0, "height": 0, "resolution": 0.05},
    ]


def render_map_crop_cases() -> list[dict[str, Any]]:
    """render_map_crop inputs: the bench call on the room map (downscaled PNG), a crop over the map corner, a robot
    off the map, an upscale-free small crop, and an int8 array grid.

    Returns:
        list[dict[str, Any]]: Keyword arguments of render_map_crop.
    """
    edge = map_edge_cells()
    origin = GOLDEN_MAP_ORIGIN_M
    side, small = GOLDEN_MAP_SIDE, GOLDEN_SMALL_MAP_SIDE
    corner = {"origin_x": origin, "origin_y": origin}
    room = {"data": occupancy_cells(side), "width": side, "height": side, "resolution": MAP_RESOLUTION_M, **corner}
    grid = {"data": occupancy_cells(small), "width": small, "height": small, "resolution": 0.1, **corner}
    return [
        {**room, "robot_x": 0.0, "robot_y": 0.0, "robot_yaw": 0.5, "radius_m": 4.0, "max_px": 120},
        {**grid, "robot_x": -4.6, "robot_y": 4.3, "robot_yaw": -2.0, "radius_m": 1.5, "max_px": 96},
        {**grid, "robot_x": 9.0, "robot_y": -9.0, "robot_yaw": 3.0, "radius_m": 1.0, "max_px": 64},
        {**grid, "robot_x": 0.73, "robot_y": -1.21, "robot_yaw": 1.2, "radius_m": 0.6, "max_px": 160},
        {
            "data": np.array(edge, np.int8),
            "width": 50,
            "height": 40,
            "resolution": 0.05,
            "origin_x": 0.0,
            "origin_y": 0.0,
            "robot_x": 1.27,
            "robot_y": 0.61,
            "robot_yaw": -0.7,
            "radius_m": 0.9,
            "max_px": 48,
        },
    ]


def draw_lidar_cases() -> list[dict[str, Any]]:
    """draw_lidar inputs: the bench scan in the deployed front and gripper scenes, the front camera with plumb-bob and
    rational distortion, a scan with invalid rows, an unknown arm offset and an empty scan.

    Returns:
        list[dict[str, Any]]: Keyword arguments of reference.camera.lidar_case.
    """
    pts = lidar_points()
    odd = pts[:40].copy()
    odd[0] = [-2.0, 0.0, 0.1, 2.0]  # behind the camera
    odd[1, :3] = np.nan  # invalid return
    odd[2, 3] = math.inf  # colour clamps to the far end
    odd[3, 3] = -1.0  # colour clamps to the near end
    odd[4] = [0.3, 0.0, -2.0, 0.3]  # far below the image
    cases = []
    for camera, points, distortion in (
        ("front", pts, None),
        ("gripper", pts, None),
        ("front", pts[:DISTORTED_POINTS], PLUMB_BOB),
        ("front", pts[:DISTORTED_POINTS], RATIONAL),
        ("front", odd, None),
    ):
        sc = scene(camera)
        intrinsics = sc.intr.model_dump()
        if distortion is not None:
            intrinsics["distortion"] = distortion
        cases.append(
            {
                "image_shape": [IMAGE_H, IMAGE_W, 3],
                "fill": 90,
                "points": points,
                "t_base_ref": sc.t_base_ref,
                "t_ref_optical": sc.t_ref_optical,
                "intrinsics": intrinsics,
            }
        )
    cases.append({**cases[0], "t_base_ref": None})
    cases.append({**cases[0], "points": np.zeros((0, 4))})
    return cases


def image() -> np.ndarray:
    """A noisy 640 x 480 BGR test image (JPEG sizes are realistic only with texture).

    Returns:
        np.ndarray: BGR image.
    """
    rng = np.random.default_rng(RNG_SEED)
    base = np.linspace(0, 255, IMAGE_W, dtype=np.uint8)[None, :, None] * np.ones((IMAGE_H, 1, 3), np.uint8)
    return cv2.add(base, rng.integers(0, 40, (IMAGE_H, IMAGE_W, 3), dtype=np.uint8))


def image_call(kind: str) -> Call:
    """The image kernels of get_camera_image: encode_jpeg, image_to_bgr or recompress_jpeg.

    Args:
        kind: 'encode', 'decode' or 'recompress'.

    Returns:
        Call: One conversion of a 640 x 480 frame.
    """
    img = image()
    ok, buf = cv2.imencode(".jpg", img)
    assert ok
    jpeg = buf.tobytes()
    raw = img.tobytes()

    def call() -> None:
        if kind == "encode":
            perception.encode_jpeg(img, 480)
        elif kind == "decode":
            perception.image_to_bgr("bgr8", IMAGE_H, IMAGE_W, IMAGE_W * 3, raw)
        else:
            perception.recompress_jpeg(jpeg, 320)

    return call


def ik_call(kind: str) -> Call:
    """ArmKinematics kernels on a reachable base-frame target.

    Args:
        kind: 'inverse' (tool point, cold cache), 'flange' (flange point, cold cache), 'forward' or 'frames'.

    Returns:
        Call: One solve.
    """
    kin = kinematics()
    flange = kind == "flange"
    target = kin.forward(SEED, tuple(-kin.tool_offset) if flange else None)

    def call() -> None:
        if kind == "forward":
            kin.forward(SEED)
        elif kind == "frames":
            kin.link_frames(SEED)
        else:
            kin.solutions.clear()
            if kind == "inverse":
                kin.inverse(target.x, target.y, target.z, target.pitch, SEED)
            else:
                kin.inverse_flange(target.x, target.y, target.z, target.pitch, SEED)

    return call


def point_reach_call() -> Call:
    """point_reach of a reachable base_link tool point (the envelope is not needed for a reachable point).

    Returns:
        Call: One reach query.
    """
    chk = checker()
    pose = kinematics().forward(SEED)
    mount = config().arm.base_in_base_link
    assert mount is not None
    point = {"x": pose.x + mount.x, "y": pose.y + mount.y, "z": pose.z + mount.z, "pitch": None}
    envelope = ReachEnvelope(chk, config().body_check)

    def call() -> None:
        point_reach(chk, point, dict(SEED), envelope)

    return call


def reach_map_call() -> Call:
    """build of the coarse reach map (checker included, as the CLI builds it).

    Returns:
        Call: One map build.
    """
    chk = checker_from_config(config())

    def call() -> None:
        build_reach_map(chk, REACH_MAP)

    return call


def box_points_call() -> Call:
    """box_surface_points of every link collision box.

    Returns:
        Call: One sampling of all boxes.
    """
    boxes = arm_geometry.LINK_COLLISION_BOXES

    def call() -> None:
        for box in boxes:
            arm_geometry.box_surface_points(box)

    return call


def sag_call(kind: str) -> Call:
    """Sag kernels at one pose.

    Args:
        kind: 'deflection' (SagCompensator.deflection) or 'compensate' (deflection plus the clamped command).

    Returns:
        Call: One evaluation.
    """
    cfg = config()
    model = GravityModel(cfg.arm.urdf_path)
    comp = SagCompensator(model, SAG_GAINS, SAG_MAX_RAD)
    limits = {j: (-2.0, 2.0) for j in SAG_JOINTS_POSE}

    def call() -> None:
        deflection = comp.deflection(SAG_JOINTS_POSE)
        if kind == "compensate":
            compensate(SAG_JOINTS_POSE, deflection, limits, 0.05)

    return call


def arm_path_samples() -> list[dict[str, float]]:
    """A straight joint-space path from the seed pose to a raised pose.

    Returns:
        list[dict[str, float]]: PATH_SAMPLES joint samples, start first.
    """
    goal = {"shoulder_pan": 0.3, "shoulder_lift": -1.0, "elbow_flex": 1.5, "wrist_flex": 0.0, "wrist_roll": 0.0}
    return [
        {j: SEED[j] + (goal[j] - SEED[j]) * k / (PATH_SAMPLES - 1) for j in SEED} | {"gripper": 0.3}
        for k in range(PATH_SAMPLES)
    ]


def collision_call(kind: str) -> Call:
    """SelfCollisionChecker kernels on the planned-path sized sample list.

    Args:
        kind: 'path' (speed scales and the hard verdict of a trajectory), 'pose' (one pose) or 'geometry'.

    Returns:
        Call: One check.
    """
    chk = checker()
    samples = arm_path_samples()

    def call() -> None:
        if kind == "path":
            chk.path(samples)
        elif kind == "pose":
            chk.pose(samples[0])
        else:
            chk.geometry(samples)

    return call


def collision_target(x: float, y: float, z: float, pitch: float) -> dict[str, float]:
    """Joints placing the tool point at a base_link point with an approach pitch (deployed kinematics and mount).

    Args:
        x (float): base_link x (m).
        y (float): base_link y (m).
        z (float): base_link z (m).
        pitch (float): Approach pitch (rad).

    Returns:
        dict[str, float]: Joint configuration (measured space).
    """
    mount = config().arm.base_in_base_link
    assert mount is not None
    return kinematics().inverse(*base_link_to_arm((x, y, z), mount), pitch, seed=SEED)


def collision_poses() -> list[dict[str, float]]:
    """Scenario poses of the body check on the deployed config: clear, raised, home, into / deep in / beside the lidar
    dome and inside its hard margin (the shapes of tests/test_self_collision.py), gripper COLLISION_GRIPPER unless set.

    Returns:
        list[dict[str, float]]: Joint configurations.
    """
    lidar = checker().body.by_name("lidar")
    lx, ly = lidar.center_xy
    down = math.pi / 2
    poses = [
        collision_target(0.33, -0.10, 0.12, 0.8),
        collision_target(0.30, -0.05, 0.25, 0.6),
        COLLISION_HOME,
        collision_target(lx, ly, lidar.z_max - 0.04, down),
        collision_target(lx, ly, lidar.z_max - 0.08, down),
        collision_target(lx + 0.12, ly, lidar.z_max + 0.008, 0.8),
        collision_target(lx + 0.12, ly, lidar.z_max - 0.01, 0.8) | {"gripper": 0.6},
        collision_target(lx + 0.06, ly - 0.02, lidar.z_max - 0.05, down),
        collision_target(lx + 0.06, ly - 0.02, lidar.z_max - 0.06, down) | {"gripper": 0.2},
    ]
    return [{"gripper": COLLISION_GRIPPER} | pose for pose in poses]


def random_collision_pose(rng: np.random.Generator) -> dict[str, float]:
    """Uniform pose inside the usable joint ranges, gripper between closed and open.

    Args:
        rng (np.random.Generator): Seeded generator.

    Returns:
        dict[str, float]: Joint configuration.
    """
    pose = {j: float(rng.uniform(*joint_range(checker(), j))) for j in SEED}
    return pose | {"gripper": float(rng.uniform(0.0, 1.5))}


def collision_path_cases() -> list[dict[str, Any]]:
    """path() inputs of the self_collision.path golden: the bench path, the joint lines between every pair of
    scenario poses (allow_escape, and held to the hard margin from the poses inside it), unpruned lines, a gripper-only
    motion and seeded random lines.

    Returns:
        list[dict[str, Any]]: Keyword arguments (samples, allow_escape, prune, moving_parts).
    """
    poses = collision_poses()
    lines = [joint_line(a, b, COLLISION_LINE_STEP_RAD) for i, a in enumerate(poses) for b in poses[i + 1 :]]
    rng = np.random.default_rng(RNG_SEED)
    randoms = [
        joint_line(random_collision_pose(rng), random_collision_pose(rng), COLLISION_LINE_STEP_RAD) for _ in range(6)
    ]
    lidar_open = poses[3] | {"gripper": 0.0}
    gripper_only = joint_line(lidar_open, lidar_open | {"gripper": 1.4})

    def case(samples: list[dict[str, float]], **kw: Any) -> dict[str, Any]:
        return {"samples": samples, "allow_escape": True, "prune": True, "moving_parts": None} | kw

    return (
        [case(arm_path_samples())]
        + [case(line) for line in lines + randoms]
        + [
            case(joint_line(start, poses[0], COLLISION_LINE_STEP_RAD), allow_escape=False)
            for start in poses[3:5] + poses[7:]
        ]
        + [case(lines[k], prune=False) for k in (0, 2, 7)]
        + [case(gripper_only, moving_parts=["moving_jaw_tip"]), case(gripper_only, moving_parts=list(JAW_POINTS))]
    )


def collision_pose_cases() -> list[dict[str, Any]]:
    """pose() inputs of the self_collision.pose golden: the scenario poses and seeded random poses.

    Returns:
        list[dict[str, Any]]: Keyword arguments (joints).
    """
    rng = np.random.default_rng(RNG_SEED + 1)
    return [{"joints": pose} for pose in collision_poses() + [random_collision_pose(rng) for _ in range(24)]]


def body_outline_call(kind: str) -> Call:
    """footprint_geometry kernels on the deployed body.

    Args:
        kind: 'outline' (body_outline) or 'shapes' (union, growth, offset and same_shape of the outline).

    Returns:
        Call: One evaluation.
    """
    body = config().body
    footprint = config().nav_footprint

    def call() -> None:
        outline = footprint_geometry.body_outline(
            body.boxes, body.cylinders, footprint.circle_segments, footprint.close_gap_m
        )
        if kind == "shapes":
            points = footprint_geometry.to_points(outline, 3)
            footprint_geometry.offset_polygon(points, 0.05, 3)
            footprint_geometry.inflation_radius(points, 0.05)
            footprint_geometry.same_shape(points, points)

    return call


def signed_distance_call() -> Call:
    """BodyModel.signed_distance of 4000 points against every body obstacle.

    Returns:
        Call: One evaluation.
    """
    model = body_model.BodyModel.from_settings(config().body)
    pts = np.random.default_rng(RNG_SEED).uniform(-0.4, 0.4, (BODY_POINTS, 3))
    obstacles = model.obstacles

    def call() -> None:
        for obstacle in obstacles:
            obstacle.signed_distance(pts)

    return call


@cache
def frontier_grid() -> frontiers.MapGrid:
    """A synthetic 800 x 800 map: a walled room with two doorways and unknown space beyond.

    Returns:
        frontiers.MapGrid: The grid.
    """
    data = np.full((MAP_SIDE_CELLS, MAP_SIDE_CELLS), frontiers.UNKNOWN, np.int16)
    data[100:700, 100:700] = frontiers.FREE
    for lo, hi in ((100, 700),):
        data[lo - 1, lo:hi] = data[hi, lo:hi] = frontiers.OCCUPIED
        data[lo:hi, lo - 1] = data[lo:hi, hi] = frontiers.OCCUPIED
    data[400, 100:500] = frontiers.OCCUPIED
    data[399:402, 300:340] = frontiers.FREE
    data[99:101, 200:260] = frontiers.UNKNOWN  # doorways into the unknown
    data[699:701, 450:520] = frontiers.UNKNOWN
    data[260:330, 99:101] = frontiers.UNKNOWN
    return frontiers.MapGrid.from_array(data, MAP_RESOLUTION_M, -20.0, -20.0, 0.0, (1,))


def frontier_call(kind: str) -> Call:
    """The frontier pipeline kernels on the synthetic map.

    Args:
        kind: 'extract', 'traversability', 'geodesic', 'field', 'update' (cold tracker update) or 'observation'.

    Returns:
        Call: One evaluation.
    """
    grid = frontier_grid()
    settings = config().frontiers
    min_cells = frontiers.min_cells_for(settings.min_length_m, grid.resolution)
    pass_clearance = 0.2 / 2 + settings.pass_clearance_margin_m
    trav, clearance = frontiers.traversability(grid, pass_clearance)
    found = frontiers.extract_frontiers(grid, min_cells)
    field = frontiers.frontier_field(grid, found)
    start = (300, 300)

    def call() -> None:
        if kind == "extract":
            frontiers.extract_frontiers(grid, min_cells)
        elif kind == "traversability":
            frontiers.traversability(grid, pass_clearance)
        elif kind == "geodesic":
            frontiers.geodesic_from(trav, start, settings.geodesic_downsample, grid.resolution)
        elif kind == "field":
            frontiers.frontier_field(grid, found)
        elif kind == "update":
            frontiers.FrontierTracker(settings, footprint_width_m=0.2).update(grid)
        else:
            for f in found:
                frontiers.observation_pose(
                    f,
                    grid,
                    clearance,
                    field,
                    settings.view_clearance_margin_m,
                    (settings.standoff_min_m, settings.standoff_max_m),
                    settings.los_candidates,
                )

    return call


def point_reach_cases() -> list[dict[str, Any]]:
    """Golden cases of point_reach: the bench point, points near the lidar and the deck, out-of-reach points and
    fixed-seed random points (pitch None = every POINT_PITCHES_DEG, or one pitch), from two IK seeds.

    Returns:
        list[dict[str, Any]]: Keyword arguments robot, point, seed.
    """
    robot = robot_inputs(config())
    pose = kinematics().forward(SEED)
    mount = config().arm.base_in_base_link
    assert mount is not None
    lidar = next(c for c in config().body.cylinders if c.name == "lidar")
    points: list[dict[str, float | None]] = [
        {"x": pose.x + mount.x, "y": pose.y + mount.y, "z": pose.z + mount.z, "pitch": None},
        {"x": 0.30, "y": -0.05, "z": 0.12, "pitch": None},
        {"x": lidar.x + 0.06, "y": lidar.y - 0.02, "z": lidar.z_max - 0.03, "pitch": None},
        {"x": lidar.x, "y": lidar.y, "z": lidar.z_max - 0.04, "pitch": None},
        {"x": 0.33, "y": -0.10, "z": 0.12, "pitch": 0.8},
        {"x": 0.05, "y": -0.05, "z": 0.11, "pitch": None},
        {"x": 0.80, "y": 0.0, "z": 0.2, "pitch": None},
        {"x": -0.40, "y": -0.30, "z": 0.05, "pitch": 0.3},
    ]
    rng = random.Random(RNG_SEED)
    for k in range(REACH_POINTS + REACH_PITCHED_POINTS):
        x, y, z = (rng.uniform(lo, hi) for lo, hi in REACH_POINT_BOX)
        pitch = rng.uniform(*REACH_PITCH_RANGE) if k >= REACH_POINTS else None
        points.append({"x": x, "y": y, "z": z, "pitch": pitch})
    seeds = [dict(SEED), dict(GRIPPER_VIEW)]
    cases = [{"robot": robot, "point": p, "seed": seeds[i % len(seeds)]} for i, p in enumerate(points)]
    # A point whose pitched IK repeats a heading during the refinement (tests/test_ik.py).
    repeated = {"x": -0.05154221794843593, "y": -0.10587178864434088, "z": 0.436844962224659, "pitch": None}
    seed = {
        "shoulder_pan": -0.23974061970965255,
        "shoulder_lift": 0.5374641689892652,
        "elbow_flex": -0.38260157671558903,
        "wrist_flex": 0.6078724925584991,
        "wrist_roll": -0.8244794748834174,
    }
    return [*cases, {"robot": robot, "point": repeated, "seed": seed}]


def reach_map_cases() -> list[dict[str, Any]]:
    """Golden cases of reach_map.build: the bench grid and a denser one with two samples per cell.

    Returns:
        list[dict[str, Any]]: Keyword arguments robot, settings.
    """
    robot = robot_inputs(config())
    return [
        {"robot": robot, "settings": s.model_dump(mode="json", exclude={"file"})} for s in (REACH_MAP, REACH_MAP_DENSE)
    ]


def frontier_case_room(h: int, w: int, known_cols: int, doors: list[tuple[int, int]]) -> np.ndarray:
    """A walled room (rows/cols 4..h-5, w-5) with an interior wall and its gap, known up to column known_cols.

    Args:
        h (int): Map rows.
        w (int): Map columns.
        known_cols (int): Columns from this one on stay unknown (the unexplored part).
        doors (list[tuple[int, int]]): (col_from, col_to) gaps of the top wall opening into unknown space.

    Returns:
        np.ndarray: int16 h x w map.
    """
    data = np.full((h, w), frontiers.UNKNOWN, np.int16)
    data[3 : h - 3, 3 : w - 3] = frontiers.OCCUPIED
    data[4 : h - 4, 4 : w - 4] = frontiers.FREE
    data[h // 2, 4 : w - 14] = frontiers.OCCUPIED
    data[h // 2, 12:16] = frontiers.FREE
    for lo, hi in doors:
        data[3:5, lo:hi] = frontiers.UNKNOWN
    data[:, known_cols:] = frontiers.UNKNOWN
    return data


def frontier_case_clutter(h: int, w: int, seed: int) -> np.ndarray:
    """A map of mostly free space with occupied blobs, unknown holes, mid-range values and free space at the border.

    Args:
        h (int): Map rows.
        w (int): Map columns.
        seed (int): RNG seed.

    Returns:
        np.ndarray: int16 h x w map.
    """
    rng = np.random.default_rng(seed)
    data = np.full((h, w), frontiers.UNKNOWN, np.int16)
    data[4:, 6:] = rng.choice(np.array([0, 0, 0, 0, 0, 0, 10, 25, 26, 30], np.int16), size=(h - 4, w - 6))
    for value, count, size in ((frontiers.OCCUPIED, 14, 3), (frontiers.UNKNOWN, 10, 4), (64, 4, 2), (65, 4, 2)):
        for _ in range(count):
            r, c = int(rng.integers(0, h - size)), int(rng.integers(0, w - size))
            data[r : r + size, c : c + size] = value
    return data


@cache
def frontier_case_maps() -> dict[str, np.ndarray]:
    """The golden case maps of the frontiers kernels.

    Returns:
        dict[str, np.ndarray]: Name -> int16 map (room, clutter, wide clutter, all unknown, one free cell).
    """
    single = np.full((9, 9), frontiers.UNKNOWN, np.int16)
    single[4, 4] = frontiers.FREE
    return {
        "room": frontier_case_room(56, 64, 64, [(10, 22), (30, 34)]),
        "clutter": frontier_case_clutter(60, 52, RNG_SEED),
        "wide_clutter": frontier_case_clutter(30, 90, RNG_SEED + 1),
        "unknown": np.full((12, 12), frontiers.UNKNOWN, np.int16),
        "single": single,
    }


def frontier_traversability_cases() -> list[dict[str, Any]]:
    """Golden cases of frontiers.traversability.

    Returns:
        list[dict[str, Any]]: Keyword arguments (data, resolution, pass_clearance_m).
    """
    maps = frontier_case_maps()
    return [
        {"data": maps["room"], "resolution": FRONTIER_CASE_RES_M, "pass_clearance_m": FRONTIER_CASE_PASS_M},
        {"data": maps["clutter"], "resolution": FRONTIER_CASE_RES_M, "pass_clearance_m": FRONTIER_CASE_PASS_M},
        {"data": maps["wide_clutter"], "resolution": FRONTIER_CASE_COARSE_RES_M, "pass_clearance_m": 0.0},
        {"data": maps["clutter"], "resolution": FRONTIER_CASE_COARSE_RES_M, "pass_clearance_m": 0.075},
        {"data": maps["unknown"], "resolution": FRONTIER_CASE_RES_M, "pass_clearance_m": FRONTIER_CASE_PASS_M},
        {"data": maps["single"], "resolution": FRONTIER_CASE_RES_M, "pass_clearance_m": 0.0},
    ]


def frontier_geodesic_cases() -> list[dict[str, Any]]:
    """Golden cases of frontiers.geodesic_from: starts inside, on a wall (snapped), far away and off the map.

    Returns:
        list[dict[str, Any]]: Keyword arguments (trav, start_rc, downsample, resolution).
    """
    maps = frontier_case_maps()
    room = frontiers.MapGrid.from_array(maps["room"], FRONTIER_CASE_RES_M, 0.0, 0.0, 0.0, (1,))
    room_trav, _ = frontiers.traversability(room, FRONTIER_CASE_PASS_M)
    clutter_trav = frontiers.free_mask(maps["clutter"])
    wide_trav = frontiers.free_mask(maps["wide_clutter"])
    res = FRONTIER_CASE_RES_M
    return [
        {"trav": room_trav, "start_rc": [10, 10], "downsample": 2, "resolution": res},
        {"trav": room_trav, "start_rc": [40, 50], "downsample": 1, "resolution": res},
        {"trav": room_trav, "start_rc": [28, 30], "downsample": 3, "resolution": res},  # on the interior wall
        {"trav": clutter_trav, "start_rc": [30, 30], "downsample": 2, "resolution": res},
        {"trav": clutter_trav, "start_rc": [59, 51], "downsample": 1, "resolution": res},
        {"trav": wide_trav, "start_rc": [15, 45], "downsample": 4, "resolution": FRONTIER_CASE_COARSE_RES_M},
        {"trav": room_trav, "start_rc": [0, 0], "downsample": 2, "resolution": res},  # nothing within the snap
        {"trav": room_trav, "start_rc": [-3, 5], "downsample": 2, "resolution": res},  # off the map
    ]


def frontier_field_cases() -> list[dict[str, Any]]:
    """Golden cases of frontiers.frontier_field (frontiers from extract_frontiers).

    Returns:
        list[dict[str, Any]]: Keyword arguments (data, resolution, min_cells).
    """
    maps = frontier_case_maps()
    res = FRONTIER_CASE_RES_M
    return [
        {"data": maps["room"], "resolution": res, "min_cells": 12},
        {"data": maps["clutter"], "resolution": res, "min_cells": 1},
        {"data": maps["clutter"], "resolution": res, "min_cells": 12},
        {"data": maps["wide_clutter"], "resolution": FRONTIER_CASE_COARSE_RES_M, "min_cells": 3},
        {"data": maps["unknown"], "resolution": res, "min_cells": 1},
        {"data": maps["single"], "resolution": res, "min_cells": 1},
    ]


def frontier_tracker_cases() -> list[dict[str, Any]]:
    """Golden cases of FrontierTracker.update: map sequences that keep, retire, split and reset frontier ids.

    Returns:
        list[dict[str, Any]]: Keyword arguments (maps, settings, footprint_width_m).
    """
    res = FRONTIER_CASE_RES_M
    settings = config().frontiers.model_dump()
    split = frontier_case_room(40, 48, 48, [(8, 34)])
    split[3:5, 20:22] = frontiers.OCCUPIED
    exploring = [
        {"data": frontier_case_room(40, 48, 24, [(10, 26)]), "resolution": res},
        {"data": frontier_case_room(40, 48, 30, [(10, 26)]), "resolution": res},
        {"data": frontier_case_room(40, 48, 48, [(8, 34)]), "resolution": res},
        {"data": split, "resolution": res},
        {"data": frontier_case_room(40, 48, 48, []), "resolution": res},
        {"data": frontier_case_room(40, 48, 14, [(10, 12)]), "resolution": res},  # most of the map lost: new epoch
        {"data": frontier_case_room(40, 48, 30, [(10, 26)]), "resolution": FRONTIER_CASE_COARSE_RES_M},
    ]
    clutter = frontier_case_clutter(44, 40, RNG_SEED + 2)
    growing = []
    for known in (20, 30, 40):
        data = clutter.copy()
        data[:, known:] = frontiers.UNKNOWN
        growing.append({"data": data, "resolution": res})
    return [
        {"maps": exploring, "settings": settings, "footprint_width_m": FRONTIER_CASE_FOOTPRINT_M},
        {"maps": growing, "settings": settings, "footprint_width_m": FRONTIER_CASE_FOOTPRINT_M},
    ]
