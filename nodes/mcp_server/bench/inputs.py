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
from mcp_server.arm_reach import ReachEnvelope, point_reach
from mcp_server.camera_candidates import CandidateStore, RobotSnapshot
from mcp_server.config import REPO_ROOT, McpServerConfig, ReachMapSettings
from mcp_server.floor_guard import FloorGuard, JawModel, SurfaceModel
from mcp_server.grasp import GraspPlanner, ObjectSpec, grasp_params
from mcp_server.grasp_check import GripMeasurements, band_verdict, fuse, grip_evidence
from mcp_server.ik import ArmKinematics
from mcp_server.jaw_crop import jaw_crop
from mcp_server.reach_map import build as build_reach_map
from mcp_server.reach_map import checker_from_config
from mcp_server.sag import GravityModel, SagCompensator, compensate
from mcp_server.self_collision import SelfCollisionChecker
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
IMAGE_W, IMAGE_H = 640, 480
LIDAR_POINTS = 720
PATH_SAMPLES = 40
BODY_POINTS = 4000
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


def occupancy_cells() -> list[int]:
    """A synthetic 800 x 800 occupancy grid (room walls, free interior, unknown outside) as a flat list.

    Returns:
        list[int]: Row-major cells.
    """
    grid = np.full((MAP_SIDE_CELLS, MAP_SIDE_CELLS), -1, np.int16)
    grid[100:700, 100:700] = 0
    grid[100:104, 100:700] = grid[696:700, 100:700] = 100
    grid[100:700, 100:104] = grid[100:700, 696:700] = 100
    grid[300:304, 100:500] = 100
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
