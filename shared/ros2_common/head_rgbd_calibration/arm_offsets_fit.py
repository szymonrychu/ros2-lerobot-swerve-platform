"""Joint head chain and arm joint offsets fit from the head D435 (design D9, Q2): the gripper-held ChArUco plate in the
D435 colour camera plus floor planes in the D435 depth frame give the arm joint offsets, the head chain and the tool
point.

Plate-view model: plate corner (`board_corners_3d(cfg.plate)`) -> plate pose in gripper_link (6 DOF from
`plate_mount.xyz_init` / `rpy_init`, or the rotation fixed to the CAD `plate_mount.rpy` and only the translation
free) -> arm FK (so101_arm.urdf, measured joints plus the offsets) -> `base_in_base_link` -> inverse of the
`d435_color_optical_frame` pose of the head chain at the view's measured pan/tilt -> `cv2.projectPoints` with the
D435 colour intrinsics. Floor views are the floor planes (base_link z = 0) in `d435_depth_optical_frame` predicted by
the same head chain (floor_fit), weighted by the floor fit's residual scales.

Free: the head parameters of the head-from-arm fit (`head_arm_fit.fit_param_names`: head_mount_joint x, y, z, roll,
pitch, yaw, tilt zero and roll, plus the camera mount and the pan axis when the config frees them); the
shoulder_lift, elbow_flex and wrist_flex offsets; the wrist_roll offset only with a CAD plate rotation; the plate
pose. A wrist_roll zero error turns the plate about the wrist_roll axis exactly, so without a CAD rotation wrist_roll
stays at `cfg.wrist_roll_offset_rad` (the recorded servo neutral) and the plate rotation absorbs any error; the FK tool
point then moves by the error times its 8 mm distance from that axis. Fixed: the shoulder_pan offset (the gauge: an
arm turn about the pan axis is a head mount x, y, yaw change), `base_in_base_link`, the link lengths and the
intrinsics. The floor views have no free poses of their own (the floor is base_link z = 0).

Acceptance is on held-out plate views (every `arm_limits.holdout_every`-th, covering at least
`arm_limits.min_holdout_head_settings` head settings): reprojection px and the FK tool point against the view's
solvePnP plate pose (mm), never on the fit residual alone (design D9). The tool point (Q2) is the fitted plate pose
composed with the CAD board-to-jaw-tip offset `plate_mount.tool_in_board_m`, accepted only below
`arm_limits.max_tool_holdout_mm`. numpy, scipy and OpenCV only, no ROS.
"""

import math
from collections.abc import Mapping
from dataclasses import dataclass, replace

import numpy as np
from scipy.optimize import least_squares

from ros2_common.head_rgbd_calibration.chain import UrdfChain, arm_in_robot_base, pose_from_xyz_rpy, xyz_rpy_from_pose
from ros2_common.head_rgbd_calibration.charuco import board_corners_3d
from ros2_common.head_rgbd_calibration.config import ARM_JOINT_NAMES, HeadArmFitConfig
from ros2_common.head_rgbd_calibration.floor_fit import (
    D435_DEPTH_FRAME,
    FloorObservation,
    fit_head_chain_floor,
    observation_errors,
    pose_mean_errors,
    tangent_basis,
)
from ros2_common.head_rgbd_calibration.head_arm_fit import (
    D435_COLOR_FRAME,
    GRIPPER_LINK,
    LENGTH_PARAMS,
    MM_PER_M,
    SOLVER_TOLERANCE,
    LimitCheck,
    board_mount_pose,
    board_pnp,
    camera_mount_joint,
    camera_pose,
    corner_errors,
    fit_param_names,
    head_origins,
    head_vector,
    initial_board_mount,
    number,
    numeric_jacobian,
    param_scale,
    rms_px,
)
from ros2_common.head_rgbd_calibration.urdf_text import Origin
from ros2_common.head_rgbd_calibration.views import ArmBoardView, require_joints

GRIPPER_FRAME_LINK = "gripper_frame_link"
GAUGE_JOINT = "shoulder_pan"
WRIST_ROLL = "wrist_roll"
FREE_OFFSET_JOINTS = ("shoulder_lift", "elbow_flex", "wrist_flex")
OFFSET_SUFFIX = "_offset_rad"
MIN_VIEWS = 3
PNP_INIT_VIEWS = 10
ROTATION_PARAMS = 3
SNIPPET_INDENT = " " * 8  # the keys of the client.yml mcp_server `arm:` block
OFFSET_DECIMALS = 4
FLOOR_PREFIX = "floor:"
WRIST_ROLL_FIXED_NOTE = "wrist_roll fixed at the recorded neutral; plate rotation fitted"
WRIST_ROLL_FITTED_NOTE = "wrist_roll fitted; plate rotation fixed to the CAD plate_mount.rpy"
GAUGE_NOTE = "shoulder_pan offset fixed (the gauge)"
TOOL_FITTED_NOTE = "tool point fitted: the plate pose composed with plate_mount.tool_in_board_m"
TOOL_NOT_MEASURED_NOTE = (
    "tool_in_board_m not set (CAD board-to-jaw-tip offset not measured); current tool_offset_m kept"
)
TOOL_HELD_OUT_NOTE = "held-out tool error {value} mm is not below {limit} mm; current tool_offset_m kept"


@dataclass(frozen=True)
class HeadArmFit:
    """Result of `fit_head_arm`.

    Attributes:
        origins: New head origins (head_mount_joint and head_tilt, plus d435_joint and head_pan when the config frees
            them; full xyz/rpy, every component not fitted is the head URDF value).
        offsets: All five arm joint offsets (rad); shoulder_pan is the current value, wrist_roll the recorded neutral
            unless `wrist_roll_fitted`.
        current_offsets: The offsets the fit started from (rad).
        params: Fitted value per head parameter (m or deg) and per free offset (`<joint>_offset_rad`, rad).
        sigma: One-sigma uncertainty per `params` entry; inf when it cannot be computed.
        plate_pose: Fitted plate frame in gripper_link, `{"xyz": [m] * 3, "rpy": [rad] * 3}`.
        tool_offset_m: Tool point in gripper_frame_link (m): the fitted one when `tool_fitted`, else the current one.
        tool_fitted: True only with `plate_mount.tool_in_board_m` and `holdout_tool_mm` below the limit.
        tool_note: Why the tool point is or is not fitted.
        rms_px: Corner reprojection RMS over the fit plate views (px).
        holdout_rms_px: Corner reprojection RMS over the held-out plate views (px); nan without held-out views.
        holdout_tool_mm: RMS over the held-out views of the distance between the arm FK tool point and the tool point
            of the view's solvePnP plate pose through the fitted head chain and plate mount (mm); the candidate fitted
            tool point with `tool_in_board_m`, else the current one.
        wrist_roll_fitted: True with a CAD plate rotation.
        views: Number of fit plate views.
        holdout_views: Number of held-out plate views.
        holdout_head_settings: Distinct head settings (head offset indices) of the held-out views.
        floor_height_rms_mm: RMS over the floor poses of the pose-mean height residual at the solution (mm).
        floor_normal_rms_deg: RMS over the floor poses of the pose-mean normal angle at the solution (deg).
        limits: Every acceptance check (name, value, limit, ok), in check order.
        failed_limits: Names of the failed checks.
        passed: True only when no check fails.
    """

    origins: dict[str, Origin]
    offsets: dict[str, float]
    current_offsets: dict[str, float]
    params: dict[str, float]
    sigma: dict[str, float]
    plate_pose: dict[str, list[float]]
    tool_offset_m: tuple[float, float, float]
    tool_fitted: bool
    tool_note: str
    rms_px: float
    holdout_rms_px: float
    holdout_tool_mm: float
    wrist_roll_fitted: bool
    views: int
    holdout_views: int
    holdout_head_settings: int
    floor_height_rms_mm: float
    floor_normal_rms_deg: float
    limits: list[LimitCheck]
    failed_limits: list[str]
    passed: bool


def offset_param(joint: str) -> str:
    """Name of a free offset in `HeadArmFit.params` (`<joint>_offset_rad`)."""
    return f"{joint}{OFFSET_SUFFIX}"


def chain_with_origins(chain: UrdfChain, origins: Mapping[str, Origin]) -> UrdfChain:
    """`chain` with the given joint origins replaced.

    Args:
        chain: Parsed URDF chain.
        origins: Origin per joint name.

    Returns:
        UrdfChain: A new chain; joints not named keep their origin.
    """
    return UrdfChain(
        [
            replace(j, xyz=origins[j.name].xyz, rpy=origins[j.name].rpy) if j.name in origins else j
            for j in chain.joints.values()
        ]
    )


def gripper_frame_pose(arm: UrdfChain) -> np.ndarray:
    """Pose of gripper_frame_link in gripper_link (fixed joints only between them).

    Args:
        arm: Arm URDF chain.

    Returns:
        np.ndarray: 4x4 pose.
    """
    zeros = {name: 0.0 for name, joint in arm.joints.items() if joint.movable}
    return np.linalg.solve(arm.transform(GRIPPER_LINK, zeros), arm.transform(GRIPPER_FRAME_LINK, zeros))


def floor_checks(
    chain: UrdfChain,
    floor_views: list[FloorObservation],
    origins: dict[str, Origin],
    cfg: HeadArmFitConfig,
) -> tuple[float, float, list[LimitCheck]]:
    """The floor part at the solution: pose-mean residuals and the floor fit's coverage and residual limits.

    Args:
        chain: Start head chain.
        floor_views: Floor observations.
        origins: Fitted head origins.
        cfg: Fit config (`floor_fit.limits`).

    Returns:
        tuple: Height RMS (mm), normal RMS (deg) and the `floor:*` checks; RMS nan without floor views.
    """
    limits = cfg.floor_fit.limits
    heights: list[float] = []
    normals: list[float] = []
    if floor_views:
        _, dh, _ = observation_errors(chain, floor_views, origins, D435_DEPTH_FRAME)
        for pose in sorted({o.pose for o in floor_views}):
            idx = [i for i, o in enumerate(floor_views) if o.pose == pose]
            height, normal = pose_mean_errors(chain, floor_views, origins, idx, dh, D435_DEPTH_FRAME)
            heights.append(height * MM_PER_M)
            normals.append(normal)
    height_rms = float(np.sqrt(np.mean(np.square(heights)))) if heights else math.nan
    normal_rms = float(np.sqrt(np.mean(np.square(normals)))) if normals else math.nan
    pans = [math.degrees(o.pan_rad) for o in floor_views]
    span = float(np.ptp(pans)) if pans else 0.0
    poses = len({o.pose for o in floor_views})
    checks: list[LimitCheck] = [
        (
            f"{FLOOR_PREFIX}min_observations",
            len(floor_views),
            limits.min_observations,
            len(floor_views) >= limits.min_observations,
        ),
        (f"{FLOOR_PREFIX}min_poses", poses, limits.min_poses, poses >= limits.min_poses),
        (f"{FLOOR_PREFIX}min_pan_span_deg", span, limits.min_pan_span_deg, span >= limits.min_pan_span_deg),
        (
            f"{FLOOR_PREFIX}max_height_rms_mm",
            height_rms,
            limits.max_height_rms_mm,
            height_rms <= limits.max_height_rms_mm,
        ),
        (
            f"{FLOOR_PREFIX}max_normal_rms_deg",
            normal_rms,
            limits.max_normal_rms_deg,
            normal_rms <= limits.max_normal_rms_deg,
        ),
    ]
    return height_rms, normal_rms, checks


def fit_head_arm(
    floor_views: list[FloorObservation],
    plate_views: list[ArmBoardView],
    camera_matrix: np.ndarray,
    dist: np.ndarray,
    head_urdf: str,
    arm_urdf: str,
    cfg: HeadArmFitConfig,
    current_offsets: dict[str, float],
) -> HeadArmFit:
    """Fit the arm joint offsets, the head chain, the plate pose and the tool point (design D9, Q2).

    Init: the head chain from `fit_head_chain_floor` on the floor views (the head URDF values when it cannot run),
    the offsets from `current_offsets` (wrist_roll at `cfg.wrist_roll_offset_rad` without a CAD rotation), the plate
    pose from `plate_mount.xyz_init` / `rpy_init` refined by solvePnP of the first `PNP_INIT_VIEWS` fit views (the
    rotation the CAD `plate_mount.rpy` when set). Then one Huber least squares (`cfg.huber_f_scale_px`) over the fit
    plate views' corner residuals (px) and the floor residuals (tangent normal error over `floor_fit.sigma_normal_deg`
    and height error over `floor_fit.sigma_height_mm`, as in floor_fit). Every `arm_limits.holdout_every`-th plate
    view is held out. Sigma from `(J^T J)^-1 * sum(r^2) / (m - n)` at the solution.

    Args:
        floor_views: Floor planes in d435_depth_optical_frame at the measured head angles.
        plate_views: Views of the gripper-held plate with all seven measured joints.
        camera_matrix: 3x3 intrinsics of the D435 colour camera (the factory camera_info).
        dist: Distortion coefficients of the D435 colour camera.
        head_urdf: Head URDF text with the applied chain.
        arm_urdf: Arm URDF text (so101_arm.urdf).
        cfg: Fit config (`plate`, `plate_mount`, `wrist_roll_offset_rad`, `tool_offset_m`, `floor_fit`, `arm_limits`,
            the head parameter switches, `base_in_base_link`, `huber_f_scale_px`).
        current_offsets: Current arm joint offsets (rad), the five arm joints.

    Returns:
        HeadArmFit: Offsets, head chain, plate pose, tool point, uncertainties and every failed limit.

    Raises:
        ValueError: Fewer than 3 plate views, a view lacks a joint, `current_offsets` does not name exactly the arm
            joints, `cfg.plate_mount` is not set, or the plate rotation is fitted without `wrist_roll_offset_rad`.
    """
    if len(plate_views) < MIN_VIEWS:
        raise ValueError(f"the arm offsets fit needs at least {MIN_VIEWS} plate views, got {len(plate_views)}")
    for view in plate_views:
        require_joints(view)
    missing = [name for name in ARM_JOINT_NAMES if name not in current_offsets]
    extra = sorted(set(current_offsets) - set(ARM_JOINT_NAMES))
    if missing or extra:
        raise ValueError(f"current_offsets must name exactly the arm joints: missing {missing}, extra {extra}")
    mount_cfg = cfg.plate_mount
    if mount_cfg is None:
        raise ValueError("the arm offsets fit needs plate_mount (the plate start pose and the CAD tool offset)")
    cad = mount_cfg.rpy is not None
    if not cad and cfg.wrist_roll_offset_rad is None:
        raise ValueError("wrist_roll_offset_rad (the recorded wrist_roll neutral) is required without plate_mount.rpy")

    limits = cfg.arm_limits
    floor = fit_head_chain_floor(floor_views, head_urdf, cfg.floor_fit, D435_DEPTH_FRAME)
    chain = chain_with_origins(UrdfChain.from_text(head_urdf), floor.origins)
    arm = UrdfChain.from_text(arm_urdf)
    names = fit_param_names(cfg)
    camera_joint = camera_mount_joint(chain, D435_COLOR_FRAME)
    free = (*FREE_OFFSET_JOINTS, WRIST_ROLL) if cad else FREE_OFFSET_JOINTS
    start_offsets = {name: float(current_offsets[name]) for name in ARM_JOINT_NAMES}
    if not cad:
        start_offsets[WRIST_ROLL] = float(cfg.wrist_roll_offset_rad)

    k = np.asarray(camera_matrix, dtype=float)
    d = np.asarray(dist, dtype=float)
    base = arm_in_robot_base(cfg.base_in_base_link.model_dump())
    corners = board_corners_3d(cfg.plate)
    views = list(plate_views)
    objects = [corners[np.asarray(v.detection.ids, dtype=int)] for v in views]
    images = [np.asarray(v.detection.points, dtype=float).reshape(-1, 2) for v in views]
    hold_idx = list(range(limits.holdout_every - 1, len(views), limits.holdout_every))
    held = set(hold_idx)
    fit_idx = [i for i in range(len(views)) if i not in held]
    all_idx = list(range(len(views)))

    def grippers(offsets: Mapping[str, float], idx: list[int]) -> list[np.ndarray]:
        return [base @ arm.transform(GRIPPER_LINK, views[i].joints, offsets=offsets) for i in idx]

    x_head = head_vector(chain, names, camera_joint)
    init_idx = fit_idx[:PNP_INIT_VIEWS]
    mount_start = initial_board_mount(
        chain,
        head_origins(chain, x_head, names, camera_joint),
        [views[i] for i in init_idx],
        grippers(start_offsets, init_idx),
        [objects[i] for i in init_idx],
        [images[i] for i in init_idx],
        k,
        d,
        pose_from_xyz_rpy(mount_cfg.xyz_init, mount_cfg.rpy_init),
        D435_COLOR_FRAME,
    ).copy()
    if cad:
        mount_start[:3, :3] = pose_from_xyz_rpy((0.0, 0.0, 0.0), mount_cfg.rpy)[:3, :3]

    n_head = len(names)
    n_free = len(free)

    def unpack(x: np.ndarray) -> tuple[np.ndarray, dict[str, float], np.ndarray]:
        offsets = {
            **start_offsets,
            **{name: float(v) for name, v in zip(free, x[n_head : n_head + n_free], strict=True)},
        }
        plate = x[n_head + n_free :]
        delta = np.concatenate([np.zeros(ROTATION_PARAMS), plate]) if cad else plate
        return x[:n_head], offsets, delta

    def plate_errors(x: np.ndarray, idx: list[int]) -> list[np.ndarray]:
        head_x, offsets, delta = unpack(x)
        return corner_errors(
            chain,
            np.concatenate([head_x, delta]),
            names,
            mount_start,
            [views[i] for i in idx],
            grippers(offsets, idx),
            [objects[i] for i in idx],
            [images[i] for i in idx],
            k,
            d,
            D435_COLOR_FRAME,
            camera_joint,
        )

    sigma_normal = math.radians(cfg.floor_fit.sigma_normal_deg)
    bases = np.asarray([tangent_basis(o.plane.normal / np.linalg.norm(o.plane.normal)) for o in floor_views])

    def floor_residuals(x: np.ndarray) -> np.ndarray:
        if not floor_views:
            return np.zeros(0)
        origins = head_origins(chain, x[:n_head], names, camera_joint)
        dn, dh, _ = observation_errors(chain, floor_views, origins, D435_DEPTH_FRAME)
        tangent = np.einsum("mij,mj->mi", bases, dn) / sigma_normal
        return np.column_stack([tangent, dh * MM_PER_M / cfg.floor_fit.sigma_height_mm]).ravel()

    def residuals(x: np.ndarray) -> np.ndarray:
        return np.concatenate([*(e.ravel() for e in plate_errors(x, fit_idx)), floor_residuals(x)])

    plate_params = ROTATION_PARAMS if cad else 2 * ROTATION_PARAMS
    x0 = np.concatenate([x_head, [start_offsets[name] for name in free], np.zeros(plate_params)])
    solution = least_squares(
        residuals,
        x0,
        loss="huber",
        f_scale=cfg.huber_f_scale_px,
        x_scale="jac",
        ftol=SOLVER_TOLERANCE,
        xtol=SOLVER_TOLERANCE,
        gtol=SOLVER_TOLERANCE,
    ).x

    jac = numeric_jacobian(residuals, solution)
    residual = residuals(solution)
    dof = len(residual) - len(solution)
    cov = np.full((len(solution), len(solution)), math.inf)
    if dof > 0:
        try:
            cov = np.linalg.inv(jac.T @ jac) * (float(residual @ residual) / dof)
        except np.linalg.LinAlgError:
            pass
    sigma_x = np.sqrt(np.abs(np.diag(cov)))

    head_x, offsets, delta = unpack(solution)
    origins = head_origins(chain, head_x, names, camera_joint)
    plate = board_mount_pose(mount_start, delta)
    errors = plate_errors(solution, all_idx)

    gripper_frame = gripper_frame_pose(arm)
    tool_in_board = mount_cfg.tool_in_board_m
    if tool_in_board is not None:
        tool_gripper = plate @ np.append(tool_in_board, 1.0)
    else:
        tool_gripper = gripper_frame @ np.append(cfg.tool_offset_m, 1.0)
    tool_mm = []
    for i, gripper in zip(hold_idx, grippers(offsets, hold_idx), strict=True):
        camera = camera_pose(chain, views[i].joints, origins, D435_COLOR_FRAME)
        board = board_pnp(objects[i], images[i], k, d, np.linalg.solve(camera, gripper @ plate))
        if board is None:
            tool_mm.append(math.nan)
            continue
        observed = camera @ board @ np.linalg.inv(plate)
        tool_mm.append(float(np.linalg.norm((observed @ tool_gripper - gripper @ tool_gripper)[:3])) * MM_PER_M)
    holdout_tool_mm = float(np.sqrt(np.mean(np.square(tool_mm)))) if tool_mm else math.nan

    tool_limit = limits.max_tool_holdout_mm
    tool_fitted = tool_in_board is not None and holdout_tool_mm < tool_limit
    candidate = np.linalg.solve(gripper_frame, tool_gripper)[:3]
    tool_offset = (float(candidate[0]), float(candidate[1]), float(candidate[2]))
    if tool_in_board is None:
        tool_note = TOOL_NOT_MEASURED_NOTE
    elif tool_fitted:
        tool_note = TOOL_FITTED_NOTE
    else:
        tool_note = TOOL_HELD_OUT_NOTE.format(value=number(holdout_tool_mm, ".2f"), limit=number(tool_limit, ".2f"))
    if not tool_fitted:
        x, y, z = cfg.tool_offset_m
        tool_offset = (float(x), float(y), float(z))

    params = {name: float(solution[i] * param_scale(name)) for i, name in enumerate(names)}
    sigma = {name: float(sigma_x[i] * param_scale(name)) for i, name in enumerate(names)}
    for j, joint in enumerate(free):
        params[offset_param(joint)] = float(solution[n_head + j])
        sigma[offset_param(joint)] = float(sigma_x[n_head + j])

    fit_rms = rms_px([errors[i] for i in fit_idx])
    holdout_rms = rms_px([errors[i] for i in hold_idx])
    settings = len({views[i].head_offset for i in hold_idx})
    height_rms, normal_rms, floor_part = floor_checks(chain, floor_views, origins, cfg)
    checks: list[LimitCheck] = [
        ("rms_px", fit_rms, limits.rms_px, fit_rms <= limits.rms_px),
        ("holdout_rms_px", holdout_rms, limits.holdout_rms_px, holdout_rms <= limits.holdout_rms_px),
        ("holdout_tool_mm", holdout_tool_mm, tool_limit, holdout_tool_mm < tool_limit),
        ("min_views", len(fit_idx), limits.min_views, len(fit_idx) >= limits.min_views),
        (
            "holdout_coverage",
            settings,
            limits.min_holdout_head_settings,
            settings >= limits.min_holdout_head_settings,
        ),
        *floor_part,
    ]
    failed = [name for name, _, _, ok in checks if not ok]
    plate_xyz, plate_rpy = xyz_rpy_from_pose(plate)
    return HeadArmFit(
        origins=origins,
        offsets={name: offsets[name] for name in ARM_JOINT_NAMES},
        current_offsets={name: float(current_offsets[name]) for name in ARM_JOINT_NAMES},
        params=params,
        sigma=sigma,
        plate_pose={"xyz": list(plate_xyz), "rpy": list(plate_rpy)},
        tool_offset_m=tool_offset,
        tool_fitted=tool_fitted,
        tool_note=tool_note,
        rms_px=fit_rms,
        holdout_rms_px=holdout_rms,
        holdout_tool_mm=holdout_tool_mm,
        wrist_roll_fitted=cad,
        views=len(fit_idx),
        holdout_views=len(hold_idx),
        holdout_head_settings=settings,
        floor_height_rms_mm=height_rms,
        floor_normal_rms_deg=normal_rms,
        limits=checks,
        failed_limits=failed,
        passed=not failed,
    )


def offsets_snippet_yaml(fit: HeadArmFit, session: str, date: str) -> str:
    """The reviewed snippet for the client.yml mcp_server `arm:` block (keys indented by `SNIPPET_INDENT`).

    Args:
        fit: Fit result.
        session: Session id.
        date: Fit date (YYYY-MM-DD).

    Returns:
        str: One comment line (date, session, rms, held-out tool error), the `joint_offsets_rad:` block with the five
        joints (4 decimals) and, when the tool point is fitted, `tool_offset_m:`.
    """
    lines = [
        f"{SNIPPET_INDENT}# arm offsets fit {date}, session {session}: rms {number(fit.rms_px, '.2f')} px, "
        f"held-out tool {number(fit.holdout_tool_mm, '.1f')} mm",
        f"{SNIPPET_INDENT}joint_offsets_rad:",
        *[f"{SNIPPET_INDENT}  {name}: {fit.offsets[name]:.{OFFSET_DECIMALS}f}" for name in ARM_JOINT_NAMES],
    ]
    if fit.tool_fitted:
        x, y, z = (f"{v:.{OFFSET_DECIMALS}f}" for v in fit.tool_offset_m)
        lines.append(f"{SNIPPET_INDENT}tool_offset_m: {{x: {x}, y: {y}, z: {z}}}")
    return "\n".join(lines) + "\n"


def arm_report_markdown(fit: HeadArmFit, cfg: HeadArmFitConfig, session: str, date: str) -> str:
    """Markdown report of an arm offsets fit.

    Args:
        fit: Fit result.
        cfg: Fit config (the plate mount mode and the CAD tool offset).
        session: Session id.
        date: Report date (YYYY-MM-DD).

    Returns:
        str: The report: notes (wrist_roll mode, gauge, tool point), offsets table, head parameters, the tool point,
        residuals and coverage, the plate pose and every limit with PASS or FAIL.
    """
    tool_in_board = cfg.plate_mount.tool_in_board_m if cfg.plate_mount is not None else None
    lines = [
        f"# Arm offsets fit {session}",
        "",
        f"- Date: {date}",
        f"- Result: {'PASS' if fit.passed else 'FAIL'}",
        f"- Note: {WRIST_ROLL_FITTED_NOTE if fit.wrist_roll_fitted else WRIST_ROLL_FIXED_NOTE}",
        f"- Note: {GAUGE_NOTE}",
        f"- Tool point: {fit.tool_note}",
        "",
        "## Joint offsets",
        "",
        "| joint | current (rad) | fitted (rad) | change (deg) | sigma (deg) |",
        "|---|---|---|---|---|",
    ]
    for joint in ARM_JOINT_NAMES:
        current, fitted = fit.current_offsets[joint], fit.offsets[joint]
        name = offset_param(joint)
        sigma = number(math.degrees(fit.sigma[name]), ".3f") if name in fit.sigma else "fixed"
        lines.append(
            f"| {joint} | {current:.4f} | {fitted:.4f} | {number(math.degrees(fitted - current), '+.3f')} | {sigma} |"
        )
    lines += [
        "",
        "## Head chain",
        "",
        "Lengths in m (fitted) and mm (sigma); angles in deg.",
        "",
        "| parameter | fitted | sigma |",
        "|---|---|---|",
    ]
    for name, value in fit.params.items():
        if name.endswith(OFFSET_SUFFIX):
            continue
        spec, change = (".4f", MM_PER_M) if name in LENGTH_PARAMS else (".3f", 1.0)
        lines.append(f"| {name} | {number(value, spec)} | {number(fit.sigma[name] * change, '.3f')} |")
    tool_mm = ", ".join(number(v * MM_PER_M, ".1f") for v in fit.tool_offset_m)
    cad_mm = "not set" if tool_in_board is None else ", ".join(number(v * MM_PER_M, ".1f") for v in tool_in_board)
    xyz_mm = ", ".join(number(v * MM_PER_M, ".1f") for v in fit.plate_pose["xyz"])
    rpy_deg = ", ".join(number(math.degrees(v), ".2f") for v in fit.plate_pose["rpy"])
    lines += [
        "",
        "## Tool point",
        "",
        f"- tool_offset_m in {GRIPPER_FRAME_LINK} (mm): {tool_mm} ({'fitted' if fit.tool_fitted else 'current'})",
        f"- plate_mount.tool_in_board_m (mm): {cad_mm}",
        f"- {fit.tool_note}",
        "",
        "## Residuals and coverage",
        "",
        f"- rms_px (fit views): {number(fit.rms_px, '.3f')}",
        f"- holdout_rms_px: {number(fit.holdout_rms_px, '.3f')}",
        f"- holdout_tool_mm: {number(fit.holdout_tool_mm, '.2f')}",
        f"- plate views: {fit.views} fit, {fit.holdout_views} held out over {fit.holdout_head_settings} head settings",
        f"- floor (pose mean): height {number(fit.floor_height_rms_mm, '.2f')} mm, "
        f"normal {number(fit.floor_normal_rms_deg, '.3f')} deg",
        "",
        "## Plate in gripper_link",
        "",
        f"- xyz (mm): {xyz_mm}; rpy (deg): {rpy_deg}",
        "",
        "## Limits",
        "",
        "| limit | value | limit value | result |",
        "|---|---|---|---|",
    ]
    for name, value, limit, ok in fit.limits:
        lines.append(
            f"| {name} | {number(float(value), '.3f')} | {number(float(limit), '.3f')} | {'PASS' if ok else 'FAIL'} |"
        )
    return "\n".join(lines) + "\n"
