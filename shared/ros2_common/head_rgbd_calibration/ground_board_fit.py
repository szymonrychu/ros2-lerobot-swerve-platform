"""Ground board fit (user decisions 312, 324, 326): the FULL ChArUco board lying flat on the floor at one or more
unknown placements, seen by the head D435 colour camera over a pan/tilt sweep, gives the camera to ground head chain.

Model per view: board corner -> board on the floor (pattern face up at z = board thickness above base_link z = 0, the
floor; free x, y and yaw per placement) -> inverse of the camera pose of the head chain at the measured pan/tilt with
the fitted origins (`head_arm_fit.head_origins`) -> `cv2.projectPoints` with the D435 colour intrinsics.

Free: the head mount z, roll and pitch, the tilt zero and tilt roll (the floor-fit set), the D435 mount x, z, roll and
yaw (`fit_camera_mount`, as head_arm_fit: its y and pitch are degenerate with the mount y and the tilt zero), the pan
axis x, y (`fit_pan_axis`) and per placement the board x, y and yaw. Gauge (decision 326): the head mount x, y and
yaw stay at the applied values, because a head shift or turn in the floor plane is exactly a shift or turn of every
free board (the synthetic scene's Jacobian condition number goes from 75 to 1e11 with them free). The camera height
is absolute (the board lies on the floor), the pan axis and the camera mount come from the pan and tilt spread.

Depth floor planes (the floor sweep's planes of the D435 cloud at each pose) are a separate cross-check, not
residuals: the D435 depth carries its own structured bias (about 1 deg plane scatter cloud to cloud), which would
leak into the colour chain, and kept apart they validate the fit independently (pose-mean height and normal RMS
under the fitted and the applied chain, as the floor fit limits).

Every `limits.holdout_every`-th distinct head pose (with all its frames) is held out of the fit and scored. Accuracy
examples propagate the head parameter covariance `(J^T J)^-1 s^2` (J the fit corner rows over every free parameter,
s^2 the reduced chi-square) to plain quantities at the head's home pose in the depth sensor frame: the floor height
`near_m` and `far_m` ahead of the head, the obstacle position `far_m` ahead (lateral and range), the head pitch and
roll; 1 sigma and 95 % (1.96 sigma) of the scalar. numpy, scipy and OpenCV only, no ROS.
"""

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, replace

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.stats import norm

from ros2_common.head_rgbd_calibration.arm_camera_fit import ArmFk, check_entry, wrapped_deg
from ros2_common.head_rgbd_calibration.chain import UrdfChain, pose_from_xyz_rpy
from ros2_common.head_rgbd_calibration.charuco import Detection, board_corners_3d
from ros2_common.head_rgbd_calibration.config import BoardConfig, GroundBoardFitConfig
from ros2_common.head_rgbd_calibration.floor_fit import (
    PAN_JOINT,
    TILT_JOINT,
    FloorObservation,
    json_float,
    observation_errors,
    pose_mean_errors,
    rms,
)
from ros2_common.head_rgbd_calibration.head_arm_fit import (
    CAMERA_MOUNT_PARAMS,
    D435_COLOR_FRAME,
    LENGTH_PARAMS,
    MM_PER_M,
    PAN_AXIS_PARAMS,
    PARAM_NAMES,
    Correlation,
    camera_mount_joint,
    head_origins,
    head_vector,
    largest_correlation,
    number,
    numeric_jacobian,
    param_scale,
    rms_px,
)
from ros2_common.head_rgbd_calibration.urdf_text import Origin

D435_DEPTH_FRAME = "d435_depth_optical_frame"
FLOOR_CHAIN_PARAMS = ("mount_z_m", "mount_roll_deg", "mount_pitch_deg", "tilt_zero_deg", "tilt_roll_deg")
GAUGE_PARAMS = ("mount_x_m", "mount_y_m", "mount_yaw_deg")
PLACEMENT_KEYS = ("x", "y", "yaw")
MIN_VIEWS = 3
SOLVER_TOLERANCE = 1e-10
# Column-scaled Jacobian condition number above which the covariance (sigmas, accuracy examples) is unknown (inf).
MAX_CONDITION = 1e8
P95_1D_FACTOR = float(norm.ppf(0.975))
DEG = "deg"
MM = "mm"
NEAR_HEIGHT_LABEL = "floor height {distance:g} m ahead of the head"
FAR_HEIGHT_LABEL = "floor height {distance:g} m ahead of the head"
LATERAL_LABEL = "obstacle position {distance:g} m ahead, lateral (head heading as applied, not calibrated here)"
RANGE_LABEL = "obstacle position {distance:g} m ahead, range"
PITCH_LABEL = "head pitch (optical axis to the floor)"
ROLL_LABEL = "head roll (image horizon to the floor)"
ACCURACY_LABELS = (
    NEAR_HEIGHT_LABEL.format(distance=1.0),
    FAR_HEIGHT_LABEL.format(distance=2.0),
    LATERAL_LABEL.format(distance=2.0),
    RANGE_LABEL.format(distance=2.0),
    PITCH_LABEL,
    ROLL_LABEL,
)
SIGMA_SUFFIX = ", 1 sigma"
P95_SUFFIX = ", 95 %"
GAUGE_NOTE = (
    "gauge: head mount x, y and yaw kept at the applied values (a head shift or turn in the floor plane equals a "
    "shift or turn of the free board)"
)
CAMERA_MOUNT_NOTE = "camera mount x, z, roll, yaw free; its y and pitch kept (degenerate with mount y and tilt zero)"
PAN_AXIS_NOTE = "pan axis free: head_pan origin x, y moved, head_tilt origin moved back (head unchanged at pan 0)"
CROSS_CHECK_NOTE = "depth floor planes are a cross-check, not fitted"
EXAMPLES_NOTE = (
    "accuracy examples at the head home pose, in the depth sensor frame the ground estimate uses; they leave out the "
    "head x, y and heading, which this fit does not calibrate (kept as applied)"
)


@dataclass(frozen=True)
class GroundBoardView:
    """One colour frame of the board on the floor.

    Attributes:
        view_id: Unique id (placement, pose and frame).
        placement: Index of the board placement (the user moved the board between placements).
        pose: Index of the head pose in the session (unique over placements; frames of one pose share it).
        pan_rad: Measured head_pan (rad).
        tilt_rad: Measured head_tilt (rad), + is down.
        detection: Board corners (ids into the FULL board, sub-pixel points).
    """

    view_id: str
    placement: int
    pose: int
    pan_rad: float
    tilt_rad: float
    detection: Detection


@dataclass(frozen=True)
class AccuracyExample:
    """Estimated error of one plain quantity.

    Attributes:
        label: What the example is (one of `ACCURACY_LABELS`, with the configured distances).
        unit: "mm" or "deg".
        sigma: One-sigma error; inf when unknown.
        p95: 95 % error (1.96 sigma); inf when unknown.
        shift: Fitted minus applied value of the quantity.
    """

    label: str
    unit: str
    sigma: float
    p95: float
    shift: float


@dataclass(frozen=True)
class CurrentScore:
    """The applied head chain on this session (placements refitted): fit and held-out corner RMS (px)."""

    rms_px: float
    holdout_rms_px: float


@dataclass(frozen=True)
class GroundBoardFit:
    """Result of `fit_ground_board`.

    Attributes:
        origins: New head origins (head_mount_joint, head_tilt, and with their parameters the camera mount joint and
            head_pan); every component not fitted is the applied value.
        names: Every head parameter of the model (`model_names`), gauge included.
        free: Free head parameter names (`free_head_names`).
        params: Value per model parameter (m or deg); gauge parameters at the applied value.
        sigma: One-sigma uncertainty per free head parameter (m or deg); inf when unknown.
        start_params: Applied value per model parameter (m or deg).
        covariance: Covariance of every free parameter (head then placements; m, rad).
        max_correlation: Free head parameter pair with the largest absolute correlation.
        condition_number: Condition number of the column-normalised Jacobian at the solution.
        placements: Per placement {placement, x_m, y_m, yaw_deg, sigma_x_mm, sigma_y_mm, sigma_yaw_deg, views, rms_px}.
        rms_px: Corner RMS over the fit views (px).
        holdout_rms_px: Corner RMS over the held-out views (px); nan without any.
        views: Fit views.
        holdout_views: Held-out views.
        pose_count: Distinct head poses.
        pan_span_deg: Peak-to-peak measured pan over all views.
        tilt_span_deg: Peak-to-peak measured tilt over all views.
        poses: Per head pose {pose, placement, pan_deg, tilt_deg, views, rms_px, held_out}.
        accuracy: The accuracy examples in `ACCURACY_LABELS` order.
        current: The applied chain scored on this session.
        cross_check: {planes, poses, fitted {height_rms_mm, normal_rms_deg}, applied {...}}: depth floor plane
            pose-mean residuals under the fitted and the applied chain (nan values without planes).
        notes: Gauge and model notes.
        solver: Solver summary {status, message, nfev, cost, start_rms_px, free}.
    """

    origins: dict[str, Origin]
    names: tuple[str, ...]
    free: tuple[str, ...]
    params: dict[str, float]
    sigma: dict[str, float]
    start_params: dict[str, float]
    covariance: np.ndarray
    max_correlation: Correlation
    condition_number: float
    placements: list[dict[str, float | int]]
    rms_px: float
    holdout_rms_px: float
    views: int
    holdout_views: int
    pose_count: int
    pan_span_deg: float
    tilt_span_deg: float
    poses: list[dict[str, float | int | bool]]
    accuracy: list[AccuracyExample]
    current: CurrentScore
    cross_check: dict[str, object]
    notes: list[str]
    solver: dict[str, object]


def free_head_names(cfg: GroundBoardFitConfig) -> tuple[str, ...]:
    """The free head parameters of a config.

    Args:
        cfg: Fit config.

    Returns:
        tuple[str, ...]: `FLOOR_CHAIN_PARAMS`, then `CAMERA_MOUNT_PARAMS` with `fit_camera_mount`, then
        `PAN_AXIS_PARAMS` with `fit_pan_axis`.
    """
    camera = CAMERA_MOUNT_PARAMS if cfg.fit_camera_mount else ()
    pan_axis = PAN_AXIS_PARAMS if cfg.fit_pan_axis else ()
    return (*FLOOR_CHAIN_PARAMS, *camera, *pan_axis)


def model_names(cfg: GroundBoardFitConfig) -> tuple[str, ...]:
    """Every head parameter of the model (the `head_arm_fit.head_origins` layout): the eight mount and tilt
    parameters, then the camera mount and pan axis parameters the config frees.

    Args:
        cfg: Fit config.

    Returns:
        tuple[str, ...]: Parameter names.
    """
    camera = CAMERA_MOUNT_PARAMS if cfg.fit_camera_mount else ()
    pan_axis = PAN_AXIS_PARAMS if cfg.fit_pan_axis else ()
    return (*PARAM_NAMES, *camera, *pan_axis)


def board_on_floor(board: BoardConfig, x: float, y: float, yaw: float) -> np.ndarray:
    """Pose in base_link of the board lying face up on the floor.

    Args:
        board: Board geometry (squares, thickness).
        x: Board centre x in base_link (m).
        y: Board centre y in base_link (m).
        yaw: Turn of the board about base_link z (rad); 0 puts the board x axis along base_link x.

    Returns:
        np.ndarray: 4x4 pose of the board frame: pattern face at z = thickness, board z (into the board) down.
    """
    pose = pose_from_xyz_rpy((0.0, 0.0, 0.0), (math.pi, 0.0, yaw))
    centre = board_corners_3d(board).mean(axis=0)
    pose[:3, 3] = np.array([x, y, board.thickness_mm / MM_PER_M]) - pose[:3, :3] @ centre
    return pose


def locate_board(
    camera: np.ndarray, detection: Detection, camera_matrix: np.ndarray, dist: np.ndarray, board: BoardConfig
) -> tuple[float, float, float] | None:
    """Board placement on the floor from one view: each corner's ray meets the board plane (z = thickness), then a
    2D rigid fit of the board corners to those points. No PnP, so no planar pose flip at oblique views.

    Args:
        camera: 4x4 pose of the camera optical frame in base_link.
        detection: Detected corners.
        camera_matrix: 3x3 intrinsics.
        dist: Distortion coefficients.
        board: Board geometry.

    Returns:
        tuple[float, float, float] | None: (x, y, yaw) as `board_on_floor` takes them; None when fewer than 3 rays meet
        the plane in front of the camera.
    """
    height = board.thickness_mm / MM_PER_M
    pixels = np.asarray(detection.points, dtype=float).reshape(-1, 1, 2)
    rays = cv2.undistortPoints(pixels, np.asarray(camera_matrix, dtype=float), np.asarray(dist, dtype=float))
    rays = np.column_stack([rays.reshape(-1, 2), np.ones(len(pixels))]) @ camera[:3, :3].T
    origin = camera[:3, 3]
    with np.errstate(divide="ignore", invalid="ignore"):
        scale = (height - origin[2]) / rays[:, 2]
    hits = np.isfinite(scale) & (scale > 0)
    if int(hits.sum()) < MIN_VIEWS:
        return None
    floor = origin[:2] + scale[hits, None] * rays[hits, :2]
    corners = board_corners_3d(board)
    local = corners[np.asarray(detection.ids, dtype=int)[hits], :2] - corners[:, :2].mean(axis=0)
    local[:, 1] = -local[:, 1]  # the face-up board's y runs against base_link y at yaw 0 (board_on_floor)
    src, dst = local - local.mean(axis=0), floor - floor.mean(axis=0)
    cross = src.T @ dst
    yaw = math.atan2(cross[0, 1] - cross[1, 0], cross[0, 0] + cross[1, 1])
    turn = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    centre = floor.mean(axis=0) - turn @ local.mean(axis=0)
    return float(centre[0]), float(centre[1]), yaw


def chain_with(chain: UrdfChain, origins: Mapping[str, Origin]) -> UrdfChain:
    """A copy of `chain` with the given joint origins.

    Args:
        chain: Parsed head URDF.
        origins: Origin per joint name.

    Returns:
        UrdfChain: New chain; joints not named keep their origin.
    """
    return UrdfChain(
        [
            replace(j, xyz=tuple(origins[j.name].xyz), rpy=tuple(origins[j.name].rpy)) if j.name in origins else j
            for j in chain.joints.values()
        ]
    )


def circular_mean(angles: Sequence[float]) -> float:
    """Mean direction of angles (rad)."""
    return math.atan2(float(np.mean(np.sin(angles))), float(np.mean(np.cos(angles))))


class GroundBoardProblem:
    """The residual model of one session: views grouped by head pose (frames of a pose share the camera pose),
    corners, placements and the held-out poses."""

    def __init__(
        self,
        views: Sequence[GroundBoardView],
        camera_matrix: np.ndarray,
        dist: np.ndarray,
        head_urdf: str,
        cfg: GroundBoardFitConfig,
        camera_frame: str = D435_COLOR_FRAME,
    ) -> None:
        """Precompute what does not change during the fit.

        Args:
            views: Board views.
            camera_matrix: 3x3 colour intrinsics.
            dist: Distortion coefficients.
            head_urdf: Head URDF text with the applied chain (the start and the gauge).
            cfg: Fit config.
            camera_frame: Optical frame of the camera in the head URDF.
        """
        self.cfg = cfg
        self.views = list(views)
        self.k = np.asarray(camera_matrix, dtype=float)
        self.d = np.asarray(dist, dtype=float)
        self.chain = UrdfChain.from_text(head_urdf)
        self.camera_frame = camera_frame
        self.camera_joint = camera_mount_joint(self.chain, camera_frame)
        self.names = model_names(cfg)
        self.free = free_head_names(cfg)
        self.free_idx = [self.names.index(name) for name in self.free]
        self.start = head_vector(self.chain, self.names, self.camera_joint)
        corners = board_corners_3d(cfg.board)
        self.objects = [corners[np.asarray(v.detection.ids, dtype=int)] for v in self.views]
        self.images = [np.asarray(v.detection.points, dtype=float).reshape(-1, 2) for v in self.views]
        self.pose_ids = sorted({v.pose for v in self.views})
        first = {pose: next(v for v in self.views if v.pose == pose) for pose in self.pose_ids}
        self.pans = np.array([first[p].pan_rad for p in self.pose_ids], dtype=float)
        self.tilts = np.array([first[p].tilt_rad for p in self.pose_ids], dtype=float)
        self.view_pose = [self.pose_ids.index(v.pose) for v in self.views]
        self.placement_ids = sorted({v.placement for v in self.views})
        self.view_placement = [self.placement_ids.index(v.placement) for v in self.views]
        held = set(self.pose_ids[cfg.limits.holdout_every - 1 :: cfg.limits.holdout_every])
        for placement in self.placement_ids:
            poses = sorted({v.pose for v in self.views if v.placement == placement})
            if poses and set(poses) <= held:
                held.discard(poses[-1])  # a placement keeps a fit pose, else its x, y, yaw are unobserved
        self.held = held
        self.fit_idx = [i for i, v in enumerate(self.views) if v.pose not in held]
        self.hold_idx = [i for i, v in enumerate(self.views) if v.pose in held]

    def head(self, free: np.ndarray) -> np.ndarray:
        """Every model parameter (m, rad) with the free ones from `free`, the others at the applied values."""
        full = self.start.copy()
        full[self.free_idx] = free
        return full

    def origins(self, full: np.ndarray) -> dict[str, Origin]:
        """Head origins of a model parameter vector."""
        return head_origins(self.chain, full, self.names, self.camera_joint)

    def cameras(self, full: np.ndarray) -> np.ndarray:
        """Camera poses in base_link at every head pose, (n_poses, 4, 4)."""
        fk = ArmFk(chain_with(self.chain, self.origins(full)), self.camera_frame)
        return fk.poses({PAN_JOINT: self.pans, TILT_JOINT: self.tilts}, {})

    def view_errors(self, full: np.ndarray, placements: np.ndarray, idx: Sequence[int]) -> list[np.ndarray]:
        """Projected minus detected corners (px) of the given views.

        Args:
            full: Model parameters (m, rad).
            placements: (n_placements, 3) board x, y (m) and yaw (rad).
            idx: View indices.

        Returns:
            list[np.ndarray]: (N, 2) errors per view.
        """
        picked = list(idx)
        if not picked:
            return []
        cameras = self.cameras(full)
        boards = [board_on_floor(self.cfg.board, *row) for row in placements]
        errors = []
        for i in picked:
            pose = np.linalg.solve(cameras[self.view_pose[i]], boards[self.view_placement[i]])
            rvec = cv2.Rodrigues(pose[:3, :3])[0]
            projected = cv2.projectPoints(self.objects[i], rvec, pose[:3, 3], self.k, self.d)[0].reshape(-1, 2)
            errors.append(projected - self.images[i])
        return errors

    def initial_placements(self) -> np.ndarray:
        """Start placements: per placement the median x, y and the circular mean yaw of `locate_board` over its views
        through the applied chain.

        Returns:
            np.ndarray: (n_placements, 3).

        Raises:
            ValueError: A placement has no view whose rays meet the floor.
        """
        cameras = self.cameras(self.start)
        rows = []
        for placement in self.placement_ids:
            found = [
                located
                for i, v in enumerate(self.views)
                if v.placement == placement
                and (located := locate_board(cameras[self.view_pose[i]], v.detection, self.k, self.d, self.cfg.board))
                is not None
            ]
            if not found:
                raise ValueError(f"placement {placement}: no view locates the board on the floor")
            values = np.array(found)
            rows.append([float(np.median(values[:, 0])), float(np.median(values[:, 1])), circular_mean(values[:, 2])])
        return np.array(rows, dtype=float)

    def split(self, vector: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """(model parameters, placements) of a solver vector [free head, placements flat]."""
        count = len(self.free)
        return self.head(vector[:count]), vector[count:].reshape(-1, len(PLACEMENT_KEYS))

    def residuals(self, vector: np.ndarray) -> np.ndarray:
        """Fit-view corner residuals (px) of a solver vector."""
        full, placements = self.split(vector)
        return np.concatenate([e.ravel() for e in self.view_errors(full, placements, self.fit_idx)])

    def solve(self, start: np.ndarray, head_free: bool) -> tuple[np.ndarray, dict[str, object]]:
        """Huber least squares from `start` (solver layout); with `head_free` False only the placements move.

        Args:
            start: Start vector [free head, placements flat].
            head_free: Whether the free head parameters move.

        Returns:
            tuple: (solution vector, solver summary {status, message, nfev, cost, start_rms_px, free}).
        """
        count = len(self.free)
        head = start[:count]

        def fun(x: np.ndarray) -> np.ndarray:
            return self.residuals(x if head_free else np.concatenate([head, x]))

        x0 = start if head_free else start[count:]
        outcome = least_squares(
            fun,
            x0,
            loss="huber",
            f_scale=self.cfg.huber_f_scale_px,
            x_scale="jac",
            ftol=SOLVER_TOLERANCE,
            xtol=SOLVER_TOLERANCE,
            gtol=SOLVER_TOLERANCE,
        )
        full, placements = self.split(start)
        summary: dict[str, object] = {
            "status": int(outcome.status),
            "message": str(outcome.message),
            "nfev": int(outcome.nfev),
            "cost": float(outcome.cost),
            "start_rms_px": rms_px(self.view_errors(full, placements, self.fit_idx)),
            "free": list(self.free) if head_free else [],
        }
        return (outcome.x if head_free else np.concatenate([head, outcome.x])), summary


def sensor_pose(problem: GroundBoardProblem, full: np.ndarray, home: tuple[float, float], frame: str) -> np.ndarray:
    """Pose of `frame` in base_link at the home head angles with the given model parameters."""
    overrides = {name: (o.xyz, o.rpy) for name, o in problem.origins(full).items()}
    return problem.chain.transform(frame, {PAN_JOINT: home[0], TILT_JOINT: home[1]}, overrides)


def example_quantities(
    problem: GroundBoardProblem, full: np.ndarray, reference: np.ndarray, cfg: GroundBoardFitConfig, frame: str
) -> np.ndarray:
    """The accuracy example quantities under model parameters `full` (mm and deg, `ACCURACY_LABELS` order).

    The floor points are fixed in the sensor frame where the `reference` chain puts them (`near_m` / `far_m` ahead of
    the sensor along its heading at home, on the floor): the quantity is where `full` puts that same measurement.

    Args:
        problem: The session problem.
        full: Model parameters (m, rad).
        reference: Model parameters that place the floor points (the fitted chain).
        cfg: Fit config (distances, home pose).
        frame: Depth sensor frame.

    Returns:
        np.ndarray: (6,) near height, far height, far lateral, far range (mm), pitch, roll (deg).
    """
    home = (math.radians(cfg.home_pan_deg), math.radians(cfg.home_tilt_deg))
    base = sensor_pose(problem, reference, home, frame)
    heading = base[:3, 2].copy()
    heading[2] = 0.0
    heading /= np.linalg.norm(heading)
    lateral = np.array([-heading[1], heading[0], 0.0])
    pose = sensor_pose(problem, full, home, frame)

    def seen(distance: float) -> np.ndarray:
        point = np.append(base[:3, 3] + distance * heading, 1.0)
        point[2] = 0.0
        return (pose @ np.linalg.solve(base, point))[:3]

    near, far = seen(cfg.near_m), seen(cfg.far_m)
    axis, right = pose[:3, 2], pose[:3, 0]
    return np.array(
        [
            near[2] * MM_PER_M,
            far[2] * MM_PER_M,
            float(far @ lateral) * MM_PER_M,
            float(far @ heading) * MM_PER_M,
            -math.degrees(math.asin(float(np.clip(axis[2], -1.0, 1.0)))),
            math.degrees(math.asin(float(np.clip(right[2], -1.0, 1.0)))),
        ]
    )


def accuracy_examples(
    problem: GroundBoardProblem, solution: np.ndarray, cov: np.ndarray, cfg: GroundBoardFitConfig, frame: str
) -> list[AccuracyExample]:
    """Propagate the head covariance to the example quantities.

    Args:
        problem: The session problem.
        solution: Solver vector at the solution.
        cov: Covariance of the solver vector.
        cfg: Fit config.
        frame: Depth sensor frame.

    Returns:
        list[AccuracyExample]: One per quantity; shift is fitted minus applied.
    """
    count = len(problem.free)
    fitted = problem.head(solution[:count])

    def quantities(free: np.ndarray) -> np.ndarray:
        return example_quantities(problem, problem.head(free), fitted, cfg, frame)

    jac = numeric_jacobian(quantities, solution[:count])
    head_cov = cov[:count, :count]
    finite = bool(np.all(np.isfinite(head_cov)))
    values = quantities(solution[:count])
    applied = example_quantities(problem, problem.start, fitted, cfg, frame)
    labels = (
        NEAR_HEIGHT_LABEL.format(distance=cfg.near_m),
        FAR_HEIGHT_LABEL.format(distance=cfg.far_m),
        LATERAL_LABEL.format(distance=cfg.far_m),
        RANGE_LABEL.format(distance=cfg.far_m),
        PITCH_LABEL,
        ROLL_LABEL,
    )
    examples = []
    for i, label in enumerate(labels):
        sigma = math.sqrt(max(float(jac[i] @ head_cov @ jac[i]), 0.0)) if finite else math.inf
        unit = MM if i < 4 else DEG
        examples.append(AccuracyExample(label, unit, sigma, P95_1D_FACTOR * sigma, float(values[i] - applied[i])))
    return examples


def cross_check(
    chain: UrdfChain, planes: Sequence[FloorObservation], origins: dict[str, Origin], frame: str
) -> dict[str, float]:
    """Pose-mean residuals of the depth floor planes under a head chain (floor_fit.pose_mean_errors).

    Args:
        chain: Head URDF chain.
        planes: Floor planes in the depth sensor frame.
        origins: Origins to predict with ({} for the chain as it is).
        frame: Depth sensor frame.

    Returns:
        dict[str, float]: {height_rms_mm, normal_rms_deg}; nan without planes.
    """
    observations = list(planes)
    if not observations:
        return {"height_rms_mm": math.nan, "normal_rms_deg": math.nan}
    _, dh, _ = observation_errors(chain, observations, origins, frame)
    heights, normals = [], []
    for pose in sorted({o.pose for o in observations}):
        idx = [i for i, o in enumerate(observations) if o.pose == pose]
        mean_dh, angle = pose_mean_errors(chain, observations, origins, idx, dh, frame)
        heights.append(mean_dh * MM_PER_M)
        normals.append(angle)
    return {"height_rms_mm": rms(heights), "normal_rms_deg": rms(normals)}


def fit_ground_board(
    views: Sequence[GroundBoardView],
    planes: Sequence[FloorObservation],
    camera_matrix: np.ndarray,
    dist: np.ndarray,
    head_urdf: str,
    cfg: GroundBoardFitConfig,
    camera_frame: str = D435_COLOR_FRAME,
    sensor_frame: str = D435_DEPTH_FRAME,
) -> GroundBoardFit:
    """Fit the camera to ground head chain from views of the board on the floor.

    Args:
        views: Board views (any number of placements).
        planes: Depth floor planes of the session's head poses (the cross-check); may be empty.
        camera_matrix: 3x3 colour intrinsics (the D435 factory camera_info).
        dist: Distortion coefficients.
        head_urdf: Head URDF text with the applied chain (start values and the gauge).
        cfg: Fit config.
        camera_frame: Optical frame of the colour camera in the head URDF.
        sensor_frame: Optical frame of the depth sensor (planes and accuracy examples).

    Returns:
        GroundBoardFit: Fitted chain, uncertainties, scores, cross-check and accuracy examples.

    Raises:
        ValueError: Fewer than 3 views, no fit view, or a placement whose board cannot be located.
    """
    if len(views) < MIN_VIEWS:
        raise ValueError(f"the ground board fit needs at least {MIN_VIEWS} views, got {len(views)}")
    problem = GroundBoardProblem(views, camera_matrix, dist, head_urdf, cfg, camera_frame)
    if not problem.fit_idx:
        raise ValueError("the ground board fit has no view left after the held-out poses")
    placements = problem.initial_placements()
    start = np.concatenate([problem.start[problem.free_idx], placements.ravel()])
    solution, solver = problem.solve(start, head_free=True)
    current, _ = problem.solve(start, head_free=False)

    jac = numeric_jacobian(problem.residuals, solution)
    residual = problem.residuals(solution)
    dof = len(residual) - len(solution)
    cov = np.full((len(solution), len(solution)), math.inf)
    norms = np.linalg.norm(jac, axis=0)
    scales = np.where(norms > 0.0, norms, 1.0)
    scaled = jac / scales
    condition = float(np.linalg.cond(scaled))
    if dof > 0 and condition < MAX_CONDITION:
        # inverted column scaled and only below MAX_CONDITION: the inverse of a near singular normal matrix can be
        # huge finite numbers instead of an error; above it the sigmas stay inf (unknown accuracy, refused)
        cov = np.linalg.inv(scaled.T @ scaled) / np.outer(scales, scales) * (float(residual @ residual) / dof)
    sigma_x = np.sqrt(np.abs(np.diag(cov)))

    count = len(problem.free)
    full, fitted_placements = problem.split(solution)
    errors = problem.view_errors(full, fitted_placements, range(len(problem.views)))
    current_full, current_placements = problem.split(current)
    current_errors = problem.view_errors(current_full, current_placements, range(len(problem.views)))
    origins = problem.origins(full)

    placement_rows = []
    for row, placement in enumerate(problem.placement_ids):
        idx = [i for i, v in enumerate(problem.views) if v.placement == placement]
        base = count + row * len(PLACEMENT_KEYS)
        placement_rows.append(
            {
                "placement": int(placement),
                "x_m": float(fitted_placements[row, 0]),
                "y_m": float(fitted_placements[row, 1]),
                "yaw_deg": wrapped_deg(math.degrees(fitted_placements[row, 2])),
                "sigma_x_mm": float(sigma_x[base]) * MM_PER_M,
                "sigma_y_mm": float(sigma_x[base + 1]) * MM_PER_M,
                "sigma_yaw_deg": math.degrees(float(sigma_x[base + 2])),
                "views": len(idx),
                "rms_px": rms_px([errors[i] for i in idx]),
            }
        )
    pose_rows = []
    for k, pose in enumerate(problem.pose_ids):
        idx = [i for i, v in enumerate(problem.views) if v.pose == pose]
        pose_rows.append(
            {
                "pose": int(pose),
                "placement": int(problem.views[idx[0]].placement),
                "pan_deg": math.degrees(problem.pans[k]),
                "tilt_deg": math.degrees(problem.tilts[k]),
                "views": len(idx),
                "rms_px": rms_px([errors[i] for i in idx]),
                "held_out": pose in problem.held,
            }
        )
    pans = np.degrees([v.pan_rad for v in problem.views])
    tilts = np.degrees([v.tilt_rad for v in problem.views])
    observations = list(planes)
    notes = [
        GAUGE_NOTE,
        *([CAMERA_MOUNT_NOTE] if cfg.fit_camera_mount else []),
        *([PAN_AXIS_NOTE] if cfg.fit_pan_axis else []),
        CROSS_CHECK_NOTE,
        EXAMPLES_NOTE,
    ]
    return GroundBoardFit(
        origins=origins,
        names=problem.names,
        free=problem.free,
        params={name: float(full[i] * param_scale(name)) for i, name in enumerate(problem.names)},
        sigma={name: float(sigma_x[i] * param_scale(name)) for i, name in enumerate(problem.free)},
        start_params={name: float(problem.start[i] * param_scale(name)) for i, name in enumerate(problem.names)},
        covariance=cov,
        max_correlation=largest_correlation(cov, problem.free),
        condition_number=condition,
        placements=placement_rows,
        rms_px=rms_px([errors[i] for i in problem.fit_idx]),
        holdout_rms_px=rms_px([errors[i] for i in problem.hold_idx]),
        views=len(problem.fit_idx),
        holdout_views=len(problem.hold_idx),
        pose_count=len(problem.pose_ids),
        pan_span_deg=float(np.ptp(pans)),
        tilt_span_deg=float(np.ptp(tilts)),
        poses=pose_rows,
        accuracy=accuracy_examples(problem, solution, cov, cfg, sensor_frame),
        current=CurrentScore(
            rms_px([current_errors[i] for i in problem.fit_idx]),
            rms_px([current_errors[i] for i in problem.hold_idx]),
        ),
        cross_check={
            "planes": len(observations),
            "poses": len({o.pose for o in observations}),
            "fitted": cross_check(problem.chain, observations, origins, sensor_frame),
            "applied": cross_check(problem.chain, observations, {}, sensor_frame),
        },
        notes=notes,
        solver=solver,
    )


def change(fit: GroundBoardFit, name: str) -> float:
    """Fitted minus applied value of a parameter (m or deg; angles wrapped), 0 for a parameter the fit lacks."""
    if name not in fit.params:
        return 0.0
    delta = fit.params[name] - fit.start_params[name]
    return delta if name in LENGTH_PARAMS else wrapped_deg(delta)


def assess(fit: GroundBoardFit, cfg: GroundBoardFitConfig) -> tuple[list[dict], list[dict]]:
    """Refusals (physically implausible, decision 324: the result cannot be accepted) and warnings (advisory limits).

    Refusals: a non-finite parameter, placement or score (`non_finite`); a non-finite sigma or accuracy example
    (`accuracy_unknown`: the Jacobian condition over `MAX_CONDITION` or no dof); the head mount z moved over
    `max_mount_move_mm` (`mount_move_mm`); the mount roll or pitch turned over `max_mount_turn_deg`
    (`mount_turn_deg`); the tilt zero or roll changed over `max_tilt_change_deg` (`tilt_change_deg`); the camera
    mount moved over `max_camera_move_mm` (`camera_move_mm`) or turned over `max_camera_turn_deg`
    (`camera_turn_deg`); the pan axis moved over `max_pan_axis_move_mm` (`pan_axis_move_mm`); a placement farther
    than `max_board_distance_m` from the head mount (`board_distance_m:<placement>`). Warnings: `rms_px`,
    `holdout_rms_px`, `max_sigma_mm`, `max_sigma_deg`, `min_views`, `min_poses`, `min_placements`,
    `min_pan_span_deg`, `min_tilt_span_deg`, `floor_height_rms_mm`, `floor_normal_rms_deg` (the depth cross-check
    under the fitted chain), `no_depth_planes` and `no_holdout`.

    Args:
        fit: Fit result.
        cfg: Fit config (`bounds`, `limits`).

    Returns:
        tuple[list[dict], list[dict]]: (refusals, warnings), each entry {name, value, limit, unit, message}.
    """
    bounds, limits = cfg.bounds, cfg.limits
    refusals: list[dict] = []
    numbers = [
        *fit.params.values(),
        fit.rms_px,
        *(float(row[key]) for row in fit.placements for key in ("x_m", "y_m", "yaw_deg")),
    ]
    if not all(math.isfinite(v) for v in numbers):
        refusals.append(
            check_entry("non_finite", math.nan, math.nan, "", "the solver gave non-finite values; capture again")
        )
    uncertain = [*(fit.sigma.get(name, math.inf) for name in fit.free), *(e.sigma for e in fit.accuracy)]
    if not all(math.isfinite(v) for v in uncertain):
        refusals.append(
            check_entry(
                "accuracy_unknown",
                math.nan,
                math.nan,
                "",
                "the estimated accuracy is unknown (the parameters are not all observable from these views); capture "
                "again with more poses per placement",
            )
        )
    camera = [name for name in CAMERA_MOUNT_PARAMS if name in fit.params]
    checks = [
        (
            "mount_move_mm",
            abs(change(fit, "mount_z_m")) * MM_PER_M,
            bounds.max_mount_move_mm,
            MM,
            "head mount height change",
        ),
        (
            "mount_turn_deg",
            max(abs(change(fit, "mount_roll_deg")), abs(change(fit, "mount_pitch_deg"))),
            bounds.max_mount_turn_deg,
            DEG,
            "head mount roll or pitch change",
        ),
        (
            "tilt_change_deg",
            max(abs(change(fit, "tilt_zero_deg")), abs(change(fit, "tilt_roll_deg"))),
            bounds.max_tilt_change_deg,
            DEG,
            "tilt zero or tilt roll change",
        ),
    ]
    if camera:
        checks += [
            (
                "camera_move_mm",
                math.hypot(change(fit, "camera_x_m"), change(fit, "camera_z_m")) * MM_PER_M,
                bounds.max_camera_move_mm,
                MM,
                "camera mount move",
            ),
            (
                "camera_turn_deg",
                max(abs(change(fit, "camera_roll_deg")), abs(change(fit, "camera_yaw_deg"))),
                bounds.max_camera_turn_deg,
                DEG,
                "camera mount turn",
            ),
        ]
    if PAN_AXIS_PARAMS[0] in fit.params:
        checks.append(
            (
                "pan_axis_move_mm",
                math.hypot(change(fit, "pan_axis_x_m"), change(fit, "pan_axis_y_m")) * MM_PER_M,
                bounds.max_pan_axis_move_mm,
                MM,
                "pan axis move",
            )
        )
    mount = (fit.params["mount_x_m"], fit.params["mount_y_m"])
    for row in fit.placements:
        distance = math.hypot(float(row["x_m"]) - mount[0], float(row["y_m"]) - mount[1])
        checks.append(
            (
                f"board_distance_m:{row['placement']}",
                distance,
                bounds.max_board_distance_m,
                "m",
                f"board placement {row['placement']} distance from the head",
            )
        )
    for name, value, limit, unit, what in checks:
        if math.isfinite(value) and not value <= limit:
            refusals.append(
                check_entry(
                    name,
                    value,
                    limit,
                    unit,
                    f"{what} {number(value, '.3g')} {unit} is over the {limit:g} {unit} plausibility bound",
                )
            )

    warnings: list[dict] = []
    lengths = [fit.sigma[name] * MM_PER_M for name in fit.free if name in LENGTH_PARAMS]
    angles = [fit.sigma[name] for name in fit.free if name not in LENGTH_PARAMS]
    fitted = fit.cross_check["fitted"]
    upper = [
        ("rms_px", fit.rms_px, limits.rms_px, "px", "fit corner RMS"),
        ("holdout_rms_px", fit.holdout_rms_px, limits.holdout_rms_px, "px", "held-out corner RMS"),
        ("max_sigma_mm", max(lengths, default=0.0), limits.max_sigma_mm, MM, "largest length sigma"),
        ("max_sigma_deg", max(angles, default=0.0), limits.max_sigma_deg, DEG, "largest angle sigma"),
        (
            "floor_height_rms_mm",
            float(fitted["height_rms_mm"]),
            limits.max_floor_height_rms_mm,
            MM,
            "depth floor plane height RMS under the fitted chain",
        ),
        (
            "floor_normal_rms_deg",
            float(fitted["normal_rms_deg"]),
            limits.max_floor_normal_rms_deg,
            DEG,
            "depth floor plane normal RMS under the fitted chain",
        ),
    ]
    for name, value, limit, unit, what in upper:
        if not math.isfinite(value) and name.startswith("max_sigma"):
            warnings.append(check_entry(name, value, limit, unit, f"{what} is unknown (singular fit)"))
        elif math.isfinite(value) and value > limit:
            warnings.append(
                check_entry(
                    name,
                    value,
                    limit,
                    unit,
                    f"{what} {number(value, '.3g')} {unit} is over the advisory {limit:g} {unit}",
                )
            )
    lower = [
        ("min_views", fit.views, limits.min_views, "views", "fit views"),
        ("min_poses", fit.pose_count, limits.min_poses, "poses", "head poses"),
        ("min_placements", len(fit.placements), limits.min_placements, "placements", "board placements"),
        ("min_pan_span_deg", fit.pan_span_deg, limits.min_pan_span_deg, DEG, "pan span"),
        ("min_tilt_span_deg", fit.tilt_span_deg, limits.min_tilt_span_deg, DEG, "tilt span"),
    ]
    for name, value, limit, unit, what in lower:
        if value < limit:
            warnings.append(
                check_entry(
                    name, value, limit, unit, f"{what} {number(value, '.3g')} are fewer than the advisory {limit:g}"
                )
            )
    if fit.cross_check["planes"] == 0:
        warnings.append(
            check_entry("no_depth_planes", 0, 1, "planes", "no depth floor plane: the depth cross-check did not run")
        )
    if fit.holdout_views == 0:
        warnings.append(check_entry("no_holdout", 0, 1, "views", "no held-out views: the result is not validated"))
    return refusals, warnings


def param_entries(fit: GroundBoardFit) -> list[dict]:
    """The report parameter list: every head parameter of the model in plain units.

    Args:
        fit: Fit result.

    Returns:
        list[dict]: {name, value, sigma, unit, applied_value, change}; lengths in mm, angles in deg; sigma None for
        a gauge parameter (kept at the applied value).
    """
    entries = []
    for name in fit.names:
        scale, unit = (MM_PER_M, MM) if name in LENGTH_PARAMS else (1.0, DEG)
        sigma = fit.sigma.get(name)
        entries.append(
            {
                "name": name,
                "value": json_float(fit.params[name] * scale),
                "sigma": None if sigma is None else json_float(sigma * scale),
                "unit": unit,
                "applied_value": json_float(fit.start_params[name] * scale),
                "change": json_float(change(fit, name) * scale),
            }
        )
    return entries


def plain(values: Mapping[str, object]) -> dict[str, object]:
    """A dict with every float made strict JSON (non-finite as None)."""
    return {key: json_float(v) if isinstance(v, float) else v for key, v in values.items()}


def ground_board_report(fit: GroundBoardFit, cfg: GroundBoardFitConfig, applied: dict | None = None) -> dict:
    """The quality report of the final gate, plain JSON types.

    Args:
        fit: Fit result.
        cfg: Fit config (bounds and advisory limits).
        applied: The head chain in effect ({source, session, result, ...}), None when only the repo URDF is; reported
            as comparison.applied.

    Returns:
        dict: {plausible, refusals, warnings, score {rms_px, heldout_rms_px, views, poses, placements,
        floor_height_rms_mm, floor_normal_rms_deg}, params, placements, accuracy_examples [{label, value, unit}] (1
        sigma and 95 % per example), comparison {applied, current_score {rms_px, heldout_rms_px}, shift_examples
        [{label, value, unit}]}, cross_check, poses, notes}.
    """
    refusals, warnings = assess(fit, cfg)
    examples = []
    for example in fit.accuracy:
        examples.append(
            {"label": f"{example.label}{SIGMA_SUFFIX}", "value": json_float(example.sigma), "unit": example.unit}
        )
        examples.append(
            {"label": f"{example.label}{P95_SUFFIX}", "value": json_float(example.p95), "unit": example.unit}
        )
    fitted = fit.cross_check["fitted"]
    return {
        "plausible": not refusals,
        "refusals": refusals,
        "warnings": warnings,
        "score": {
            "rms_px": json_float(fit.rms_px),
            "heldout_rms_px": json_float(fit.holdout_rms_px),
            "views": int(fit.views + fit.holdout_views),
            "poses": int(fit.pose_count),
            "placements": len(fit.placements),
            "floor_height_rms_mm": json_float(float(fitted["height_rms_mm"])),
            "floor_normal_rms_deg": json_float(float(fitted["normal_rms_deg"])),
        },
        "params": param_entries(fit),
        "placements": [plain(row) for row in fit.placements],
        "accuracy_examples": examples,
        "comparison": {
            "applied": applied,
            "current_score": {
                "rms_px": json_float(fit.current.rms_px),
                "heldout_rms_px": json_float(fit.current.holdout_rms_px),
            },
            "shift_examples": [
                {"label": example.label, "value": json_float(example.shift), "unit": example.unit}
                for example in fit.accuracy
            ],
        },
        "cross_check": {
            "planes": int(fit.cross_check["planes"]),
            "poses": int(fit.cross_check["poses"]),
            "fitted": plain(fit.cross_check["fitted"]),
            "applied": plain(fit.cross_check["applied"]),
        },
        "poses": [plain(row) for row in fit.poses],
        "notes": list(fit.notes),
    }


def fitted_origins_json(fit: GroundBoardFit) -> dict[str, dict[str, list[float]]]:
    """The fitted origins as {joint: {xyz, rpy}} (m, rad)."""
    return {
        name: {"xyz": [float(v) for v in o.xyz], "rpy": [float(v) for v in o.rpy]} for name, o in fit.origins.items()
    }
