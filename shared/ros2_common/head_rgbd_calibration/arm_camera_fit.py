"""Arm to camera fit (user decisions 315, 324, 325, 326, m577, m578): the ChArUco board held in the gripper, seen by
the head D435 colour camera through the FIXED applied head chain, gives the arm calibration in the head camera's
frame: the arm joint offsets, the arm base in base_link, the board seat in the gripper and a gravity sag.

Model per view: board corner -> board pose in gripper_link (the CAD `board_mount` moved by a fitted 6-DOF seat, then
turned about the grip axis by the sag) -> arm FK (measured joints plus the fitted offsets) -> arm base in base_link
(fitted x, y and, with `fit_base_z`, z) -> inverse of the camera pose of the applied head chain at the measured
pan/tilt (fixed: no head parameter is fitted, no floor session is read) -> `cv2.projectPoints` with the D435 colour
intrinsics.

Gauges (exact degeneracies, one side fixed at the applied value): the arm base yaw (a yaw turn about the base_link z
axis is exactly a shoulder_pan offset change plus a base x, y shift, because the shoulder_pan axis is vertical and sits
38.8 mm off the arm root), and the wrist_roll offset unless `fit_wrist_roll` (a wrist_roll zero change is exactly a
board turn about the wrist_roll axis, so only the board seat prior can pin it).

Sag: the board turns about the grip axis (the jaw closing axis, through the grip point) by `sag` (rad per N m) times
the gravity torque of the board about that axis at the view's arm pose (board mass at its centre of mass, gravity
-z in base_link); with `fit_droop` it also moves along gravity by `droop` (m per N m) times the absolute torque. Both
have priors toward zero, as the seat has one toward the CAD mount; every prior is one residual per component in sigma
units, against the corner residuals divided by `pixel_sigma_px`.

Validation is on held-out arm poses (every `limits.holdout_every`-th distinct arm pose with all its views): corner
RMS and the tool point error (arm FK tool point against the view's solvePnP board pose, mm). Accuracy examples
propagate the parameter covariance `(Jc^T Jc / s^2 + Jp^T Jp)^-1` (Jc the corner rows, s^2 their reduced
chi-square `rc^T rc / (mc - n)`, Jp the prior rows at their configured weight) to the tool point in base_link at three
plain arm poses (1 sigma: the RMS 3D error sqrt(trace); 95 %: the 95 % chi-square radius of the largest principal
axis, an upper bound of the 95 % error radius). numpy, scipy and OpenCV only, no ROS.
"""

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from scipy.stats import chi2

from ros2_common.head_rgbd_calibration.arm_offsets_fit import OFFSET_DECIMALS, SNIPPET_INDENT, gripper_frame_pose
from ros2_common.head_rgbd_calibration.chain import UrdfChain, arm_in_robot_base, pose_from_xyz_rpy, xyz_rpy_from_pose
from ros2_common.head_rgbd_calibration.charuco import board_corners_3d
from ros2_common.head_rgbd_calibration.config import ARM_JOINT_NAMES, HeadArmFitConfig
from ros2_common.head_rgbd_calibration.floor_fit import PAN_JOINT, TILT_JOINT, json_float
from ros2_common.head_rgbd_calibration.head_arm_fit import (
    D435_COLOR_FRAME,
    DEG_PER_RAD,
    GRIPPER_LINK,
    MM_PER_M,
    SOLVER_TOLERANCE,
    Correlation,
    board_mount_pose,
    board_pnp,
    camera_pose,
    initial_board_mount,
    largest_correlation,
    number,
    numeric_jacobian,
    rms_px,
)
from ros2_common.head_rgbd_calibration.views import ArmBoardView, require_joints

GRAVITY_M_PER_S2 = 9.80665
GRAVITY_DIRECTION = np.array([0.0, 0.0, -1.0])  # base_link z is up
MIN_VIEWS = 3
PRIOR_ROWS = 8  # board seat rotation (3) and translation (3), sag, droop
PNP_INIT_VIEWS = 10
WRIST_ROLL = "wrist_roll"
PAN_GAUGE_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")
OFFSET_SUFFIX = "_offset"
BASE_KEYS = ("x", "y", "z", "yaw")
BOARD_ROTATION_PARAMS = ("board_rx", "board_ry", "board_rz")
BOARD_POSITION_PARAMS = ("board_x", "board_y", "board_z")
# The seat turn against the CAD mount in the board frame (skew.rpy_deg); its sigma is the rotation vector's, equal to
# first order for the small turns the bounds allow.
BOARD_REPORT_NAMES = ("board_skew_roll", "board_skew_pitch", "board_skew_yaw")
SAG_PARAM = "sag"
DROOP_PARAM = "droop"
RPY_SEQUENCE = "xyz"
P95_3D_FACTOR = math.sqrt(float(chi2.ppf(0.95, 3)))
DEG = "deg"
MM = "mm"
SAG_UNIT = "deg/(N m)"
DROOP_UNIT = "mm/(N m)"
GRID_PAN_STEPS = 9
GRID_JOINT_STEPS = 15
REACH_HEIGHT_BAND_M = 0.05
REACH_LABEL = "tool point at {reach:g} cm reach"
FULL_REACH_LABEL = "tool point at full reach"
CAMERA_LABEL = "gripper position seen by the head camera at {distance:g} cm"
SIGMA_SUFFIX = ", 1 sigma"
P95_SUFFIX = ", 95 %"
CM_PER_M = 100.0
YAW_GAUGE_NOTE = (
    "gauge: arm base yaw fixed at the applied value (a yaw turn equals a shoulder_pan offset change plus a base x, y "
    "shift)"
)
ROLL_GAUGE_NOTE = (
    "gauge: wrist_roll offset fixed at the applied value (a wrist_roll zero change equals a board turn about the "
    "wrist_roll axis)"
)
ROLL_PRIOR_NOTE = "wrist_roll offset fitted: only the board seat prior pins it (a board turn about its axis)"
BASE_Z_FIXED_NOTE = "arm base z fixed at the applied value"
HEAD_FIXED_NOTE = "head chain fixed: the applied calibration, not refitted"
TOOL_NOTE = "tool point: the applied tool_offset_m in gripper_frame_link"


@dataclass(frozen=True)
class AccuracyExample:
    """Estimated tool point error at one plain arm pose.

    Attributes:
        label: What the example is (e.g. "tool point at 30 cm reach").
        joints: Measured arm joints of the pose (rad).
        sigma_mm: RMS 3D error, sqrt of the trace of the propagated covariance (mm); inf when unknown.
        p95_mm: 95 % chi-square radius along the largest principal axis (mm); inf when unknown.
        shift_mm: Distance between the tool point of the applied and the fitted calibration at the pose (mm).
    """

    label: str
    joints: dict[str, float]
    sigma_mm: float
    p95_mm: float
    shift_mm: float


@dataclass(frozen=True)
class CurrentScore:
    """The applied offsets and base on this session (board seat and sag refitted): fit RMS (px), held-out RMS (px)
    and held-out tool error (mm)."""

    rms_px: float
    holdout_rms_px: float
    holdout_tool_mm: float


@dataclass(frozen=True)
class ArmCameraFit:
    """Result of `fit_arm_camera`.

    Attributes:
        offsets: All five arm joint offsets (rad); a fixed one is its applied value.
        current_offsets: The applied offsets the fit started from (rad).
        base: Arm base in base_link {x, y, z (m), yaw (rad)}; a fixed key is its applied value.
        current_base: The applied arm base.
        board_mount: Fitted board seat (without sag) in gripper_link, `{"xyz": [m] * 3, "rpy": [rad] * 3}`.
        cad_mount: The CAD board mount (`cfg.board_mount`), same layout.
        skew: Seat against the CAD mount: `xyz_mm` (gripper_link) and `rpy_deg` (rotation CAD to seat, board frame).
        sag: deg_per_nm, sigma_deg_per_nm, reference_torque_nm, at_reference_deg (sag turn at the reference torque),
            range_deg ([min, max] over the views), droop_mm_per_nm, sigma_droop_mm_per_nm, droop_at_reference_mm
            (None without `fit_droop`).
        free: Free parameter names in solver order.
        values: Value per free parameter (rad, m, rad per N m, m per N m; board rotation as a rotation vector change).
        sigma: One-sigma uncertainty per free parameter (same units); inf when unknown.
        covariance: Covariance of the free parameters (solver order).
        max_correlation: Free parameter pair with the largest absolute correlation.
        condition_number: Condition number of the column-normalised Jacobian at the solution.
        rms_px: Corner RMS over the fit views (px).
        holdout_rms_px: Corner RMS over the held-out views (px); nan without held-out views.
        holdout_tool_mm: Held-out tool point error RMS (mm); nan without held-out views.
        views: Fit views.
        holdout_views: Held-out views.
        arm_poses: Distinct arm poses over all views.
        accuracy: The accuracy examples, in `example_poses` order.
        current: The applied calibration scored on this session.
        notes: Gauge and model notes.
        solver: Solver summary {status, message, nfev, cost, start_rms_px, free}.
    """

    offsets: dict[str, float]
    current_offsets: dict[str, float]
    base: dict[str, float]
    current_base: dict[str, float]
    board_mount: dict[str, list[float]]
    cad_mount: dict[str, list[float]]
    skew: dict[str, list[float]]
    sag: dict[str, float | list[float] | None]
    free: tuple[str, ...]
    values: dict[str, float]
    sigma: dict[str, float]
    covariance: np.ndarray
    max_correlation: Correlation
    condition_number: float
    rms_px: float
    holdout_rms_px: float
    holdout_tool_mm: float
    views: int
    holdout_views: int
    arm_poses: int
    accuracy: list[AccuracyExample]
    current: CurrentScore
    notes: list[str]
    solver: dict[str, object]

    @property
    def shifts(self) -> list[tuple[str, float]]:
        """(label, shift mm) of every accuracy example: how far the fit moves the tool point there."""
        return [(example.label, example.shift_mm) for example in self.accuracy]


def offset_name(joint: str) -> str:
    """Parameter name of an arm joint offset (`<joint>_offset`)."""
    return f"{joint}{OFFSET_SUFFIX}"


def gripper_in_base(
    arm: UrdfChain, joints: Mapping[str, float], offsets: Mapping[str, float], base: Mapping[str, float]
) -> np.ndarray:
    """Pose of gripper_link in robot base_link.

    Args:
        arm: Arm URDF chain.
        joints: Measured arm joints (rad).
        offsets: Arm joint offsets (rad), urdf = measured + offset.
        base: Arm base in base_link {x, y, z (m), yaw (rad)}.

    Returns:
        np.ndarray: 4x4 pose.
    """
    return arm_in_robot_base(base) @ arm.transform(GRIPPER_LINK, joints, offsets=offsets)


def prediction_error_mm(jac: np.ndarray, cov: np.ndarray) -> tuple[float, float]:
    """Propagate a parameter covariance to a 3D point.

    Args:
        jac: (3, n) Jacobian of the point (mm) by the parameters.
        cov: (n, n) parameter covariance.

    Returns:
        tuple[float, float]: (RMS 3D error sqrt(trace), 95 % chi-square radius of the largest principal axis) in mm;
        (inf, inf) when the covariance is not finite.
    """
    if not np.all(np.isfinite(cov)):
        return math.inf, math.inf
    point = jac @ cov @ jac.T
    largest = float(np.max(np.linalg.eigvalsh((point + point.T) / 2.0)))
    return math.sqrt(max(float(np.trace(point)), 0.0)), P95_3D_FACTOR * math.sqrt(max(largest, 0.0))


def wrapped_deg(value: float) -> float:
    """An angle in degrees wrapped into (-180, 180]."""
    wrapped = math.fmod(value + 180.0, 360.0)
    return (wrapped + 360.0 if wrapped <= 0.0 else wrapped) - 180.0


def free_names(cfg: HeadArmFitConfig) -> tuple[str, ...]:
    """The free parameters of a config, in solver order.

    Args:
        cfg: Fit config (`arm_camera` switches).

    Returns:
        tuple[str, ...]: Offsets (wrist_roll with `fit_wrist_roll`), base x, y (z with `fit_base_z`), the six board seat
        changes, sag (droop with `fit_droop`).
    """
    model = cfg.arm_camera
    joints = (*PAN_GAUGE_JOINTS, WRIST_ROLL) if model.fit_wrist_roll else PAN_GAUGE_JOINTS
    base = ("base_x", "base_y", "base_z") if model.fit_base_z else ("base_x", "base_y")
    droop = (DROOP_PARAM,) if model.fit_droop else ()
    return (
        *(offset_name(j) for j in joints),
        *base,
        *BOARD_ROTATION_PARAMS,
        *BOARD_POSITION_PARAMS,
        SAG_PARAM,
        *droop,
    )


def axis_rotation(cross: np.ndarray, cross_squared: np.ndarray, angle: np.ndarray) -> np.ndarray:
    """Rodrigues rotations about one unit axis, given its cross-product matrix K and K @ K.

    Args:
        cross: 3x3 cross-product matrix of the unit axis.
        cross_squared: `cross @ cross`.
        angle: (n,) rotation angles (rad).

    Returns:
        np.ndarray: (n, 3, 3) rotation matrices.
    """
    angle = np.asarray(angle, dtype=float).reshape(-1, 1, 1)
    return np.eye(3) + np.sin(angle) * cross + (1.0 - np.cos(angle)) * cross_squared


def cross_matrix(axis: np.ndarray) -> np.ndarray:
    """3x3 cross-product matrix of a vector."""
    x, y, z = (float(v) for v in axis)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


class ArmFk:
    """Forward kinematics of one arm link with the joint origins precomputed and many joint sets at once: the fit
    evaluates it for every view tens of thousands of times, `UrdfChain.transform` rebuilds every origin each call."""

    def __init__(self, arm: UrdfChain, link: str = GRIPPER_LINK) -> None:
        """Precompute the path to `link`.

        Args:
            arm: Arm URDF chain.
            link: Target link.
        """
        self.steps: list[tuple[np.ndarray, str | None, np.ndarray, np.ndarray]] = []
        for joint in arm.path(link):
            axis = np.asarray(joint.axis, dtype=float)
            cross = cross_matrix(axis / np.linalg.norm(axis)) if joint.movable else np.zeros((3, 3))
            self.steps.append(
                (pose_from_xyz_rpy(joint.xyz, joint.rpy), joint.name if joint.movable else None, cross, cross @ cross)
            )

    def poses(self, joints: Mapping[str, np.ndarray], offsets: Mapping[str, float]) -> np.ndarray:
        """Poses of the link in the arm root for n joint sets.

        Args:
            joints: (n,) measured angles (rad) per movable joint on the path.
            offsets: Joint offsets (rad), urdf = measured + offset.

        Returns:
            np.ndarray: (n, 4, 4) poses.

        Raises:
            KeyError: A movable joint on the path is missing from `joints`.
        """
        count = len(next(iter(joints.values())))
        pose = np.broadcast_to(np.eye(4), (count, 4, 4))
        for origin, name, cross, cross_squared in self.steps:
            pose = pose @ origin
            if name is not None:
                spin = np.broadcast_to(np.eye(4), (count, 4, 4)).copy()
                angles = np.asarray(joints[name], dtype=float) + offsets.get(name, 0.0)
                spin[:, :3, :3] = axis_rotation(cross, cross_squared, angles)
                pose = pose @ spin
        return pose

    def pose(self, joints: Mapping[str, float], offsets: Mapping[str, float]) -> np.ndarray:
        """Pose of the link in the arm root (equal to `UrdfChain.transform(link, joints, offsets=offsets)`).

        Args:
            joints: Measured joint angles (rad).
            offsets: Joint offsets (rad), urdf = measured + offset.

        Returns:
            np.ndarray: 4x4 pose.
        """
        return self.poses({name: np.array([value]) for name, value in joints.items()}, offsets)[0]


class ArmCameraProblem:
    """The residual model of one session: precomputed camera poses (the head is fixed), corners and the board mass
    model; parameters are a dict of every model parameter (offsets, base, board seat change, sag, droop)."""

    def __init__(
        self,
        views: Sequence[ArmBoardView],
        camera_matrix: np.ndarray,
        dist: np.ndarray,
        head_urdf: str,
        arm_urdf: str,
        cfg: HeadArmFitConfig,
        camera_frame: str = D435_COLOR_FRAME,
    ) -> None:
        """Precompute what does not change during the fit.

        Args:
            views: Board views with all seven measured joints.
            camera_matrix: 3x3 colour intrinsics.
            dist: Distortion coefficients.
            head_urdf: Head URDF text with the applied chain.
            arm_urdf: Arm URDF text.
            cfg: Fit config.
            camera_frame: Optical frame of the camera in the head URDF.
        """
        self.cfg = cfg
        self.model = cfg.arm_camera
        self.views = list(views)
        self.k = np.asarray(camera_matrix, dtype=float)
        self.d = np.asarray(dist, dtype=float)
        self.head = UrdfChain.from_text(head_urdf)
        self.arm = UrdfChain.from_text(arm_urdf)
        self.fk = ArmFk(self.arm)
        self.joint_angles = {
            name: np.array([v.joints[name] for v in self.views], dtype=float) for name in ARM_JOINT_NAMES
        }
        self.camera_frame = camera_frame
        self.cameras = [camera_pose(self.head, v.joints, None, camera_frame) for v in self.views]
        self.inverse_cameras = [np.linalg.inv(c) for c in self.cameras]
        corners = board_corners_3d(cfg.board)
        self.objects = [corners[np.asarray(v.detection.ids, dtype=int)] for v in self.views]
        self.images = [np.asarray(v.detection.points, dtype=float).reshape(-1, 2) for v in self.views]
        self.cad = pose_from_xyz_rpy(cfg.board_mount.xyz_init, cfg.board_mount.rpy_init)
        self.current_offsets = {name: float(cfg.joint_offsets_rad[name]) for name in ARM_JOINT_NAMES}
        self.current_base = {key: float(v) for key, v in cfg.base_in_base_link.model_dump().items()}
        self.grip = np.asarray(self.model.grip_point_m, dtype=float)
        self.axis = np.asarray(self.model.grip_axis, dtype=float) / np.linalg.norm(self.model.grip_axis)
        self.axis_cross = cross_matrix(self.axis)
        self.axis_cross_squared = self.axis_cross @ self.axis_cross
        self.lever = np.asarray(self.model.board_com_m, dtype=float) - self.grip
        self.weight_n = self.model.board_mass_kg * GRAVITY_M_PER_S2
        perpendicular = self.lever - (self.lever @ self.axis) * self.axis
        self.reference_torque = self.weight_n * float(np.linalg.norm(perpendicular))
        self.tool_gripper = gripper_frame_pose(self.arm) @ np.append(np.asarray(cfg.tool_offset_m, dtype=float), 1.0)
        arm_poses = sorted({v.arm_pose for v in self.views})
        held = set(arm_poses[cfg.limits.holdout_every - 1 :: cfg.limits.holdout_every])
        self.arm_poses = len(arm_poses)
        self.fit_idx = [i for i, v in enumerate(self.views) if v.arm_pose not in held]
        self.hold_idx = [i for i, v in enumerate(self.views) if v.arm_pose in held]

    def start(self) -> dict[str, float]:
        """Start parameters: the applied offsets and base, the board seat from solvePnP of the first fit views, no sag.

        Returns:
            dict[str, float]: Value per model parameter.
        """
        init = self.fit_idx[:PNP_INIT_VIEWS]
        grippers = [
            gripper_in_base(self.arm, self.views[i].joints, self.current_offsets, self.current_base) for i in init
        ]
        mount = initial_board_mount(
            self.head,
            {},
            [self.views[i] for i in init],
            grippers,
            [self.objects[i] for i in init],
            [self.images[i] for i in init],
            self.k,
            self.d,
            self.cad,
            self.camera_frame,
        )
        rotation = Rotation.from_matrix(self.cad[:3, :3].T @ mount[:3, :3]).as_rotvec()
        shift = mount[:3, 3] - self.cad[:3, 3]
        return {
            **{offset_name(j): self.current_offsets[j] for j in ARM_JOINT_NAMES},
            **{f"base_{key}": self.current_base[key] for key in BASE_KEYS},
            **dict(zip(BOARD_ROTATION_PARAMS, (float(v) for v in rotation), strict=True)),
            **dict(zip(BOARD_POSITION_PARAMS, (float(v) for v in shift), strict=True)),
            SAG_PARAM: 0.0,
            DROOP_PARAM: 0.0,
        }

    def offsets(self, p: Mapping[str, float]) -> dict[str, float]:
        """Arm joint offsets of a parameter set (rad)."""
        return {j: float(p[offset_name(j)]) for j in ARM_JOINT_NAMES}

    def base(self, p: Mapping[str, float]) -> dict[str, float]:
        """Arm base of a parameter set {x, y, z, yaw}."""
        return {key: float(p[f"base_{key}"]) for key in BASE_KEYS}

    def seat(self, p: Mapping[str, float]) -> np.ndarray:
        """Board seat (without sag) in gripper_link of a parameter set."""
        delta = np.array([p[name] for name in (*BOARD_ROTATION_PARAMS, *BOARD_POSITION_PARAMS)], dtype=float)
        return board_mount_pose(self.cad, delta)

    def torques(self, seat: np.ndarray, grippers: np.ndarray) -> np.ndarray:
        """Gravity torque of the board about the grip axis (N m) at each gripper pose.

        Args:
            seat: 4x4 board seat in gripper_link.
            grippers: (n, 4, 4) gripper_link poses in base_link.

        Returns:
            np.ndarray: (n,) torques about the grip axis (board frame axis direction).
        """
        weights = np.einsum("nji,j->ni", grippers[:, :3, :3], GRAVITY_DIRECTION * self.weight_n)
        return np.cross(seat[:3, :3] @ self.lever, weights) @ (seat[:3, :3] @ self.axis)

    def boards_in_gripper(self, p: Mapping[str, float], seat: np.ndarray, grippers: np.ndarray) -> np.ndarray:
        """Board poses in gripper_link at each gripper pose: the seat turned about the grip axis (through the grip
        point) by the sag times the torque and moved along gravity by the droop times the absolute torque.

        Args:
            p: Parameters (sag, droop).
            seat: 4x4 board seat in gripper_link.
            grippers: (n, 4, 4) gripper_link poses in base_link.

        Returns:
            np.ndarray: (n, 4, 4) poses.
        """
        torque = self.torques(seat, grippers)
        turn = np.broadcast_to(np.eye(4), (len(grippers), 4, 4)).copy()
        turn[:, :3, :3] = axis_rotation(self.axis_cross, self.axis_cross_squared, float(p[SAG_PARAM]) * torque)
        turn[:, :3, 3] = self.grip - turn[:, :3, :3] @ self.grip
        poses = seat @ turn
        down = np.einsum("nji,j->ni", grippers[:, :3, :3], GRAVITY_DIRECTION)
        poses[:, :3, 3] += float(p[DROOP_PARAM]) * np.abs(torque)[:, None] * down
        return poses

    def grippers(self, p: Mapping[str, float], idx: Sequence[int]) -> np.ndarray:
        """gripper_link poses in base_link of the given views, (n, 4, 4)."""
        picked = list(idx)
        joints = {name: angles[picked] for name, angles in self.joint_angles.items()}
        return arm_in_robot_base(self.base(p)) @ self.fk.poses(joints, self.offsets(p))

    def boards(self, p: Mapping[str, float], idx: Sequence[int]) -> tuple[np.ndarray, np.ndarray]:
        """Board poses in base_link and in gripper_link of the given views, each (n, 4, 4)."""
        grippers = self.grippers(p, idx)
        boards = self.boards_in_gripper(p, self.seat(p), grippers)
        return grippers @ boards, boards

    def view_errors(self, p: Mapping[str, float], idx: Sequence[int]) -> list[np.ndarray]:
        """Projected minus detected corners (px) of the given views."""
        errors = []
        picked = list(idx)
        if not picked:
            return errors
        in_base, _ = self.boards(p, picked)
        for i, pose in zip(picked, in_base, strict=True):
            board = self.inverse_cameras[i] @ pose
            rvec = cv2.Rodrigues(board[:3, :3])[0]
            projected = cv2.projectPoints(self.objects[i], rvec, board[:3, 3], self.k, self.d)[0].reshape(-1, 2)
            errors.append(projected - self.images[i])
        return errors

    def priors(self, p: Mapping[str, float]) -> np.ndarray:
        """Prior residuals in sigma units: board seat rotation and translation, sag and droop at the reference torque.

        Args:
            p: Parameters.

        Returns:
            np.ndarray: (8,) residuals.
        """
        rotation = np.degrees([p[name] for name in BOARD_ROTATION_PARAMS]) / self.model.board_prior_deg
        shift = np.array([p[name] for name in BOARD_POSITION_PARAMS]) * MM_PER_M / self.model.board_prior_mm
        sag = math.degrees(p[SAG_PARAM] * self.reference_torque) / self.model.sag_prior_deg
        droop = p[DROOP_PARAM] * self.reference_torque * MM_PER_M / self.model.droop_prior_mm
        return np.concatenate([rotation, shift, [sag, droop]])

    def residuals(self, p: Mapping[str, float]) -> np.ndarray:
        """Fit-view corner residuals over `pixel_sigma_px`, then the priors."""
        corners = [e.ravel() / self.model.pixel_sigma_px for e in self.view_errors(p, self.fit_idx)]
        return np.concatenate([*corners, self.priors(p)])

    def solve(
        self, start: Mapping[str, float], free: tuple[str, ...]
    ) -> tuple[dict[str, float], np.ndarray, dict[str, object]]:
        """Huber least squares over the free parameters, the others held at `start`.

        Args:
            start: Start value of every model parameter.
            free: Names of the free parameters.

        Returns:
            tuple: (solution of every model parameter, free solution vector, solver summary {status, message, nfev,
            cost, start_rms_px, free}).
        """

        def fun(x: np.ndarray) -> np.ndarray:
            return self.residuals({**start, **dict(zip(free, (float(v) for v in x), strict=True))})

        x0 = np.array([start[name] for name in free], dtype=float)
        outcome = least_squares(
            fun,
            x0,
            loss="huber",
            f_scale=self.cfg.huber_f_scale_px / self.model.pixel_sigma_px,
            x_scale="jac",
            ftol=SOLVER_TOLERANCE,
            xtol=SOLVER_TOLERANCE,
            gtol=SOLVER_TOLERANCE,
        )
        x = outcome.x
        summary: dict[str, object] = {
            "status": int(outcome.status),
            "message": str(outcome.message),
            "nfev": int(outcome.nfev),
            "cost": float(outcome.cost),
            "start_rms_px": rms_px(self.view_errors(start, self.fit_idx)),
            "free": list(free),
        }
        return {**start, **dict(zip(free, (float(v) for v in x), strict=True))}, x, summary

    def holdout_tool_mm(self, p: Mapping[str, float]) -> float:
        """Held-out tool error: RMS over the held-out views of the distance between the arm FK tool point and the tool
        point of the view's solvePnP board pose (mm), over the views solvePnP solves; nan without any."""
        errors = []
        if not self.hold_idx:
            return math.nan
        in_base, in_gripper = self.boards(p, self.hold_idx)
        for i, predicted, board in zip(self.hold_idx, in_base, in_gripper, strict=True):
            observed = board_pnp(self.objects[i], self.images[i], self.k, self.d, self.inverse_cameras[i] @ predicted)
            if observed is None:
                continue  # a view solvePnP cannot solve is left out of the tool error
            tool_board = np.linalg.solve(board, self.tool_gripper)
            delta = (self.cameras[i] @ observed @ tool_board - predicted @ tool_board)[:3]
            errors.append(float(np.linalg.norm(delta)) * MM_PER_M)
        return float(np.sqrt(np.mean(np.square(errors)))) if errors else math.nan

    def score(self, p: Mapping[str, float]) -> CurrentScore:
        """Fit RMS, held-out RMS and held-out tool error of a parameter set."""
        errors = self.view_errors(p, range(len(self.views)))
        return CurrentScore(
            rms_px([errors[i] for i in self.fit_idx]),
            rms_px([errors[i] for i in self.hold_idx]),
            self.holdout_tool_mm(p),
        )

    def tool_point(self, p: Mapping[str, float], joints: Mapping[str, float]) -> np.ndarray:
        """Tool point in base_link (m) at measured joints."""
        return (arm_in_robot_base(self.base(p)) @ self.fk.pose(joints, self.offsets(p)) @ self.tool_gripper)[:3]


def example_poses(problem: ArmCameraProblem) -> list[tuple[str, dict[str, float]]]:
    """The plain arm poses of the accuracy examples, picked from a joint grid (pan, lift, elbow over their URDF limits,
    wrist_flex and wrist_roll zero) with the applied calibration.

    Args:
        problem: The session problem (arm, applied offsets and base, head chain, views, `arm_camera` settings).

    Returns:
        list[tuple[str, dict[str, float]]]: (label, measured joints) for the tool point at `reach_m` (pan zero, height
        nearest the shoulder_pan origin), at full reach (pan zero, the largest reach within `REACH_HEIGHT_BAND_M` of
        that height) and the pose whose tool point is nearest the point `camera_distance_m` along the optical axis at
        the median head angles of the views.
    """
    model = problem.model
    arm = problem.arm
    applied = {**{offset_name(j): problem.current_offsets[j] for j in ARM_JOINT_NAMES}}
    applied.update({f"base_{key}": problem.current_base[key] for key in BASE_KEYS})

    def span(joint: str, steps: int) -> np.ndarray:
        low, high = arm.limits(joint) or (-math.pi, math.pi)
        return np.linspace(low, high, steps)

    pan_xyz = np.asarray(arm.origin("shoulder_pan")[0], dtype=float)
    arm_frame = {**applied, "base_x": 0.0, "base_y": 0.0, "base_z": 0.0, "base_yaw": 0.0}
    planar = []
    for lift in span("shoulder_lift", GRID_JOINT_STEPS):
        for elbow in span("elbow_flex", GRID_JOINT_STEPS):
            joints = {
                "shoulder_pan": 0.0,
                "shoulder_lift": lift,
                "elbow_flex": elbow,
                "wrist_flex": 0.0,
                WRIST_ROLL: 0.0,
            }
            tool = problem.tool_point(arm_frame, joints)
            planar.append((joints, math.hypot(tool[0] - pan_xyz[0], tool[1] - pan_xyz[1]), tool[2] - pan_xyz[2]))
    at_reach = min(planar, key=lambda entry: (entry[1] - model.reach_m) ** 2 + entry[2] ** 2)[0]
    band = [entry for entry in planar if abs(entry[2]) <= REACH_HEIGHT_BAND_M] or planar
    full = max(band, key=lambda entry: entry[1])[0]

    pan = float(np.median([v.joints[PAN_JOINT] for v in problem.views]))
    tilt = float(np.median([v.joints[TILT_JOINT] for v in problem.views]))
    camera = camera_pose(problem.head, {PAN_JOINT: pan, TILT_JOINT: tilt}, None, problem.camera_frame)
    target = camera[:3, 3] + model.camera_distance_m * camera[:3, 2]
    best: tuple[float, dict[str, float]] | None = None
    for pan_joint in span("shoulder_pan", GRID_PAN_STEPS):
        for lift in span("shoulder_lift", GRID_JOINT_STEPS):
            for elbow in span("elbow_flex", GRID_JOINT_STEPS):
                joints = {
                    "shoulder_pan": pan_joint,
                    "shoulder_lift": lift,
                    "elbow_flex": elbow,
                    "wrist_flex": 0.0,
                    WRIST_ROLL: 0.0,
                }
                distance = float(np.linalg.norm(problem.tool_point(applied, joints) - target))
                if best is None or distance < best[0]:
                    best = (distance, joints)
    assert best is not None
    return [
        (REACH_LABEL.format(reach=model.reach_m * CM_PER_M), at_reach),
        (FULL_REACH_LABEL, full),
        (CAMERA_LABEL.format(distance=model.camera_distance_m * CM_PER_M), best[1]),
    ]


def accuracy_examples(
    problem: ArmCameraProblem, solution: Mapping[str, float], free: tuple[str, ...], cov: np.ndarray
) -> list[AccuracyExample]:
    """Propagate the covariance to the tool point at each example pose.

    Args:
        problem: The session problem.
        solution: Fitted value of every model parameter.
        free: Free parameter names (covariance order).
        cov: Covariance of the free parameters.

    Returns:
        list[AccuracyExample]: One per `example_poses` entry.
    """
    applied = {
        **solution,
        **{offset_name(j): problem.current_offsets[j] for j in ARM_JOINT_NAMES},
        **{f"base_{key}": problem.current_base[key] for key in BASE_KEYS},
    }
    examples = []
    for label, joints in example_poses(problem):

        def point_mm(x: np.ndarray, joints: dict[str, float] = joints) -> np.ndarray:
            return problem.tool_point({**solution, **dict(zip(free, x, strict=True))}, joints) * MM_PER_M

        x = np.array([solution[name] for name in free], dtype=float)
        sigma, p95 = prediction_error_mm(numeric_jacobian(point_mm, x), cov)
        shift = float(np.linalg.norm(problem.tool_point(solution, joints) - problem.tool_point(applied, joints)))
        examples.append(AccuracyExample(label, joints, sigma, p95, shift * MM_PER_M))
    return examples


def fit_arm_camera(
    views: Sequence[ArmBoardView],
    camera_matrix: np.ndarray,
    dist: np.ndarray,
    head_urdf: str,
    arm_urdf: str,
    cfg: HeadArmFitConfig,
    camera_frame: str = D435_COLOR_FRAME,
) -> ArmCameraFit:
    """Fit the arm calibration against the fixed applied head chain.

    The applied calibration the fit starts from and compares against is `cfg.joint_offsets_rad` and
    `cfg.base_in_base_link`; the tool point is `cfg.tool_offset_m`. Free: `free_names(cfg)`. Every
    `cfg.limits.holdout_every`-th distinct arm pose is held out with all its views.

    Args:
        views: Board views with all seven measured joints.
        camera_matrix: 3x3 colour intrinsics (the D435 factory camera_info).
        dist: Distortion coefficients.
        head_urdf: Head URDF text with the applied chain (fixed).
        arm_urdf: Arm URDF text.
        cfg: Fit config (board, board_mount, joint_offsets_rad, base_in_base_link, tool_offset_m, limits, arm_camera).
        camera_frame: Optical frame of the camera in the head URDF.

    Returns:
        ArmCameraFit: Fitted calibration, uncertainties, scores, accuracy examples and the applied one's score.

    Raises:
        ValueError: Fewer than 3 views or a view lacks a joint.
    """
    if len(views) < MIN_VIEWS:
        raise ValueError(f"the arm to camera fit needs at least {MIN_VIEWS} views, got {len(views)}")
    for view in views:
        require_joints(view)
    problem = ArmCameraProblem(views, camera_matrix, dist, head_urdf, arm_urdf, cfg, camera_frame)
    start = problem.start()
    free = free_names(cfg)
    solution, x, solver = problem.solve(start, free)

    def fun(vector: np.ndarray) -> np.ndarray:
        return problem.residuals({**solution, **dict(zip(free, (float(v) for v in vector), strict=True))})

    jac = numeric_jacobian(fun, x)
    residual = fun(x)
    corners = len(residual) - PRIOR_ROWS
    dof = corners - len(x)
    cov = np.full((len(x), len(x)), math.inf)
    if dof > 0:
        # The corner rows' reduced chi-square scales only the corner information; the priors keep their weight.
        scale = float(residual[:corners] @ residual[:corners]) / dof
        corner_jac, prior_jac = jac[:corners], jac[corners:]
        try:
            cov = np.linalg.inv(corner_jac.T @ corner_jac / scale + prior_jac.T @ prior_jac)
        except np.linalg.LinAlgError:
            pass
    sigma_x = np.sqrt(np.abs(np.diag(cov)))
    norms = np.linalg.norm(jac, axis=0)
    condition = float(np.linalg.cond(jac / np.where(norms > 0.0, norms, 1.0)))

    current_free = tuple(
        n for n in free if n in (*BOARD_ROTATION_PARAMS, *BOARD_POSITION_PARAMS, SAG_PARAM, DROOP_PARAM)
    )
    current_solution, _, _ = problem.solve(start, current_free)

    seat = problem.seat(solution)
    seat_xyz, seat_rpy = xyz_rpy_from_pose(seat)
    cad_xyz, cad_rpy = xyz_rpy_from_pose(problem.cad)
    skew_rpy = Rotation.from_matrix(problem.cad[:3, :3].T @ seat[:3, :3]).as_euler(RPY_SEQUENCE)
    sigma = {name: float(sigma_x[i]) for i, name in enumerate(free)}
    turns = np.degrees(
        solution[SAG_PARAM] * problem.torques(seat, problem.grippers(solution, range(len(problem.views))))
    )
    droop_free = DROOP_PARAM in free
    sag: dict[str, float | list[float] | None] = {
        "deg_per_nm": math.degrees(solution[SAG_PARAM]),
        "sigma_deg_per_nm": math.degrees(sigma[SAG_PARAM]),
        "reference_torque_nm": problem.reference_torque,
        "at_reference_deg": math.degrees(solution[SAG_PARAM] * problem.reference_torque),
        "range_deg": [float(min(turns)), float(max(turns))],
        "droop_mm_per_nm": solution[DROOP_PARAM] * MM_PER_M if droop_free else None,
        "sigma_droop_mm_per_nm": sigma[DROOP_PARAM] * MM_PER_M if droop_free else None,
        "droop_at_reference_mm": solution[DROOP_PARAM] * problem.reference_torque * MM_PER_M if droop_free else None,
    }
    model = cfg.arm_camera
    notes = [
        HEAD_FIXED_NOTE,
        YAW_GAUGE_NOTE,
        ROLL_PRIOR_NOTE if model.fit_wrist_roll else ROLL_GAUGE_NOTE,
        *([] if model.fit_base_z else [BASE_Z_FIXED_NOTE]),
        TOOL_NOTE,
    ]
    score = problem.score(solution)
    return ArmCameraFit(
        offsets=problem.offsets(solution),
        current_offsets=dict(problem.current_offsets),
        base=problem.base(solution),
        current_base=dict(problem.current_base),
        board_mount={"xyz": list(seat_xyz), "rpy": list(seat_rpy)},
        cad_mount={"xyz": list(cad_xyz), "rpy": list(cad_rpy)},
        skew={
            "xyz_mm": [float(v) for v in (seat[:3, 3] - problem.cad[:3, 3]) * MM_PER_M],
            "rpy_deg": [float(v) for v in np.degrees(skew_rpy)],
        },
        sag=sag,
        free=free,
        values={name: float(x[i]) for i, name in enumerate(free)},
        sigma=sigma,
        covariance=cov,
        max_correlation=largest_correlation(cov, free),
        condition_number=condition,
        rms_px=score.rms_px,
        holdout_rms_px=score.holdout_rms_px,
        holdout_tool_mm=score.holdout_tool_mm,
        views=len(problem.fit_idx),
        holdout_views=len(problem.hold_idx),
        arm_poses=problem.arm_poses,
        accuracy=accuracy_examples(problem, solution, free, cov),
        current=problem.score(current_solution),
        notes=notes,
        solver=solver,
    )


def check_entry(name: str, value: float, limit: float, unit: str, message: str) -> dict[str, float | str | None]:
    """One refusal or warning: {name, value, limit, unit, message} (non-finite numbers as None)."""
    return {"name": name, "value": json_float(value), "limit": json_float(limit), "unit": unit, "message": message}


def assess(fit: ArmCameraFit, cfg: HeadArmFitConfig) -> tuple[list[dict], list[dict]]:
    """Refusals (physically implausible, decision 324: the result cannot be accepted) and warnings (the advisory
    limits of today's pass/fail checks, m578).

    Refusals: a non-finite offset, base or score (`non_finite`); an offset change over `max_offset_change_deg`
    (`offset_change:<joint>`); the base moved over `max_base_move_mm` (`base_move_mm`) or turned over
    `max_base_turn_deg` (`base_turn_deg`); the seat over `max_skew_mm` / `max_skew_deg` (`skew_mm`, `skew_deg`); the
    sag at the reference torque over `max_sag_deg` (`sag_deg`); the droop at the reference torque over `max_droop_mm`
    (`droop_mm`). Warnings: `rms_px`, `holdout_rms_px` (`arm_limits`), `holdout_tool_mm`
    (`arm_limits.max_tool_holdout_mm`), `min_views` (`arm_limits`), `min_arm_poses` (`limits`) and `no_holdout`
    without held-out views. The largest correlation is not a warning: the shoulder_pan offset and the base y correlate
    at about 0.99 by geometry (a pan turn moves the board sideways like a base shift); it is logged with the solver.

    Args:
        fit: Fit result.
        cfg: Fit config (`arm_camera.bounds`, `arm_limits`, `limits`).

    Returns:
        tuple[list[dict], list[dict]]: (refusals, warnings), each entry {name, value, limit, unit, message}.
    """
    bounds = cfg.arm_camera.bounds
    refusals: list[dict] = []
    numbers = [*fit.offsets.values(), *fit.base.values(), fit.rms_px, *fit.skew["xyz_mm"], *fit.skew["rpy_deg"]]
    if not all(math.isfinite(v) for v in numbers):
        refusals.append(
            check_entry("non_finite", math.nan, math.nan, "", "the solver gave non-finite values; capture again")
        )
    for joint in ARM_JOINT_NAMES:
        change = abs(math.degrees(fit.offsets[joint] - fit.current_offsets[joint]))
        if change > bounds.max_offset_change_deg:
            refusals.append(
                check_entry(
                    f"offset_change:{joint}",
                    change,
                    bounds.max_offset_change_deg,
                    DEG,
                    f"{joint} offset change {number(change, '.1f')} deg is over the "
                    f"{bounds.max_offset_change_deg:g} deg plausibility bound",
                )
            )
    move = math.dist([fit.base[k] for k in ("x", "y", "z")], [fit.current_base[k] for k in ("x", "y", "z")]) * MM_PER_M
    turn = abs(wrapped_deg(math.degrees(fit.base["yaw"] - fit.current_base["yaw"])))
    skew_mm = float(np.linalg.norm(fit.skew["xyz_mm"]))
    skew_deg = math.degrees(
        float(np.linalg.norm(Rotation.from_euler(RPY_SEQUENCE, fit.skew["rpy_deg"], degrees=True).as_rotvec()))
    )
    sag_deg = abs(float(fit.sag["at_reference_deg"]))
    droop = fit.sag.get("droop_at_reference_mm")
    checks = [
        ("base_move_mm", move, bounds.max_base_move_mm, MM, "arm base move"),
        ("base_turn_deg", turn, bounds.max_base_turn_deg, DEG, "arm base turn"),
        ("skew_mm", skew_mm, bounds.max_skew_mm, MM, "board seat shift against the CAD mount"),
        ("skew_deg", skew_deg, bounds.max_skew_deg, DEG, "board seat turn against the CAD mount"),
        ("sag_deg", sag_deg, bounds.max_sag_deg, DEG, "board sag at the reference torque"),
        *(
            [("droop_mm", abs(float(droop)), bounds.max_droop_mm, MM, "board droop at the reference torque")]
            if droop is not None
            else []
        ),
    ]
    for name, value, limit, unit, what in checks:
        if not value <= limit:
            refusals.append(
                check_entry(
                    name,
                    value,
                    limit,
                    unit,
                    f"{what} {number(value, '.1f')} {unit} is over the {limit:g} {unit} plausibility bound",
                )
            )
    limits, arm = cfg.limits, cfg.arm_limits
    warnings: list[dict] = []
    upper = [
        ("rms_px", fit.rms_px, arm.rms_px, "px", "fit corner RMS"),
        ("holdout_rms_px", fit.holdout_rms_px, arm.holdout_rms_px, "px", "held-out corner RMS"),
        ("holdout_tool_mm", fit.holdout_tool_mm, arm.max_tool_holdout_mm, MM, "held-out tool point error"),
    ]
    for name, value, limit, unit, what in upper:
        if math.isfinite(value) and value > limit:
            warnings.append(
                check_entry(
                    name,
                    value,
                    limit,
                    unit,
                    f"{what} {number(value, '.3g')} {unit} is over the advisory {limit:g} {unit} limit",
                )
            )
    lower = [
        ("min_views", fit.views, arm.min_views, "views", "fit views"),
        ("min_arm_poses", fit.arm_poses, limits.min_arm_poses, "poses", "arm poses"),
    ]
    for name, value, limit, unit, what in lower:
        if value < limit:
            warnings.append(
                check_entry(name, value, limit, unit, f"{value} {what} are fewer than the advisory {limit}")
            )
    if fit.holdout_views == 0:
        warnings.append(check_entry("no_holdout", 0, 1, "views", "no held-out views: the result is not validated"))
    return refusals, warnings


def param_entries(fit: ArmCameraFit) -> list[dict]:
    """The report parameter list: every offset, the base, the board seat, the sag (and droop), in plain units.

    Args:
        fit: Fit result.

    Returns:
        list[dict]: {name, value, sigma, held, unit, applied_value, change}; held True and sigma None for a fixed
        parameter (a gauge); sigma None with held False for a fitted parameter whose sigma is not finite (a singular
        covariance);
        lengths in mm, angles in deg, the sag in deg/(N m), the droop in mm/(N m); the applied value of the board seat
        is the CAD mount for board_x, y, z, zero for the seat turn `board_skew_roll`, `board_skew_pitch`,
        `board_skew_yaw` (the skew rotation, board frame, as `skew.rpy_deg`), the sag and the droop.
    """

    def entry(name: str, value: float, sigma: float | None, unit: str, applied: float) -> dict:
        change = wrapped_deg(value - applied) if unit == DEG else value - applied
        return {
            "name": name,
            "value": json_float(value),
            "sigma": None if sigma is None else json_float(sigma),
            "held": sigma is None,
            "unit": unit,
            "applied_value": json_float(applied),
            "change": json_float(change),
        }

    def sigma_of(name: str, scale: float) -> float | None:
        return fit.sigma[name] * scale if name in fit.sigma else None

    entries = [
        entry(
            offset_name(j),
            math.degrees(fit.offsets[j]),
            sigma_of(offset_name(j), DEG_PER_RAD),
            DEG,
            math.degrees(fit.current_offsets[j]),
        )
        for j in ARM_JOINT_NAMES
    ]
    for key in ("x", "y", "z"):
        name = f"base_{key}"
        entries.append(
            entry(name, fit.base[key] * MM_PER_M, sigma_of(name, MM_PER_M), MM, fit.current_base[key] * MM_PER_M)
        )
    entries.append(entry("base_yaw", math.degrees(fit.base["yaw"]), None, DEG, math.degrees(fit.current_base["yaw"])))
    for axis, name in enumerate(BOARD_POSITION_PARAMS):
        entries.append(
            entry(
                name,
                fit.board_mount["xyz"][axis] * MM_PER_M,
                sigma_of(name, MM_PER_M),
                MM,
                fit.cad_mount["xyz"][axis] * MM_PER_M,
            )
        )
    for axis, (name, free) in enumerate(zip(BOARD_REPORT_NAMES, BOARD_ROTATION_PARAMS, strict=True)):
        entries.append(entry(name, fit.skew["rpy_deg"][axis], sigma_of(free, DEG_PER_RAD), DEG, 0.0))
    entries.append(entry(SAG_PARAM, float(fit.sag["deg_per_nm"]), float(fit.sag["sigma_deg_per_nm"]), SAG_UNIT, 0.0))
    if fit.sag.get("droop_mm_per_nm") is not None:
        entries.append(
            entry(
                DROOP_PARAM, float(fit.sag["droop_mm_per_nm"]), float(fit.sag["sigma_droop_mm_per_nm"]), DROOP_UNIT, 0.0
            )
        )
    return entries


def arm_camera_report(fit: ArmCameraFit, cfg: HeadArmFitConfig, applied: dict | None = None) -> dict:
    """The quality report of the final gate (m578), plain JSON types.

    Args:
        fit: Fit result.
        cfg: Fit config (bounds and advisory limits).
        applied: The accepted arm to camera calibration in effect ({result, accuracy_examples, ...}), None when none
            is accepted; reported as comparison.applied.

    Returns:
        dict: {plausible, refusals, warnings, score {rms_px, heldout_rms_px, heldout_tool_mm, views, poses}, params,
        accuracy_examples [{label, value, unit}] (two per example: 1 sigma and 95 %), comparison {applied,
        current_score {rms_px, heldout_rms_px, heldout_tool_mm}, shift_examples [{label, value, unit}]}, skew
        {xyz_mm, rpy_deg}, sag {...}, notes}.
    """
    refusals, warnings = assess(fit, cfg)
    examples = []
    for example in fit.accuracy:
        examples.append({"label": f"{example.label}{SIGMA_SUFFIX}", "value": json_float(example.sigma_mm), "unit": MM})
        examples.append({"label": f"{example.label}{P95_SUFFIX}", "value": json_float(example.p95_mm), "unit": MM})
    return {
        "plausible": not refusals,
        "refusals": refusals,
        "warnings": warnings,
        "score": {
            "rms_px": json_float(fit.rms_px),
            "heldout_rms_px": json_float(fit.holdout_rms_px),
            "heldout_tool_mm": json_float(fit.holdout_tool_mm),
            "views": int(fit.views + fit.holdout_views),
            "poses": int(fit.arm_poses),
        },
        "params": param_entries(fit),
        "accuracy_examples": examples,
        "comparison": {
            "applied": applied,
            "current_score": {
                "rms_px": json_float(fit.current.rms_px),
                "heldout_rms_px": json_float(fit.current.holdout_rms_px),
                "heldout_tool_mm": json_float(fit.current.holdout_tool_mm),
            },
            "shift_examples": [{"label": label, "value": json_float(mm), "unit": MM} for label, mm in fit.shifts],
        },
        "skew": {key: [json_float(v) for v in values] for key, values in fit.skew.items()},
        "sag": {
            key: [json_float(v) for v in value]
            if isinstance(value, list)
            else (None if value is None else json_float(value))
            for key, value in fit.sag.items()
        },
        "notes": list(fit.notes),
    }


def calibration_snippet_yaml(fit: ArmCameraFit, session: str, result: str, date: str) -> str:
    """The reviewed snippet for the client.yml mcp_server `arm:` block (keys indented by `SNIPPET_INDENT`).

    Args:
        fit: Fit result.
        session: Session id.
        result: Result id.
        date: Fit date (YYYY-MM-DD).

    Returns:
        str: A comment line (date, session, result, RMS, held-out tool error), `joint_offsets_rad:` with the five
        joints, `arm_base_height_m:` (mcp_server requires it equal to the base z) and `base_in_base_link:`, 4 decimals.
    """
    base = {key: f"{fit.base[key]:.{OFFSET_DECIMALS}f}" for key in BASE_KEYS}
    lines = [
        f"{SNIPPET_INDENT}# arm to camera fit {date}, session {session}, result {result}: "
        f"rms {number(fit.rms_px, '.2f')} px, held-out tool {number(fit.holdout_tool_mm, '.1f')} mm",
        f"{SNIPPET_INDENT}joint_offsets_rad:",
        *[f"{SNIPPET_INDENT}  {name}: {fit.offsets[name]:.{OFFSET_DECIMALS}f}" for name in ARM_JOINT_NAMES],
        f"{SNIPPET_INDENT}arm_base_height_m: {base['z']}",
        f"{SNIPPET_INDENT}base_in_base_link: {{x: {base['x']}, y: {base['y']}, z: {base['z']}, yaw: {base['yaw']}}}",
    ]
    return "\n".join(lines) + "\n"
