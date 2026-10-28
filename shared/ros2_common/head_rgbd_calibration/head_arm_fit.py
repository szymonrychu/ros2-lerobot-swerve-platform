"""Head-from-arm head chain fit: the ChArUco board clipped to the gripper, moved by the trusted arm and seen by the
D435 colour camera, gives the head mount x, y, z, roll, pitch, yaw, the head_tilt origin pitch (tilt zero) and roll
(design D31-D35) and, with `fit_pan_axis`, the head pan axis position relative to the head.

Model per view: board corner -> board pose in gripper_link (fitted, 6 DOF) -> arm FK (measured joints plus the joint
offsets) -> arm root in robot base_link (`base_in_base_link`) -> inverse of the `camera_frame` pose (default
`d435_color_optical_frame`) of the head chain at the measured pan/tilt with the fitted origins -> `cv2.projectPoints`
with the D435 colour intrinsics (the factory camera_info). The pan zero is folded into `mount_yaw_deg` (D33). The pan
axis parameters move the head_pan origin x, y in head_pan_base and the head_tilt origin back by the same shift, so the
head stays where it is at pan 0 and only the axis it turns about moves: the 2026-10-23 session put that axis 38 mm
behind the head URDF value, which the fixed-axis model could not absorb (13 px at the +-10 deg pan offsets). The
camera mount parameters are the x, z, roll and yaw of the camera mount joint (`d435_joint`, the joint after head_tilt
on the path to `camera_frame`), refined from its CAD start. Its y and pitch keep the head URDF values: y runs along the
tilt axis and is fully degenerate with the mount y (Jacobian condition number 8e10 on the synthetic scene), and pitch
commutes with the tilt rotation and folds into tilt_zero (correlation 0.9995). With x, z, roll and yaw free the
synthetic scene's largest correlation is 0.905 (camera yaw against mount yaw), camera x against the pan axis x 0.89.
The head_pan rpy and z, head_tilt z and the D435 colour and optical joints keep the head URDF values (D32; the pan axis
direction relative to the head is the tilt roll and zero already, and a pan scale factor is degenerate with the axis
shift). numpy, scipy and OpenCV only, no ROS.
"""

import math
from collections.abc import Callable, Mapping
from dataclasses import dataclass, replace

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from ros2_common.head_rgbd_calibration import floor_fit
from ros2_common.head_rgbd_calibration.chain import UrdfChain, arm_in_robot_base, pose_from_xyz_rpy, xyz_rpy_from_pose
from ros2_common.head_rgbd_calibration.charuco import board_corners_3d
from ros2_common.head_rgbd_calibration.config import BoardConfig, HeadArmFitConfig, HeadArmFitLimits
from ros2_common.head_rgbd_calibration.floor_fit import MOUNT_JOINT, PAN_JOINT, TILT_JOINT, as_xyz_rpy, json_float
from ros2_common.head_rgbd_calibration.urdf_text import Origin
from ros2_common.head_rgbd_calibration.views import ArmBoardView, require_joints

D435_COLOR_FRAME = "d435_color_optical_frame"
D435_JOINT = "d435_joint"
GRIPPER_LINK = "gripper_link"
PARAM_NAMES = (
    "mount_x_m",
    "mount_y_m",
    "mount_z_m",
    "mount_roll_deg",
    "mount_pitch_deg",
    "mount_yaw_deg",
    "tilt_zero_deg",
    "tilt_roll_deg",
)
CAMERA_MOUNT_PARAMS = ("camera_x_m", "camera_z_m", "camera_roll_deg", "camera_yaw_deg")
PAN_AXIS_PARAMS = ("pan_axis_x_m", "pan_axis_y_m")
FLOOR_PARAMS = floor_fit.PARAM_NAMES
LENGTH_PARAMS = ("mount_x_m", "mount_y_m", "mount_z_m", "camera_x_m", "camera_z_m", *PAN_AXIS_PARAMS)
ANGLE_PARAMS = tuple(name for name in (*PARAM_NAMES, *CAMERA_MOUNT_PARAMS) if name not in LENGTH_PARAMS)
START_DELTA_PARAMS = ("mount_x_m", "mount_y_m", "mount_yaw_deg", *CAMERA_MOUNT_PARAMS)
BOARD_PARAM_COUNT = 6
MIN_VIEWS = 3
MM_PER_M = 1000.0
DEG_PER_RAD = 180.0 / math.pi
PRIOR_UNIT_M = 0.001
PRIOR_UNIT_DEG = 0.1
SOLVER_TOLERANCE = 1e-12
AIM_TOLERANCE = 1e-15
JACOBIAN_STEP = 1e-6
PAN_ZERO_NOTE = "pan zero folded into mount_yaw_deg"
CAMERA_MOUNT_NOTE = "camera mount x, z, roll, yaw free; its y and pitch kept (degenerate with mount y and tilt zero)"
PAN_AXIS_NOTE = "pan axis free: head_pan origin x, y moved, head_tilt origin moved back (head unchanged at pan 0)"

LimitCheck = tuple[str, float, float, bool]


@dataclass(frozen=True)
class Correlation:
    """The head parameter pair with the largest absolute correlation and its signed value (nan when unknown)."""

    pair: tuple[str, str]
    value: float


@dataclass(frozen=True)
class ViewError:
    """Reprojection RMS of one view (px); `held_out` views are not in the fit."""

    view_id: str
    arm_pose: int
    rms_px: float
    held_out: bool


@dataclass(frozen=True)
class HeadArmFit:
    """Result of `fit_head_from_arm`.

    Attributes:
        origins: New `head_mount_joint` and `head_tilt` origins, plus `head_pan` when the pan axis is fitted (full
            xyz/rpy); every component that is not fitted is the head URDF value.
        params: Fitted value per fitted parameter (`fit_param_names`, m or deg).
        sigma: One-sigma uncertainty per fitted parameter (m or deg); inf when it cannot be computed.
        start_params: Start value per fitted parameter, read from the head URDF (the applied chain).
        floor_delta: Fitted minus floor value per `floor_fit.PARAM_NAMES` entry (m or deg); empty without a floor.
        max_correlation: The head parameter pair with the largest absolute correlation.
        board_mount: Fitted board frame in gripper_link, `{"xyz": [m] * 3, "rpy": [rad] * 3}` (fixed-axis xyz).
        rms_px: Corner reprojection RMS over the fit views (px).
        holdout_rms_px: Corner reprojection RMS over the held-out views (px); nan without held-out views.
        holdout_board_mm: RMS over the held-out views of the board centre distance between arm FK and the view's
            solvePnP through the fitted head chain (mm); nan without held-out views.
        views: Number of fit views (held-out views not counted).
        arm_poses: Number of distinct arm poses over all views.
        holdout_views: Number of held-out views.
        pan_span_deg: Peak-to-peak of the measured head_pan over all views (deg).
        tilt_span_deg: Peak-to-peak of the measured head_tilt over all views (deg).
        per_view: Reprojection RMS per view, held-out views included.
        offset_rms_px: Corner reprojection RMS per head offset index over all views, held-out views included (px).
        offset_views: Number of views per head offset index.
        failed_limits: Names of the limits exceeded, in check order.
        passed: True only when no limit is exceeded.
    """

    origins: dict[str, Origin]
    params: dict[str, float]
    sigma: dict[str, float]
    start_params: dict[str, float]
    floor_delta: dict[str, float]
    max_correlation: Correlation
    board_mount: dict[str, list[float]]
    rms_px: float
    holdout_rms_px: float
    holdout_board_mm: float
    views: int
    arm_poses: int
    holdout_views: int
    pan_span_deg: float
    tilt_span_deg: float
    per_view: list[ViewError]
    offset_rms_px: dict[int, float]
    offset_views: dict[int, int]
    failed_limits: list[str]
    passed: bool

    def to_json(self) -> dict:
        """Plain JSON types; NaN and infinities become None.

        Returns:
            dict: JSON-serialisable result (strict JSON).
        """
        return {
            "origins": {
                name: {"xyz": [float(v) for v in o.xyz], "rpy": [float(v) for v in o.rpy]}
                for name, o in self.origins.items()
            },
            "params": {name: json_float(v) for name, v in self.params.items()},
            "sigma": {name: json_float(v) for name, v in self.sigma.items()},
            "start_params": {name: json_float(v) for name, v in self.start_params.items()},
            "floor_delta": {name: json_float(v) for name, v in self.floor_delta.items()},
            "max_correlation": {
                "pair": list(self.max_correlation.pair),
                "value": json_float(self.max_correlation.value),
            },
            "board_mount": {key: [float(v) for v in values] for key, values in self.board_mount.items()},
            "rms_px": json_float(self.rms_px),
            "holdout_rms_px": json_float(self.holdout_rms_px),
            "holdout_board_mm": json_float(self.holdout_board_mm),
            "views": int(self.views),
            "arm_poses": int(self.arm_poses),
            "holdout_views": int(self.holdout_views),
            "pan_span_deg": json_float(self.pan_span_deg),
            "tilt_span_deg": json_float(self.tilt_span_deg),
            "per_view": [
                {
                    "view_id": entry.view_id,
                    "arm_pose": int(entry.arm_pose),
                    "rms_px": json_float(entry.rms_px),
                    "held_out": bool(entry.held_out),
                }
                for entry in self.per_view
            ],
            "offset_rms_px": [
                {"head_offset": int(k), "views": int(self.offset_views[k]), "rms_px": json_float(v)}
                for k, v in sorted(self.offset_rms_px.items())
            ],
            "failed_limits": list(self.failed_limits),
            "passed": bool(self.passed),
        }


def param_scale(name: str) -> float:
    """Factor from the internal unit (m, rad) to the reported unit (m, deg) of a parameter.

    Args:
        name: Entry of `PARAM_NAMES`, `CAMERA_MOUNT_PARAMS` or `PAN_AXIS_PARAMS`.

    Returns:
        float: 1 for lengths, degrees per radian for angles.
    """
    return 1.0 if name in LENGTH_PARAMS else DEG_PER_RAD


def fit_param_names(cfg: HeadArmFitConfig) -> tuple[str, ...]:
    """The head parameters a config fits.

    Args:
        cfg: Fit config.

    Returns:
        tuple[str, ...]: `PARAM_NAMES`, then `CAMERA_MOUNT_PARAMS` when `cfg.fit_camera_mount`, then
        `PAN_AXIS_PARAMS` when `cfg.fit_pan_axis`.
    """
    camera = CAMERA_MOUNT_PARAMS if cfg.fit_camera_mount else ()
    pan_axis = PAN_AXIS_PARAMS if cfg.fit_pan_axis else ()
    return (*PARAM_NAMES, *camera, *pan_axis)


def camera_mount_joint(chain: UrdfChain, camera_frame: str) -> str:
    """The camera mount joint: the joint after head_tilt on the path to the camera frame.

    Args:
        chain: Head URDF chain.
        camera_frame: Optical frame of the camera in the head URDF.

    Returns:
        str: Joint name (`d435_joint` for the D435 colour frame).

    Raises:
        ValueError: The camera frame does not hang below head_tilt.
    """
    joints = chain.joint_names(camera_frame)
    if TILT_JOINT not in joints[:-1]:
        raise ValueError(f"{camera_frame} has no joint after {TILT_JOINT}: {joints}")
    return joints[joints.index(TILT_JOINT) + 1]


def head_values(chain: UrdfChain, camera_joint: str = D435_JOINT) -> dict[str, float]:
    """Every head parameter of a head chain in metres and radians.

    Args:
        chain: Head URDF chain.
        camera_joint: Camera mount joint (`camera_mount_joint`).

    Returns:
        dict[str, float]: Value per `PARAM_NAMES`, `CAMERA_MOUNT_PARAMS` and `PAN_AXIS_PARAMS` entry.
    """
    (mx, my, mz), (mroll, mpitch, myaw) = chain.origin(MOUNT_JOINT)
    troll, tpitch, _ = chain.origin(TILT_JOINT)[1]
    (cx, _, cz), (croll, _, cyaw) = chain.origin(camera_joint)
    px, py, _ = chain.origin(PAN_JOINT)[0]
    return {
        "mount_x_m": mx,
        "mount_y_m": my,
        "mount_z_m": mz,
        "mount_roll_deg": mroll,
        "mount_pitch_deg": mpitch,
        "mount_yaw_deg": myaw,
        "tilt_zero_deg": tpitch,
        "tilt_roll_deg": troll,
        "camera_x_m": cx,
        "camera_z_m": cz,
        "camera_roll_deg": croll,
        "camera_yaw_deg": cyaw,
        "pan_axis_x_m": px,
        "pan_axis_y_m": py,
    }


def head_vector(chain: UrdfChain, names: tuple[str, ...] = PARAM_NAMES, camera_joint: str = D435_JOINT) -> np.ndarray:
    """The values of the named head parameters of a head chain in metres and radians.

    Args:
        chain: Head URDF chain.
        names: Head parameter names (see `fit_param_names`).
        camera_joint: Camera mount joint (`camera_mount_joint`).

    Returns:
        np.ndarray: Value per name, in `names` order.
    """
    values = head_values(chain, camera_joint)
    return np.array([values[name] for name in names], dtype=float)


def start_params(
    chain: UrdfChain, names: tuple[str, ...] = PARAM_NAMES, camera_joint: str = D435_JOINT
) -> dict[str, float]:
    """Start value of every named parameter, read from the head chain (the applied chain).

    Args:
        chain: Head URDF chain.
        names: Head parameter names (see `fit_param_names`).
        camera_joint: Camera mount joint (`camera_mount_joint`).

    Returns:
        dict[str, float]: Value per name (m or deg).
    """
    x = head_vector(chain, names, camera_joint)
    return {name: float(x[i] * param_scale(name)) for i, name in enumerate(names)}


def head_origins(
    chain: UrdfChain, x: np.ndarray, names: tuple[str, ...] = PARAM_NAMES, camera_joint: str = D435_JOINT
) -> dict[str, Origin]:
    """Head origins for a parameter vector.

    The mount takes the six mount parameters and the head_tilt origin the tilt roll and zero (pitch); its yaw stays at
    the chain value. With the camera mount parameters the camera mount joint takes x, z, roll and yaw, its y and
    pitch stay at the chain value. With the pan axis parameters the head_pan origin x, y become the pan axis values
    (its z and rpy stay) and the head_tilt origin xyz moves back by the same shift, expressed in the head_pan frame,
    so the camera pose at pan 0 does not change; without them head_pan is not returned and head_tilt keeps the chain
    xyz.

    Args:
        chain: Head URDF chain (the start chain the shift is measured from).
        x: Parameters in `names` order, metres and radians.
        names: `PARAM_NAMES`, optionally with `CAMERA_MOUNT_PARAMS` and `PAN_AXIS_PARAMS` (see `fit_param_names`).
        camera_joint: Camera mount joint (`camera_mount_joint`).

    Returns:
        dict[str, Origin]: Origins of head_mount_joint, head_tilt and, with their parameters, the camera mount joint
        and head_pan; every other joint keeps its URDF origin.
    """
    v = {name: float(value) for name, value in zip(names, x, strict=True)}
    txyz, (_, _, tyaw) = chain.origin(TILT_JOINT)
    origins = {
        MOUNT_JOINT: Origin(
            (v["mount_x_m"], v["mount_y_m"], v["mount_z_m"]),
            (v["mount_roll_deg"], v["mount_pitch_deg"], v["mount_yaw_deg"]),
        ),
        TILT_JOINT: Origin(txyz, (v["tilt_roll_deg"], v["tilt_zero_deg"], tyaw)),
    }
    if CAMERA_MOUNT_PARAMS[0] in v:
        (_, cy, _), (_, cpitch, _) = chain.origin(camera_joint)
        origins[camera_joint] = Origin(
            (v["camera_x_m"], cy, v["camera_z_m"]), (v["camera_roll_deg"], cpitch, v["camera_yaw_deg"])
        )
    if PAN_AXIS_PARAMS[0] in v:
        pxyz, prpy = chain.origin(PAN_JOINT)
        moved = np.array([v["pan_axis_x_m"], v["pan_axis_y_m"], pxyz[2]])
        back = pose_from_xyz_rpy(pxyz, prpy)[:3, :3].T @ (np.asarray(pxyz, dtype=float) - moved)
        origins[PAN_JOINT] = Origin((float(moved[0]), float(moved[1]), float(moved[2])), prpy)
        tilt_xyz = np.asarray(txyz, dtype=float) + back
        origins[TILT_JOINT] = Origin(
            (float(tilt_xyz[0]), float(tilt_xyz[1]), float(tilt_xyz[2])), origins[TILT_JOINT].rpy
        )
    return origins


def board_centre(board: BoardConfig) -> np.ndarray:
    """Centre of the board's inner corners (the board rectangle centre) in the board frame.

    Args:
        board: Board definition.

    Returns:
        np.ndarray: (3,) metres.
    """
    return board_corners_3d(board).mean(axis=0)


def camera_pose(
    chain: UrdfChain,
    joints: Mapping[str, float],
    origins: Mapping[str, Origin] | None = None,
    camera_frame: str = D435_COLOR_FRAME,
) -> np.ndarray:
    """Pose of the camera's optical frame (default the D435 colour camera) in base_link at the measured head angles.

    Args:
        chain: Head URDF chain.
        joints: Joint angles (rad); head_pan and head_tilt are read.
        origins: Per-joint origin replacement; None keeps the URDF.
        camera_frame: Optical frame of the camera in the head URDF.

    Returns:
        np.ndarray: 4x4 pose.
    """
    overrides = {name: as_xyz_rpy(o) for name, o in (origins or {}).items()}
    return chain.transform(camera_frame, {PAN_JOINT: joints[PAN_JOINT], TILT_JOINT: joints[TILT_JOINT]}, overrides)


def aim_head(chain: UrdfChain, point_base: np.ndarray, camera_frame: str = D435_COLOR_FRAME) -> tuple[float, float]:
    """Head angles that put a point on the optical axis of the camera (default the D435 colour camera).

    Solved numerically on `chain` (the camera is off the pan and tilt axes), then clamped to the chain's head_pan and
    head_tilt limits.

    Args:
        chain: Head URDF chain (with any overrides already in it).
        point_base: (3,) point in robot base_link (m).
        camera_frame: Optical frame of the camera in the head URDF.

    Returns:
        tuple[float, float]: (pan, tilt) in rad, + tilt is down.
    """
    target = np.append(np.asarray(point_base, dtype=float).reshape(3), 1.0)

    def residual(angles: np.ndarray) -> np.ndarray:
        local = np.linalg.solve(
            camera_pose(chain, {PAN_JOINT: angles[0], TILT_JOINT: angles[1]}, camera_frame=camera_frame), target
        )
        return np.array([math.atan2(local[0], local[2]), math.atan2(local[1], local[2])])

    delta = target[:3] - camera_pose(chain, {PAN_JOINT: 0.0, TILT_JOINT: 0.0}, camera_frame=camera_frame)[:3, 3]
    start = np.array([math.atan2(delta[1], delta[0]), math.atan2(-delta[2], math.hypot(delta[0], delta[1]))])
    pan, tilt = least_squares(
        residual, start, method="lm", xtol=AIM_TOLERANCE, ftol=AIM_TOLERANCE, gtol=AIM_TOLERANCE
    ).x
    pan = (pan + math.pi) % (2.0 * math.pi) - math.pi
    pan_low, pan_high = chain.limits(PAN_JOINT) or (-math.pi, math.pi)
    tilt_low, tilt_high = chain.limits(TILT_JOINT) or (-math.pi, math.pi)
    return float(np.clip(pan, pan_low, pan_high)), float(np.clip(tilt, tilt_low, tilt_high))


def pose_from_rvec_tvec(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    """4x4 transform of an OpenCV rotation vector and translation.

    Args:
        rvec: (3,) Rodrigues vector.
        tvec: (3,) translation.

    Returns:
        np.ndarray: 4x4 pose.
    """
    pose = np.eye(4)
    pose[:3, :3] = cv2.Rodrigues(np.asarray(rvec, dtype=float).reshape(3, 1))[0]
    pose[:3, 3] = np.asarray(tvec, dtype=float).reshape(3)
    return pose


def board_mount_pose(base: np.ndarray, delta: np.ndarray) -> np.ndarray:
    """Board pose in gripper_link: `base` turned by the rotation vector `delta[:3]` (board frame) and moved by
    `delta[3:]` (gripper frame).

    Args:
        base: 4x4 start pose of the board in gripper_link.
        delta: (6,) [rotation vector (rad), translation (m)].

    Returns:
        np.ndarray: 4x4 pose.
    """
    pose = base.copy()
    pose[:3, :3] = base[:3, :3] @ Rotation.from_rotvec(delta[:3]).as_matrix()
    pose[:3, 3] = base[:3, 3] + delta[3:]
    return pose


def board_pnp(
    objects: np.ndarray, image: np.ndarray, camera_matrix: np.ndarray, dist: np.ndarray, guess: np.ndarray
) -> np.ndarray | None:
    """Board pose in the camera from one view's corners, iterated from a predicted pose.

    Args:
        objects: (N, 3) board corners in the board frame (m).
        image: (N, 2) detected corners (px).
        camera_matrix: 3x3 intrinsics.
        dist: Distortion coefficients.
        guess: 4x4 predicted board pose in the camera (avoids the planar pose flip).

    Returns:
        np.ndarray | None: 4x4 board pose in the camera, None when solvePnP fails.
    """
    ok, rvec, tvec = cv2.solvePnP(
        objects,
        image,
        camera_matrix,
        dist,
        rvec=cv2.Rodrigues(guess[:3, :3])[0],
        tvec=guess[:3, 3].reshape(3, 1).copy(),
        useExtrinsicGuess=True,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    return pose_from_rvec_tvec(rvec, tvec) if ok else None


def initial_board_mount(
    chain: UrdfChain,
    origins: dict[str, Origin],
    views: list[ArmBoardView],
    grippers: list[np.ndarray],
    objects: list[np.ndarray],
    images: list[np.ndarray],
    camera_matrix: np.ndarray,
    dist: np.ndarray,
    nominal: np.ndarray,
    camera_frame: str = D435_COLOR_FRAME,
) -> np.ndarray:
    """Board mount start: each view's solvePnP (seeded from `nominal`) mapped through the start head chain and the arm
    FK into gripper_link; median translation and mean rotation.

    Args:
        chain: Head URDF chain.
        origins: Start head origins.
        views: Views to use.
        grippers: 4x4 gripper_link pose in base_link per view.
        objects: (N, 3) board corners per view.
        images: (N, 2) detected corners per view.
        camera_matrix: 3x3 intrinsics.
        dist: Distortion coefficients.
        nominal: 4x4 configured board mount start (`cfg.board_mount`).
        camera_frame: Optical frame of the camera in the head URDF.

    Returns:
        np.ndarray: 4x4 board pose in gripper_link; `nominal` when no view solves.
    """
    rotations = []
    translations = []
    for view, gripper, obj, img in zip(views, grippers, objects, images, strict=True):
        camera = camera_pose(chain, view.joints, origins, camera_frame)
        board = board_pnp(obj, img, camera_matrix, dist, np.linalg.solve(camera, gripper @ nominal))
        if board is None:
            continue
        mount = np.linalg.solve(gripper, camera @ board)
        rotations.append(mount[:3, :3])
        translations.append(mount[:3, 3])
    if not rotations:
        return nominal
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_matrix(np.array(rotations)).mean().as_matrix()
    pose[:3, 3] = np.median(np.array(translations), axis=0)
    return pose


def corner_errors(
    chain: UrdfChain,
    x: np.ndarray,
    names: tuple[str, ...],
    mount_start: np.ndarray,
    views: list[ArmBoardView],
    grippers: list[np.ndarray],
    objects: list[np.ndarray],
    images: list[np.ndarray],
    camera_matrix: np.ndarray,
    dist: np.ndarray,
    camera_frame: str = D435_COLOR_FRAME,
    camera_joint: str = D435_JOINT,
) -> list[np.ndarray]:
    """Projected minus detected corners of each view.

    Args:
        chain: Head URDF chain.
        x: Head parameters (`names` order, m and rad) then the 6 board mount changes (`board_mount_pose`).
        names: The head parameter names (`fit_param_names`).
        mount_start: 4x4 board mount the change applies to.
        views: Views.
        grippers: 4x4 gripper_link pose in base_link per view.
        objects: (N, 3) board corners per view.
        images: (N, 2) detected corners per view.
        camera_matrix: 3x3 intrinsics.
        dist: Distortion coefficients.
        camera_frame: Optical frame of the camera in the head URDF.
        camera_joint: Camera mount joint (`camera_mount_joint`).

    Returns:
        list[np.ndarray]: (N, 2) pixel errors per view.
    """
    origins = head_origins(chain, x[: len(names)], names, camera_joint)
    mount = board_mount_pose(mount_start, x[len(names) :])
    errors = []
    for view, gripper, obj, img in zip(views, grippers, objects, images, strict=True):
        board = np.linalg.solve(camera_pose(chain, view.joints, origins, camera_frame), gripper @ mount)
        rvec = cv2.Rodrigues(board[:3, :3])[0]
        projected = cv2.projectPoints(obj, rvec, board[:3, 3], camera_matrix, dist)[0].reshape(-1, 2)
        errors.append(projected - img)
    return errors


def numeric_jacobian(fun: Callable[[np.ndarray], np.ndarray], x: np.ndarray, step: float = JACOBIAN_STEP) -> np.ndarray:
    """Central-difference Jacobian.

    Args:
        fun: Vector function of x.
        x: (n,) point.
        step: Difference step per component.

    Returns:
        np.ndarray: (m, n) Jacobian.
    """
    columns = []
    for i in range(len(x)):
        dx = np.zeros(len(x))
        dx[i] = step
        columns.append((fun(x + dx) - fun(x - dx)) / (2.0 * step))
    return np.column_stack(columns)


def rms_px(errors: list[np.ndarray]) -> float:
    """RMS corner error (px, the 2D distance) over the given views; nan for none."""
    if not errors:
        return math.nan
    return float(np.sqrt(np.mean(np.sum(np.square(np.concatenate(errors)), axis=1))))


def largest_correlation(cov: np.ndarray, names: tuple[str, ...] = PARAM_NAMES) -> Correlation:
    """The head parameter pair with the largest absolute correlation.

    Args:
        cov: Covariance whose first `len(names)` rows and columns are the head parameters.
        names: The head parameter names in covariance order.

    Returns:
        Correlation: Pair and signed value; value nan when the covariance is not finite.
    """
    n = len(names)
    head = cov[:n, :n]
    sd = np.sqrt(np.abs(np.diag(head)))
    with np.errstate(invalid="ignore", divide="ignore"):
        corr = head / np.outer(sd, sd)
    best = Correlation((names[0], names[1]), math.nan)
    for i in range(n):
        for j in range(i + 1, n):
            value = float(corr[i, j])
            if math.isfinite(value) and not abs(value) <= abs(best.value):
                best = Correlation((names[i], names[j]), value)
    return best


def fit_head_from_arm(
    views: list[ArmBoardView],
    camera_matrix: np.ndarray,
    dist: np.ndarray,
    head_urdf: str,
    arm_urdf: str,
    cfg: HeadArmFitConfig,
    floor: dict[str, float] | None,
    camera_frame: str = D435_COLOR_FRAME,
) -> HeadArmFit:
    """Fit the head chain from views of the gripper board in the D435 colour camera (design D31-D35, D10).

    Free: the eight `PARAM_NAMES`, the four `CAMERA_MOUNT_PARAMS` with `cfg.fit_camera_mount` (x, z, roll, yaw of
    the camera mount joint, `d435_joint` for the D435 colour frame), the two `PAN_AXIS_PARAMS` with
    `cfg.fit_pan_axis`, and the 6-DOF board mount in gripper_link. Every other head origin component (the camera mount
    y and pitch and the D435 optical joints included), the arm, the joint offsets, the gauge and the intrinsics are
    fixed. Every `limits.holdout_every`-th distinct arm pose (sorted) is held out with all its views. One Huber least
    squares over the fit views' corner residuals (px); with `cfg.floor_prior_weight > 0` and a floor, one residual per
    floor parameter `floor_prior_weight * (value - floor[name]) / unit` (1 mm, 0.1 deg). Sigma from
    `(J^T J)^-1 * sum(r^2) / (m - n)` on the corner residuals at the solution.

    Args:
        views: Views of the board with all seven measured joints.
        camera_matrix: 3x3 intrinsics of the camera (the D435 colour factory camera_info).
        dist: Distortion coefficients of the camera.
        head_urdf: Head URDF text with the applied chain (the start values).
        arm_urdf: Arm URDF text.
        cfg: Fit config.
        floor: Applied floor-fit values per `floor_fit.PARAM_NAMES` entry (m or deg); None when there is none, which
            fails `floor_cross_check`.
        camera_frame: Optical frame of the camera in the head URDF (default the D435 colour frame).

    Returns:
        HeadArmFit: Fitted chain, uncertainties, validation and every failed limit.

    Raises:
        ValueError: Fewer than 3 views, a view lacks a joint, or `floor` lacks a floor parameter.
    """
    if len(views) < MIN_VIEWS:
        raise ValueError(f"the head-from-arm fit needs at least {MIN_VIEWS} views, got {len(views)}")
    for view in views:
        require_joints(view)
    if floor is not None:
        missing = [name for name in FLOOR_PARAMS if name not in floor]
        if missing:
            raise ValueError(f"floor values are missing: {', '.join(missing)}")

    chain = UrdfChain.from_text(head_urdf)
    arm = UrdfChain.from_text(arm_urdf)
    limits = cfg.limits
    k = np.asarray(camera_matrix, dtype=float)
    d = np.asarray(dist, dtype=float)
    base = arm_in_robot_base(cfg.base_in_base_link.model_dump())
    corners = board_corners_3d(cfg.board)
    grippers = [base @ arm.transform(GRIPPER_LINK, v.joints, offsets=cfg.joint_offsets_rad) for v in views]
    objects = [corners[np.asarray(v.detection.ids, dtype=int)] for v in views]
    images = [np.asarray(v.detection.points, dtype=float).reshape(-1, 2) for v in views]

    arm_poses = sorted({v.arm_pose for v in views})
    held = set(arm_poses[limits.holdout_every - 1 :: limits.holdout_every])
    fit_idx = [i for i, v in enumerate(views) if v.arm_pose not in held]
    hold_idx = [i for i, v in enumerate(views) if v.arm_pose in held]

    def subset(idx: list[int]) -> tuple[list, list, list, list]:
        return [views[i] for i in idx], [grippers[i] for i in idx], [objects[i] for i in idx], [images[i] for i in idx]

    fit_set = subset(fit_idx)
    names = fit_param_names(cfg)
    camera_joint = camera_mount_joint(chain, camera_frame)
    x_start = head_vector(chain, names, camera_joint)
    nominal = pose_from_xyz_rpy(cfg.board_mount.xyz_init, cfg.board_mount.rpy_init)
    mount_start = initial_board_mount(
        chain, head_origins(chain, x_start, names, camera_joint), *fit_set, k, d, nominal, camera_frame
    )

    def corner_residuals(x: np.ndarray) -> np.ndarray:
        errors = corner_errors(chain, x, names, mount_start, *fit_set, k, d, camera_frame, camera_joint)
        return np.concatenate([e.ravel() for e in errors])

    floor_values = floor or {}
    prior = list(FLOOR_PARAMS) if floor_values and cfg.floor_prior_weight > 0 else []

    def residuals(x: np.ndarray) -> np.ndarray:
        terms = [
            cfg.floor_prior_weight
            * (x[names.index(name)] * param_scale(name) - floor_values[name])
            / (PRIOR_UNIT_M if name in LENGTH_PARAMS else PRIOR_UNIT_DEG)
            for name in prior
        ]
        return np.concatenate([corner_residuals(x), terms])

    x0 = np.concatenate([x_start, np.zeros(BOARD_PARAM_COUNT)])
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

    jac = numeric_jacobian(corner_residuals, solution)
    residual = corner_residuals(solution)
    dof = len(residual) - len(solution)
    unknown = np.full((len(solution), len(solution)), math.inf)
    cov = unknown
    if dof > 0:
        try:
            cov = np.linalg.inv(jac.T @ jac) * (float(residual @ residual) / dof)
        except np.linalg.LinAlgError:
            cov = unknown
    sigma_x = np.sqrt(np.abs(np.diag(cov)))

    origins = head_origins(chain, solution[: len(names)], names, camera_joint)
    mount = board_mount_pose(mount_start, solution[len(names) :])
    errors = corner_errors(
        chain, solution, names, mount_start, views, grippers, objects, images, k, d, camera_frame, camera_joint
    )
    centre = np.append(board_centre(cfg.board), 1.0)
    board_mm = []
    for i in hold_idx:
        camera = camera_pose(chain, views[i].joints, origins, camera_frame)
        from_arm = grippers[i] @ mount
        board = board_pnp(objects[i], images[i], k, d, np.linalg.solve(camera, from_arm))
        if board is None:
            board_mm.append(math.nan)
            continue
        board_mm.append(float(np.linalg.norm((camera @ board @ centre - from_arm @ centre)[:3])) * MM_PER_M)

    params = {name: float(solution[i] * param_scale(name)) for i, name in enumerate(names)}
    offsets = sorted({v.head_offset for v in views})
    offset_idx = {offset: [i for i, v in enumerate(views) if v.head_offset == offset] for offset in offsets}
    pans = [math.degrees(v.joints[PAN_JOINT]) for v in views]
    tilts = [math.degrees(v.joints[TILT_JOINT]) for v in views]
    mount_xyz, mount_rpy = xyz_rpy_from_pose(mount)
    result = HeadArmFit(
        origins=origins,
        params=params,
        sigma={name: float(sigma_x[i] * param_scale(name)) for i, name in enumerate(names)},
        start_params=start_params(chain, names, camera_joint),
        floor_delta={name: params[name] - floor_values[name] for name in FLOOR_PARAMS} if floor_values else {},
        max_correlation=largest_correlation(cov, names),
        board_mount={"xyz": list(mount_xyz), "rpy": list(mount_rpy)},
        rms_px=rms_px([errors[i] for i in fit_idx]),
        holdout_rms_px=rms_px([errors[i] for i in hold_idx]),
        holdout_board_mm=float(np.sqrt(np.mean(np.square(board_mm)))) if board_mm else math.nan,
        views=len(fit_idx),
        arm_poses=len(arm_poses),
        holdout_views=len(hold_idx),
        pan_span_deg=float(np.ptp(pans)),
        tilt_span_deg=float(np.ptp(tilts)),
        per_view=[
            ViewError(v.view_id, int(v.arm_pose), rms_px([errors[i]]), v.arm_pose in held) for i, v in enumerate(views)
        ],
        offset_rms_px={offset: rms_px([errors[i] for i in idx]) for offset, idx in offset_idx.items()},
        offset_views={offset: len(idx) for offset, idx in offset_idx.items()},
        failed_limits=[],
        passed=False,
    )
    failed = [name for name, _, _, ok in limit_checks(result, limits) if not ok]
    return replace(result, failed_limits=failed, passed=not failed)


def limit_checks(fit: HeadArmFit, limits: HeadArmFitLimits) -> list[LimitCheck]:
    """Every acceptance limit of a fit, in check order.

    Args:
        fit: Fit result (its `failed_limits` and `passed` are not read).
        limits: Acceptance limits.

    Returns:
        list[LimitCheck]: (name, value, limit, ok); lengths in mm, angles in deg, the correlation as its absolute
        value. NaN values fail.
    """
    sigma_mm = float(np.max([fit.sigma[name] for name in LENGTH_PARAMS if name in fit.sigma])) * MM_PER_M
    sigma_deg = float(np.max([fit.sigma[name] for name in ANGLE_PARAMS if name in fit.sigma]))
    correlation = abs(fit.max_correlation.value)
    checks: list[LimitCheck] = [
        ("rms_px", fit.rms_px, limits.rms_px, fit.rms_px <= limits.rms_px),
        ("holdout_rms_px", fit.holdout_rms_px, limits.holdout_rms_px, fit.holdout_rms_px <= limits.holdout_rms_px),
        (
            "holdout_board_mm",
            fit.holdout_board_mm,
            limits.holdout_board_mm,
            fit.holdout_board_mm <= limits.holdout_board_mm,
        ),
        ("max_sigma_mm", sigma_mm, limits.max_sigma_mm, sigma_mm <= limits.max_sigma_mm),
        ("max_sigma_deg", sigma_deg, limits.max_sigma_deg, sigma_deg <= limits.max_sigma_deg),
        ("max_correlation", correlation, limits.max_correlation, correlation <= limits.max_correlation),
        ("min_views", fit.views, limits.min_views, fit.views >= limits.min_views),
        ("min_arm_poses", fit.arm_poses, limits.min_arm_poses, fit.arm_poses >= limits.min_arm_poses),
        ("min_pan_span_deg", fit.pan_span_deg, limits.min_pan_span_deg, fit.pan_span_deg >= limits.min_pan_span_deg),
        (
            "min_tilt_span_deg",
            fit.tilt_span_deg,
            limits.min_tilt_span_deg,
            fit.tilt_span_deg >= limits.min_tilt_span_deg,
        ),
    ]
    for name, delta in fit.floor_delta.items():
        value, limit = (
            (abs(delta) * MM_PER_M, limits.max_floor_delta_mm)
            if name in LENGTH_PARAMS
            else (abs(delta), limits.max_floor_delta_deg)
        )
        checks.append((f"floor_delta:{name}", value, limit, value <= limit))
    for name in (n for n in START_DELTA_PARAMS if n in fit.params):
        delta = abs(fit.params[name] - fit.start_params[name])
        value, limit = (
            (delta * MM_PER_M, limits.max_start_delta_mm)
            if name in LENGTH_PARAMS
            else (delta, limits.max_start_delta_deg)
        )
        checks.append((f"start_delta:{name}", value, limit, value <= limit))
    for name in PAN_AXIS_PARAMS:
        if name in fit.params:
            value = abs(fit.params[name] - fit.start_params[name]) * MM_PER_M
            checks.append(
                (f"start_delta:{name}", value, limits.max_pan_axis_delta_mm, value <= limits.max_pan_axis_delta_mm)
            )
    if not fit.floor_delta:
        checks.append(("floor_cross_check", math.nan, math.nan, False))
    return checks


def number(value: float, spec: str) -> str:
    """`format(value, spec)`, or `n/a` when the value is not finite."""
    return format(value, spec) if math.isfinite(value) else "n/a"


def head_arm_report_markdown(fit: HeadArmFit, cfg: HeadArmFitConfig, session: str, date: str) -> str:
    """Markdown report of a head-from-arm fit.

    Args:
        fit: Fit result.
        cfg: Fit config (limits).
        session: Session id.
        date: Report date (YYYY-MM-DD).

    Returns:
        str: The report: parameter table, residuals (overall and per head offset), coverage, the largest
        correlation, the board mount and every limit with PASS or FAIL.
    """
    lines = [
        f"# Head-from-arm fit {session}",
        "",
        f"- Date: {date}",
        f"- Result: {'PASS' if fit.passed else 'FAIL'}",
        f"- Note: {PAN_ZERO_NOTE}",
        *([f"- Note: {CAMERA_MOUNT_NOTE}"] if any(name in fit.params for name in CAMERA_MOUNT_PARAMS) else []),
        *([f"- Note: {PAN_AXIS_NOTE}"] if any(name in fit.params for name in PAN_AXIS_PARAMS) else []),
        "",
        "## Parameters",
        "",
        "Lengths in m (start, fitted, floor) and mm (delta, sigma, floor delta); angles in deg.",
        "",
        "| parameter | start | fitted | delta | sigma | floor | floor delta |",
        "|---|---|---|---|---|---|---|",
    ]
    for name in fit.params:
        value_spec, change = (".4f", MM_PER_M) if name in LENGTH_PARAMS else (".3f", 1.0)
        start, fitted = fit.start_params[name], fit.params[name]
        floor_cells = ["-", "-"]
        if name in fit.floor_delta:
            delta = fit.floor_delta[name]
            floor_cells = [number(fitted - delta, value_spec), number(delta * change, "+.3f")]
        lines.append(
            f"| {name} | {number(start, value_spec)} | {number(fitted, value_spec)} | "
            f"{number((fitted - start) * change, '+.3f')} | {number(fit.sigma[name] * change, '.3f')} | "
            f"{floor_cells[0]} | {floor_cells[1]} |"
        )
    xyz_mm = ", ".join(number(v * MM_PER_M, ".1f") for v in fit.board_mount["xyz"])
    rpy_deg = ", ".join(number(math.degrees(v), ".2f") for v in fit.board_mount["rpy"])
    lines += [
        "",
        "## Residuals",
        "",
        f"- rms_px (fit views): {number(fit.rms_px, '.3f')}",
        f"- holdout_rms_px: {number(fit.holdout_rms_px, '.3f')}",
        f"- holdout_board_mm: {number(fit.holdout_board_mm, '.2f')}",
        "",
        "## Residuals per head offset",
        "",
        "Head offset index into arm_sequence.head_offsets_deg; all views, held-out views included.",
        "",
        "| head offset | views | rms px |",
        "|---|---|---|",
        *[
            f"| {offset} | {fit.offset_views[offset]} | {number(value, '.3f')} |"
            for offset, value in sorted(fit.offset_rms_px.items())
        ],
        "",
        "## Coverage",
        "",
        f"- fit views: {fit.views}, held-out views: {fit.holdout_views}, arm poses: {fit.arm_poses}",
        f"- pan span: {number(fit.pan_span_deg, '.1f')} deg, tilt span: {number(fit.tilt_span_deg, '.1f')} deg",
        "",
        "## Largest correlation",
        "",
        f"- {fit.max_correlation.pair[0]} / {fit.max_correlation.pair[1]}: {number(fit.max_correlation.value, '+.3f')}",
        "",
        "## Board mount in gripper_link",
        "",
        f"- xyz (mm): {xyz_mm}; rpy (deg): {rpy_deg}",
        "",
        "## Limits",
        "",
        "| limit | value | limit value | result |",
        "|---|---|---|---|",
    ]
    for name, value, limit, ok in limit_checks(fit, cfg.limits):
        shown = "no floor result" if name == "floor_cross_check" else number(float(value), ".3f")
        lines.append(f"| {name} | {shown} | {number(float(limit), '.3f')} | {'PASS' if ok else 'FAIL'} |")
    return "\n".join(lines) + "\n"
