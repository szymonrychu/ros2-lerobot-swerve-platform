"""Camera tools: pixel -> ground, annotated images with overlays, numbered candidate points, mount calibration.

All tools are read-only (sensor class): they never command the robot. Pixel coordinates are in the calibrated image
size (the intrinsics width x height); every image these tools return has that size.
"""

import base64
import json
import math
import time
from collections.abc import Iterator
from contextlib import contextmanager
from typing import Annotated, Any, Literal

import cv2
import numpy as np
from mcp.server.mcpserver.exceptions import ToolError
from mcp.types import CallToolResult, ImageContent, TextContent
from pydantic import BaseModel, Field
from ros2_common.camera_geometry import CameraIntrinsics, MountPose, pixel_to_point_camera

from .arm import ArmError
from .camera_calib import CalibrationError, SampleStore, mount_yaml, solve_samples
from .camera_candidates import CandidateStore, RobotSnapshot, base_moved, head_moved
from .camera_overlay import (
    COLOR_GRIPPER,
    COLOR_PLANNED,
    COLOR_REACH,
    candidate_pixels,
    circle_runs,
    draw_candidates,
    draw_grid,
    draw_legend,
    draw_lidar,
    draw_marker,
    draw_polylines,
    thin,
)
from .camera_point import (
    ARM_FRAME,
    BASE_FRAME,
    COLOR_OPTICAL_FRAME,
    MAP_FRAME,
    PointFrames,
    calibration_block,
    frames_of_point,
    head_pose_block,
    position,
    residual_notes,
)
from .camera_scene import (
    APPROX_NOTE,
    Camera,
    CameraSetup,
    Scene,
    base_to_map,
    build_scene,
    camera_setup,
    ground_fields,
    ground_report,
    head_parent_transform,
    head_parent_transform_at,
    offset_matrix,
    picture_sizes,
    pixel_input,
    pixel_scale,
)
from .config import HARD_MAX_IMAGE_PX
from .depth_point import DepthWindow, window_depth
from .depth_source import DepthFrame
from .head import HeadPoseAtStamp
from .lookup_tools import record_lookup
from .models import BasePose, RobotError
from .tool_context import ToolContext

TOOL_NAMES = (
    "pixel_to_ground",
    "pixel_to_point",
    "camera_point_to_arm",
    "get_annotated_camera_image",
    "mark_candidate_points",
    "resolve_candidate",
    "capture_calibration_sample",
    "solve_camera_calibration",
    "clear_calibration_samples",
)
SurfaceHeight = Annotated[
    float,
    Field(
        ge=-0.5,
        le=0.5,
        description=(
            "Height (m) of the surface the pixel lies on relative to the robot's floor: positive above, negative "
            "below (top of a 3 cm box 0.03, a floor 10 cm lower -0.10); 0 = the floor"
        ),
    ),
]
ImageWidth = Annotated[
    int | None,
    Field(
        ge=16,
        le=4096,
        description=(
            "Width (px) of the image the pixel was read from, e.g. 384 for a default get_camera_image picture "
            "(each camera's default size is in the tool description); with image_height the pixel is scaled to the "
            "calibrated image size. Omit only for pixels read from the calibrated-size image "
            "(get_annotated_camera_image, mark_candidate_points)"
        ),
    ),
]
ImageHeight = Annotated[
    int | None,
    Field(
        ge=16,
        le=4096,
        description=(
            "Height (px) of the image the pixel was read from (216 for the 384x216 front picture, 288 for a 384x288 "
            "one; each camera's default size is in the tool description)"
        ),
    ),
]
Overlay = Literal["grid", "reach", "gripper", "planned_gripper", "lidar"]
# Warning in the pixel-taking tool descriptions: pixels of a smaller picture are wrong without the image size.
PIXEL_SIZE_NOTE = (
    "IMPORTANT: u, v are pixels of the CALIBRATED image size (get_annotated_camera_image and mark_candidate_points "
    "images, the result's 'image') unless image_width and image_height give the size of the image the pixel came "
    "from: get_camera_image returns {sizes} by default, so pass those as image_width, image_height for its pixels "
    "(an unscaled small-image pixel lands at the wrong place; another aspect is refused)."
)
MAX_GROUND_RANGE_M = 6.0  # candidate points whose floor point is farther are skipped (near-horizon pixels are noise)
MIN_GOOD_SAMPLES = 6  # six mount parameters
HIGH_RMS_PX = 3.0
UNCERTAIN_TEXT = (
    "Pixel coordinates are in the calibrated image size; ground points assume a flat floor and the configured mount."
)


class Point3(BaseModel):
    """A point in the arm base frame (the frame of move_arm_cartesian)."""

    x: float = Field(description="m, forward of the arm base")
    y: float = Field(description="m, left of the arm base")
    z: float = Field(description="m, up from the arm mount plane (the floor is at the negative arm floor_z_m)")


class Region(BaseModel):
    """Pixel rectangle of an image."""

    u0: float
    v0: float
    u1: float
    v1: float


class MountGuess(BaseModel):
    """Starting mount pose for the solver (the parent frame comes from the config)."""

    x: float = Field(description="m")
    y: float = Field(description="m")
    z: float = Field(description="m")
    roll: float = Field(description="rad about fixed x")
    pitch: float = Field(description="rad about fixed y")
    yaw: float = Field(description="rad about fixed z")


CameraOpticalFrame = Literal["d435_color_optical_frame", "d435_depth_optical_frame"]
WindowPx = Annotated[
    int | None,
    Field(
        ge=1,
        le=31,
        description=(
            "Odd side (px) of the square window the depth is read from (median of its valid pixels); default "
            "camera_arm.default_window_px (5). A larger window averages more noise on a flat top, a smaller one fits "
            "a small object"
        ),
    ),
]
ImageStampS = Annotated[
    float | None,
    Field(
        description=(
            "stamp_s of the picture the pixel was read from (get_camera_image metadata): the head pan/tilt at that "
            "instant places the ray. Omit it only when the head has not moved since the picture; the tool then uses "
            "the current pose and refuses while the head is moving"
        ),
    ),
]
REGISTRATION_TOLERANCE_PX = 0.5  # depth CameraInfo may differ from the colour one by this much (fx, fy, cx, cy)
MM_PER_M = 1000.0


def check_registered(frame: DepthFrame, intr: CameraIntrinsics) -> None:
    """Require the aligned depth to be registered to the colour camera the pixel was read in.

    Args:
        frame (DepthFrame): Aligned depth frame with its CameraInfo.
        intr (CameraIntrinsics): Colour intrinsics.

    Raises:
        ValueError: When the depth image size differs from the colour size, or its CameraInfo (fx, fy, cx, cy) from
            the colour one by more than REGISTRATION_TOLERANCE_PX.
    """
    height, width = frame.depth.shape
    if (width, height) != (intr.width, intr.height):
        raise ValueError(
            f"the aligned depth image is {width}x{height} but the colour camera is calibrated at "
            f"{intr.width}x{intr.height}: the depth is not registered to the colour image (check the realsense "
            "align_depth setting and resolution)"
        )
    k = frame.info.k
    depth_k = (k[0], k[4], k[2], k[5])
    colour_k = (intr.fx, intr.fy, intr.cx, intr.cy)
    if any(abs(a - b) > REGISTRATION_TOLERANCE_PX for a, b in zip(depth_k, colour_k, strict=True)):
        raise ValueError(
            "the aligned depth CameraInfo (fx, fy, cx, cy) = "
            f"{tuple(round(v, 1) for v in depth_k)} differs from the colour camera's {tuple(round(v, 1) for v in colour_k)}: "
            "the depth is not registered to the colour camera"
        )


def depth_report(win: DepthWindow) -> dict[str, Any]:
    """The depth part of a pixel_to_point result.

    Args:
        win (DepthWindow): The window reading.

    Returns:
        dict[str, Any]: z_m, spread_m, valid_fraction, n_valid, n_flagged, n_window, window_px and sigma_m.
    """
    return {
        "z_m": round(win.z_m, 4),
        "spread_m": round(win.spread_m, 4),
        "valid_fraction": round(win.valid_fraction, 3),
        "n_valid": win.n_valid,
        "n_flagged": win.n_flagged,
        "n_window": win.n_window,
        "window_px": win.window_px,
        "sigma_m": round(win.sigma_m, 5),
    }


@contextmanager
def camera_errors() -> Iterator[None]:
    """Turn camera, arm and calibration failures into MCP tool errors the model can read.

    Yields:
        None: Context body.
    """
    try:
        yield
    except (ArmError, RobotError, ValueError) as exc:
        raise ToolError(str(exc)) from exc


def data_result(data: dict[str, Any]) -> CallToolResult:
    """Structured tool result with the payload also as a JSON text block.

    Args:
        data (dict[str, Any]): Payload.

    Returns:
        CallToolResult: Result for a structured tool.
    """
    return CallToolResult(content=[TextContent(type="text", text=json.dumps(data))], structured_content=data)


def image_result(jpeg: bytes, data: dict[str, Any]) -> list[ImageContent | TextContent]:
    """JPEG image plus its metadata as JSON text.

    Args:
        jpeg (bytes): Encoded image.
        data (dict[str, Any]): Metadata.

    Returns:
        list[ImageContent | TextContent]: Content blocks.
    """
    image = ImageContent(type="image", data=base64.b64encode(jpeg).decode(), mime_type="image/jpeg")
    return [image, TextContent(type="text", text=json.dumps(data))]


def register(ctx: ToolContext) -> None:
    """Register the camera tools.

    Args:
        ctx (ToolContext): Shared registration context.
    """
    server, robot, config = ctx.server, ctx.robot, ctx.config
    sizes = picture_sizes(config)
    picture_text = " and ".join(f"{w}x{h} ({camera})" for camera, (w, h) in sorted(sizes.items(), reverse=True))
    pixel_size_note = PIXEL_SIZE_NOTE.format(sizes=picture_text)
    default_px = config.limits.default_camera_px
    candidates = CandidateStore()
    samples = SampleStore(config.cameras.calibration_dir)
    kin = robot.arm.kin
    floor_note = "Frame: base_link (x forward, y left, floor z = 0)."

    def map_pose() -> BasePose | None:
        """Current map pose, None when unknown."""
        try:
            return robot.robot_state().pose
        except RobotError:
            return None

    def arm_joints() -> dict[str, float] | None:
        """Measured arm joints from a fresh sample, None when stale (the tool point overlay)."""
        sample = robot.arm.fresh_sample()
        return None if sample is None else robot.arm.measured(sample)

    gate_mm = config.camera_arm.max_residual_mm
    camera_arm = config.camera_arm

    def pose_at(stamp_s: float, what: str) -> HeadPoseAtStamp:
        """Head pan/tilt at a capture stamp (from the /head/joint_states history).

        Raises ValueError when the stamp is not covered by the history or the head moved around it.
        """
        pose = robot.head.pose_at(stamp_s)
        if pose.moving:
            raise ValueError(
                f"the head moved {math.degrees(pose.window_motion_rad):.2f} deg within +-{config.head.stamp_window_s:g} s "
                f"of the {what} (limit {math.degrees(config.head.max_motion_rad):.2f} deg): the pan/tilt at the capture "
                "is not reliable, let the head stop and look again"
            )
        return pose

    def scene_for(camera: Camera, pose: HeadPoseAtStamp | None = None) -> tuple[Scene, CameraSetup]:
        """Scene of a calibrated camera at the head pose of a capture stamp, or at the measured head pose."""
        setup = camera_setup(config, camera, source=robot)
        if pose is None:
            t_parent = head_parent_transform(config, robot.head)
        else:
            t_parent = head_parent_transform_at(config, robot.head, pose.pan, pose.tilt)
        return build_scene(setup, config, t_parent), setup

    def grab(camera: Camera, setup: CameraSetup) -> tuple[np.ndarray, float]:
        """Fresh frame as a BGR image at the calibrated size, with its capture stamp."""
        intr = setup.intrinsics
        frame = robot.camera_image(camera, min(HARD_MAX_IMAGE_PX, max(intr.width, intr.height)))
        img = cv2.imdecode(np.frombuffer(frame.jpeg, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            raise RobotError(f"cannot decode the {camera} camera frame")
        if img.shape[1] != intr.width or img.shape[0] != intr.height:
            img = cv2.resize(img, (intr.width, intr.height), interpolation=cv2.INTER_AREA)
        return img, frame.stamp_s

    def framed_scene(camera: Camera) -> tuple[Scene, CameraSetup, np.ndarray]:
        """A fresh frame and the scene placed with the head pose at that frame's capture stamp."""
        setup = camera_setup(config, camera, source=robot)  # not calibrated: refused before any capture
        img, stamp_s = grab(camera, setup)
        scene, _ = scene_for(camera, pose_at(stamp_s, "camera frame"))
        return scene, setup, img

    def current_pose_block() -> dict[str, Any]:
        """Head pose block of a tool that uses the current measured pose (no capture stamp)."""
        angles = robot.head.measured() or {}
        pan, tilt = (angles.get(name, math.nan) for name in config.head.joint_names)
        motion = robot.head.recent_motion_rad()
        return {
            "pan_deg": round(math.degrees(pan), 4),
            "tilt_deg": round(math.degrees(tilt), 4),
            "stamp_s": None,
            "nearest_sample_gap_s": None,
            "window_motion_deg": None if motion is None else round(math.degrees(motion), 4),
            "source": "current_measured",
        }

    def encode(img: np.ndarray) -> bytes:
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, config.limits.jpeg_quality])
        if not ok:
            raise RobotError("cannot encode the annotated image")
        return buf.tobytes()

    def candidate_image(camera: Camera) -> dict[str, int] | None:
        """Calibrated image size the candidate pixels refer to (None when the camera has no intrinsics)."""
        try:
            intr = camera_setup(config, camera, need_mount=False, source=robot).intrinsics
        except ValueError:
            return None
        return {"width": intr.width, "height": intr.height}

    def draw_point_with_floor(
        img: np.ndarray, scene: Scene, ref_point: np.ndarray, color: tuple[int, int, int], shape: str, label: str
    ) -> bool:
        """Draw a 3D point and its floor projection joined by a line; False when the point itself is off-image."""
        floor_point = np.array([ref_point[0], ref_point[1], scene.ground_z])
        tip, foot = scene.project(ref_point), scene.project(floor_point)
        if foot is not None:
            draw_marker(img, foot, color, "cross")
        if tip is not None:
            draw_marker(img, tip, color, shape, label)
        if tip is not None and foot is not None:
            draw_polylines(img, [[tip, foot]], color)
        return tip is not None or foot is not None

    @server.tool(
        description=(
            "Position seen at a camera pixel (read-only). Projects the pixel ray through the configured camera "
            "model and mount onto a horizontal plane: the flat floor by default, or the surface the pixel lies on when "
            "surface_height_m is given (height relative to the robot's floor, positive above, negative below: the top "
            "of a 3 cm box is 0.03, a floor 10 cm lower is -0.10); the result reports surface_height_m and the "
            "coordinates have z = floor + surface_height_m. 'front' is the D435 colour camera on the pan/tilt head, "
            "posed from the head pan/tilt at image_stamp_s (the stamp_s of the picture you read the pixel from) or, "
            "without it, the CURRENT measured angles (refused while the head moves). Returns ground_base_link "
            "{x,y,z} (m, x forward, y left, floor z = 0), ground_arm {frame 'arm', x, y, z}: the same point in the arm "
            "base frame (what move_arm_cartesian and ObjectSpec frame 'arm' take), ground_map {x,y} when the map pose "
            "is known, distance_from_base_m, bearing_deg, method, uncertainty_note, head_pose_at_capture, head_moving "
            "and calibration {residual_mm}. The height is a GUESS (surface_height_m): to pick up an object use "
            f"pixel_to_point, which measures its position and height with the D435 depth. {floor_note} Pixel (u right, "
            f"v down). {pixel_size_note} The result reports pixel "
            "(calibrated size), image (calibrated size) and pixel_input (what you passed) when scaled. Errors if the "
            "camera is not calibrated (intrinsics and mount missing in the config), the pixel does not see the floor "
            "or the head is moving."
        )
    )
    def pixel_to_ground(
        camera: Annotated[Camera, Field(description="'front' (the head D435 colour camera)")],
        u: Annotated[float, Field(description="Pixel x (right)")],
        v: Annotated[float, Field(description="Pixel y (down)")],
        surface_height_m: SurfaceHeight = 0.0,
        image_width: ImageWidth = None,
        image_height: ImageHeight = None,
        image_stamp_s: ImageStampS = None,
    ) -> CallToolResult:
        """Pixel -> floor point; the tool description is passed to the decorator (it states the frames)."""
        with camera_errors():
            if image_stamp_s is None:
                scene, _ = scene_for(camera)
                motion = robot.head.recent_motion_rad()
                if motion is not None and motion > config.head.max_motion_rad:
                    raise ValueError(
                        f"the head is moving ({math.degrees(motion):.2f} deg in the last "
                        f"{config.head.stamp_window_s:g} s): the current pan/tilt does not place the picture; wait for "
                        "it to stop or pass image_stamp_s"
                    )
                head_extra: dict[str, Any] = {"head_pose_at_capture": current_pose_block()}
                head_extra["head_moving"] = None if motion is None else False
            else:
                pose = pose_at(image_stamp_s, "picture")
                scene, _ = scene_for(camera, pose)
                head_extra = {"head_pose_at_capture": head_pose_block(pose), "head_moving": False}
            scale = pixel_scale(image_width, image_height, scene.intr.width, scene.intr.height, default_px)
            pixel = (u, v) if scale is None else (u * scale, v * scale)
            report = ground_report(scene, camera, pixel[0], pixel[1], map_pose(), surface_height_m)
            if scale is not None:
                assert image_width is not None and image_height is not None
                report["pixel_input"] = pixel_input(u, v, image_width, image_height, scale)
            report.update(head_extra)
            report["calibration"] = calibration_block(robot.head.calibration(), gate_mm)
            return data_result(report)

    def point_report(
        frames: PointFrames,
        sigma_m: float | None,
        stamp_s: float,
        pose: HeadPoseAtStamp,
        approximate: bool,
        camera_frame: str = COLOR_OPTICAL_FRAME,
    ) -> dict[str, Any]:
        """The frames, error budget, head pose and calibration parts shared by pixel_to_point and camera_point_to_arm."""
        info = robot.head.calibration()
        calibration = calibration_block(info, gate_mm)
        map_xy = None
        map_state = map_pose()
        if map_state is not None:
            mx, my = base_to_map(map_state, float(frames.base_link[0]), float(frames.base_link[1]))
            map_xy = {
                "frame": MAP_FRAME,
                "x": round(mx, 3),
                "y": round(my, 3),
                "z": round(float(frames.base_link[2]), 3),
            }
        residual_mm = calibration["residual_mm"]
        budget = None
        if residual_mm is not None:
            budget = round(math.hypot(residual_mm, (sigma_m or 0.0) * MM_PER_M), 2)
        ready = calibration["within_gate"] is True and frames.arm is not None
        out: dict[str, Any] = {
            "camera_optical": position(camera_frame, frames.camera),
            "base_link": position(BASE_FRAME, frames.base_link),
            ARM_FRAME: position(ARM_FRAME, frames.arm),
            "map": map_xy,
            "uncertainty_m": None if sigma_m is None else round(sigma_m, 5),
            "error_budget_mm": budget,
            "head_pose_at_capture": head_pose_block(pose),
            "head_moving": False,
            "image_stamp_s": stamp_s,
            "calibration": calibration,
            "grasp_ready": ready,
            "notes": residual_notes(info, gate_mm, frames.arm is not None),
        }
        if approximate:
            out["notes"].append(APPROX_NOTE)
        return out

    @server.tool(
        description=(
            "Measured 3D position of what a colour pixel sees (read-only): reads the D435 aligned depth at (u, v) "
            "(median of the valid pixels of a window_px square after dropping flying/mixed pixels at depth edges), "
            "deprojects it with the colour intrinsics and converts it through the head pan/tilt AT THE FRAME STAMP and "
            "the applied head calibration. Returns camera_optical {frame 'd435_color_optical_frame', x right, y down, z "
            "forward}, base_link {frame, x forward, y left, z up from the floor}, arm {frame 'arm': the frame of "
            "move_arm_cartesian and ObjectSpec frame 'arm'; null while arm.base_in_base_link is not set}, map {x, y, z} "
            "when the map pose is known, depth {z_m, spread_m, valid_fraction, window_px, sigma_m}, uncertainty_m (the "
            "D435 z^2 depth error, 1 sigma; it grows with the square of the distance, so work from "
            f"{camera_arm.min_range_m:g} to {camera_arm.max_range_m:g} m), error_budget_mm (calibration residual and "
            "depth error combined), head_pose_at_capture, head_moving and calibration {source, session, result, "
            "residual_mm: the held-out camera-to-arm error of the applied head-from-arm calibration, gate_mm, "
            "within_gate}. grasp_ready is true only when the residual is known and within the "
            f"{gate_mm:g} mm gate and the arm point exists: pass position_source 'camera' on the ObjectSpec of a grasp "
            "built from this position, grasp_object refuses it otherwise. Use it for the position AND the height of an "
            "object (its top: z in the arm or base_link frame), not pixel_to_ground with a guessed surface_height_m. "
            "Aim at the object's interior (a pixel on its outline gives 'depth edge'), not nearer than "
            f"{camera_arm.min_range_m:g} m. Refuses (instead of guessing) when there is no depth, the colour and depth "
            "frames are not the same frameset, the head moved around the frame, or the pixel is on a hole, an edge or "
            f"beyond the range. {pixel_size_note}"
        )
    )
    def pixel_to_point(
        camera: Annotated[Camera, Field(description="'front' (the head D435 colour camera)")],
        u: Annotated[float, Field(description="Pixel x (right)")],
        v: Annotated[float, Field(description="Pixel y (down)")],
        window_px: WindowPx = None,
        image_width: ImageWidth = None,
        image_height: ImageHeight = None,
    ) -> CallToolResult:
        """Pixel + aligned depth -> camera, base_link, arm and map points; the description is passed to the decorator."""
        with camera_errors():
            setup = camera_setup(config, camera, source=robot)
            intr = setup.intrinsics
            scale = pixel_scale(image_width, image_height, intr.width, intr.height, default_px)
            pu, pv = (u, v) if scale is None else (u * scale, v * scale)
            frame = robot.aligned_depth(camera)
            check_registered(frame, intr)
            pose = pose_at(frame.stamp_s, "depth frame")
            scene, _ = scene_for(camera, pose)
            win = window_depth(frame.depth, pu, pv, window_px or camera_arm.default_window_px, camera_arm, intr.fx)
            point = pixel_to_point_camera(intr, pu, pv, win.z_m)
            frames = frames_of_point(scene.t_ref_optical, scene.t_arm_ref, point)
            report: dict[str, Any] = {
                "camera": camera,
                "pixel": {"u": pu, "v": pv},
                "image": {"width": intr.width, "height": intr.height},
            }
            if scale is not None:
                assert image_width is not None and image_height is not None
                report["pixel_input"] = pixel_input(u, v, image_width, image_height, scale)
            report.update(point_report(frames, win.sigma_m, frame.stamp_s, pose, scene.approximate))
            report["depth"] = depth_report(win)
            report["depth_frame"] = {
                "stamp_s": frame.stamp_s,
                "colour_stamp_s": frame.color_stamp_s,
                "pair_dt_s": round(abs(frame.color_stamp_s - frame.stamp_s), 4),
                "age_s": frame.age_s,
            }
            return data_result(report)

    @server.tool(
        description=(
            "Convert a 3D point already in a D435 optical frame (a marker pose, a cloud point; x right, y down, z "
            "forward, metres, z > 0) to base_link and the arm base frame (read-only), through the head pan/tilt at "
            "stamp_s and the applied head calibration. frame is 'd435_color_optical_frame' (the colour image and the "
            "aligned depth are in it) or 'd435_depth_optical_frame' (the raw depth, point clouds). stamp_s is the "
            "header stamp of the image or cloud the point came from: without it the newest head joint state is used, "
            "which is only right when the head has not moved since. Returns camera_optical {frame, x, y, z}, "
            "base_link, arm, head_pose_at_capture, head_moving, calibration {residual_mm, gate_mm, within_gate} and "
            "grasp_ready, as pixel_to_point. Refuses when the head moved around stamp_s or the stamp is outside the "
            "joint-state history."
        )
    )
    def camera_point_to_arm(
        frame: Annotated[CameraOpticalFrame, Field(description="Optical frame the point is expressed in")],
        x: Annotated[float, Field(description="m, right of the optical axis")],
        y: Annotated[float, Field(description="m, below the optical axis")],
        z: Annotated[float, Field(description="m, along the optical axis (forward); > 0")],
        stamp_s: Annotated[
            float | None, Field(description="Header stamp (ROS time, s) of the image or cloud the point came from")
        ] = None,
    ) -> CallToolResult:
        """Camera-frame point -> base_link and arm; the tool description is passed to the decorator."""
        with camera_errors():
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z) and z > 0.0):
                raise ValueError("z must be positive (the depth along the optical axis, in metres) and x, y finite")
            notes: list[str] = []
            if stamp_s is None:
                newest = robot.head.history.newest_stamp()
                if newest is None:
                    raise ValueError(
                        f"no {config.head.state_topic} samples received yet: the head pose is unknown "
                        "(pass stamp_s of the image)"
                    )
                if robot.head.measured() is None:
                    raise ValueError(
                        f"head joint states ({config.head.state_topic}) are missing or stale: the head pose is unknown"
                    )
                stamp_s = newest
                notes.append(
                    "no stamp_s given: the head pose is the newest joint state, right only if the head has not moved "
                    "since the point was measured"
                )
            pose = pose_at(stamp_s, "point's frame")
            t_base_optical = robot.head.kinematics.link_pose(pose.pan, pose.tilt, frame)
            offset = config.arm.base_in_base_link
            t_arm_base = None if offset is None else np.linalg.inv(offset_matrix(offset))
            frames = frames_of_point(t_base_optical, t_arm_base, np.array([x, y, z], dtype=float))
            report = point_report(frames, None, stamp_s, pose, False, frame)
            report["notes"] = [*notes, *report["notes"]]
            return data_result(report)

    @server.tool(
        structured_output=False,
        description=(
            "Take a fresh photo from a camera and draw metric overlays on it (read-only): 'grid' floor grid with (x,y) "
            "metre labels every grid_step_m in base_link; 'reach' the arm's reachable floor annulus; 'gripper' the "
            "current gripper tool point plus its projection straight down onto the floor; 'planned_gripper' a marker "
            "for planned_gripper {x,y,z} (arm base frame, as in move_arm_cartesian) so you can check the target BEFORE "
            "moving; 'lidar' latest scan points as range-coloured dots (red near, blue far; needs base_link "
            "coordinates). Returns a JPEG plus metadata (frame, overlays drawn, notes for overlays that could not be "
            "drawn). Errors if the camera is not calibrated."
        ),
    )
    def get_annotated_camera_image(
        camera: Annotated[Camera, Field(description="'front' (the head D435 colour camera)")],
        overlays: Annotated[
            list[Overlay], Field(description="Overlays to draw: grid, reach, gripper, planned_gripper, lidar")
        ] = Field(default_factory=lambda: ["grid"]),
        planned_gripper: Annotated[
            Point3 | None,
            Field(description="Planned tool point, arm base frame (needed for the planned_gripper overlay)"),
        ] = None,
        grid_step_m: Annotated[float, Field(ge=0.05, le=1.0, description="Floor grid spacing in metres")] = 0.1,
    ) -> list[ImageContent | TextContent]:
        """Annotated camera image; the tool description is passed to the decorator."""
        with camera_errors():
            wanted = list(dict.fromkeys(overlays))
            if "planned_gripper" in wanted and planned_gripper is None:
                raise ValueError("the planned_gripper overlay needs the planned_gripper {x, y, z} argument")
            scene, setup, img = framed_scene(camera)
            notes: list[str] = []
            meta: dict[str, Any] = {"camera": camera, "frame": scene.frame_name}
            legend = [f"{camera} camera, frame {scene.frame_name} (x fwd, y left, z up)"]
            if "grid" in wanted:
                draw_grid(img, scene, grid_step_m)
                legend.append(f"grid {grid_step_m:g} m, labels (x,y) m")
            if "reach" in wanted:
                pan = kin.pan_axis_xy
                centre = scene.arm_to_ref(np.array([pan[0], pan[1], 0.0]))
                if centre is None:
                    notes.append("reach not drawn: arm.base_in_base_link is not configured")
                else:
                    for radius in (config.arm.reach_outer_m, config.arm.reach_inner_m):
                        if radius > 0.0:
                            draw_polylines(img, circle_runs(scene, (centre[0], centre[1]), radius), COLOR_REACH)
                    legend.append(f"reach floor annulus {config.arm.reach_inner_m:g}-{config.arm.reach_outer_m:g} m")
            if "gripper" in wanted:
                joints = arm_joints()
                if joints is None:
                    notes.append("gripper not drawn: no fresh arm joint states")
                else:
                    pose = kin.forward(joints)
                    meta["tool_point"] = {"x": round(pose.x, 3), "y": round(pose.y, 3), "z": round(pose.z, 3)}
                    ref = scene.arm_to_ref(np.array([pose.x, pose.y, pose.z]))
                    if ref is None:
                        notes.append("gripper not drawn: arm.base_in_base_link is not configured")
                    elif draw_point_with_floor(img, scene, ref, COLOR_GRIPPER, "circle", "tool"):
                        legend.append("gripper tool point (circle) and floor foot (cross)")
                    else:
                        notes.append("gripper tool point is outside the camera view")
            if "planned_gripper" in wanted and planned_gripper is not None:
                ref = scene.arm_to_ref(np.array([planned_gripper.x, planned_gripper.y, planned_gripper.z]))
                if ref is None:
                    notes.append("planned_gripper not drawn: arm.base_in_base_link is not configured")
                elif draw_point_with_floor(img, scene, ref, COLOR_PLANNED, "circle", "plan"):
                    legend.append("planned gripper (circle) and floor foot (cross)")
                else:
                    notes.append("planned_gripper is outside the camera view")
            if "lidar" in wanted:
                scan = robot.scan_points()
                if scan is None:
                    notes.append("lidar not drawn: no fresh scan or no base_link <- laser transform")
                elif scene.t_base_ref is None:
                    notes.append("lidar not drawn: arm.base_in_base_link is not configured")
                else:
                    shown = draw_lidar(img, scene, np.array(scan.points, dtype=np.float64).reshape(-1, 4))
                    meta["lidar_points_shown"] = shown
                    legend.append(f"lidar {shown} pts (red near, blue far)")
            if scene.approximate:
                legend.append("APPROXIMATE intrinsics (hfov)")
            draw_legend(img, legend)
            meta.update(
                overlays=wanted,
                grid_step_m=grid_step_m,
                approximate_intrinsics=scene.approximate,
                image={"width": scene.intr.width, "height": scene.intr.height},
                notes=notes,
            )
            jpeg = encode(img)
            meta["lookup_id"] = record_lookup(
                ctx, "get_annotated_camera_image", camera, jpeg, scene.intr.width, scene.intr.height
            )
            return image_result(jpeg, meta)

    @server.tool(
        structured_output=False,
        description=(
            "Take a fresh photo and overlay numbered dots on a pixel grid (read-only) so you can pick floor targets "
            "by number: returns the JPEG and a table {set_id, surface_height_m, points:[{n, u, v, "
            "ground_base_link{x,y,z}|null, ground_map{x,y}|null}]}. "
            "Pass surface_height_m (height of the surface the dots lie on relative to the floor, positive above, "
            "negative below; 0 = the floor) when the target is on a box top or another level, so the points are "
            "intersected with that plane. Dots whose pixel does not see "
            f"the surface (sky, behind) or lies beyond {MAX_GROUND_RANGE_M:g} m are skipped and counted in "
            "skipped_no_ground. The last 10 sets are kept; look at the picture, choose a number, then call "
            "resolve_candidate(set_id, n). Optional region {u0,v0,u1,v1} limits the grid; spacing_px sets the grid; "
            "at most max_points dots (evenly thinned). The image and the points' u, v are in the calibrated image "
            "size (table 'image'); region is in that size too unless image_width / image_height give the size of "
            "the image you read it from (default get_camera_image pictures: "
            f"{picture_text}). Errors if the camera is "
            "not calibrated."
        ),
    )
    def mark_candidate_points(
        camera: Annotated[Camera, Field(description="'front' (the head D435 colour camera)")],
        region: Annotated[Region | None, Field(description="Pixel rectangle; the whole image when omitted")] = None,
        spacing_px: Annotated[
            int, Field(ge=8, le=320, description="Grid spacing in pixels of the calibrated image")
        ] = 40,
        max_points: Annotated[int, Field(ge=1, le=200, description="Maximum number of dots")] = 40,
        surface_height_m: SurfaceHeight = 0.0,
        image_width: ImageWidth = None,
        image_height: ImageHeight = None,
    ) -> list[ImageContent | TextContent]:
        """Numbered candidate dots; the tool description is passed to the decorator."""
        with camera_errors():
            scene, setup, img = framed_scene(camera)
            scale = pixel_scale(image_width, image_height, scene.intr.width, scene.intr.height, default_px) or 1.0
            pose = map_pose()
            box = (
                None if region is None else (region.u0 * scale, region.v0 * scale, region.u1 * scale, region.v1 * scale)
            )
            pixels = candidate_pixels(scene.intr.width, scene.intr.height, box, spacing_px, 10**6)
            valid: list[dict[str, Any]] = []
            skipped = 0
            for u, v in pixels:
                ground = scene.ground_point(u, v, surface_height_m)
                if ground is None or float(np.hypot(ground[0], ground[1])) > MAX_GROUND_RANGE_M:
                    skipped += 1
                    continue
                fields = ground_fields(scene, ground, pose)
                base = fields.get("ground_base_link")
                valid.append(
                    {
                        "u": round(u, 1),
                        "v": round(v, 1),
                        "ground_base_link": None if base is None else {"x": base["x"], "y": base["y"], "z": base["z"]},
                        "ground_map": fields.get("ground_map"),
                    }
                )
            kept = [{"n": n, **p} for n, p in enumerate(thin(valid, max_points), start=1)]
            snapshot = RobotSnapshot(None if pose is None else (pose.x, pose.y, pose.yaw), robot.head.measured())
            item = candidates.add(camera, snapshot, kept, surface_height_m)
            draw_candidates(img, [(p["n"], p["u"], p["v"]) for p in kept])
            draw_legend(img, [f"{camera} camera: {len(kept)} candidates, set {item.set_id}"])
            table = {
                "set_id": item.set_id,
                "camera": camera,
                "frame": scene.frame_name,
                "surface_height_m": surface_height_m,
                "image": {"width": scene.intr.width, "height": scene.intr.height},
                "points": kept,
                "skipped_no_ground": skipped,
                "approximate_intrinsics": scene.approximate,
            }
            return image_result(encode(img), table)

    @server.tool(
        description=(
            "Coordinates of a numbered candidate point from mark_candidate_points (read-only). Returns the values "
            "STORED when the set was made (they are not recomputed), age_s since then, and whether the base "
            "(robot_moved_since) or the pan/tilt head carrying the camera (head_moved_since) moved since; if the base "
            "moved, base_link coordinates are stale (ground_map stays valid), and if the head moved the pixel no "
            "longer matches the live image. "
            "surface_height_m is the plane the set was intersected with (given to mark_candidate_points). "
            "Only the last 10 sets are kept."
        )
    )
    def resolve_candidate(
        set_id: Annotated[str, Field(description="set_id returned by mark_candidate_points")],
        n: Annotated[int, Field(ge=1, description="Point number shown on the image")],
    ) -> CallToolResult:
        """Stored candidate coordinates; the tool description is passed to the decorator."""
        with camera_errors():
            try:
                item = candidates.get(set_id)
            except KeyError:
                raise ValueError(f"unknown candidate set {set_id!r} (only the last 10 sets are kept)") from None
            point = next((p for p in item.points if p["n"] == n), None)
            if point is None:
                raise ValueError(f"set {set_id} has no point {n} (points 1..{len(item.points)})")
            pose = map_pose()
            now_pose = None if pose is None else (pose.x, pose.y, pose.yaw)
            robot_moved = base_moved(item.snapshot.pose, now_pose)
            head_moved_flag = head_moved(item.snapshot.head, robot.head.measured())
            note = ["values are the ones stored when the set was made"]
            if robot_moved:
                note.append("the base moved since: ground_base_link is stale, ground_map is still valid")
            if head_moved_flag:
                note.append("the head moved since: the pixel no longer matches the live image")
            if robot_moved is None or head_moved_flag is None:
                note.append("motion check incomplete: the map pose or the head joint states were unavailable")
            return data_result(
                {
                    "set_id": set_id,
                    "camera": item.camera,
                    "surface_height_m": item.surface_height_m,
                    "point": point,
                    "image": candidate_image(item.camera),
                    "age_s": round(time.monotonic() - item.created, 1),
                    "robot_moved_since": robot_moved,
                    "head_moved_since": head_moved_flag,
                    "note": "; ".join(note),
                }
            )

    @server.tool(
        description=(
            "Store one calibration sample (read-only for the robot; writes a JSON file): the pixel (u, v) of a marker "
            "whose floor position you measured, with that ground point in base_link (floor z = 0) and the parent-link "
            "pose at this moment (the current head pan/tilt: the front camera's parent is a head link, so keep the "
            "head still or capture views at several head poses). Returns the sample count. Use several well spread "
            "markers, then call solve_camera_calibration. Works before the camera is calibrated. The pixel is in the "
            "calibrated image size (the intrinsics' width x height) unless image_width / image_height give the size "
            "of the image it came from (default get_camera_image pictures: "
            f"{picture_text}; scaling needs the intrinsics)."
        )
    )
    def capture_calibration_sample(
        camera: Annotated[Camera, Field(description="'front' (the head D435 colour camera)")],
        u: Annotated[float, Field(description="Marker pixel x in the image")],
        v: Annotated[float, Field(description="Marker pixel y in the image")],
        ground_x: Annotated[float, Field(description="Marker x (m) in base_link")],
        ground_y: Annotated[float, Field(description="Marker y (m) in base_link")],
        ground_z: Annotated[float, Field(description="Marker z (m) in base_link; 0 on the floor")] = 0.0,
        image_width: ImageWidth = None,
        image_height: ImageHeight = None,
    ) -> CallToolResult:
        """Store a calibration sample; the tool description is passed to the decorator."""
        with camera_errors():
            if image_width is not None or image_height is not None:
                intr = camera_setup(config, camera, need_mount=False, source=robot).intrinsics
                scale = pixel_scale(image_width, image_height, intr.width, intr.height, default_px) or 1.0
                u, v = u * scale, v * scale
            parent = str(getattr(config.cameras, camera).parent_frame)
            t_parent = head_parent_transform(config, robot.head)
            count = samples.add(camera, parent, t_parent, (u, v), (ground_x, ground_y, ground_z))
            return data_result(
                {
                    "camera": camera,
                    "samples": count,
                    "parent_frame": parent,
                    "file": str(samples.path(camera)),
                    "note": f"need at least {MIN_GOOD_SAMPLES} well spread samples for the 6 mount parameters",
                }
            )

    @server.tool(
        description=(
            "Fit the camera mount pose to the stored calibration samples (never edits the config). Needs the "
            "configured intrinsics (error otherwise) and a starting mount: the configured one or `initial` {x,y,z,roll,"
            "pitch,yaw} (m, rad; body frame x forward, y left, z up in the parent frame). Returns rms_px (aim for "
            "about 1 px or less), the sample count and a YAML snippet to paste under `cameras:` in the mcp_server "
            "section of ansible/group_vars/client.yml (then redeploy mcp_server)."
        )
    )
    def solve_camera_calibration(
        camera: Annotated[Camera, Field(description="'front' (the head D435 colour camera)")],
        initial: Annotated[
            MountGuess | None, Field(description="Starting mount; defaults to the configured one")
        ] = None,
    ) -> CallToolResult:
        """Solve the mount; the tool description is passed to the decorator."""
        with camera_errors():
            setup = camera_setup(config, camera, need_mount=False, source=robot)
            parent = str(getattr(config.cameras, camera).parent_frame)
            if initial is not None:
                start = MountPose(parent_frame=parent, **initial.model_dump())
            elif setup.mount is not None:
                start = setup.mount
            else:
                raise ValueError(
                    f"no starting mount for {camera}: set cameras.{camera}.mount in the config or pass `initial`"
                )
            stored_parent, stored = samples.load(camera)
            if stored and stored_parent != parent:
                raise CalibrationError(
                    f"stored samples use parent frame {stored_parent!r} but the config says {parent!r}; clear them"
                )
            mount, rms = solve_samples(stored, setup.intrinsics, start)
            notes = ["the config was not modified; paste the YAML into ansible/group_vars/client.yml and redeploy"]
            if len(stored) < MIN_GOOD_SAMPLES:
                notes.append(f"only {len(stored)} samples: the 6 mount parameters are not well determined")
            if rms > HIGH_RMS_PX:
                notes.append(f"rms {rms:.1f} px is high: check marker positions, intrinsics and the initial mount")
            if setup.approximate:
                notes.append(APPROX_NOTE)
            return data_result(
                {
                    "camera": camera,
                    "samples": len(stored),
                    "rms_px": round(rms, 3),
                    "mount": mount.model_dump(),
                    "yaml": mount_yaml(camera, mount),
                    "note": "; ".join(notes),
                }
            )

    @server.tool(
        description=(
            "Delete all stored calibration samples of a camera (the JSON file under the calibration directory); "
            "returns how many were removed. Use it to start a new calibration run."
        )
    )
    def clear_calibration_samples(
        camera: Annotated[Camera, Field(description="'front' (the head D435 colour camera)")],
    ) -> CallToolResult:
        """Delete calibration samples; the tool description is passed to the decorator."""
        with camera_errors():
            return data_result({"camera": camera, "removed": samples.clear(camera)})
