"""Camera tools: pixel -> ground, annotated images with overlays, numbered candidate points, mount calibration.

All tools are read-only (sensor class): they never command the robot. Pixel coordinates are in the calibrated image
size (the intrinsics width x height); every image these tools return has that size.
"""

import base64
import json
import time
from collections.abc import Iterator
from contextlib import contextmanager
from typing import Annotated, Any, Literal

import cv2
import numpy as np
from mcp.server.mcpserver.exceptions import ToolError
from mcp.types import CallToolResult, ImageContent, TextContent
from pydantic import BaseModel, Field
from ros2_common.camera_geometry import MountPose

from .arm import ArmError
from .camera_calib import CalibrationError, SampleStore, mount_yaml, solve_samples
from .camera_candidates import CandidateStore, RobotSnapshot, arm_moved, base_moved
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
from .camera_scene import (
    APPROX_NOTE,
    CameraSetup,
    Scene,
    build_scene,
    camera_setup,
    ground_fields,
    ground_report,
    parent_transform,
)
from .config import HARD_MAX_IMAGE_PX
from .models import BasePose, RobotError
from .tool_context import ToolContext

TOOL_NAMES = (
    "pixel_to_ground",
    "get_annotated_camera_image",
    "mark_candidate_points",
    "resolve_candidate",
    "capture_calibration_sample",
    "solve_camera_calibration",
    "clear_calibration_samples",
)
Camera = Literal["gripper", "front"]
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
Overlay = Literal["grid", "reach", "gripper", "planned_gripper", "lidar"]
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
    candidates = CandidateStore()
    samples = SampleStore(config.cameras.calibration_dir)
    kin = robot.arm.kin
    floor_note = (
        f"Frames: the front camera reports in base_link (x forward, y left, floor z = 0); the gripper camera in the arm "
        f"base frame (z = 0 at the arm mount plane, floor z = {config.arm.floor_z_m:.3f} m)"
        + (
            "; base_link coordinates are added because arm.base_in_base_link is configured."
            if config.arm.base_in_base_link is not None
            else "; arm.base_in_base_link is not configured, so gripper results have no base_link/map coordinates."
        )
    )

    def map_pose() -> BasePose | None:
        """Current map pose, None when unknown."""
        try:
            return robot.robot_state().pose
        except RobotError:
            return None

    def arm_joints(required: bool) -> dict[str, float] | None:
        """Measured arm joints; an error when required and stale, else None."""
        sample = robot.arm.require_sample() if required else robot.arm.fresh_sample()
        return None if sample is None else robot.arm.measured(sample)

    def scene_for(camera: str) -> tuple[Scene, CameraSetup, dict[str, float] | None]:
        """Scene of a calibrated camera at the current arm pose."""
        setup = camera_setup(config, camera)
        joints = arm_joints(required=camera == "gripper")
        return build_scene(setup, config, parent_transform(config, camera, kin, joints)), setup, joints

    def grab(camera: str, setup: CameraSetup) -> np.ndarray:
        """Fresh frame as a BGR image at the calibrated size."""
        intr = setup.intrinsics
        frame = robot.camera_image(camera, min(HARD_MAX_IMAGE_PX, max(intr.width, intr.height)))
        img = cv2.imdecode(np.frombuffer(frame.jpeg, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            raise RobotError(f"cannot decode the {camera} camera frame")
        if img.shape[1] != intr.width or img.shape[0] != intr.height:
            img = cv2.resize(img, (intr.width, intr.height), interpolation=cv2.INTER_AREA)
        return img

    def encode(img: np.ndarray) -> bytes:
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, config.limits.jpeg_quality])
        if not ok:
            raise RobotError("cannot encode the annotated image")
        return buf.tobytes()

    def arm_point_in_ref(scene: Scene, point: np.ndarray) -> np.ndarray | None:
        """Arm-base-frame point in the scene's reference frame (None when the arm offset is unknown)."""
        return point if scene.camera == "gripper" else scene.arm_to_ref(point)

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
            "coordinates have z = floor + surface_height_m. 'gripper' uses the CURRENT measured arm joints (the camera moves with "
            "the arm); 'front' is the fixed overhead camera. Returns ground_base_link {x,y,z} (m, x forward, y left, "
            "floor z = 0), ground_map {x,y} when the map pose is known, distance_from_base_m, bearing_deg, method and "
            f"uncertainty_note. {floor_note} Pixel (u right, v down) is in the calibrated image size, e.g. read it "
            "from get_annotated_camera_image or mark_candidate_points. Errors if the camera is not calibrated "
            "(intrinsics and mount missing in the config) or the pixel does not see the floor."
        )
    )
    def pixel_to_ground(
        camera: Annotated[Camera, Field(description="'gripper' or 'front'")],
        u: Annotated[float, Field(description="Pixel x (right)")],
        v: Annotated[float, Field(description="Pixel y (down)")],
        surface_height_m: SurfaceHeight = 0.0,
    ) -> CallToolResult:
        """Pixel -> floor point; the tool description is passed to the decorator (it states the frames)."""
        with camera_errors():
            scene, _, _ = scene_for(camera)
            return data_result(ground_report(scene, camera, u, v, map_pose(), surface_height_m))

    @server.tool(
        structured_output=False,
        description=(
            "Take a fresh photo from a camera and draw metric overlays on it (read-only): 'grid' floor grid with "
            "(x,y) metre labels every grid_step_m in the camera's frame (base_link for 'front', the arm base frame for "
            "'gripper'); 'reach' the arm's reachable floor annulus; 'gripper' the current gripper tool point plus its "
            "projection straight down onto the floor; 'planned_gripper' a marker for planned_gripper {x,y,z} (arm base "
            "frame, as in move_arm_cartesian) so you can check the target BEFORE moving; 'lidar' latest scan points as "
            "range-coloured dots (red near, blue far; needs base_link coordinates). Returns a JPEG plus metadata "
            "(frame, overlays drawn, notes for overlays that could not be drawn). Errors if the camera is not "
            "calibrated."
        ),
    )
    def get_annotated_camera_image(
        camera: Annotated[Camera, Field(description="'gripper' or 'front'")],
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
            scene, setup, joints = scene_for(camera)
            img = grab(camera, setup)
            notes: list[str] = []
            meta: dict[str, Any] = {"camera": camera, "frame": scene.frame_name}
            legend = [f"{camera} camera, frame {scene.frame_name} (x fwd, y left, z up)"]
            if "grid" in wanted:
                draw_grid(img, scene, grid_step_m)
                legend.append(f"grid {grid_step_m:g} m, labels (x,y) m")
            if "reach" in wanted:
                pan = kin.pan_axis_xy
                centre = arm_point_in_ref(scene, np.array([pan[0], pan[1], 0.0]))
                if centre is None:
                    notes.append("reach not drawn: arm.base_in_base_link is not configured")
                else:
                    for radius in (config.arm.reach_outer_m, config.arm.reach_inner_m):
                        if radius > 0.0:
                            draw_polylines(img, circle_runs(scene, (centre[0], centre[1]), radius), COLOR_REACH)
                    legend.append(f"reach floor annulus {config.arm.reach_inner_m:g}-{config.arm.reach_outer_m:g} m")
            if "gripper" in wanted:
                if joints is None:
                    joints = arm_joints(required=False)
                if joints is None:
                    notes.append("gripper not drawn: no fresh arm joint states")
                else:
                    pose = kin.forward(joints)
                    meta["tool_point"] = {"x": round(pose.x, 3), "y": round(pose.y, 3), "z": round(pose.z, 3)}
                    ref = arm_point_in_ref(scene, np.array([pose.x, pose.y, pose.z]))
                    if ref is None:
                        notes.append("gripper not drawn: arm.base_in_base_link is not configured")
                    elif draw_point_with_floor(img, scene, ref, COLOR_GRIPPER, "circle", "tool"):
                        legend.append("gripper tool point (circle) and floor foot (cross)")
                    else:
                        notes.append("gripper tool point is outside the camera view")
            if "planned_gripper" in wanted and planned_gripper is not None:
                ref = arm_point_in_ref(scene, np.array([planned_gripper.x, planned_gripper.y, planned_gripper.z]))
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
            return image_result(encode(img), meta)

    @server.tool(
        structured_output=False,
        description=(
            "Take a fresh photo and overlay numbered dots on a pixel grid (read-only) so you can pick floor targets "
            "by number: returns the JPEG and a table {set_id, surface_height_m, points:[{n, u, v, "
            "ground_base_link{x,y,z}|null, ground_map{x,y}|null}]} (the gripper camera also gives ground_arm_base). "
            "Pass surface_height_m (height of the surface the dots lie on relative to the floor, positive above, "
            "negative below; 0 = the floor) when the target is on a box top or another level, so the points are "
            "intersected with that plane. Dots whose pixel does not see "
            f"the surface (sky, behind) or lies beyond {MAX_GROUND_RANGE_M:g} m are skipped and counted in "
            "skipped_no_ground. The last 10 sets are kept; look at the picture, choose a number, then call "
            "resolve_candidate(set_id, n). Optional region {u0,v0,u1,v1} limits the grid; spacing_px sets the grid; "
            "at most max_points dots (evenly thinned). Errors if the camera is not calibrated."
        ),
    )
    def mark_candidate_points(
        camera: Annotated[Camera, Field(description="'gripper' or 'front'")],
        region: Annotated[Region | None, Field(description="Pixel rectangle; the whole image when omitted")] = None,
        spacing_px: Annotated[int, Field(ge=8, le=320, description="Grid spacing in pixels")] = 40,
        max_points: Annotated[int, Field(ge=1, le=200, description="Maximum number of dots")] = 40,
        surface_height_m: SurfaceHeight = 0.0,
    ) -> list[ImageContent | TextContent]:
        """Numbered candidate dots; the tool description is passed to the decorator."""
        with camera_errors():
            scene, setup, joints = scene_for(camera)
            pose = map_pose()
            img = grab(camera, setup)
            box = None if region is None else (region.u0, region.v0, region.u1, region.v1)
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
                point: dict[str, Any] = {
                    "u": round(u, 1),
                    "v": round(v, 1),
                    "ground_base_link": None if base is None else {"x": base["x"], "y": base["y"], "z": base["z"]},
                    "ground_map": fields.get("ground_map"),
                }
                if scene.camera == "gripper":
                    arm = fields["ground_arm_base"]
                    point["ground_arm_base"] = {"x": arm["x"], "y": arm["y"], "z": arm["z"]}
                valid.append(point)
            kept = [{"n": n, **p} for n, p in enumerate(thin(valid, max_points), start=1)]
            snapshot = RobotSnapshot(
                None if pose is None else (pose.x, pose.y, pose.yaw),
                joints if joints is not None else arm_joints(False),
            )
            item = candidates.add(camera, snapshot, kept, surface_height_m)
            draw_candidates(img, [(p["n"], p["u"], p["v"]) for p in kept])
            draw_legend(img, [f"{camera} camera: {len(kept)} candidates, set {item.set_id}"])
            table = {
                "set_id": item.set_id,
                "camera": camera,
                "frame": scene.frame_name,
                "surface_height_m": surface_height_m,
                "points": kept,
                "skipped_no_ground": skipped,
                "approximate_intrinsics": scene.approximate,
            }
            return image_result(encode(img), table)

    @server.tool(
        description=(
            "Coordinates of a numbered candidate point from mark_candidate_points (read-only). Returns the values "
            "STORED when the set was made (they are not recomputed), age_s since then, and whether the base "
            "(robot_moved_since) or the arm (arm_moved_since) moved since; if the base moved, base_link coordinates "
            "are stale (ground_map stays valid), and if the arm moved the pixel no longer matches the live image. "
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
            arm_moved_flag = arm_moved(item.snapshot.joints, arm_joints(required=False))
            note = ["values are the ones stored when the set was made"]
            if robot_moved:
                note.append("the base moved since: ground_base_link is stale, ground_map is still valid")
            if arm_moved_flag:
                note.append("the arm moved since: the pixel no longer matches the live image")
            if robot_moved is None or arm_moved_flag is None:
                note.append("motion check incomplete: the pose or joint states were unavailable")
            return data_result(
                {
                    "set_id": set_id,
                    "camera": item.camera,
                    "surface_height_m": item.surface_height_m,
                    "point": point,
                    "age_s": round(time.monotonic() - item.created, 1),
                    "robot_moved_since": robot_moved,
                    "arm_moved_since": arm_moved_flag,
                    "note": "; ".join(note),
                }
            )

    @server.tool(
        description=(
            "Store one calibration sample (read-only for the robot; writes a JSON file): the pixel (u, v) of a marker "
            "whose floor position you measured, with that ground point in the camera's reference frame (arm base "
            f"frame for 'gripper', where the floor is z = {config.arm.floor_z_m:.3f}; base_link for 'front', floor z = 0) "
            "and the parent-link pose at this moment (current arm joints for 'gripper', identity for 'front'). Returns "
            "the sample count. Use several well spread markers (and for 'gripper' several arm poses), then call "
            "solve_camera_calibration. Works before the camera is calibrated."
        )
    )
    def capture_calibration_sample(
        camera: Annotated[Camera, Field(description="'gripper' or 'front'")],
        u: Annotated[float, Field(description="Marker pixel x in the image")],
        v: Annotated[float, Field(description="Marker pixel y in the image")],
        ground_x: Annotated[float, Field(description="Marker x (m) in the camera's reference frame")],
        ground_y: Annotated[float, Field(description="Marker y (m) in the camera's reference frame")],
        ground_z: Annotated[float, Field(description="Marker z (m); 0 for the front camera on the floor")] = 0.0,
    ) -> CallToolResult:
        """Store a calibration sample; the tool description is passed to the decorator."""
        with camera_errors():
            parent = str(getattr(config.cameras, camera).parent_frame)
            joints = arm_joints(required=camera == "gripper")
            t_parent = parent_transform(config, camera, kin, joints)
            count = samples.add(camera, parent, t_parent, (u, v), (ground_x, ground_y, ground_z), joints=joints)
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
        camera: Annotated[Camera, Field(description="'gripper' or 'front'")],
        initial: Annotated[
            MountGuess | None, Field(description="Starting mount; defaults to the configured one")
        ] = None,
    ) -> CallToolResult:
        """Solve the mount; the tool description is passed to the decorator."""
        with camera_errors():
            setup = camera_setup(config, camera, need_mount=False)
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
        camera: Annotated[Camera, Field(description="'gripper' or 'front'")],
    ) -> CallToolResult:
        """Delete calibration samples; the tool description is passed to the decorator."""
        with camera_errors():
            return data_result({"camera": camera, "removed": samples.clear(camera)})
