"""Programmatic MuJoCo scene: the vendored SO-101 plus floor, optional ledge/stair and a free box."""

from pathlib import Path

import mujoco
import numpy as np

from grasp_sim.config import SceneConfig
from grasp_sim.tcp import jaw_shift, shift_jaws

ARM_XML = Path(__file__).resolve().parents[2] / "assets" / "so101" / "so101.xml"
JOINT_NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
OBJECT_BODY = "object"
OBJECT_GEOM = "object_box"
FLOOR_GEOM = "floor"
SUPPORT_GEOM = "support"
RAIL_GEOMS = ("support_rail_left", "support_rail_right")
RAIL_WIDTH_M = 0.004
SLAB_THICKNESS_M = 0.3
FLOOR_BACK_X_M = -0.6
FLOOR_FRONT_X_M = 1.2
FLOOR_HALF_WIDTH_M = 0.8
OBJECT_DROP_GAP_M = 0.0005
OBJECT_SOLREF = (0.01, 1.0)
FLOOR_RGBA = (0.35, 0.4, 0.5, 1.0)
SUPPORT_RGBA = (0.6, 0.45, 0.3, 1.0)
OBJECT_RGBA = (0.1, 0.9, 0.2, 1.0)
BOX = mujoco.mjtGeom.mjGEOM_BOX


def add_slab(
    spec: mujoco.MjSpec,
    name: str,
    x_range: tuple[float, float],
    half_width: float,
    top_z: float,
    friction: tuple[float, float, float],
    rgba: tuple[float, float, float, float],
) -> None:
    """Add a static box slab whose top face is at top_z and spans x_range in the arm frame.

    Args:
        spec (mujoco.MjSpec): Model spec to extend.
        name (str): Geom name.
        x_range (tuple[float, float]): Slab extent along x in m.
        half_width (float): Half extent along y in m.
        top_z (float): Height of the top face in m.
        friction (tuple[float, float, float]): MuJoCo friction triple.
        rgba (tuple[float, float, float, float]): Colour.
    """
    half_x = (x_range[1] - x_range[0]) / 2
    spec.worldbody.add_geom(
        name=name,
        type=BOX,
        size=[half_x, half_width, SLAB_THICKNESS_M / 2],
        pos=[(x_range[0] + x_range[1]) / 2, 0.0, top_z - SLAB_THICKNESS_M / 2],
        friction=list(friction),
        rgba=list(rgba),
    )


def object_start_height(scene: SceneConfig) -> float:
    """Initial height of the object centre, resting on its support (m, arm frame).

    Args:
        scene (SceneConfig): Scene configuration with an object.

    Returns:
        float: Centre z in m.
    """
    assert scene.object is not None
    return scene.support_top_z + scene.object.gap_below_m + scene.object.size_m[2] / 2 + OBJECT_DROP_GAP_M


def add_rails(spec: mujoco.MjSpec, scene: SceneConfig, friction: tuple[float, float, float]) -> None:
    """Two static rails under the object's +-y edges (along its x axis) that hold it gap_below_m above the support.

    Args:
        spec (mujoco.MjSpec): Model spec to extend.
        scene (SceneConfig): Scene with an object whose gap_below_m > 0.
        friction (tuple[float, float, float]): MuJoCo friction triple.
    """
    assert scene.object is not None
    obj = scene.object
    across = np.array([-np.sin(obj.yaw_rad), np.cos(obj.yaw_rad)])
    offset = obj.size_m[1] / 2 - RAIL_WIDTH_M / 2
    quat = [float(np.cos(obj.yaw_rad / 2)), 0.0, 0.0, float(np.sin(obj.yaw_rad / 2))]
    for name, sign in zip(RAIL_GEOMS, (1.0, -1.0), strict=True):
        xy = np.array([obj.x_m, obj.y_m]) + sign * offset * across
        spec.worldbody.add_geom(
            name=name,
            type=BOX,
            size=[obj.size_m[0] / 2, RAIL_WIDTH_M / 2, obj.gap_below_m / 2],
            pos=[float(xy[0]), float(xy[1]), scene.support_top_z + obj.gap_below_m / 2],
            quat=quat,
            friction=list(friction),
            rgba=list(SUPPORT_RGBA),
        )


def build_model(scene: SceneConfig) -> mujoco.MjModel:
    """Compile the arm, the ground and the configured object into one MuJoCo model.

    Args:
        scene (SceneConfig): Scene configuration (arm frame: base at the origin, floor top at -base_height_m).

    Returns:
        mujoco.MjModel: Compiled model.
    """
    spec = mujoco.MjSpec.from_file(str(ARM_XML))
    for joint, (low, high) in scene.limit_overrides_rad.items():
        spec.joint(joint).range = [low, high]
        spec.actuator(joint).ctrlrange = [low, high]
    if scene.tool_offset_m is not None:
        shift_jaws(spec, jaw_shift(scene.tool_offset_m))
    kind = scene.support_kind
    floor_end = scene.support_start_x if kind == "stair" else FLOOR_FRONT_X_M
    add_slab(
        spec,
        FLOOR_GEOM,
        (FLOOR_BACK_X_M, floor_end),
        FLOOR_HALF_WIDTH_M,
        scene.floor_z,
        scene.support_friction,
        FLOOR_RGBA,
    )
    if kind != "floor":
        start = scene.support_start_x
        add_slab(
            spec,
            SUPPORT_GEOM,
            (start, start + scene.support_depth_m),
            scene.support_width_m / 2,
            scene.support_top_z,
            scene.support_friction,
            SUPPORT_RGBA,
        )
    if scene.object is not None and scene.object.gap_below_m > 0.0:
        add_rails(spec, scene, scene.support_friction)
    if scene.object is not None:
        obj = scene.object
        body = spec.worldbody.add_body(
            name=OBJECT_BODY,
            pos=[obj.x_m, obj.y_m, object_start_height(scene)],
            quat=[float(np.cos(obj.yaw_rad / 2)), 0.0, 0.0, float(np.sin(obj.yaw_rad / 2))],
        )
        body.add_freejoint()
        body.add_geom(
            name=OBJECT_GEOM,
            type=BOX,
            size=[s / 2 for s in obj.size_m],
            mass=obj.mass_kg,
            friction=list(obj.friction),
            solref=list(OBJECT_SOLREF),
            condim=4,
            rgba=list(OBJECT_RGBA),
        )
    return spec.compile()
