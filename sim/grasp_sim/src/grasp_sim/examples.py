"""Hand-built example grasp plans: a side-grasp "scoop" with the jaws skimming the support surface.

The scoop is derived by simple IK (grasp_sim.ik): the jaws open, the gripper descends along a steep line to the
object so the jaw tips end up next to the support, the jaws close, the object is lifted and the arm retreats.
The "tip" variant is misaligned so a jaw drives into the object during the approach (tipping / pushing).
"""

from typing import Literal

import numpy as np

from grasp_sim.config import BoxObjectConfig, SceneConfig
from grasp_sim.ik import ArmIK
from grasp_sim.plan import JOINT_ORDER, LABELS

Variant = Literal["pass", "tip"]
CaseName = Literal["floor", "ledge", "stair", "tip"]

JAW_INNER_FACE_X_M = -0.0105
JAW_CLEARANCE_M = 0.008
GRASP_DEPTH_Z_M = -0.085
APPROACH_PITCH_RAD = 1.0
WRIST_ROLL_URDF_RAD = -1.57
GRIPPER_OPEN_RAD = 0.6
GRIPPER_CLOSED_RAD = -0.1
PRE_GRASP_BACKOFF_M = 0.07
APPROACH_STEP_M = 0.005
APPROACH_SPEED_M_S = 0.03
JOINT_SPEED_RAD_S = 0.8
GRIPPER_SPEED_RAD_S = 0.5
SAMPLE_DT_S = 0.05
MIN_SEGMENT_S = 0.3
LIFT_HEIGHT_M = 0.06
RETREAT_BACK_M = 0.04
GRASP_HOLD_S = 0.3
TIP_LATERAL_OFFSET_M = 0.025
TIP_BOX = BoxObjectConfig(size_m=(0.02, 0.02, 0.07), mass_kg=0.03, x_m=0.2)
LEDGE_HEIGHT_M = 0.07
STAIR_DEPTH_M = 0.06
STAIR_OBJECT_X_M = 0.17
STAIR_PITCH_RAD = 1.2


def example_case(name: CaseName, base_height_m: float = 0.15) -> tuple[SceneConfig, list[dict]]:
    """A scene and its example scoop plan.

    - floor: 3x3x4 cm box on the floor, passing scoop.
    - ledge: the same box on a ledge 7 cm above the floor, passing scoop.
    - stair: the box on a stair 6 cm below the floor plane (closer, steeper approach), passing scoop.
    - tip: a tall narrow box on the floor and a sideways-offset approach: the jaw topples it (failing scoop).

    Args:
        name (CaseName): floor, ledge, stair or tip.
        base_height_m (float): Arm base height above the floor.

    Returns:
        tuple[SceneConfig, list[dict]]: Scene and replay-format plan (measured joint space).
    """
    floor_z = -base_height_m
    if name == "tip":
        scene = SceneConfig(base_height_m=base_height_m, object=TIP_BOX)
        return scene, build_plan(scene, "tip")
    if name == "stair":
        box = BoxObjectConfig(x_m=STAIR_OBJECT_X_M)
        scene = SceneConfig(base_height_m=base_height_m, support_z_m=floor_z - STAIR_DEPTH_M, object=box)
        return scene, build_plan(scene, "pass", STAIR_PITCH_RAD)
    support = floor_z + LEDGE_HEIGHT_M if name == "ledge" else None
    scene = SceneConfig(base_height_m=base_height_m, support_z_m=support, object=BoxObjectConfig(x_m=0.2))
    return scene, build_plan(scene, "pass")


def jaw_point(box: BoxObjectConfig) -> tuple[float, float, float]:
    """Point in the gripper_link frame that sits at the object centre when the jaws straddle it.

    Args:
        box (BoxObjectConfig): Object to grasp (its y size is the jaw gap direction at the example wrist roll).

    Returns:
        tuple[float, float, float]: Local point in m.
    """
    return (JAW_INNER_FACE_X_M + JAW_CLEARANCE_M + box.size_m[1] / 2, 0.0, GRASP_DEPTH_Z_M)


def build_plan(scene: SceneConfig, variant: Variant = "pass", pitch_rad: float = APPROACH_PITCH_RAD) -> list[dict]:
    """Build a replay-format scoop plan for the scene's object (joint values in the measured space).

    Args:
        scene (SceneConfig): Scene with an object.
        variant (Variant): "pass" aligns the jaws with the object, "tip" is offset sideways so a jaw hits it.
        pitch_rad (float): Approach pitch (rad, positive = jaws pointing down); steeper reaches lower targets.

    Returns:
        list[dict]: Samples {"t", "joints", "label"?} ready for grasp_sim.replay.simulate (JSON-serialisable).

    Raises:
        ValueError: If the scene has no object.
        grasp_sim.ik.IKError: If a waypoint is unreachable.
    """
    if scene.object is None:
        raise ValueError("the scoop plan needs an object in the scene")
    box = scene.object
    ik = ArmIK(scene)
    offsets = scene.joint_offsets_rad.model_dump()
    local = jaw_point(box)
    center = np.array([box.x_m, box.y_m, scene.support_top_z + box.size_m[2] / 2])
    if variant == "tip":
        center = center + np.array([0.0, TIP_LATERAL_OFFSET_M, 0.0])
    radial = center[:2] / np.linalg.norm(center[:2])
    approach_dir = np.array([radial[0] * np.cos(pitch_rad), radial[1] * np.cos(pitch_rad), -np.sin(pitch_rad)])

    def pose(target: np.ndarray, seed: dict[str, float] | None) -> dict[str, float]:
        return ik.solve(target, local, pitch_rad, WRIST_ROLL_URDF_RAD, seed=seed)

    pre = center - approach_dir * PRE_GRASP_BACKOFF_M
    q = pose(pre, None)
    samples: list[dict] = []
    clock = 0.0

    def emit(joints_urdf: dict[str, float], gripper: float, t: float, label: str | None) -> None:
        measured = {j: float(joints_urdf[j] - offsets[j]) for j in JOINT_ORDER[:5]}
        measured["gripper"] = gripper
        entry: dict = {"t": round(t, 4), "joints": measured}
        if label is not None:
            assert label in LABELS
            entry["label"] = label
        samples.append(entry)

    def move_to(path: list[np.ndarray], gripper: float, speed_m_s: float | None, label: str) -> None:
        nonlocal q, clock
        first = True
        for target in path:
            nxt = pose(target, q)
            delta = max(abs(nxt[j] - q[j]) for j in JOINT_ORDER[:5])
            dt = max(delta / JOINT_SPEED_RAD_S, SAMPLE_DT_S) if speed_m_s is None else APPROACH_STEP_M / speed_m_s
            clock += dt
            q = nxt
            emit(q, gripper, clock, label if first else None)
            first = False

    def hold_gripper(start: float, end: float, label: str) -> None:
        nonlocal clock
        steps = max(int(abs(end - start) / GRIPPER_SPEED_RAD_S / SAMPLE_DT_S), 1)
        for i in range(1, steps + 1):
            clock += SAMPLE_DT_S
            emit(q, start + (end - start) * i / steps, clock, label if i == 1 else None)

    emit(q, GRIPPER_CLOSED_RAD, clock, "open")
    hold_gripper(GRIPPER_CLOSED_RAD, GRIPPER_OPEN_RAD, "open")
    clock += MIN_SEGMENT_S
    emit(q, GRIPPER_OPEN_RAD, clock, "pre_grasp")
    steps = int(PRE_GRASP_BACKOFF_M / APPROACH_STEP_M)
    line = [pre + approach_dir * APPROACH_STEP_M * (i + 1) for i in range(steps)]
    move_to(line, GRIPPER_OPEN_RAD, APPROACH_SPEED_M_S, "approach")
    clock += GRASP_HOLD_S
    emit(q, GRIPPER_OPEN_RAD, clock, "grasp")
    hold_gripper(GRIPPER_OPEN_RAD, GRIPPER_CLOSED_RAD, "close")
    clock += GRASP_HOLD_S
    emit(q, GRIPPER_CLOSED_RAD, clock, None)
    up = int(LIFT_HEIGHT_M / APPROACH_STEP_M)
    lift = [center + np.array([0.0, 0.0, APPROACH_STEP_M * (i + 1)]) for i in range(up)]
    move_to(lift, GRIPPER_CLOSED_RAD, 0.06, "lift")
    top = center + np.array([0.0, 0.0, LIFT_HEIGHT_M])
    back = int(RETREAT_BACK_M / APPROACH_STEP_M)
    retreat = [top - np.array([radial[0], radial[1], 0.0]) * APPROACH_STEP_M * (i + 1) for i in range(back)]
    move_to(retreat, GRIPPER_CLOSED_RAD, 0.06, "retreat")
    return samples
