"""Replay a joint-space plan on the MuJoCo scene and judge it."""

import math
from bisect import bisect_right
from collections.abc import Callable
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from grasp_sim.config import SceneConfig, SimConfig
from grasp_sim.plan import JOINT_ORDER, JointSample, interpolate_joints, label_at, parse_plan
from grasp_sim.report import ClearanceStats, ContactEvent, ObjectReport, SegmentReport, SimReport
from grasp_sim.scene import FLOOR_GEOM, OBJECT_BODY, OBJECT_GEOM, SUPPORT_GEOM, build_model
from grasp_sim.tcp import jaw_shift

JAW_BODIES = ("gripper", "moving_jaw_so101_v1", "camera_mount")
WRIST_BODIES = ("wrist",)
GRIP_POINT_LOCAL = np.array([-0.0081, 0.0, -0.075])
SATURATION_FRACTION = 0.999
CLIP_WARN_RAD = 1e-6
LIFT_LABEL = "lift"
UNLABELLED = "unlabelled"
Observer = Callable[[mujoco.MjModel, mujoco.MjData, float, str | None], None]


class Rig:
    """Compiled scene plus the lookup tables the replay loop needs."""

    def __init__(self, scene: SceneConfig) -> None:
        """Compile the scene and classify every collision geom.

        Args:
            scene (SceneConfig): Scene configuration.
        """
        self.scene = scene
        self.model = build_model(scene)
        self.data = mujoco.MjData(self.model)
        self.qpos_adr = {j: int(self.model.joint(j).qposadr[0]) for j in JOINT_ORDER}
        self.ctrl_range = {j: self.model.actuator(j).ctrlrange.copy() for j in JOINT_ORDER}
        self.force_limit = {j: float(self.model.actuator(j).forcerange[1]) for j in JOINT_ORDER}
        self.offsets = scene.joint_offsets_rad.model_dump()
        self.role: dict[int, str] = {}
        for gid in range(self.model.ngeom):
            if not (self.model.geom_contype[gid] or self.model.geom_conaffinity[gid]):
                continue
            name = self.model.geom(gid).name
            body = self.model.body(self.model.geom_bodyid[gid]).name
            if name == FLOOR_GEOM:
                self.role[gid] = "floor"
            elif name == SUPPORT_GEOM:
                self.role[gid] = "support"
            elif name == OBJECT_GEOM:
                self.role[gid] = "object"
            elif body in JAW_BODIES:
                self.role[gid] = "jaw"
            elif body in WRIST_BODIES:
                self.role[gid] = "wrist"
            else:
                self.role[gid] = "arm"
        self.groups = {r: [g for g, v in self.role.items() if v == r] for r in ("jaw", "wrist", "floor", "support")}
        self.has_object = scene.object is not None
        self.grip_local = GRIP_POINT_LOCAL + jaw_shift(scene.tool_offset_m)

    def to_ctrl(self, measured: dict[str, float], warnings: set[str]) -> np.ndarray:
        """Measured joint angles to clipped MuJoCo actuator targets (urdf = measured + offset).

        Args:
            measured (dict[str, float]): Joint name -> measured rad.
            warnings (set[str]): Collects a message per joint that had to be clipped.

        Returns:
            np.ndarray: Control vector in actuator order.
        """
        out = np.zeros(len(JOINT_ORDER))
        for i, joint in enumerate(JOINT_ORDER):
            value = measured[joint] + self.offsets[joint]
            low, high = self.ctrl_range[joint]
            clipped = float(np.clip(value, low, high))
            if abs(clipped - value) > CLIP_WARN_RAD:
                warnings.add(f"{joint} target {value:.3f} rad (urdf) clipped to [{low:.3f}, {high:.3f}]")
            out[i] = clipped
        return out

    def geom_name(self, gid: int) -> str:
        """Readable geom name (falls back to body name and index for unnamed geoms)."""
        name = self.model.geom(gid).name
        return name or f"{self.model.body(self.model.geom_bodyid[gid]).name}#{gid}"

    def min_distance(self, geoms: list[int], surfaces: list[int], distmax: float) -> float | None:
        """Smallest geom-to-geom distance between two geom lists (m, negative = penetration)."""
        if not geoms or not surfaces:
            return None
        best = distmax
        fromto = np.zeros(6)
        for a in geoms:
            for b in surfaces:
                best = min(best, mujoco.mj_geomDistance(self.model, self.data, a, b, distmax, fromto))
        return best

    def grip_point(self) -> np.ndarray:
        """World position of the point between the jaws near their tips."""
        body = self.data.body("gripper")
        return body.xpos + body.xmat.reshape(3, 3) @ self.grip_local


def classify_contact(role_a: str, role_b: str) -> str | None:
    """Name a contact kind from the two geom roles, or None for contacts that are not judged.

    Args:
        role_a (str): Role of the first geom (jaw, wrist, arm, floor, support, object).
        role_b (str): Role of the second geom.

    Returns:
        str | None: arm_floor, arm_support, jaw_object, arm_object, jaw_floor, jaw_support, or None.
    """
    pair = {role_a, role_b}
    arm_roles = pair & {"jaw", "wrist", "arm"}
    other = pair - {"jaw", "wrist", "arm"}
    if len(arm_roles) != 1 or len(other) != 1 or role_a == role_b:
        return None
    arm_role, surface = next(iter(arm_roles)), next(iter(other))
    if surface in ("floor", "support"):
        return f"jaw_{surface}" if arm_role == "jaw" else f"arm_{surface}"
    return "jaw_object" if arm_role == "jaw" else "arm_object"


def object_tilt_deg(xmat: np.ndarray) -> float:
    """Angle between the object's z axis and world up.

    Args:
        xmat (np.ndarray): Row-major 3x3 (or flat 9) body rotation matrix.

    Returns:
        float: Tilt in degrees.
    """
    return math.degrees(math.acos(float(np.clip(np.asarray(xmat).reshape(3, 3)[2, 2], -1.0, 1.0))))


def is_unintended(kind: str, label: str | None, cfg: SimConfig) -> bool:
    """Whether a contact of this kind in this label counts against the plan."""
    if kind in ("jaw_floor", "jaw_support"):
        return not cfg.allow_jaw_surface_contact
    if kind in ("arm_floor", "arm_support"):
        return True
    return label not in cfg.contact_allowed_labels


def lift_state(rig: Rig, start_z: float, cfg: SimConfig) -> tuple[float, bool]:
    """Object height gain over its start and whether it is up and still between the jaws.

    Args:
        rig (Rig): Rig with current data.
        start_z (float): Object centre height before the replay.
        cfg (SimConfig): Thresholds.

    Returns:
        tuple[float, bool]: Height gain in m, and True if it exceeds lift_min_height_m with the object near the jaws.
    """
    pos = rig.data.body(OBJECT_BODY).xpos
    gain = float(pos[2] - start_z)
    near = float(np.linalg.norm(pos - rig.grip_point())) <= cfg.lift_hold_distance_m
    return gain, gain >= cfg.lift_min_height_m and near


class Tracker:
    """Accumulates per-label statistics, contacts and object motion while stepping."""

    def __init__(self, rig: Rig, cfg: SimConfig, plan: list[JointSample]) -> None:
        """Start tracking from the rig's settled state.

        Args:
            rig (Rig): Rig after settling.
            cfg (SimConfig): Thresholds.
            plan (list[JointSample]): Parsed plan (for the label timeline).
        """
        self.rig, self.cfg = rig, cfg
        self.segments: dict[str | None, dict[str, Any]] = {}
        self.first_contact: ContactEvent | None = None
        self.counts: dict[str, int] = {}
        self.max_force = dict.fromkeys(JOINT_ORDER, 0.0)
        self.saturated: set[str] = set()
        self.approach_tilt = 0.0
        self.approach_disp = 0.0
        self.lift_result: tuple[float, bool] | None = None
        self.step = 0
        self.start_pos = rig.data.body(OBJECT_BODY).xpos.copy() if rig.has_object else None
        self.plan_has_lift = any(s.label == LIFT_LABEL for s in plan)

    def segment(self, label: str | None, t: float) -> dict[str, Any]:
        """Get or create the accumulator for a label."""
        if label not in self.segments:
            self.segments[label] = {
                "t_start": t,
                "t_end": t,
                "clear": dict.fromkeys(("jaws_floor", "jaws_support", "wrist_floor", "wrist_support")),
                "force": 0.0,
                "tilt": 0.0 if self.rig.has_object else None,
                "disp": 0.0 if self.rig.has_object else None,
            }
        return self.segments[label]

    def record(self, label: str | None, t: float) -> None:
        """Record one physics step: clearances, forces, contacts and object motion."""
        rig, cfg, data = self.rig, self.cfg, self.rig.data
        seg = self.segment(label, t)
        seg["t_end"] = t
        forces = np.abs(data.actuator_force)
        for i, joint in enumerate(JOINT_ORDER):
            self.max_force[joint] = max(self.max_force[joint], float(forces[i]))
            if forces[i] >= SATURATION_FRACTION * rig.force_limit[joint]:
                self.saturated.add(joint)
        seg["force"] = max(seg["force"], float(forces.max()))
        if self.step % cfg.clearance_stride == 0:
            self.record_clearance(seg)
        self.record_contacts(label, t)
        if rig.has_object and self.start_pos is not None:
            pos = data.body(OBJECT_BODY).xpos
            tilt = object_tilt_deg(data.body(OBJECT_BODY).xmat)
            disp = float(np.linalg.norm(pos[:2] - self.start_pos[:2]))
            seg["tilt"], seg["disp"] = max(seg["tilt"], tilt), max(seg["disp"], disp)
            if label not in cfg.contact_allowed_labels:
                self.approach_tilt, self.approach_disp = max(self.approach_tilt, tilt), max(self.approach_disp, disp)
            if label == LIFT_LABEL:
                self.lift_result = lift_state(rig, float(self.start_pos[2]), cfg)
        self.step += 1

    def record_clearance(self, seg: dict[str, Any]) -> None:
        """Update the minimum jaw/wrist distances to floor and support."""
        rig, dist = self.rig, self.cfg.clearance_distmax_m
        for group, key in (("jaw", "jaws"), ("wrist", "wrist")):
            for surface in ("floor", "support"):
                value = rig.min_distance(rig.groups[group], rig.groups[surface], dist)
                slot = f"{key}_{surface}"
                if value is not None and (seg["clear"][slot] is None or value < seg["clear"][slot]):
                    seg["clear"][slot] = value

    def record_contacts(self, label: str | None, t: float) -> None:
        """Classify the active contacts and remember the first unintended one."""
        rig, data = self.rig, self.rig.data
        for c in data.contact[: data.ncon]:
            kind = classify_contact(rig.role.get(int(c.geom1), ""), rig.role.get(int(c.geom2), ""))
            if kind is None:
                continue
            self.counts[kind] = self.counts.get(kind, 0) + 1
            if self.first_contact is None and is_unintended(kind, label, self.cfg):
                self.first_contact = ContactEvent(
                    t=t, label=label, kind=kind, geom_a=rig.geom_name(int(c.geom1)), geom_b=rig.geom_name(int(c.geom2))
                )


def settle(rig: Rig, ctrl: np.ndarray, seconds: float) -> None:
    """Hold the first pose and let the object come to rest.

    Args:
        rig (Rig): Rig to step.
        ctrl (np.ndarray): Actuator targets to hold.
        seconds (float): Settling time in s.
    """
    for joint, value in zip(JOINT_ORDER, ctrl, strict=True):
        rig.data.qpos[rig.qpos_adr[joint]] = value
    rig.data.ctrl[:] = ctrl
    mujoco.mj_forward(rig.model, rig.data)
    for _ in range(int(seconds / rig.model.opt.timestep)):
        mujoco.mj_step(rig.model, rig.data)


def build_report(
    rig: Rig, tracker: Tracker, cfg: SimConfig, plan: list[JointSample], duration: float, warnings: set[str]
) -> SimReport:
    """Turn the tracker state into a SimReport with pass/fail reasons."""
    reasons: list[str] = []
    obj: ObjectReport | None = None
    grasp: bool | None = None
    if rig.has_object and tracker.start_pos is not None:
        pos = rig.data.body(OBJECT_BODY).xpos
        gain, at_end = lift_state(rig, float(tracker.start_pos[2]), cfg)
        after_lift = tracker.lift_result[1] if tracker.lift_result else None
        tipped = tracker.approach_tilt > cfg.tilt_threshold_deg
        pushed = tracker.approach_disp > cfg.push_threshold_m
        grasp = bool(at_end and (after_lift is None or after_lift))
        obj = ObjectReport(
            start_pos=tuple(float(v) for v in tracker.start_pos),
            final_pos=tuple(float(v) for v in pos),
            approach_max_tilt_deg=tracker.approach_tilt,
            approach_max_displacement_m=tracker.approach_disp,
            tipped=tipped,
            pushed=pushed,
            lift_height_m=gain,
            lifted_after_lift=after_lift,
            lifted_at_end=at_end,
        )
        if tipped:
            reasons.append(
                f"object tipped {tracker.approach_tilt:.1f} deg before the grasp (limit {cfg.tilt_threshold_deg})"
            )
        if pushed:
            reasons.append(
                f"object pushed {tracker.approach_disp * 1000:.1f} mm before the grasp (limit {cfg.push_threshold_m * 1000:.1f})"
            )
        if tracker.plan_has_lift and not grasp:
            reasons.append("grasp failed: object not lifted with the gripper")
    if tracker.first_contact is not None:
        c = tracker.first_contact
        reasons.insert(
            0, f"unintended {c.kind} contact in {c.label or UNLABELLED} at t={c.t:.2f}s ({c.geom_a} vs {c.geom_b})"
        )
    segments = [
        SegmentReport(
            label=label,
            t_start=s["t_start"],
            t_end=s["t_end"],
            min_clearance=ClearanceStats(**s["clear"]),
            max_actuator_force_nm=s["force"],
            max_object_tilt_deg=s["tilt"],
            max_object_displacement_m=s["disp"],
        )
        for label, s in tracker.segments.items()
    ]
    return SimReport(
        passed=not reasons,
        reasons=reasons,
        warnings=sorted(warnings),
        duration_s=duration,
        segments=segments,
        first_unintended_contact=tracker.first_contact,
        event_counts=dict(sorted(tracker.counts.items())),
        object=obj,
        grasp_success=grasp,
        max_actuator_force_nm=tracker.max_force,
        saturated_actuators=sorted(tracker.saturated),
    )


def simulate(
    plan_json: str | Path | list[Any] | dict[str, Any],
    scene_cfg: SceneConfig | None = None,
    sim_cfg: SimConfig | None = None,
    observer: Observer | None = None,
) -> SimReport:
    """Replay a timed joint plan and report clearances, contacts, object motion and grasp success.

    Args:
        plan_json (str | Path | list | dict): Replay plan (see grasp_sim.plan.parse_plan), joints in the measured
            joint space of mcp_server (radians); the scene's joint offsets map them to MuJoCo angles.
        scene_cfg (SceneConfig | None): Scene (defaults: box on the floor 0.2 m ahead).
        sim_cfg (SimConfig | None): Thresholds and options.
        observer (Observer | None): Called after every physics step with (model, data, plan label of the step)
            for live viewing or frame capture.

    Returns:
        SimReport: Per-label statistics, first unintended contact, object motion, grasp success, pass/fail reasons.
    """
    scene = scene_cfg or SceneConfig()
    cfg = sim_cfg or SimConfig()
    plan = parse_plan(plan_json)
    rig = Rig(scene)
    warnings: set[str] = set()
    settle(rig, rig.to_ctrl(plan[0].joints, warnings), cfg.settle_s)
    tracker = Tracker(rig, cfg, plan)
    labels = [label_at(plan, s.t) for s in plan]
    times = [s.t for s in plan]
    dt = rig.model.opt.timestep
    steps = math.ceil((plan[-1].t - plan[0].t + cfg.hold_end_s) / dt)
    for k in range(steps):
        t = plan[0].t + k * dt
        label = labels[max(bisect_right(times, t) - 1, 0)]
        rig.data.ctrl[:] = rig.to_ctrl(interpolate_joints(plan, t), warnings)
        mujoco.mj_step(rig.model, rig.data)
        tracker.record(label, t)
        if observer is not None:
            observer(rig.model, rig.data, t, label)
    return build_report(rig, tracker, cfg, plan, steps * dt, warnings)
