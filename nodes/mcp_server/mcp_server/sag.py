"""Gravity sag model of the SO-101 arm: static gravity torques from the URDF inertials, predicted steady-state
deflection per joint, compensated joint targets, and the gain fit from logged (target, settled measured) pairs.

The STS3215 position servos are proportional controllers: under a static load torque tau a joint settles short of
its target by about k * tau (compliance k in rad per N m). The model computes tau for every gravity-loaded joint
(shoulder_lift, elbow_flex, wrist_flex; the vertical shoulder_pan axis and the wrist_roll axis carry about none)
from the link masses and centres of mass of the URDF, with the arm base level (gravity along -z of base_link). The
deflection d_j = k_j * tau_j (saturated at max_rad) points in the direction gravity turns the joint; commanding
target - d makes the joint settle on the target.

Everything here is pure (no ROS, no clock); ArmController applies it when arm.sag_compensation.enabled is set.
"""

import json
import math
import xml.etree.ElementTree as ET
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

BASE_LINK = "base_link"
# Joints the sag model predicts a deflection for (gravity-loaded pitch axes of the arm).
SAG_JOINTS = ("shoulder_lift", "elbow_flex", "wrist_flex")
GRAVITY_MPS2 = 9.81
GRAVITY = np.array([0.0, 0.0, -GRAVITY_MPS2])
# Marker of the structured settle record ArmController logs per arm move: "arm_settle {json}".
SETTLE_LOG_MARKER = "arm_settle"
# Approach modes of a joint: its final approach went against gravity (lifting), with gravity (lowering), or it is
# held at a measured pose (hold).
APPROACH_LIFTING = "lifting"
APPROACH_LOWERING = "lowering"
APPROACH_HOLD = "hold"
APPROACH_EPS_RAD = 1e-4  # a planned point closer than this to the goal does not count as the approach

JointPairs = Sequence[tuple[Mapping[str, float], Mapping[str, float]]]


@dataclass(frozen=True)
class UrdfJoint:
    """One URDF joint: its fixed origin transform, axis (joint frame) and parent/child links."""

    name: str
    parent: str
    child: str
    origin: np.ndarray  # 4x4 transform parent link -> joint frame at angle 0
    axis: np.ndarray  # unit 3-vector in the joint frame (zero for a fixed joint)
    movable: bool


def origin_transform(element: ET.Element | None) -> np.ndarray:
    """4x4 transform of a URDF <origin xyz rpy> element (identity when missing).

    Args:
        element (ET.Element | None): The origin element.

    Returns:
        np.ndarray: Homogeneous transform.
    """
    transform = np.eye(4)
    if element is None:
        return transform
    xyz = [float(v) for v in element.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in element.get("rpy", "0 0 0").split()]
    transform[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()  # URDF rpy: fixed axes x, y, z
    transform[:3, 3] = xyz
    return transform


class GravityModel:
    """Static gravity torques about the arm joints from the URDF link inertials (link masses and centres of mass)."""

    def __init__(self, urdf_path: Path, joints: Sequence[str] = SAG_JOINTS, base_link: str = BASE_LINK) -> None:
        """Parse the URDF tree below base_link.

        Args:
            urdf_path (Path): URDF file.
            joints (Sequence[str]): Joints whose gravity torque torques() reports.
            base_link (str): Root link; gravity is -z of this frame (level arm base).

        Raises:
            ValueError: For a requested joint that is not a movable URDF joint.
        """
        root = ET.parse(urdf_path).getroot()
        self.base_link = base_link
        self.masses: dict[str, float] = {}
        self.coms: dict[str, np.ndarray] = {}
        for link in root.findall("link"):
            inertial = link.find("inertial")
            mass = None if inertial is None else inertial.find("mass")
            if inertial is None or mass is None:
                continue
            self.masses[link.attrib["name"]] = float(mass.attrib["value"])
            self.coms[link.attrib["name"]] = origin_transform(inertial.find("origin"))[:3, 3]
        self.by_parent: dict[str, list[UrdfJoint]] = {}
        movable: dict[str, UrdfJoint] = {}
        for element in root.findall("joint"):  # top-level joints (not <transmission> ones)
            parent, child = element.find("parent"), element.find("child")
            if parent is None or child is None:
                raise ValueError(f"URDF {urdf_path}: joint {element.get('name')!r} lacks a parent or child link")
            axis_element = element.find("axis")
            axis_text = "1 0 0" if axis_element is None else axis_element.get("xyz", "1 0 0")
            axis = np.array([float(v) for v in axis_text.split()])
            is_movable = element.get("type") in ("revolute", "continuous")
            joint = UrdfJoint(
                name=element.attrib["name"],
                parent=parent.attrib["link"],
                child=child.attrib["link"],
                origin=origin_transform(element.find("origin")),
                axis=axis / np.linalg.norm(axis) if is_movable else np.zeros(3),
                movable=is_movable,
            )
            self.by_parent.setdefault(joint.parent, []).append(joint)
            if is_movable:
                movable[joint.name] = joint
        unknown = [j for j in joints if j not in movable]
        if unknown:
            raise ValueError(f"URDF {urdf_path} has no movable joints {unknown}")
        self.joints = tuple(joints)
        self.movable = movable
        self.subtree = {j: self.descendants(movable[j].child) for j in self.joints}

    def descendants(self, link: str) -> set[str]:
        """Links at and below a link.

        Args:
            link (str): Link name.

        Returns:
            set[str]: The link and every link below it.
        """
        out = {link}
        for joint in self.by_parent.get(link, []):
            out |= self.descendants(joint.child)
        return out

    def frames(self, angles: Mapping[str, float]) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
        """Link frames and joint frames (axis frames) in base_link for URDF joint angles.

        Args:
            angles (Mapping[str, float]): Joint name -> URDF rad (missing movable joints are 0).

        Returns:
            tuple[dict[str, np.ndarray], dict[str, np.ndarray]]: Link name -> 4x4 pose, joint name -> 4x4 pose of
                the joint frame (origin on the axis, axis in its local coordinates).
        """
        links = {self.base_link: np.eye(4)}
        joints: dict[str, np.ndarray] = {}
        stack = [self.base_link]
        while stack:
            parent = stack.pop()
            for joint in self.by_parent.get(parent, []):
                frame = links[parent] @ joint.origin
                joints[joint.name] = frame
                motion = np.eye(4)
                if joint.movable:
                    motion[:3, :3] = Rotation.from_rotvec(joint.axis * float(angles.get(joint.name, 0.0))).as_matrix()
                links[joint.child] = frame @ motion
                stack.append(joint.child)
        return links, joints

    def torques(self, angles: Mapping[str, float]) -> dict[str, float]:
        """Gravity torque about each modelled joint axis, positive along the joint's +axis (N m).

        Args:
            angles (Mapping[str, float]): Joint name -> URDF rad (missing movable joints are 0).

        Returns:
            dict[str, float]: Joint name -> torque the link weights exert about the joint (N m).
        """
        links, joints = self.frames(angles)
        weights = {
            name: (links[name][:3, :3] @ self.coms[name] + links[name][:3, 3], self.masses[name] * GRAVITY)
            for name in self.masses
            if name in links
        }
        out: dict[str, float] = {}
        for j in self.joints:
            frame = joints[j]
            axis = frame[:3, :3] @ self.movable[j].axis
            origin = frame[:3, 3]
            total = np.zeros(3)
            for name in self.subtree[j]:
                if name in weights:
                    com, force = weights[name]
                    total += np.cross(com - origin, force)
            out[j] = float(total @ axis)
        return out


def predict_deflection(torques: Mapping[str, float], gains: Mapping[str, float], max_rad: float) -> dict[str, float]:
    """Steady-state deflection d_j = k_j * tau_j, saturated at +-max_rad (rad, in the direction gravity turns j).

    Args:
        torques (Mapping[str, float]): Joint name -> gravity torque (N m).
        gains (Mapping[str, float]): Joint name -> compliance k (rad per N m); missing joints deflect 0.
        max_rad (float): Saturation of every joint's deflection (rad).

    Returns:
        dict[str, float]: Joint name -> deflection (rad) for every joint in torques.
    """
    return {j: max(-max_rad, min(max_rad, gains.get(j, 0.0) * tau)) for j, tau in torques.items()}


def compensate(
    target: Mapping[str, float],
    deflection: Mapping[str, float],
    limits: Mapping[str, tuple[float, float]],
    margin: float,
    overrides: Mapping[str, float] | None = None,
) -> dict[str, float]:
    """Commanded joints = target - deflection, kept inside the limit band [lower + margin, upper - margin].

    A target already outside the band (e.g. a hold at a measured pose inside the margin) is never pushed further
    out: its bound widens to the target itself. Joints without a deflection (or without limits) pass through.

    Args:
        target (Mapping[str, float]): Joint name -> target rad (same space as limits).
        deflection (Mapping[str, float]): Joint name -> predicted deflection (rad).
        limits (Mapping[str, tuple[float, float]]): Joint name -> (lower, upper) rad.
        margin (float): Margin kept from each limit (rad).
        overrides (Mapping[str, float] | None): Per-joint margins replacing margin.

    Returns:
        dict[str, float]: Commanded joint values.
    """
    out = dict(target)
    for j, d in deflection.items():
        if j not in target or j not in limits:
            continue
        lo, hi = limits[j]
        m = (overrides or {}).get(j, margin)
        value = target[j]
        out[j] = min(max(value - d, min(lo + m, value)), max(hi - m, value))
    return out


class SagCompensator:
    """Deflection of measured-space joint maps: GravityModel, the two approach gains, saturation, zero offsets.

    A servo with gearbox friction stops at one edge of its friction band: a joint whose last motion went against
    gravity (lifting) stops with the full load deflection, one that came down with gravity (lowering) stops early.
    Each joint therefore has a lifting gain k and a lowering gain k_lowering; a hold at a measured pose (lease
    acquire, stop, timeout) uses the middle of the band, (k + k_lowering) / 2, where the arm neither rises nor sags.
    """

    def __init__(
        self,
        model: GravityModel,
        k: Mapping[str, float],
        max_rad: float,
        offsets: Mapping[str, float] | None = None,
        k_lowering: Mapping[str, float] | None = None,
    ) -> None:
        """Bind the model to its gains.

        Args:
            model (GravityModel): Gravity torque model.
            k (Mapping[str, float]): Joint name -> compliance (rad per N m) after a lifting approach.
            max_rad (float): Deflection saturation (rad).
            offsets (Mapping[str, float] | None): Follower zero offsets, urdf = measured + offset.
            k_lowering (Mapping[str, float] | None): Joint name -> compliance after a lowering approach (missing = 0).
        """
        self.model = model
        self.k = dict(k)
        self.k_lowering = dict(k_lowering or {})
        self.max_rad = max_rad
        self.offsets = dict(offsets or {})

    def torques(self, measured: Mapping[str, float]) -> dict[str, float]:
        """Gravity torques of the modelled joints at a measured-space configuration.

        Args:
            measured (Mapping[str, float]): Joint name -> measured rad.

        Returns:
            dict[str, float]: Joint name -> torque (N m).
        """
        return self.model.torques({j: v + self.offsets.get(j, 0.0) for j, v in measured.items()})

    def deflection(self, measured: Mapping[str, float], gains: Mapping[str, float] | None = None) -> dict[str, float]:
        """Predicted deflection at a measured-space configuration (the pose the arm should settle on).

        Args:
            measured (Mapping[str, float]): Joint name -> measured rad.
            gains (Mapping[str, float] | None): Gains to apply; None for the lifting gains k.

        Returns:
            dict[str, float]: Joint name -> deflection (rad) for the modelled joints (offsets cancel: additive).
        """
        return predict_deflection(self.torques(measured), self.k if gains is None else gains, self.max_rad)

    def hold_modes(self) -> dict[str, str]:
        """Approach mode of every modelled joint for a hold at a measured pose.

        Returns:
            dict[str, str]: Joint name -> APPROACH_HOLD.
        """
        return {j: APPROACH_HOLD for j in self.model.joints}

    def gains_for(self, modes: Mapping[str, str]) -> dict[str, float]:
        """Gain per joint for its approach mode (lifting k, lowering k_lowering, hold the middle of both).

        Args:
            modes (Mapping[str, str]): Joint name -> approach mode.

        Returns:
            dict[str, float]: Joint name -> gain (rad per N m).
        """
        gains: dict[str, float] = {}
        for j, mode in modes.items():
            lifting, lowering = self.k.get(j, 0.0), self.k_lowering.get(j, 0.0)
            gains[j] = (
                lifting
                if mode == APPROACH_LIFTING
                else lowering
                if mode == APPROACH_LOWERING
                else (lifting + lowering) / 2.0
            )
        return gains

    def approach_modes(
        self, points: Sequence[Mapping[str, float]], goal: Mapping[str, float], prior: Mapping[str, str]
    ) -> dict[str, str]:
        """Approach mode of each joint from the final approach of a planned motion to its goal.

        The last planned point of a joint that differs from its goal gives the approach direction: toward the gravity
        torque at the goal is lowering, against it lifting. A joint the motion does not move keeps its prior mode.

        Args:
            points (Sequence[Mapping[str, float]]): Planned setpoints (measured space), ending at the goal.
            goal (Mapping[str, float]): Goal pose (every arm joint, measured space).
            prior (Mapping[str, str]): Current mode per joint.

        Returns:
            dict[str, str]: Joint name -> approach mode for every modelled joint.
        """
        torques = self.torques(goal)
        modes: dict[str, str] = {}
        for j in self.model.joints:
            before = next((p[j] for p in reversed(points) if j in p and abs(p[j] - goal[j]) > APPROACH_EPS_RAD), None)
            if before is None:
                modes[j] = prior.get(j, APPROACH_HOLD)
            else:
                modes[j] = APPROACH_LOWERING if (goal[j] - before) * torques[j] > 0.0 else APPROACH_LIFTING
        return modes

    @staticmethod
    def blend_gains(start: Mapping[str, float], end: Mapping[str, float], fraction: float) -> dict[str, float]:
        """Gains a fraction of the way from start to end (the compensation ramps along a motion, no step).

        Args:
            start (Mapping[str, float]): Gains at the motion start.
            end (Mapping[str, float]): Gains at the goal.
            fraction (float): Progress in [0, 1].

        Returns:
            dict[str, float]: Interpolated gains for every joint of end.
        """
        if fraction >= 1.0:
            return dict(end)
        return {j: start.get(j, 0.0) + (v - start.get(j, 0.0)) * fraction for j, v in end.items()}


def fit_gains(pairs: JointPairs, joints: Sequence[str] = SAG_JOINTS) -> dict[str, float]:
    """Least-squares compliance through the origin per joint: k_j = sum(tau d) / sum(tau^2), never negative.

    Args:
        pairs (JointPairs): (gravity torques N m, observed deflection rad = settled measured - target) per move.
        joints (Sequence[str]): Joints to fit.

    Returns:
        dict[str, float]: Joint name -> k (rad per N m); 0 for a joint without samples.
    """
    gains: dict[str, float] = {}
    for j in joints:
        rows = [(tau[j], d[j]) for tau, d in pairs if j in tau and j in d]
        denominator = sum(t * t for t, _ in rows)
        gains[j] = max(0.0, sum(t * d for t, d in rows) / denominator) if denominator > 0.0 else 0.0
    return gains


def rms_by_joint(
    pairs: JointPairs, gains: Mapping[str, float], max_rad: float, joints: Sequence[str] = SAG_JOINTS
) -> tuple[dict[str, float], dict[str, float]]:
    """RMS settled error per joint without and with compensation (assuming the arm settles d - prediction off).

    Args:
        pairs (JointPairs): (torques, observed deflection) per move.
        gains (Mapping[str, float]): Fitted gains.
        max_rad (float): Deflection saturation (rad).
        joints (Sequence[str]): Joints to report.

    Returns:
        tuple[dict[str, float], dict[str, float]]: (before, after) RMS per joint (rad); joints without samples omitted.
    """
    before: dict[str, float] = {}
    after: dict[str, float] = {}
    for j in joints:
        rows = [(tau, d) for tau, d in pairs if j in tau and j in d]
        if not rows:
            continue
        errors = [d[j] - predict_deflection({j: tau[j]}, gains, max_rad)[j] for tau, d in rows]
        before[j] = math.sqrt(sum(d[j] ** 2 for _, d in rows) / len(rows))
        after[j] = math.sqrt(sum(e * e for e in errors) / len(errors))
    return before, after


def cross_validate(
    pairs: JointPairs, groups: Sequence[str], joints: Sequence[str] = SAG_JOINTS, max_rad: float = 0.12
) -> dict[str, object]:
    """Leave-one-group-out validation: fit on every other group, score the held-out moves.

    Args:
        pairs (JointPairs): (torques, observed deflection) per move.
        groups (Sequence[str]): Group label per pair (e.g. the pose/touch name); moves of one group are held out
            together.
        joints (Sequence[str]): Joints to fit.
        max_rad (float): Deflection saturation (rad).

    Returns:
        dict[str, object]: folds, held-out RMS before/after per joint, reduction (1 - after/before) per joint, the
            gains fitted on all pairs and the per-fold gains.
    """
    held_errors: dict[str, list[float]] = {j: [] for j in joints}
    held_raw: dict[str, list[float]] = {j: [] for j in joints}
    fold_gains: dict[str, dict[str, float]] = {}
    for group in sorted(set(groups)):
        train = [p for p, g in zip(pairs, groups, strict=True) if g != group]
        test = [p for p, g in zip(pairs, groups, strict=True) if g == group]
        gains = fit_gains(train, joints)
        fold_gains[group] = gains
        for tau, d in test:
            for j in joints:
                if j in tau and j in d:
                    held_raw[j].append(d[j])
                    held_errors[j].append(d[j] - predict_deflection({j: tau[j]}, gains, max_rad)[j])
    before = {j: math.sqrt(sum(v * v for v in held_raw[j]) / len(held_raw[j])) for j in joints if held_raw[j]}
    after = {j: math.sqrt(sum(v * v for v in held_errors[j]) / len(held_errors[j])) for j in joints if held_errors[j]}
    reduction = {j: (1.0 - after[j] / before[j]) if before[j] > 0.0 else 0.0 for j in before}
    return {
        "folds": len(fold_gains),
        "before": before,
        "after": after,
        "reduction": reduction,
        "samples": {j: len(held_raw[j]) for j in joints if held_raw[j]},
        "gains": fit_gains(pairs, joints),
        "fold_gains": fold_gains,
    }


def pool_reports(reports: Sequence[Mapping[str, Mapping[str, float]]]) -> dict[str, dict[str, float]]:
    """Pool held-out RMS of several cross_validate reports (e.g. the lifting and the lowering moves), weighted by
    their sample counts.

    Args:
        reports (Sequence[Mapping[str, Mapping[str, float]]]): Reports with before, after and samples per joint.

    Returns:
        dict[str, dict[str, float]]: Pooled before, after, reduction and samples per joint.
    """
    sums: dict[str, list[float]] = {}
    for report in reports:
        for j, n in report["samples"].items():
            acc = sums.setdefault(j, [0.0, 0.0, 0.0])
            acc[0] += n * report["before"][j] ** 2
            acc[1] += n * report["after"][j] ** 2
            acc[2] += n
    before = {j: math.sqrt(b / n) for j, (b, _, n) in sums.items()}
    after = {j: math.sqrt(a / n) for j, (_, a, n) in sums.items()}
    return {
        "before": before,
        "after": after,
        "reduction": {j: (1.0 - after[j] / before[j]) if before[j] > 0.0 else 0.0 for j in before},
        "samples": {j: int(n) for j, (_, _, n) in sums.items()},
    }


def parse_settle_records(lines: Iterable[str]) -> list[dict[str, object]]:
    """Structured settle records ("arm_settle {json}", logged by ArmController per arm move) from journal lines.

    Args:
        lines (Iterable[str]): Journal lines (e.g. journalctl -u ros2-mcp_server output).

    Returns:
        list[dict[str, object]]: The decoded records in order; lines without a valid record are skipped.
    """
    marker = SETTLE_LOG_MARKER + " "
    records: list[dict[str, object]] = []
    for line in lines:
        at = line.find(marker)
        if at < 0:
            continue
        try:
            record = json.loads(line[at + len(marker) :])
        except json.JSONDecodeError:
            continue
        if isinstance(record, dict):
            records.append(record)
    return records


def record_pairs(
    records: Sequence[Mapping[str, object]], model: GravityModel, offsets: Mapping[str, float] | None = None
) -> dict[str, list[tuple[dict[str, float], dict[str, float], str]]]:
    """Fit pairs from settled arm_settle records, split by the approach mode of each joint.

    The observed deflection is measured - commanded (the servo compliance under load, whether or not compensation
    was on); the torque is evaluated at the settled measured pose. Joints held (mode hold) or unsettled records are
    skipped.

    Args:
        records (Sequence[Mapping[str, object]]): parse_settle_records output.
        model (GravityModel): Gravity torque model.
        offsets (Mapping[str, float] | None): Follower zero offsets of the logged poses (urdf = measured + offset).

    Returns:
        dict[str, list[tuple[dict[str, float], dict[str, float], str]]]: APPROACH_LIFTING / APPROACH_LOWERING ->
            ({joint: torque}, {joint: deflection}, group = record index) per joint and record.
    """
    out: dict[str, list[tuple[dict[str, float], dict[str, float], str]]] = {APPROACH_LIFTING: [], APPROACH_LOWERING: []}
    for index, record in enumerate(records):
        measured, commanded, modes = record.get("measured"), record.get("commanded"), record.get("modes")
        if not record.get("settled") or not isinstance(measured, dict) or not isinstance(commanded, dict):
            continue
        modes = modes if isinstance(modes, dict) else {}
        torques = model.torques({j: float(v) + (offsets or {}).get(j, 0.0) for j, v in measured.items()})
        for j in model.joints:
            mode = modes.get(j)
            if mode in out and j in measured and j in commanded:
                out[mode].append(({j: torques[j]}, {j: float(measured[j]) - float(commanded[j])}, str(index)))
    return out
