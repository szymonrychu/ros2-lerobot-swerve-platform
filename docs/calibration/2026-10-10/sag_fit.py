"""Fit the gravity sag gains of arm.sag_compensation (k after a lifting approach, k_lowering after a lowering one).

Inputs are listed in sag_fit.yaml: the 2026-10-10 touch-downs (session/touches.json) and any saved mcp_server
journals with arm_settle records. Run from the mcp_server node so mcp_server imports:

    cd nodes/mcp_server && uv run python ../../docs/calibration/2026-10-10/sag_fit.py

Touch-downs: each touch commanded move_arm_cartesian(x, y, z, pitch 1.57, wrist_roll -1.6) with the model deployed
at the time (the 2026-10-08 joint offsets and tool point below). The commanded joint targets are recomputed with that
model's IK (seeded along the same move chain: transit z 0, pre-hover z -0.074, hover z -0.094; lift z -0.074 and up
z 0 from the contact pose). Hover poses give the full deflection vector (measured - target); the lift and up moves
logged only the shoulder_lift residual (the other joints were inside their converge tolerance and are not used).
Torques use the current offsets (the best estimate of the physical joint angles).
"""

import json
import math
from pathlib import Path

import yaml
from mcp_server.config import McpServerConfig
from mcp_server.ik import ArmKinematics
from mcp_server.sag import (
    APPROACH_LIFTING,
    APPROACH_LOWERING,
    SAG_JOINTS,
    GravityModel,
    cross_validate,
    fit_gains,
    parse_settle_records,
    pool_reports,
    predict_deflection,
    record_pairs,
)

HERE = Path(__file__).resolve().parent
CONFIG_FILE = HERE / "sag_fit.yaml"
# Model deployed during the touch-downs (client.yml before commit 0dd6be1): 2026-10-08 offsets and tool point.
OLD_OFFSETS = {
    "shoulder_pan": -0.0619,
    "shoulder_lift": -0.0103,
    "elbow_flex": -0.1381,
    "wrist_flex": 0.2477,
    "wrist_roll": -0.0710,
}
OLD_TOOL = (0.0104, -0.0282, -0.0017)
LIMIT_OVERRIDES = {"shoulder_lift": (-1.745, 1.9)}
MARGIN_RAD = 0.05
MOUNT_X_M, MOUNT_Y_M = 0.0592, -0.05  # label frame (base_link) -> arm frame of the touch commands
PITCH, ROLL = 1.57, -1.6
HOVER_Z, PRE_HOVER_Z, LIFT_Z, UP_Z = -0.094, -0.074, -0.074, 0.0

Pair = tuple[dict[str, float], dict[str, float], str]


def mode_of(before: float, goal: float, torque: float) -> str:
    """Approach mode of a joint from its last planned value before the goal.

    Args:
        before (float): Joint value the approach started from (rad).
        goal (float): Goal value (rad).
        torque (float): Gravity torque at the goal (N m).

    Returns:
        str: APPROACH_LOWERING when the motion goes with gravity, else APPROACH_LIFTING.
    """
    return APPROACH_LOWERING if (goal - before) * torque > 0.0 else APPROACH_LIFTING


def touch_pairs(
    touches: list[dict], old: ArmKinematics, model: GravityModel, offsets: dict[str, float]
) -> tuple[dict[str, list[Pair]], list[dict]]:
    """Fit pairs of the touch-downs, split by approach mode, plus the hover poses for the tool-point check.

    Args:
        touches (list[dict]): touches.json entries.
        old (ArmKinematics): The model deployed at the time of the touches.
        model (GravityModel): Gravity torque model.
        offsets (dict[str, float]): Current zero offsets (torques are evaluated in this URDF space).

    Returns:
        tuple[dict[str, list[Pair]], list[dict]]: Pairs per mode, and per touch {name, target, measured, modes}.
    """
    split: dict[str, list[Pair]] = {APPROACH_LIFTING: [], APPROACH_LOWERING: []}
    hovers: list[dict] = []

    def torques(pose: dict[str, float]) -> dict[str, float]:
        return model.torques({j: v + offsets.get(j, 0.0) for j, v in pose.items()})

    for touch in touches:
        name = touch["name"]
        x = touch["commanded_bl_mm_deployed_model"][0] / 1000.0 - MOUNT_X_M
        y = touch["commanded_bl_mm_deployed_model"][1] / 1000.0 - MOUNT_Y_M
        measured = dict(touch["hover"]["joints"])
        transit = old.inverse(x, y, UP_Z, PITCH, seed=measured | {"wrist_roll": ROLL})
        pre = old.inverse(x, y, PRE_HOVER_Z, PITCH, seed=transit)
        hover = old.inverse(x, y, HOVER_Z, PITCH, seed=pre)
        tau = torques(measured)
        modes = {j: mode_of(pre[j], hover[j], tau[j]) for j in SAG_JOINTS}
        for j in SAG_JOINTS:
            split[modes[j]].append(({j: tau[j]}, {j: measured[j] - hover[j]}, name))
        hovers.append({"name": name, "target": hover, "measured": measured, "modes": modes})
        logged = {}
        for entry in touch["log"]:
            for label in ("lift", "up"):
                if entry.startswith(f"{label}: converged "):
                    logged[label] = json.loads(entry.split("converged ", 1)[1])
        if not logged:
            continue
        lift = old.inverse(x, y, LIFT_Z, PITCH, seed=touch["contact"]["joints"])
        up = old.inverse(x, y, UP_Z, PITCH, seed=lift)
        for label, start, goal in (("lift", touch["contact"]["joints"], lift), ("up", lift, up)):
            residual = logged.get(label, {})
            if "shoulder_lift" not in residual:
                continue
            settled = goal | {"shoulder_lift": goal["shoulder_lift"] - residual["shoulder_lift"]}
            t = torques(settled)["shoulder_lift"]
            mode = mode_of(start["shoulder_lift"], goal["shoulder_lift"], t)
            split[mode].append(({"shoulder_lift": t}, {"shoulder_lift": -residual["shoulder_lift"]}, name))
    return split, hovers


def tool_errors(
    hovers: list[dict], folds: dict[str, dict[str, dict[str, float]]], kin: ArmKinematics, max_rad: float
) -> dict:
    """Held-out tool-point error of the hover poses without and with compensation (mm).

    With compensation the joint lands at target + (deflection - prediction), the prediction from the gains fitted
    without that touch.

    Args:
        hovers (list[dict]): touch_pairs hover poses.
        folds (dict[str, dict[str, dict[str, float]]]): mode -> touch -> gains fitted without that touch.
        kin (ArmKinematics): Current kinematics (tool point).
        max_rad (float): Deflection saturation (rad).

    Returns:
        dict: Per touch {before_mm, after_mm} and the RMS of both.
    """
    out: dict[str, dict[str, float]] = {}
    for hover in hovers:
        target, measured = hover["target"], hover["measured"]
        offsets = kin.offsets
        tau = GRAVITY.torques({j: v + offsets.get(j, 0.0) for j, v in measured.items()})
        landed = dict(measured)
        for j in SAG_JOINTS:
            gains = folds[hover["modes"][j]].get(hover["name"], {})
            landed[j] = measured[j] - predict_deflection({j: tau[j]}, gains, max_rad)[j]
        goal = kin.forward(target)

        def error(pose: dict[str, float], goal: object = goal) -> float:
            p = kin.forward(pose)
            return 1000.0 * math.dist((p.x, p.y, p.z), (goal.x, goal.y, goal.z))

        out[hover["name"]] = {"before_mm": round(error(measured), 2), "after_mm": round(error(landed), 2)}
    rms = {
        key: round(math.sqrt(sum(v[key] ** 2 for v in out.values()) / len(out)), 2) for key in ("before_mm", "after_mm")
    }
    return {"per_touch": out, "rms": rms}


GRAVITY = GravityModel(McpServerConfig().arm.urdf_path)


def main() -> None:
    """Fit, validate (leave one touch / record out) and write sag_fit_results.json."""
    cfg = yaml.safe_load(CONFIG_FILE.read_text())
    urdf = McpServerConfig().arm.urdf_path
    offsets = {j: float(v) for j, v in cfg["joint_offsets_rad"].items()}
    max_rad = float(cfg["max_rad"])
    old = ArmKinematics(urdf, MARGIN_RAD, OLD_OFFSETS, OLD_TOOL, LIMIT_OVERRIDES)
    kin = ArmKinematics(urdf, MARGIN_RAD, offsets, tuple(cfg["tool_offset_m"]), LIMIT_OVERRIDES)
    touches = json.loads((HERE / cfg["touches"]).read_text())["touches"]
    split, hovers = touch_pairs(touches, old, GRAVITY, offsets)
    records = []
    for journal in cfg["journals"]:
        records += parse_settle_records((HERE / journal).read_text().splitlines())
    for mode, pairs in record_pairs(records, GRAVITY, offsets).items():
        split[mode] += [(tau, d, f"journal:{group}") for tau, d, group in pairs]
    reports = {}
    for mode, pairs in split.items():
        reports[mode] = cross_validate([(t, d) for t, d, _ in pairs], [g for _, _, g in pairs], SAG_JOINTS, max_rad)
    pooled = pool_reports([reports[APPROACH_LIFTING], reports[APPROACH_LOWERING]])
    everything = [p for pairs in split.values() for p in pairs]
    single = cross_validate([(t, d) for t, d, _ in everything], [g for _, _, g in everything], SAG_JOINTS, max_rad)
    folds = {mode: report["fold_gains"] for mode, report in reports.items()}
    results = {
        "inputs": {"touches": len(touches), "journal_records": len(records)},
        "samples": {mode: report["samples"] for mode, report in reports.items()},
        "k": fit_gains([(t, d) for t, d, _ in split[APPROACH_LIFTING]], SAG_JOINTS),
        "k_lowering": fit_gains([(t, d) for t, d, _ in split[APPROACH_LOWERING]], SAG_JOINTS),
        "held_out_by_mode": {
            mode: {key: report[key] for key in ("before", "after", "reduction", "samples")}
            for mode, report in reports.items()
        },
        "held_out_pooled": pooled,
        "held_out_single_gain": {key: single[key] for key in ("before", "after", "reduction", "samples", "gains")},
        "hover_tool_point": tool_errors(hovers, folds, kin, max_rad),
        "torque_range": {
            j: [
                round(min(t[j] for t, _, _ in everything if j in t), 4),
                round(max(t[j] for t, _, _ in everything if j in t), 4),
            ]
            for j in SAG_JOINTS
        },
    }
    (HERE / cfg["results"]).write_text(json.dumps(results, indent=2) + "\n")
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
