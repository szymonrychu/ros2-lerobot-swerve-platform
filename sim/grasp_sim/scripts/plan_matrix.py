"""Plan the grasp scenario matrix with the mcp_server planner and the deployed client.yml config.

Runs in the nodes/mcp_server uv environment (it imports mcp_server), writes <out>/index.json and one GraspPlan JSON
per scenario; `grasp-sim matrix <out>` (sim/grasp_sim environment) replays them:

    cd nodes/mcp_server && uv run python ../../sim/grasp_sim/scripts/plan_matrix.py --out /tmp/matrix
    cd sim/grasp_sim && uv run grasp-sim matrix /tmp/matrix

Optional --params <yaml/json> overrides grasp parameters on top of client.yml (for tuning runs).
"""

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import yaml
from mcp_server.config import ArmBaseOffset, McpServerConfig
from mcp_server.floor_guard import FloorGuard, JawModel, SurfaceModel
from mcp_server.grasp import GraspPlanner, ObjectSpec, grasp_params
from mcp_server.ik import ArmKinematics

REPO_ROOT = Path(__file__).resolve().parents[3]
CLIENT_GROUP_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
MCP_SERVER_NODE = "mcp_server"
# Executor default seed without fresh joint states (grasp_tools.DEFAULT_SEED): a folded arm.
SEED = {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow_flex": 1.2, "wrist_flex": 0.3, "wrist_roll": 0.0}
# Boxes: name -> (depth along the approach, width across the jaws, height) in m.
BOXES = {"4x4x4": (0.04, 0.04, 0.04), "3x3x6": (0.03, 0.03, 0.06), "6x6x3": (0.06, 0.06, 0.03)}
# Positions: name -> (x, y) of the object centre in the arm frame (m).
POSITIONS = {
    "r20": (0.20, 0.0),
    "r25": (0.25, 0.0),
    "r30": (0.30, 0.0),
    "r25a30": (0.25 * math.cos(math.radians(30)), 0.25 * math.sin(math.radians(30))),
}
# Supports: name -> height of the surface under the object above the floor (m; the floor is arm.floor_z_m, -0.104 in
# the arm frame): two ledges and a lower stair.
SUPPORTS = {"floor": 0.0, "ledge+0.07": 0.07, "ledge+0.15": 0.15, "stair-0.10": -0.10}
# Strategies: name -> (planner strategy, approach pitch deg for angled, gap under the object in m).
GAP_BELOW_M = 0.02
STRATEGIES = {
    "top_down": ("top_down", None, 0.0),
    "angled45": ("angled", 45.0, 0.0),
    "scoop": ("scoop", None, 0.0),
    "scoop_gap": ("scoop", None, GAP_BELOW_M),
    "auto": ("auto", None, 0.0),
}
INDEX_FILE = "index.json"


def client_config() -> McpServerConfig:
    """The deployed mcp_server config from ansible/group_vars/client.yml (urdf_path made absolute).

    Returns:
        McpServerConfig: Validated config.
    """
    group_vars = yaml.safe_load(CLIENT_GROUP_VARS.read_text())
    node = next(n for n in group_vars["ros2_nodes"] if n["name"] == MCP_SERVER_NODE)
    data = yaml.safe_load(node["config"]) if isinstance(node["config"], str) else node["config"]
    data["arm"]["urdf_path"] = str(REPO_ROOT / data["arm"]["urdf_path"])
    return McpServerConfig.model_validate(data)


def object_spec(x: float, y: float, bottom: float, size: tuple[float, float, float], gap: float) -> ObjectSpec:
    """ObjectSpec of a scenario; gap_below_m only when the planner knows the field (older planners lack it).

    Args:
        x (float): Centre x (arm frame, m).
        y (float): Centre y (m).
        bottom (float): Object bottom z (m).
        size (tuple[float, float, float]): Depth, width, height (m).
        gap (float): Clear height under the object (m).

    Returns:
        ObjectSpec: Object.
    """
    fields: dict[str, Any] = {
        "frame": "arm",
        "x": x,
        "y": y,
        "support_z": bottom,
        "depth_m": size[0],
        "width_m": size[1],
        "height_m": size[2],
    }
    if "gap_below_m" in ObjectSpec.model_fields:
        fields["gap_below_m"] = gap
    return ObjectSpec.model_validate(fields)


def plan_all(out: Path, overrides: dict[str, Any] | None, only: str | None = None) -> list[dict[str, Any]]:
    """Plan every scenario (or those whose key contains only) and write the plans and index.json into out.

    Args:
        out (Path): Output directory.
        overrides (dict[str, Any] | None): Grasp parameter overrides on top of the client.yml grasp block.
        only (str | None): Plan only scenarios whose key contains this text.

    Returns:
        list[dict[str, Any]]: Index entries.
    """
    cfg = client_config()
    tool = tuple(cfg.arm.tool_offset_m.model_dump().values())
    kin = ArmKinematics(
        cfg.arm.urdf_path,
        margin=cfg.limits.arm_limit_margin_rad,
        joint_offsets=cfg.arm.joint_offsets_rad.model_dump(),
        tool_offset=tool,
        limit_overrides=cfg.arm.joint_limit_overrides_rad,
    )
    mount = cfg.arm.base_in_base_link or ArmBaseOffset(z=cfg.arm.arm_base_height_m)
    jaw = JawModel(kin, cfg.arm.jaw_open_axis, cfg.arm.gripper_closed_rad)
    planner = GraspPlanner(kin, cfg, FloorGuard(kin, cfg.floor_guard, mount, jaw), jaw)
    params = grasp_params(cfg.grasp, overrides)
    floor_z = cfg.arm.floor_z_m
    out.mkdir(parents=True, exist_ok=True)
    entries: list[dict[str, Any]] = []
    for box, size in BOXES.items():
        for position, (x, y) in POSITIONS.items():
            for support, height in SUPPORTS.items():
                surface_z = floor_z + height
                for name, (strategy, pitch, gap) in STRATEGIES.items():
                    key = f"{box}_{position}_{support}_{name}"
                    if only is not None and only not in key:
                        continue
                    surface = SurfaceModel(surface_z_m=surface_z + mount.z, tilt=None, tilt_source="none", mount=mount)
                    obj = object_spec(x, y, surface_z + gap, size, gap)
                    plan = planner.plan(obj, strategy, params, surface, dict(SEED), pitch)
                    (out / f"{key}.json").write_text(plan.model_dump_json())
                    entries.append(
                        {
                            "key": key,
                            "box": box,
                            "position": position,
                            "support": support,
                            "strategy": name,
                            "x": x,
                            "y": y,
                            "support_z": surface_z + gap,
                            "surface_z": surface_z,
                            "size_m": list(size),
                            "gap_below_m": gap,
                            "feasible": plan.feasible,
                            "reasons": plan.reasons,
                            "chosen": plan.strategy,
                            "approach_pitch_deg": None
                            if plan.approach_pitch_rad is None
                            else round(math.degrees(plan.approach_pitch_rad), 1),
                            "plan_file": f"{key}.json",
                        }
                    )
    index = {
        "base_height_m": cfg.arm.arm_base_height_m,
        "floor_z_m": floor_z,
        "tool_offset_m": list(tool),
        "timing": {
            "rate_hz": cfg.limits.arm_rate_hz,
            "arm_max_joint_velocity_rps": cfg.limits.arm_max_joint_velocity_rps,
            "arm_max_speed_scale": cfg.limits.arm_max_speed_scale,
            "gripper_velocity_rps": cfg.limits.gripper_velocity_rps,
            "gripper_closed_rad": cfg.arm.gripper_closed_rad,
        },
        "planner_params": params.model_dump(mode="json"),
        "entries": entries,
    }
    (out / INDEX_FILE).write_text(json.dumps(index, indent=1))
    return entries


def main(argv: list[str] | None = None) -> int:
    """Entry point.

    Args:
        argv (list[str] | None): Arguments (defaults to sys.argv).

    Returns:
        int: 0.
    """
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", type=Path, required=True, help="output directory")
    parser.add_argument("--params", type=Path, help="grasp parameter overrides (.yaml/.json)")
    parser.add_argument("--only", help="plan only scenarios whose key contains this text, e.g. 4x4x4_r20_")
    args = parser.parse_args(argv)
    overrides = yaml.safe_load(args.params.read_text()) if args.params else None
    entries = plan_all(args.out, overrides, args.only)
    feasible = sum(e["feasible"] for e in entries)
    print(f"planned {len(entries)} scenarios, {feasible} feasible; wrote {args.out / INDEX_FILE}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
