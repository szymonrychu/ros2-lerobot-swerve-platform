"""Plan the grasp scenario matrix with the mcp_server planner and the deployed client.yml config.

Runs in the nodes/mcp_server uv environment (it imports mcp_server), writes <out>/index.json and one GraspPlan JSON
per scenario; `grasp-sim matrix <out>` (sim/grasp_sim environment) replays them:

    cd nodes/mcp_server && uv run python ../../sim/grasp_sim/scripts/plan_matrix.py --out /tmp/matrix
    cd sim/grasp_sim && uv run grasp-sim matrix /tmp/matrix

Optional --params <yaml/json> overrides grasp parameters on top of client.yml (for tuning runs).

--tipover plans the 2026-10-10 jar tip-over scenarios instead of the matrix: a light 39 mm x 6 cm upright cylinder
standing on the floor at base_link (0.316, 0.0), top_down / angled 50 / auto, replayed with the jar where the planner
was told (err0) and 3 mm toward the fixed jaw (err3, a perception error of the size seen on the robot).

Ledges and the stair start SUPPORT_EDGE_MARGIN_M before the object centre (the sim scene's default support edge, recorded
per entry as support_edge_x). The stair is passed to the planner as a surface region: the robot floor up to the edge,
a half-plane 10 cm lower beyond it, so the planner sees the step edge. Floor and ledge scenarios keep a single surface
height (surface_z_m), as before.
"""

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
from mcp_server.config import ArmBaseOffset, McpServerConfig
from mcp_server.floor_guard import FloorGuard, FloorOverride, JawModel, SurfaceModel
from mcp_server.grasp import GraspParams, GraspPlan, GraspPlanner, ObjectSpec, grasp_params
from mcp_server.ik import ArmKinematics
from mcp_server.surfaces import HalfPlaneEdge, SurfaceRegion

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
# Supports: name -> height of the surface under the object above the floor (m; the floor is arm.floor_z_m, -0.100 in
# the arm frame): two ledges and a lower stair.
SUPPORTS = {"floor": 0.0, "ledge+0.07": 0.07, "ledge+0.15": 0.15, "stair-0.10": -0.10}
# The ledge / stair edge lies this far before the object centre along arm x (grasp_sim DEFAULT_SUPPORT_MARGIN_M).
SUPPORT_EDGE_MARGIN_M = 0.08
STAIR_PREFIX = "stair"
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
# 2026-10-10 tip-over jar (grip session report): 39 mm across (jaw contact), 5.5-6.3 cm tall, light, on the floor.
JAR_BOX = "jar39x60"
JAR_SIZE_M = (0.039, 0.039, 0.06)
JAR_MASS_KG = 0.03
JAR_BASE_LINK_XY = (0.316, 0.0)
JAR_POSITION = "base0.316"
# Strategies: name -> (planner strategy, approach pitch deg). Angled 45 cannot reach the approach start here (3 mm short),
# 50 deg is the flattest feasible angled pitch.
JAR_STRATEGIES = {"top_down": ("top_down", None), "angled50": ("angled", 50.0), "auto": ("auto", None)}
# Sim placement error toward the fixed jaw (m): the camera put the jar within a few mm (3.4-3.7 vs 3.9 cm).
JAR_ERRORS_M = {"err0": 0.0, "err3": 0.003}


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


def stair_region(edge_x: float, height: float) -> SurfaceRegion:
    """The stair as a surface region: a half-plane beyond an edge across the arm at arm-frame x = edge_x.

    Args:
        edge_x (float): Edge position along arm x (m).
        height (float): Stair height relative to the robot floor (m, < 0).

    Returns:
        SurfaceRegion: The region (surface on the far side of the edge).
    """
    return SurfaceRegion(
        name="stair", frame="arm", height_m=height, edge=HalfPlaneEdge(point=(edge_x, 0.0), direction=(0.0, -1.0))
    )


@dataclass(frozen=True)
class Setup:
    """The deployed planner and what the index records about it."""

    cfg: McpServerConfig
    kin: ArmKinematics
    mount: ArmBaseOffset
    guard: FloorGuard
    planner: GraspPlanner
    params: GraspParams


def setup(overrides: dict[str, Any] | None) -> Setup:
    """Planner on the deployed client.yml config.

    Args:
        overrides (dict[str, Any] | None): Grasp parameter overrides on top of the client.yml grasp block.

    Returns:
        Setup: Config, kinematics, arm mount, floor guard, planner and grasp parameters.
    """
    cfg = client_config()
    kin = ArmKinematics(
        cfg.arm.urdf_path,
        margin=cfg.limits.arm_limit_margin_rad,
        joint_offsets=cfg.arm.joint_offsets_rad.model_dump(),
        tool_offset=tuple(cfg.arm.tool_offset_m.model_dump().values()),
        limit_overrides=cfg.arm.joint_limit_overrides_rad,
    )
    mount = cfg.arm.base_in_base_link or ArmBaseOffset(z=cfg.arm.arm_base_height_m)
    jaw = JawModel(kin, cfg.arm.jaw_open_axis, cfg.arm.gripper_closed_rad)
    guard = FloorGuard(kin, cfg.floor_guard, mount, jaw)
    return Setup(cfg, kin, mount, guard, GraspPlanner(kin, cfg, guard, jaw), grasp_params(cfg.grasp, overrides))


def write_index(out: Path, run: Setup, entries: list[dict[str, Any]]) -> None:
    """Write index.json: the scene constants, executor timing, planner parameters and the entries.

    Args:
        out (Path): Output directory.
        run (Setup): Planner setup.
        entries (list[dict[str, Any]]): Index entries.
    """
    cfg = run.cfg
    index = {
        "base_height_m": cfg.arm.arm_base_height_m,
        "floor_z_m": cfg.arm.floor_z_m,
        "tool_offset_m": list(cfg.arm.tool_offset_m.model_dump().values()),
        "pan_axis_xy": list(run.kin.pan_axis_xy),
        "timing": {
            "rate_hz": cfg.limits.arm_rate_hz,
            "arm_max_joint_velocity_rps": cfg.limits.arm_max_joint_velocity_rps,
            "arm_max_speed_scale": cfg.limits.arm_max_speed_scale,
            "gripper_velocity_rps": cfg.limits.gripper_velocity_rps,
            "gripper_closed_rad": cfg.arm.gripper_closed_rad,
        },
        "planner_params": run.params.model_dump(mode="json"),
        "entries": entries,
    }
    (out / INDEX_FILE).write_text(json.dumps(index, indent=1))


def plan_summary(plan: GraspPlan) -> dict[str, Any]:
    """Index fields of a plan's outcome.

    Args:
        plan (GraspPlan): Plan.

    Returns:
        dict[str, Any]: feasible, reasons, chosen, approach_pitch_deg.
    """
    return {
        "feasible": plan.feasible,
        "reasons": plan.reasons,
        "chosen": plan.strategy,
        "approach_pitch_deg": None
        if plan.approach_pitch_rad is None
        else round(math.degrees(plan.approach_pitch_rad), 1),
    }


def toward_fixed_jaw(plan: GraspPlan) -> tuple[float, float]:
    """Horizontal unit direction from the object centre to the fixed jaw at the grasp of a centred plan.

    Args:
        plan (GraspPlan): Feasible plan (with a grasp shift the grasp waypoint x, y is the jaw centre and its
            tool_point the fixed jaw).

    Returns:
        tuple[float, float]: Unit (x, y) in the arm frame; (0, 0) without a grasp shift.
    """
    grasp = next(w for w in plan.waypoints if w.label == "grasp")
    if grasp.tool_point is None or plan.grasp_shift is None:
        return (0.0, 0.0)
    dx, dy = grasp.tool_point["x"] - grasp.x, grasp.tool_point["y"] - grasp.y
    norm = math.hypot(dx, dy)
    return (0.0, 0.0) if norm == 0.0 else (dx / norm, dy / norm)


def plan_tipover(out: Path, overrides: dict[str, Any] | None) -> list[dict[str, Any]]:
    """Plan the jar tip-over scenarios and write the plans and index.json into out.

    Args:
        out (Path): Output directory.
        overrides (dict[str, Any] | None): Grasp parameter overrides on top of the client.yml grasp block.

    Returns:
        list[dict[str, Any]]: Index entries.
    """
    run = setup(overrides)
    floor_z = run.cfg.arm.floor_z_m
    out.mkdir(parents=True, exist_ok=True)
    surface = SurfaceModel(surface_z_m=floor_z + run.mount.z, tilt=None, tilt_source="none", mount=run.mount)
    jar = ObjectSpec(
        frame="base_link",
        x=JAR_BASE_LINK_XY[0],
        y=JAR_BASE_LINK_XY[1],
        support_z=floor_z + run.mount.z,
        depth_m=JAR_SIZE_M[0],
        width_m=JAR_SIZE_M[1],
        height_m=JAR_SIZE_M[2],
    )
    arm_jar = run.planner.to_arm(jar)
    entries: list[dict[str, Any]] = []
    for name, (strategy, pitch) in JAR_STRATEGIES.items():
        plan = run.planner.plan(jar, strategy, run.params, surface, dict(SEED), pitch)
        direction = toward_fixed_jaw(plan) if plan.feasible else (0.0, 0.0)
        for err, distance in JAR_ERRORS_M.items():
            key = f"{JAR_BOX}_{JAR_POSITION}_floor_{name}_{err}"
            (out / f"{key}.json").write_text(plan.model_dump_json())
            entries.append(
                {
                    "key": key,
                    "box": JAR_BOX,
                    "position": JAR_POSITION,
                    "support": "floor",
                    "strategy": f"{name}_{err}",
                    "x": arm_jar.x,
                    "y": arm_jar.y,
                    "support_z": arm_jar.support_z,
                    "surface_z": floor_z,
                    "size_m": list(JAR_SIZE_M),
                    "gap_below_m": 0.0,
                    "shape": "cylinder",
                    "mass_kg": JAR_MASS_KG,
                    "sim_offset_xy_m": [direction[0] * distance, direction[1] * distance],
                    "support_edge_x": None,
                    "surfaces": None,
                    **plan_summary(plan),
                    "plan_file": f"{key}.json",
                }
            )
    write_index(out, run, entries)
    return entries


def plan_all(out: Path, overrides: dict[str, Any] | None, only: str | None = None) -> list[dict[str, Any]]:
    """Plan every scenario (or those whose key contains only) and write the plans and index.json into out.

    Args:
        out (Path): Output directory.
        overrides (dict[str, Any] | None): Grasp parameter overrides on top of the client.yml grasp block.
        only (str | None): Plan only scenarios whose key contains this text.

    Returns:
        list[dict[str, Any]]: Index entries.
    """
    run = setup(overrides)
    mount, guard, planner, params = run.mount, run.guard, run.planner, run.params
    floor_z = run.cfg.arm.floor_z_m
    out.mkdir(parents=True, exist_ok=True)
    entries: list[dict[str, Any]] = []
    for box, size in BOXES.items():
        for position, (x, y) in POSITIONS.items():
            for support, height in SUPPORTS.items():
                surface_z = floor_z + height
                edge_x = None if support == "floor" else x - SUPPORT_EDGE_MARGIN_M
                regions = (
                    [stair_region(x - SUPPORT_EDGE_MARGIN_M, height)] if support.startswith(STAIR_PREFIX) else None
                )
                for name, (strategy, pitch, gap) in STRATEGIES.items():
                    key = f"{box}_{position}_{support}_{name}"
                    if only is not None and only not in key:
                        continue
                    if regions is None:
                        surface = SurfaceModel(
                            surface_z_m=surface_z + mount.z, tilt=None, tilt_source="none", mount=mount
                        )
                    else:
                        surface = guard.surface(FloorOverride(surfaces=regions), None, 0.0)
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
                            "support_edge_x": edge_x,
                            "surfaces": None if regions is None else [r.model_dump(mode="json") for r in regions],
                            **plan_summary(plan),
                            "plan_file": f"{key}.json",
                        }
                    )
    write_index(out, run, entries)
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
    parser.add_argument("--tipover", action="store_true", help="plan the 2026-10-10 jar tip-over scenarios instead")
    args = parser.parse_args(argv)
    overrides = yaml.safe_load(args.params.read_text()) if args.params else None
    entries = plan_tipover(args.out, overrides) if args.tipover else plan_all(args.out, overrides, args.only)
    feasible = sum(e["feasible"] for e in entries)
    print(f"planned {len(entries)} scenarios, {feasible} feasible; wrote {args.out / INDEX_FILE}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
