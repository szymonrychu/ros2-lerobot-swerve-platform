"""Tests for sim/grasp_sim/scripts/plan_matrix.py: the planner side of the grasp-sim scenario matrix.

The script runs in this node's uv environment (it imports mcp_server) and writes the index that
`grasp-sim matrix` replays in the MuJoCo sim (sim/README.md).
"""

import importlib.util
import json
from pathlib import Path
from types import ModuleType

import pytest

SCRIPT = Path(__file__).resolve().parents[3] / "sim" / "grasp_sim" / "scripts" / "plan_matrix.py"
INDEX_KEYS = {"base_height_m", "floor_z_m", "tool_offset_m", "pan_axis_xy", "timing", "planner_params", "entries"}
ENTRY_KEYS = {
    "key",
    "box",
    "position",
    "support",
    "strategy",
    "x",
    "y",
    "support_z",
    "surface_z",
    "size_m",
    "gap_below_m",
    "feasible",
    "reasons",
    "chosen",
    "approach_pitch_deg",
    "plan_file",
    "support_edge_x",
    "surfaces",
}


def load_script() -> ModuleType:
    spec = importlib.util.spec_from_file_location("plan_matrix", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_plan_matrix_writes_the_index_and_plans_with_the_deployed_config(tmp_path: Path) -> None:
    script = load_script()
    assert script.main(["--out", str(tmp_path), "--only", "4x4x4_r20_floor_"]) == 0
    index = json.loads((tmp_path / "index.json").read_text())
    assert set(index) == INDEX_KEYS
    assert index["tool_offset_m"] == pytest.approx([0.0010, -0.0056, -0.0014])  # client.yml arm.tool_offset_m
    assert index["pan_axis_xy"] == pytest.approx([0.0388, 0.0], abs=1e-4)  # the sim turns the box to face it
    entries = {e["key"]: e for e in index["entries"]}
    assert set(entries) == {f"4x4x4_r20_floor_{s}" for s in ("top_down", "angled45", "scoop", "scoop_gap", "auto")}
    for entry in entries.values():
        assert set(entry) == ENTRY_KEYS
        plan = json.loads((tmp_path / entry["plan_file"]).read_text())
        assert plan["feasible"] == entry["feasible"]
    assert entries["4x4x4_r20_floor_top_down"]["feasible"]
    assert entries["4x4x4_r20_floor_auto"]["chosen"] == "top_down"
    scoop = entries["4x4x4_r20_floor_scoop"]
    assert not scoop["feasible"] and "no gap under object" in scoop["reasons"][0]
    gap = entries["4x4x4_r20_floor_scoop_gap"]
    assert gap["gap_below_m"] > 0 and gap["support_z"] == pytest.approx(gap["surface_z"] + gap["gap_below_m"])


def test_params_file_overrides_the_grasp_defaults(tmp_path: Path) -> None:
    params = tmp_path / "params.yaml"
    params.write_text("scoop_max_pitch_deg: 60\n")
    script = load_script()
    assert script.main(["--out", str(tmp_path / "run"), "--params", str(params), "--only", "4x4x4_r20_floor_top"]) == 0
    index = json.loads((tmp_path / "run" / "index.json").read_text())
    assert index["planner_params"]["scoop_max_pitch_deg"] == 60
    assert [e["key"] for e in index["entries"]] == ["4x4x4_r20_floor_top_down"]


def test_supports_are_placed_relative_to_the_configured_floor(tmp_path: Path) -> None:
    """Ledges 7 and 15 cm above the floor, the stair 10 cm below it, the floor at client.yml arm.floor_z_m (-0.100)."""
    script = load_script()
    assert script.main(["--out", str(tmp_path), "--only", "4x4x4_r20_"]) == 0
    index = json.loads((tmp_path / "index.json").read_text())
    floor = index["floor_z_m"]
    assert floor == pytest.approx(-0.100) and index["base_height_m"] == pytest.approx(0.100)
    surfaces = {e["support"]: e["surface_z"] for e in index["entries"]}
    assert surfaces == pytest.approx(
        {"floor": floor, "ledge+0.07": floor + 0.07, "ledge+0.15": floor + 0.15, "stair-0.10": floor - 0.10}
    )


def test_the_stair_is_passed_to_the_planner_as_a_step_surface_at_the_scene_edge(tmp_path: Path) -> None:
    """The stair scenario gives the planner the step (half-plane 10 cm down) at the edge the sim scene builds."""
    script = load_script()
    assert script.main(["--out", str(tmp_path), "--only", "4x4x4_r30_"]) == 0
    index = json.loads((tmp_path / "index.json").read_text())
    entries = {e["key"]: e for e in index["entries"]}
    stair = entries["4x4x4_r30_stair-0.10_angled45"]
    assert stair["support_edge_x"] == pytest.approx(0.30 - script.SUPPORT_EDGE_MARGIN_M)
    assert stair["surfaces"] == [
        {
            "name": "stair",
            "frame": "arm",
            "height_m": pytest.approx(-0.10),
            "edge": {"point": [pytest.approx(0.22), 0.0], "direction": [0.0, -1.0], "side": "left"},
            "polygon": None,
        }
    ]
    plan = json.loads((tmp_path / stair["plan_file"]).read_text())
    assert plan["surface"]["surfaces"][0]["name"] == "stair"
    assert stair["feasible"] and stair["approach_pitch_deg"] > 45.0  # steeper than 45: the wrist clears the edge
    ledge = entries["4x4x4_r30_ledge+0.07_angled45"]
    assert ledge["surfaces"] is None and ledge["support_edge_x"] == pytest.approx(0.22)
    assert entries["4x4x4_r30_floor_angled45"]["support_edge_x"] is None


def test_tipover_writes_the_2026_10_10_jar_scenarios(tmp_path: Path) -> None:
    """--tipover: the light 39 mm x 6 cm jar standing on the floor at base_link (0.316, 0.0), top_down / angled 50 /
    auto, placed where the planner was told and 3 mm toward the fixed jaw (a perception error)."""
    script = load_script()
    assert script.main(["--out", str(tmp_path), "--tipover"]) == 0
    index = json.loads((tmp_path / "index.json").read_text())
    entries = {e["key"]: e for e in index["entries"]}
    assert set(entries) == {
        f"jar39x60_base0.316_floor_{s}_{e}" for s in ("top_down", "angled50", "auto") for e in ("err0", "err3")
    }
    mount = script.client_config().arm.base_in_base_link
    for key, entry in entries.items():
        assert entry["shape"] == "cylinder" and entry["mass_kg"] == pytest.approx(script.JAR_MASS_KG)
        assert entry["size_m"] == pytest.approx([0.039, 0.039, 0.06])
        assert (entry["x"], entry["y"]) == pytest.approx((0.316 - mount.x, 0.0 - mount.y))  # arm frame
        assert entry["support_z"] == pytest.approx(index["floor_z_m"])
        offset = entry["sim_offset_xy_m"]
        if key.endswith("err0") or not entry["feasible"]:
            assert offset == [0.0, 0.0]
            continue
        assert (offset[0] ** 2 + offset[1] ** 2) ** 0.5 == pytest.approx(0.003)
        plan = json.loads((tmp_path / entry["plan_file"]).read_text())
        grasp = next(w for w in plan["waypoints"] if w["label"] == "grasp")
        toward_fixed = (grasp["tool_point"]["x"] - grasp["x"], grasp["tool_point"]["y"] - grasp["y"])
        assert offset[0] * toward_fixed[0] + offset[1] * toward_fixed[1] > 0.0
    assert entries["jar39x60_base0.316_floor_top_down_err0"]["feasible"]
    assert entries["jar39x60_base0.316_floor_angled50_err0"]["feasible"]
