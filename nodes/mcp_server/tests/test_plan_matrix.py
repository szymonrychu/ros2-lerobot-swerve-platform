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
INDEX_KEYS = {"base_height_m", "floor_z_m", "tool_offset_m", "timing", "planner_params", "entries"}
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
    """Ledges 7 and 15 cm above the floor, the stair 10 cm below it, the floor at client.yml arm.floor_z_m (-0.104)."""
    script = load_script()
    assert script.main(["--out", str(tmp_path), "--only", "4x4x4_r20_"]) == 0
    index = json.loads((tmp_path / "index.json").read_text())
    floor = index["floor_z_m"]
    assert floor == pytest.approx(-0.104) and index["base_height_m"] == pytest.approx(0.104)
    surfaces = {e["support"]: e["surface_z"] for e in index["entries"]}
    assert surfaces == pytest.approx(
        {"floor": floor, "ledge+0.07": floor + 0.07, "ledge+0.15": floor + 0.15, "stair-0.10": floor - 0.10}
    )
