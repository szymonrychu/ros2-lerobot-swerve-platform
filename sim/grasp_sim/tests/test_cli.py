"""Command line: example export, plan replay, exit codes, report file, frame/GIF output."""

import json
from pathlib import Path

import pytest

from grasp_sim.cli import main
from grasp_sim.report import SimReport


def export(tmp_path: Path, name: str) -> tuple[Path, Path]:
    assert main(["example", name, "--out", str(tmp_path)]) == 0
    return tmp_path / f"{name}.plan.json", tmp_path / f"{name}.scene.json"


def test_example_command_writes_plan_and_scene_files(tmp_path: Path) -> None:
    plan, scene = export(tmp_path, "floor")
    assert isinstance(json.loads(plan.read_text()), list)
    assert json.loads(scene.read_text())["base_height_m"] == 0.104


def test_run_passing_plan_exits_zero_and_writes_report(tmp_path: Path, capsys) -> None:
    plan, scene = export(tmp_path, "ledge")
    report_path = tmp_path / "report.json"
    code = main(["run", str(plan), "--scene", str(scene), "--report", str(report_path)])
    assert code == 0
    assert "PASS" in capsys.readouterr().out
    report = SimReport.model_validate_json(report_path.read_text())
    assert report.passed and report.grasp_success


def test_run_failing_plan_exits_one_and_prints_reasons(tmp_path: Path, capsys) -> None:
    plan, scene = export(tmp_path, "tip")
    code = main(["run", str(plan), "--scene", str(scene)])
    out = capsys.readouterr().out
    assert code == 1
    assert "FAIL" in out and "tipped" in out


def test_run_accepts_a_grasp_plan_and_yaml_scene(tmp_path: Path) -> None:
    plan, _ = export(tmp_path, "floor")
    samples = json.loads(plan.read_text())
    grasp_plan = {
        "waypoints": [{"label": s.get("label"), "t": s["t"], "joints": s["joints"]} for s in samples if s.get("label")]
        + [{"t": samples[-1]["t"], "joints": samples[-1]["joints"]}]
    }
    grasp_path = tmp_path / "grasp_plan.json"
    grasp_path.write_text(json.dumps(grasp_plan))
    scene_yaml = tmp_path / "scene.yaml"
    scene_yaml.write_text("object:\n  x_m: 0.2\n")
    code = main(["run", str(grasp_path), "--grasp-plan", "--scene", str(scene_yaml)])
    assert code in (0, 1)


def test_run_writes_frames_and_gif(tmp_path: Path) -> None:
    plan, scene = export(tmp_path, "floor")
    frames = tmp_path / "frames"
    gif = tmp_path / "scoop.gif"
    code = main(
        [
            "run",
            str(plan),
            "--scene",
            str(scene),
            "--frames",
            str(frames),
            "--gif",
            str(gif),
            "--fps",
            "4",
            "--size",
            "160x120",
        ]
    )
    assert code == 0
    pngs = sorted(frames.glob("*.png"))
    assert len(pngs) >= 20
    assert gif.stat().st_size > 1000


def test_run_with_sim_thresholds_file(tmp_path: Path, capsys) -> None:
    plan, scene = export(tmp_path, "floor")
    sim = tmp_path / "sim.yaml"
    sim.write_text("lift_min_height_m: 0.5\n")
    capsys.readouterr()
    code = main(["run", str(plan), "--scene", str(scene), "--sim", str(sim), "--json"])
    assert code == 1
    assert json.loads(capsys.readouterr().out)["grasp_success"] is False


def test_render_without_mjpython_explains_how_to_launch(tmp_path: Path) -> None:
    plan, scene = export(tmp_path, "floor")
    with pytest.raises(SystemExit, match="mjpython"):
        main(["run", str(plan), "--scene", str(scene), "--render"])
