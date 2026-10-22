"""Unit tests for scripts/ground_noise_report.py: runbook metrics over /ground/state captures."""

import json
from pathlib import Path
from typing import Any

import pytest
import yaml

from scripts.ground_noise_report import main

REPO_ROOT = Path(__file__).resolve().parent.parent
CONFIG_PATH = REPO_ROOT / "scripts" / "ground_noise_report.yaml"

PLANE = {"normal": [0.0, 0.0, 1.0], "height_m": 0.0}
BOX_POINT = [0.7, 0.15]
RAMP_POINT = [0.9, -0.3]
BOARD_POINT = [0.45, 0.0]
FAR_POINT = [1.8, 0.0]


def region(point: list[float], cls: str) -> dict[str, Any]:
    return {"name": "step_1", "kind": "step", "class": cls, "edge": {"point": point}}


def area(kind: str, cx: float, cy: float, slope: float = 0.0) -> dict[str, Any]:
    polygon = [[cx - 0.05, cy - 0.05], [cx + 0.05, cy - 0.05], [cx + 0.05, cy + 0.05], [cx - 0.05, cy + 0.05]]
    return {"kind": kind, "polygon": polygon, "height_m": 0.03, "slope_deg": slope, "source": "step"}


def state(**kw: Any) -> dict[str, Any]:
    base: dict[str, Any] = {
        "plane": PLANE,
        "obstacle_points": 0,
        "drop_points": 0,
        "regions": [],
        "warnings": [],
    }
    base.update(kw)
    return base


def write_capture(path: Path, states: list[dict[str, Any]], style: str = "scraper") -> Path:
    lines = []
    for i, s in enumerate(states):
        if style == "scraper":
            lines.append(json.dumps({"timestamp_ns": i, "source": "client", "topic": "/ground/state", "value": s}))
        else:
            lines.append(json.dumps({"t": i, "topic": "/ground/state", "data": s}))
    path.write_text("\n".join(lines) + "\n")
    return path


def run(
    tmp_path: Path, scene: str, states: list[dict[str, Any]], capsys: pytest.CaptureFixture[str]
) -> tuple[int, dict]:
    capture = write_capture(tmp_path / "cap.ndjson", states)
    code = main(["--config", str(CONFIG_PATH), "--capture", str(capture), "--scene", scene])
    return code, json.loads(capsys.readouterr().out)


def test_box_scene_pass_and_fail(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    hit = state(regions=[region(BOX_POINT, "obstacle")], obstacle_points=5)
    # obstacle points alone count while nothing warning-class sits in the box ROI
    points_only = state(obstacle_points=3)
    # obstacle points but a warning region in the box: not a hit
    warned = state(obstacle_points=3, regions=[region(BOX_POINT, "warning")])
    miss = state()
    code, out = run(tmp_path, "box", [hit] * 18 + [points_only, hit], capsys)
    assert code == 0
    assert out["metrics"]["box_obstacle_share"] == 1.0
    assert out["passed"] is True
    code, out = run(tmp_path, "box", [hit] * 8 + [warned, miss], capsys)
    assert code == 1
    assert out["metrics"]["box_obstacle_share"] == pytest.approx(0.8)
    # a warning-area centroid in the box also vetoes the points-only rule
    code, out = run(tmp_path, "box", [state(obstacle_points=3, warnings=[area("step", 0.75, 0.2)])] * 2, capsys)
    assert out["metrics"]["box_obstacle_share"] == 0.0


def test_ramp_scene_informational_metrics(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    good = state(warnings=[area("ramp", 0.92, -0.30, 13.0)])
    bad_slope = state(warnings=[area("ramp", 0.92, -0.30, 6.0)])
    code, out = run(tmp_path, "ramp", [good] * 10, capsys)
    assert code == 0
    assert out["metrics"]["ramp_warning_share"] == 1.0
    assert out["metrics"]["ramp_slope_abs_error_p50"] == pytest.approx(1.0)
    assert out["metrics"]["ramp_slope_abs_error_p90"] == pytest.approx(1.0)
    # warning share and slope error are informational: a bad ramp reading still passes
    code, out = run(tmp_path, "ramp", [bad_slope] * 10 + [state()] * 10, capsys)
    assert code == 0
    assert out["metrics"]["ramp_warning_share"] == pytest.approx(0.5)
    assert out["metrics"]["ramp_slope_abs_error_p50"] == pytest.approx(8.0)
    assert set(out["thresholds"]) == {"ramp_lethal_states", "warning_churn_share"}


def test_ramp_scene_never_lethal(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    no_area = state()
    lethal = state(regions=[region(RAMP_POINT, "obstacle")])
    elsewhere = state(regions=[region(BOX_POINT, "obstacle")])
    code, out = run(tmp_path, "ramp", [no_area, elsewhere] * 5, capsys)
    assert code == 0
    assert out["metrics"]["ramp_lethal_states"] == 0
    code, out = run(tmp_path, "ramp", [no_area] * 9 + [lethal], capsys)
    assert code == 1
    assert out["metrics"]["ramp_lethal_states"] == 1
    assert out["thresholds"]["ramp_lethal_states"]["passed"] is False


def test_scene_rois_match_ground_truth() -> None:
    scene = yaml.safe_load(CONFIG_PATH.read_text())["scene"]
    assert scene["box"]["roi"] == {"x_min": 0.60, "x_max": 0.78, "y_min": 0.05, "y_max": 0.33}
    assert scene["board"]["roi"] == {"x_min": 0.37, "x_max": 0.61, "y_min": -0.12, "y_max": 0.14}
    assert scene["ramp"]["roi"] == {"x_min": 0.75, "x_max": 1.10, "y_min": -0.45, "y_max": -0.15}
    assert "height_m" in scene["box"] and "slope_deg" in scene["ramp"]
    thresholds = yaml.safe_load(CONFIG_PATH.read_text())["thresholds"]
    assert thresholds["ramp_lethal_states_max"] == 0
    assert "ramp_warning_share_min" not in thresholds
    assert "ramp_slope_abs_error_p50_max" not in thresholds


def test_board_scene_lethal_states(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    clean = state(regions=[region(BOARD_POINT, "warning")])
    lethal = state(regions=[region(BOARD_POINT, "obstacle")])
    elsewhere = state(regions=[region(BOX_POINT, "obstacle")])
    code, out = run(tmp_path, "board", [clean, elsewhere], capsys)
    assert code == 0
    assert out["metrics"]["board_lethal_states"] == 0
    code, out = run(tmp_path, "board", [clean, lethal, lethal], capsys)
    assert code == 1
    assert out["metrics"]["board_lethal_states"] == 2


def test_empty_scene_lethal_p95(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    code, out = run(tmp_path, "empty", [state()] * 20, capsys)
    assert code == 0
    assert out["metrics"]["empty_lethal_points_p95"] == 0
    noisy = [state(obstacle_points=150, drop_points=50)] * 3 + [state()] * 17
    code, out = run(tmp_path, "empty", noisy, capsys)
    assert code == 1
    assert out["metrics"]["empty_lethal_points_p95"] == 200
    # states without a plane are not counted
    code, out = run(tmp_path, "empty", [state()] * 5 + [{"plane": None, "obstacle_points": 0}], capsys)
    assert out["states_with_plane"] == 5


def test_warning_churn_share(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    a = state(warnings=[area("step", 0.75, 0.2)])
    a_jitter = state(warnings=[area("step", 0.7504, 0.2)])  # same after 1 cm rounding
    b = state(warnings=[area("step", 0.80, 0.2)])
    code, out = run(tmp_path, "empty", [a, a_jitter, a, b, b], capsys)
    assert out["metrics"]["warning_churn_share"] == pytest.approx(1 / 4)
    assert code == 1
    code, out = run(tmp_path, "empty", [a, a_jitter] * 6, capsys)
    assert out["metrics"]["warning_churn_share"] == 0.0
    assert code == 0
    # kind change counts
    c = state(warnings=[area("ramp", 0.75, 0.2)])
    code, out = run(tmp_path, "board", [a, c], capsys)
    assert out["metrics"]["warning_churn_share"] == 1.0


def test_reads_scraper_and_ws_lines(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    s = state(regions=[region(BOARD_POINT, "obstacle")])
    lines = [
        {"topic": "/ground/state", "value": s},
        {"topic": "/ground/state", "value": json.dumps(s)},
        {"t": 1.0, "topic": "/ground/state", "data": s},
        {"t": 2.0, "topic": "/ground/state", "data": json.dumps(s)},
        {"topic": "/other", "value": {"x": 1}},
    ]
    capture = tmp_path / "mixed.ndjson"
    capture.write_text("\n".join(json.dumps(x) for x in lines) + "\n")
    main(["--config", str(CONFIG_PATH), "--capture", str(capture), "--scene", "board"])
    out = json.loads(capsys.readouterr().out)
    assert out["metrics"]["board_lethal_states"] == 4
    assert out["states"] == 4
    assert out["skipped_lines"] == 1
    assert out["malformed_lines"] == 0
    ws = write_capture(tmp_path / "ws.ndjson", [s, s], style="ws")
    main(["--config", str(CONFIG_PATH), "--capture", str(ws), "--scene", "board"])
    assert json.loads(capsys.readouterr().out)["states"] == 2


def test_malformed_lines_counted(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    good = json.dumps({"topic": "/ground/state", "value": state()})
    lines = [
        good,
        "{not json",
        "[1, 2]",
        json.dumps({"topic": "/ground/state", "value": "{broken"}),
        json.dumps({"topic": "/ground/state"}),
        "",
        good,
    ]
    capture = tmp_path / "bad.ndjson"
    capture.write_text("\n".join(lines) + "\n")
    code = main(["--config", str(CONFIG_PATH), "--capture", str(capture), "--scene", "empty"])
    out = json.loads(capsys.readouterr().out)
    assert out["malformed_lines"] == 4
    assert out["states"] == 2
    assert code == 0


def test_exit_code_reflects_thresholds(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    config = yaml.safe_load(CONFIG_PATH.read_text())
    config["thresholds"]["empty_lethal_points_p95_max"] = 500
    loose = tmp_path / "loose.yaml"
    loose.write_text(yaml.safe_dump(config))
    capture = write_capture(tmp_path / "cap.ndjson", [state(obstacle_points=200)] * 5)
    assert main(["--config", str(CONFIG_PATH), "--capture", str(capture), "--scene", "empty"]) == 1
    capsys.readouterr()
    assert main(["--config", str(loose), "--capture", str(capture), "--scene", "empty"]) == 0
    assert json.loads(capsys.readouterr().out)["passed"] is True
    # an empty capture has no data: fails rather than passing vacuously
    empty = tmp_path / "none.ndjson"
    empty.write_text("")
    assert main(["--config", str(CONFIG_PATH), "--capture", str(empty), "--scene", "empty"]) == 1
    capsys.readouterr()
    with pytest.raises(SystemExit):
        main(["--config", str(CONFIG_PATH), "--capture", str(capture), "--scene", "nope"])
