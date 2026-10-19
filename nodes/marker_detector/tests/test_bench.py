"""Benchmark runner: stage statistics, decode ratio and version fields on small synthetic scenes."""

import json

import pytest
from pydantic import ValidationError

from marker_detector.bench import BenchConfig, load_bench_config, main, run_bench

STAGES = ("decode_frame", "estimate_pose", "frame")
CONFIG = {
    "iterations": 3,
    "warmup": 1,
    "seed": 1,
    "scenes": [
        {"name": "full_1", "image_size": [1152, 648], "markers": [{"size_mm": 150, "distance_m": 1.0}]},
        {
            "name": "zoom_2",
            "image_size": [1024, 576],
            "focal_scale": 2.0,
            "markers": [{"size_mm": 100, "distance_m": 2.0}, {"size_mm": 100, "distance_m": 2.3}],
        },
    ],
}


@pytest.fixture(scope="module")
def result() -> dict:
    return run_bench(BenchConfig.model_validate(CONFIG))


def test_every_stage_has_ordered_percentiles(result: dict) -> None:
    for scene in ("full_1", "zoom_2"):
        for stage in STAGES:
            stats = result[scene][stage]
            assert 0 < stats["p50_ms"] <= stats["p95_ms"] <= stats["max_ms"]


def test_near_scenes_decode_fully_and_versions_reported(result: dict) -> None:
    assert result["decoded_ratio"] == 1.0
    assert result["cv2"] and result["zxingcpp"] and result["machine"]
    json.dumps(result)


def test_invalid_config_rejected() -> None:
    with pytest.raises(ValidationError):
        BenchConfig.model_validate({**CONFIG, "scenes": [{"name": "x", "image_size": [100, 100], "markers": []}]})


def test_main_prints_json(tmp_path, capsys) -> None:
    path = tmp_path / "bench.yaml"
    path.write_text(json.dumps(CONFIG))
    assert load_bench_config(path).iterations == 3
    assert main([str(path)]) == 0
    assert "decoded_ratio" in json.loads(capsys.readouterr().out)
