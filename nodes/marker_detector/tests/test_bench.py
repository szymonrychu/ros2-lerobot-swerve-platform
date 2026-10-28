"""Benchmark runner: stage statistics, decode ratio and version fields on small synthetic scenes."""

import json
import math

import pytest
from pydantic import ValidationError

from marker_detector.bench import (
    DEFAULT_CONFIG_PATH,
    BenchConfig,
    BenchScene,
    load_bench_config,
    main,
    render_scene,
    run_bench,
)
from marker_detector.synthetic import D435_COLOR_K, D435_COLOR_SIZE

STAGES = ("decode_frame", "estimate_pose", "frame")
CONFIG = {
    "iterations": 3,
    "warmup": 1,
    "seed": 1,
    "scenes": [
        {"name": "d435_1", "image_size": [640, 360], "markers": [{"size_mm": 150, "distance_m": 1.0}]},
        {
            "name": "d435_2",
            "image_size": [640, 360],
            "markers": [{"size_mm": 100, "distance_m": 0.6}, {"size_mm": 100, "distance_m": 0.8}],
        },
    ],
}


@pytest.fixture(scope="module")
def result() -> dict:
    return run_bench(BenchConfig.model_validate(CONFIG))


def test_every_stage_has_ordered_percentiles(result: dict) -> None:
    for scene in ("d435_1", "d435_2"):
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


def test_d435_colour_intrinsics_follow_the_69_deg_fov() -> None:
    assert D435_COLOR_SIZE == (640, 360)
    assert D435_COLOR_K[0, 0] == pytest.approx(320 / math.tan(math.radians(34.5)))
    assert D435_COLOR_K[1, 1] == D435_COLOR_K[0, 0]
    assert (D435_COLOR_K[0, 2], D435_COLOR_K[1, 2]) == (320.0, 180.0)


def test_scene_renders_through_the_d435_colour_camera() -> None:
    scene = BenchScene(name="s", image_size=(640, 360), markers=[{"size_mm": 150, "distance_m": 1.0}])
    grey, k, count = render_scene(scene, 0)
    assert grey.shape == (360, 640) and count == 1
    assert k == pytest.approx(D435_COLOR_K)


def test_zoom_crop_focal_scale_rejected() -> None:
    with pytest.raises(ValidationError, match="focal_scale"):
        BenchScene(name="z", image_size=(640, 360), focal_scale=2.0, markers=[{"size_mm": 100, "distance_m": 2.0}])


def test_shipped_scenes_are_d435_colour_frames() -> None:
    scenes = load_bench_config(DEFAULT_CONFIG_PATH).scenes
    assert scenes
    assert all(tuple(scene.image_size) == D435_COLOR_SIZE for scene in scenes)
    assert not [scene.name for scene in scenes if "zoom" in scene.name]
