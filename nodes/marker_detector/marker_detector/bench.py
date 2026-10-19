"""Benchmark of decode and pose cost on synthetic scenes: ``python -m marker_detector.bench [config]``."""

import json
import platform
import sys
import time
from importlib.metadata import version
from pathlib import Path

import cv2
import numpy as np
import yaml
from pydantic import Field

from marker_detector.config import PoseConfig, StrictModel
from marker_detector.decode import decode_frame
from marker_detector.pose import estimate_pose
from marker_detector.synthetic import STREAM_K, STREAM_SIZE, pose_facing, render_marker_scene

DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "bench.yaml"
MARKER_SPACING_M = 0.2
NOISE_SIGMA = 0.5
STAGE_DECODE = "decode_frame"
STAGE_POSE = "estimate_pose"
STAGE_FRAME = "frame"


class BenchMarker(StrictModel):
    """One synthetic marker: symbol edge and distance from the camera."""

    size_mm: float = Field(gt=0)
    distance_m: float = Field(gt=0)


class BenchScene(StrictModel):
    """One scene: image size, optional focal scale (default image width / 1152) and 1-3 markers."""

    name: str
    image_size: tuple[int, int]
    focal_scale: float | None = Field(None, gt=0)
    markers: list[BenchMarker] = Field(min_length=1, max_length=3)


class BenchConfig(StrictModel):
    """Benchmark settings (config/bench.yaml)."""

    iterations: int = Field(200, gt=0)
    warmup: int = Field(20, ge=0)
    seed: int = 0
    scenes: list[BenchScene] = Field(min_length=1)


def load_bench_config(path: Path) -> BenchConfig:
    """Load and validate a bench YAML (or JSON) file.

    Args:
        path: Config file.

    Returns:
        The validated BenchConfig.
    """
    return BenchConfig.model_validate(yaml.safe_load(path.read_text()) or {})


def render_scene(scene: BenchScene, seed: int) -> tuple[np.ndarray, np.ndarray, int]:
    """Render a scene once.

    Args:
        scene: Scene definition.
        seed: Noise seed.

    Returns:
        (grey image, 3x3 intrinsics, marker count).
    """
    width, height = scene.image_size
    scale = scene.focal_scale if scene.focal_scale is not None else width / STREAM_SIZE[0]
    k = np.array([[STREAM_K[0, 0] * scale, 0, width / 2], [0, STREAM_K[1, 1] * scale, height / 2], [0, 0, 1.0]])
    count = len(scene.markers)
    payloads = [f"P1/K:BASE/N:BENCH{i}/S:{round(m.size_mm)}" for i, m in enumerate(scene.markers)]
    poses = [
        pose_facing(m.distance_m, offset_m=((i - (count - 1) / 2) * MARKER_SPACING_M * m.distance_m / scale, 0.0))
        for i, m in enumerate(scene.markers)
    ]
    image = render_marker_scene(
        payloads, [m.size_mm for m in scene.markers], poses, k, scene.image_size, noise_px=NOISE_SIGMA, seed=seed
    )
    return image, k, count


def summarise(samples_ns: list[int]) -> dict[str, float]:
    """Return p50/p95/max in milliseconds of nanosecond samples."""
    ms = np.asarray(samples_ns, dtype=float) / 1e6
    return {
        "p50_ms": float(np.percentile(ms, 50)),
        "p95_ms": float(np.percentile(ms, 95)),
        "max_ms": float(ms.max()),
    }


def run_bench(cfg: BenchConfig) -> dict:
    """Render every scene once and time decode, pose per candidate and the whole frame.

    Args:
        cfg: Benchmark settings.

    Returns:
        ``{scene: {stage: {p50_ms, p95_ms, max_ms}}, "decoded_ratio", "decoded_by_scene", "cv2", "zxingcpp", "machine"}``.
    """
    pose_cfg = PoseConfig()
    report: dict = {}
    decoded_total = expected_total = 0
    by_scene: dict[str, float] = {}
    for scene in cfg.scenes:
        grey, k, count = render_scene(scene, cfg.seed)
        decode_ns: list[int] = []
        pose_ns: list[int] = []
        frame_ns: list[int] = []
        decoded = 0
        for i in range(cfg.warmup + cfg.iterations):
            frame_start = time.perf_counter_ns()
            candidates, _ = decode_frame(grey)
            decode_end = time.perf_counter_ns()
            pose_each = []
            for candidate in candidates:
                start = time.perf_counter_ns()
                estimate_pose(grey, candidate, k, None, pose_cfg)
                pose_each.append(time.perf_counter_ns() - start)
            frame_end = time.perf_counter_ns()
            if i < cfg.warmup:
                continue
            decode_ns.append(decode_end - frame_start)
            pose_ns.extend(pose_each)
            frame_ns.append(frame_end - frame_start)
            decoded += len(candidates)
        report[scene.name] = {
            STAGE_DECODE: summarise(decode_ns),
            STAGE_POSE: summarise(pose_ns or [0]),
            STAGE_FRAME: summarise(frame_ns),
        }
        by_scene[scene.name] = decoded / (count * cfg.iterations)
        decoded_total += decoded
        expected_total += count * cfg.iterations
    report["decoded_ratio"] = decoded_total / expected_total
    report["decoded_by_scene"] = by_scene
    report["cv2"] = cv2.__version__
    report["zxingcpp"] = version("zxing-cpp")
    report["machine"] = platform.platform()
    return report


def main(argv: list[str] | None = None) -> int:
    """Run the benchmark and print the JSON report.

    Args:
        argv: Optional config path as the only argument (default config/bench.yaml).

    Returns:
        0 on success, 1 on a usage error.
    """
    args = sys.argv[1:] if argv is None else argv
    if len(args) > 1:
        print("usage: python -m marker_detector.bench [config]", file=sys.stderr)
        return 1
    path = Path(args[0]) if args else DEFAULT_CONFIG_PATH
    print(json.dumps(run_bench(load_bench_config(path)), indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
