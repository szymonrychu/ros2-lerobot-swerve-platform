"""CPU cost per pair of the Python and the C++ stereo depth paths on the same input.

Run inside the Python node's uv environment (it calls the stereo_depth functions themselves):

    cd nodes/stereo_depth && uv run --frozen python ../stereo_depth_cpp/scripts/bench_compare.py

Settings: BENCH_COMPARE_CONFIG or scripts/bench_compare.yaml next to this file. Each path runs what the node does
per synced Image pair: decode both images to grey, rectify, StereoSGBM, the depth conversion and the 16UC1 output
bytes. The Python path runs in this process (time.process_time); the C++ path is the stereo_depth_bench binary of
the stereo_depth_cpp package (cpp_command), which reads a job file written here and prints its own CPU time. Output:

    python cpu_s_per_pair=... wall_s_per_pair=... pairs=N valid_pixels=...
    cpp cpu_s_per_pair=... wall_s_per_pair=... pairs=N valid_pixels=...
    cpp/python cpu ratio=...
"""

import os
import re
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
PACKAGE_DIR = HERE.parent
STEREO_DEPTH_DIR = PACKAGE_DIR.parent / "stereo_depth"
sys.path.insert(0, str(STEREO_DEPTH_DIR))

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import tests.conftest as conftest  # noqa: E402  (ROS message doubles: the bench never touches ROS)
import yaml  # noqa: E402
from pydantic import BaseModel, ConfigDict, Field, ValidationError  # noqa: E402
from stereo_depth.calibration import load_stereo_calibration, rectification_maps, rectify  # noqa: E402
from stereo_depth.config import DepthConfig  # noqa: E402
from stereo_depth.depth import MIN_DISPARITY, compute_disparity, create_matcher, disparity_to_depth_mm  # noqa: E402
from stereo_depth.messages import depth_image_msg, image_to_grey  # noqa: E402

CONFIG_ENV_VAR = "BENCH_COMPARE_CONFIG"
DEFAULT_CONFIG_PATH = HERE / "bench_compare.yaml"
FIXTURES_DIR = PACKAGE_DIR / "test" / "fixtures"
DEFAULT_LEFT_PNG = FIXTURES_DIR / "left.png"
DEFAULT_RIGHT_PNG = FIXTURES_DIR / "right.png"
DEFAULT_CALIBRATION_DIR = FIXTURES_DIR / "ideal" / "calibration"
CPP_RESULT = re.compile(
    r"^cpp cpu_s_per_pair=(?P<cpu>\S+) wall_s_per_pair=(?P<wall>\S+) pairs=\d+ valid_pixels=(?P<valid>\d+)"
)
EXIT_CONFIG = 78
EXIT_FAILED = 1


class BenchCompareConfig(BaseModel):
    """bench_compare settings.

    Attributes:
        pairs (int): Timed pairs per path.
        warmup (int): Untimed pairs before timing.
        encoding (str): Camera Image encoding the grey pair is sent as.
        left_png (Path | None): Left grey PNG; None = the parity fixture.
        right_png (Path | None): Right grey PNG; None = the parity fixture.
        calibration_dir (Path | None): Calibration directory; None = the ideal fixture calibration.
        depth (dict[str, Any]): stereo_depth config keys for both paths (validated by DepthConfig).
        cpp_command (list[str]): Command running stereo_depth_bench; the job file path is appended.
        work_dir (Path | None): Directory of the C++ job file; None = a temporary directory.
    """

    model_config = ConfigDict(extra="forbid", frozen=True)

    pairs: int = Field(default=30, ge=1)
    warmup: int = Field(default=2, ge=0)
    encoding: str = "bgr8"
    left_png: Path | None = None
    right_png: Path | None = None
    calibration_dir: Path | None = None
    depth: dict[str, Any] = Field(default_factory=dict)
    cpp_command: list[str] = Field(min_length=1)
    work_dir: Path | None = None


def load_bench_config(path: Path) -> BenchCompareConfig:
    """Load and validate the bench settings.

    Args:
        path (Path): YAML file.

    Returns:
        BenchCompareConfig: The settings.

    Raises:
        ValueError: If the file is missing or invalid.
    """
    if not path.is_file():
        raise ValueError(f"config file not found: {path}")
    data = yaml.safe_load(path.read_text()) or {}
    try:
        config = BenchCompareConfig.model_validate(data)
        DepthConfig.model_validate(config.depth)
    except ValidationError as e:
        raise ValueError(f"{path}: {e}") from e
    return config


def read_grey(path: Path) -> np.ndarray:
    """Read a PNG as uint8 grey.

    Args:
        path (Path): Image file.

    Returns:
        np.ndarray: uint8 (H, W).
    """
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(f"cannot read {path}")
    return image


def python_bench(
    config: BenchCompareConfig, left_png: Path, right_png: Path, calibration_dir: Path
) -> dict[str, float]:
    """Time the Python node's per-pair path: image_to_grey, rectify, compute_disparity, disparity_to_depth_mm, bytes.

    Args:
        config (BenchCompareConfig): Settings.
        left_png (Path): Left grey PNG.
        right_png (Path): Right grey PNG.
        calibration_dir (Path): Calibration directory.

    Returns:
        dict[str, float]: cpu and wall seconds per pair, valid pixel count.
    """
    depth_config = DepthConfig.model_validate(config.depth)
    loaded = load_stereo_calibration(calibration_dir)
    if loaded.calibration is None:
        raise ValueError(loaded.reason)
    calibration = loaded.calibration
    maps = rectification_maps(calibration)
    cv2.setNumThreads(depth_config.matcher_threads)
    matcher = create_matcher(depth_config)
    left_msg = conftest.image_msg(read_grey(left_png), config.encoding)
    right_msg = conftest.image_msg(read_grey(right_png), config.encoding)
    header = conftest.Header()
    valid = 0

    def run() -> None:
        nonlocal valid
        left_grey, right_grey = image_to_grey(left_msg), image_to_grey(right_msg)
        assert left_grey is not None and right_grey is not None
        rect_left, rect_right = rectify(maps, left_grey, right_grey)
        disparity = compute_disparity(matcher, rect_left, rect_right)
        depth = disparity_to_depth_mm(
            disparity,
            calibration.fx,
            calibration.baseline_m,
            float(MIN_DISPARITY),
            depth_config.min_depth_m,
            depth_config.max_depth_m,
        )
        if not depth.any():
            raise RuntimeError("the Python path published no depth")
        depth_image_msg(header, depth)
        valid = int(np.count_nonzero(depth))

    for _ in range(config.warmup):
        run()
    cpu_start, wall_start = time.process_time(), time.perf_counter()
    for _ in range(config.pairs):
        run()
    cpu_s, wall_s = time.process_time() - cpu_start, time.perf_counter() - wall_start
    return {"cpu": cpu_s / config.pairs, "wall": wall_s / config.pairs, "valid": valid}


def cpp_bench(config: BenchCompareConfig, left_png: Path, right_png: Path, calibration_dir: Path) -> dict[str, float]:
    """Run stereo_depth_bench on a job file with the same input and settings.

    Args:
        config (BenchCompareConfig): Settings.
        left_png (Path): Left grey PNG.
        right_png (Path): Right grey PNG.
        calibration_dir (Path): Calibration directory.

    Returns:
        dict[str, float]: cpu and wall seconds per pair, valid pixel count.
    """
    job = {
        "calibration_dir": str(calibration_dir),
        "left_png": str(left_png),
        "right_png": str(right_png),
        "encoding": config.encoding,
        "pairs": config.pairs,
        "warmup": config.warmup,
        "depth": dict(config.depth),
    }
    with tempfile.TemporaryDirectory(dir=config.work_dir) as tmp:
        job_path = Path(tmp) / "bench_job.yaml"
        job_path.write_text(yaml.safe_dump(job, sort_keys=False))
        result = subprocess.run([*config.cpp_command, str(job_path)], capture_output=True, text=True, check=False)
    match = next((m for line in result.stdout.splitlines() if (m := CPP_RESULT.match(line))), None)
    if result.returncode != 0 or match is None:
        raise RuntimeError(f"C++ bench failed (exit {result.returncode}): {result.stdout}{result.stderr}".strip())
    return {"cpu": float(match["cpu"]), "wall": float(match["wall"]), "valid": int(match["valid"])}


def report(name: str, result: dict[str, float], pairs: int) -> str:
    """One result line.

    Args:
        name (str): python or cpp.
        result (dict[str, float]): From python_bench or cpp_bench.
        pairs (int): Timed pairs.

    Returns:
        str: The line.
    """
    return (
        f"{name} cpu_s_per_pair={result['cpu']:.6f} wall_s_per_pair={result['wall']:.6f} "
        f"pairs={pairs} valid_pixels={int(result['valid'])}"
    )


def main() -> int:
    """Run both paths and print CPU seconds per pair for each.

    Returns:
        int: 0, 78 on a config error, 1 when a path fails.
    """
    path = Path(os.environ.get(CONFIG_ENV_VAR) or DEFAULT_CONFIG_PATH)
    try:
        config = load_bench_config(path)
    except ValueError as e:
        print(f"bench_compare config error: {e}", file=sys.stderr)
        return EXIT_CONFIG
    left_png = (config.left_png or DEFAULT_LEFT_PNG).resolve()
    right_png = (config.right_png or DEFAULT_RIGHT_PNG).resolve()
    calibration_dir = (config.calibration_dir or DEFAULT_CALIBRATION_DIR).resolve()
    print(
        f"bench_compare: {config.pairs} pairs, {config.encoding}, {left_png.name}/{right_png.name}, {calibration_dir}"
    )
    try:
        python = python_bench(config, left_png, right_png, calibration_dir)
        print(report("python", python, config.pairs), flush=True)
        cpp = cpp_bench(config, left_png, right_png, calibration_dir)
        print(report("cpp", cpp, config.pairs))
    except (RuntimeError, ValueError) as e:
        print(f"bench_compare: {e}", file=sys.stderr)
        return EXIT_FAILED
    print(f"cpp/python cpu ratio={cpp['cpu'] / python['cpu']:.3f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
