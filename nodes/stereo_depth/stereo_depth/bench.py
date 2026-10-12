"""Matcher benchmark (python3 -m stereo_depth.bench): times a grid of image sizes, disparity ranges, matchers and
thread counts on a synthetic textured pair and prints the median wall time and the cores needed at the target rate.

Config: STEREO_BENCH_CONFIG or config/bench.yaml next to the package. Run it on the client to measure the real Pi cost.
The cores column is process CPU time per pair times target_fps (1.0 = one full core).
"""

import os
import statistics
import sys
import time
from pathlib import Path
from typing import Literal, NamedTuple

import cv2
import numpy as np
import yaml
from pydantic import BaseModel, ConfigDict, Field, ValidationError, field_validator

from .config import DISPARITY_MULTIPLE, MAX_BLOCK_SIZE, ConfigError

CONFIG_ENV_VAR = "STEREO_BENCH_CONFIG"
DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "bench.yaml"
MS_PER_S = 1000.0
BENCH_DISPARITY_FRACTION = 0.5  # the synthetic pair's shift as a fraction of the disparity range
TEXTURE_SEED = 7
COLUMNS = ("size", "matcher", "disparities", "threads", "median_ms", "cores@{fps:g}fps")
MatcherName = Literal["bm", "sgbm_3way", "sgbm"]
SGBM_MODES = {"sgbm_3way": cv2.StereoSGBM_MODE_SGBM_3WAY, "sgbm": cv2.StereoSGBM_MODE_SGBM}
SGBM_P1_PER_PIXEL = 8
SGBM_P2_PER_PIXEL = 32


class BenchConfig(BaseModel):
    """stereo_depth.bench settings.

    Attributes:
        sizes (list[list[int]]): Image sizes as [width, height].
        num_disparities (list[int]): Disparity ranges, positive multiples of 16.
        matchers (list[str]): bm (StereoBM), sgbm_3way (StereoSGBM MODE_SGBM_3WAY) and/or sgbm (MODE_SGBM).
        threads (list[int]): cv2.setNumThreads values.
        block_size (int): Matcher block size (odd, 5..11 so StereoBM accepts it too).
        runs (int): Timed runs per cell (one untimed warm-up run first).
        target_fps (float): Rate the cores column is computed for.
    """

    model_config = ConfigDict(extra="forbid", frozen=True)

    sizes: list[list[int]] = Field(default_factory=lambda: [[480, 360]], min_length=1)
    num_disparities: list[int] = Field(default_factory=lambda: [48], min_length=1)
    matchers: list[MatcherName] = Field(default_factory=lambda: ["sgbm_3way"], min_length=1)
    threads: list[int] = Field(default_factory=lambda: [1], min_length=1)
    block_size: int = Field(default=5, ge=5, le=MAX_BLOCK_SIZE)
    runs: int = Field(default=10, ge=1)
    target_fps: float = Field(default=5.0, gt=0.0)

    @field_validator("sizes")
    @classmethod
    def check_sizes(_cls, value: list[list[int]]) -> list[list[int]]:
        """Require [width, height] pairs of positive integers.

        Args:
            value (list[list[int]]): Raw sizes.

        Returns:
            list[list[int]]: The sizes.
        """
        for size in value:
            if len(size) != 2 or min(size) <= 0:
                raise ValueError(f"each size must be [width, height] > 0, got {size}")
        return value

    @field_validator("num_disparities")
    @classmethod
    def check_disparities(_cls, value: list[int]) -> list[int]:
        """Require positive multiples of 16.

        Args:
            value (list[int]): Raw disparity ranges.

        Returns:
            list[int]: The ranges.
        """
        for n in value:
            if n <= 0 or n % DISPARITY_MULTIPLE:
                raise ValueError(f"each num_disparities value must be a positive multiple of 16, got {n}")
        return value

    @field_validator("threads")
    @classmethod
    def check_threads(_cls, value: list[int]) -> list[int]:
        """Require positive thread counts.

        Args:
            value (list[int]): Raw thread counts.

        Returns:
            list[int]: The counts.
        """
        if min(value) < 1:
            raise ValueError("threads must be >= 1")
        return value

    @field_validator("block_size")
    @classmethod
    def check_block_size(_cls, value: int) -> int:
        """Require an odd block size.

        Args:
            value (int): Raw block size.

        Returns:
            int: The block size.
        """
        if value % 2 == 0:
            raise ValueError("block_size must be odd")
        return value


class BenchRow(NamedTuple):
    """One timed cell.

    Attributes:
        width (int): Image width.
        height (int): Image height.
        matcher (str): Matcher name.
        num_disparities (int): Disparity range.
        threads (int): cv2 thread count.
        median_ms (float): Median wall time per pair, milliseconds.
        cores_at_fps (float): Median process CPU seconds per pair times target_fps.
    """

    width: int
    height: int
    matcher: str
    num_disparities: int
    threads: int
    median_ms: float
    cores_at_fps: float


def config_path_from_env() -> Path:
    """Return the config path from STEREO_BENCH_CONFIG, or DEFAULT_CONFIG_PATH when unset.

    Returns:
        Path: Config file path.
    """
    return Path(os.environ.get(CONFIG_ENV_VAR) or DEFAULT_CONFIG_PATH)


def load_bench_config(path: Path) -> BenchConfig:
    """Load and validate the benchmark config; an empty file gives the defaults.

    Args:
        path (Path): YAML config file.

    Returns:
        BenchConfig: Validated settings.

    Raises:
        ConfigError: If the file is missing, is not a YAML mapping or fails validation.
    """
    if not path.is_file():
        raise ConfigError(f"config file not found: {path}")
    try:
        data = yaml.safe_load(path.read_text()) or {}
    except yaml.YAMLError as e:
        raise ConfigError(f"{path}: invalid YAML: {e}") from e
    if not isinstance(data, dict):
        raise ConfigError(f"{path}: expected a mapping at the top level, got {type(data).__name__}")
    try:
        return BenchConfig.model_validate(data)
    except ValidationError as e:
        raise ConfigError(f"{path}: {e}") from e


def synthetic_pair(
    width: int, height: int, disparity_px: int, seed: int = TEXTURE_SEED
) -> tuple[np.ndarray, np.ndarray]:
    """A rectified textured pair where the right image is the left shifted by disparity_px.

    Args:
        width (int): Image width.
        height (int): Image height.
        disparity_px (int): Constant disparity, pixels.
        seed (int): Texture seed.

    Returns:
        tuple[np.ndarray, np.ndarray]: (left, right) uint8 grey images (height, width).
    """
    rng = np.random.default_rng(seed)
    coarse = rng.integers(0, 256, size=(height // 2 + 1, (width + disparity_px) // 2 + 1), dtype=np.uint8)
    wide = np.kron(coarse, np.ones((2, 2), dtype=np.uint8))[:height, : width + disparity_px]
    return wide[:, :width].copy(), wide[:, disparity_px : width + disparity_px].copy()


def create_matcher(name: str, num_disparities: int, block_size: int) -> cv2.StereoMatcher:
    """Create one matcher kind.

    Args:
        name (str): bm, sgbm_3way or sgbm.
        num_disparities (int): Disparity range.
        block_size (int): Block size.

    Returns:
        cv2.StereoMatcher: The matcher.
    """
    if name == "bm":
        return cv2.StereoBM.create(numDisparities=num_disparities, blockSize=block_size)
    area = block_size * block_size
    return cv2.StereoSGBM.create(
        minDisparity=0,
        numDisparities=num_disparities,
        blockSize=block_size,
        P1=SGBM_P1_PER_PIXEL * area,
        P2=SGBM_P2_PER_PIXEL * area,
        mode=SGBM_MODES[name],
    )


def time_cell(
    config: BenchConfig, width: int, height: int, matcher_name: str, disparities: int, threads: int
) -> BenchRow:
    """Time one grid cell.

    Args:
        config (BenchConfig): runs, block_size and target_fps.
        width (int): Image width.
        height (int): Image height.
        matcher_name (str): Matcher kind.
        disparities (int): Disparity range.
        threads (int): cv2 thread count.

    Returns:
        BenchRow: The medians.
    """
    cv2.setNumThreads(threads)
    left, right = synthetic_pair(width, height, max(1, int(disparities * BENCH_DISPARITY_FRACTION)))
    matcher = create_matcher(matcher_name, disparities, config.block_size)
    matcher.compute(left, right)
    walls, cpus = [], []
    for _ in range(config.runs):
        wall0, cpu0 = time.perf_counter(), time.process_time()
        matcher.compute(left, right)
        walls.append(time.perf_counter() - wall0)
        cpus.append(time.process_time() - cpu0)
    return BenchRow(
        width=width,
        height=height,
        matcher=matcher_name,
        num_disparities=disparities,
        threads=threads,
        median_ms=statistics.median(walls) * MS_PER_S,
        cores_at_fps=statistics.median(cpus) * config.target_fps,
    )


def run_bench(config: BenchConfig) -> list[BenchRow]:
    """Time every cell of the grid (sizes x matchers x disparities x threads).

    Args:
        config (BenchConfig): Validated settings.

    Returns:
        list[BenchRow]: One row per cell, in grid order.
    """
    return [
        time_cell(config, width, height, matcher, disparities, threads)
        for width, height in config.sizes
        for matcher in config.matchers
        for disparities in config.num_disparities
        for threads in config.threads
    ]


def format_table(rows: list[BenchRow], target_fps: float) -> str:
    """Render the rows as a whitespace-aligned table with a header line.

    Args:
        rows (list[BenchRow]): Timed cells.
        target_fps (float): Rate of the cores column.

    Returns:
        str: The table, one line per cell after the header.
    """
    header = [c.format(fps=target_fps) for c in COLUMNS]
    body = [
        [
            f"{r.width}x{r.height}",
            r.matcher,
            str(r.num_disparities),
            str(r.threads),
            f"{r.median_ms:.1f}",
            f"{r.cores_at_fps:.2f}",
        ]
        for r in rows
    ]
    widths = [max(len(line[i]) for line in [header, *body]) for i in range(len(header))]
    return "\n".join(
        "  ".join(cell.ljust(w) for cell, w in zip(line, widths, strict=True)).rstrip() for line in [header, *body]
    )


def main() -> int:
    """Load the config, run the grid and print the table.

    Returns:
        int: 0 on success, 1 on a config error.
    """
    try:
        config = load_bench_config(config_path_from_env())
    except ConfigError as e:
        print(f"stereo_depth.bench config error: {e}", file=sys.stderr)
        return 1
    print(format_table(run_bench(config), config.target_fps))
    return 0


if __name__ == "__main__":
    sys.exit(main())
