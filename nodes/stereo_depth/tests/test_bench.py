"""Tests of stereo_depth.bench: config, the synthetic pair, one timed cell and the printed table."""

from pathlib import Path

import numpy as np
import pytest

from stereo_depth import bench
from stereo_depth.bench import BenchConfig, ConfigError


def one_cell() -> BenchConfig:
    """A one-cell grid that runs in well under a second.

    Returns:
        BenchConfig: The config.
    """
    return BenchConfig(sizes=[[160, 120]], num_disparities=[16], matchers=["sgbm_3way"], threads=[1], runs=2)


def test_repo_config_file_loads_with_the_documented_grid() -> None:
    config = bench.load_bench_config(bench.DEFAULT_CONFIG_PATH)
    assert bench.DEFAULT_CONFIG_PATH == Path(bench.__file__).resolve().parent.parent / "config" / "bench.yaml"
    assert [480, 360] in config.sizes
    assert 48 in config.num_disparities
    assert set(config.matchers) == {"bm", "sgbm_3way", "sgbm"}
    assert config.target_fps == 5.0


def test_config_path_comes_from_the_environment(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.delenv(bench.CONFIG_ENV_VAR, raising=False)
    assert bench.config_path_from_env() == bench.DEFAULT_CONFIG_PATH
    monkeypatch.setenv(bench.CONFIG_ENV_VAR, str(tmp_path / "b.yaml"))
    assert bench.config_path_from_env() == tmp_path / "b.yaml"


def test_config_rejects_unknown_keys_and_bad_values(tmp_path: Path) -> None:
    path = tmp_path / "b.yaml"
    path.write_text("bogus: 1\n")
    with pytest.raises(ConfigError, match="bogus"):
        bench.load_bench_config(path)
    with pytest.raises(ValueError, match="multiple of 16"):
        BenchConfig(num_disparities=[40])
    with pytest.raises(ValueError):
        BenchConfig(matchers=["bogus"])
    with pytest.raises(ValueError, match="width, height"):
        BenchConfig(sizes=[[480]])


def test_synthetic_pair_is_shifted_by_the_disparity() -> None:
    left, right = bench.synthetic_pair(64, 48, 8)
    assert left.shape == right.shape == (48, 64) and left.dtype == np.uint8
    assert np.array_equal(left[:, 8:], right[:, :-8])


@pytest.mark.parametrize("matcher", ["bm", "sgbm_3way", "sgbm"])
def test_create_matcher_builds_each_kind(matcher: str) -> None:
    left, right = bench.synthetic_pair(160, 120, 8)
    disparity = bench.create_matcher(matcher, 16, 5).compute(left, right)
    assert disparity.shape == left.shape


def test_run_bench_times_every_cell() -> None:
    rows = bench.run_bench(one_cell())
    assert len(rows) == 1
    row = rows[0]
    assert (row.width, row.height, row.matcher, row.num_disparities, row.threads) == (160, 120, "sgbm_3way", 16, 1)
    assert row.median_ms > 0.0 and row.cores_at_fps >= 0.0


def test_format_table_has_a_header_and_one_line_per_cell() -> None:
    rows = bench.run_bench(one_cell())
    lines = bench.format_table(rows, 5.0).splitlines()
    assert len(lines) == 2
    header = lines[0].split()
    assert header == ["size", "matcher", "disparities", "threads", "median_ms", "cores@5fps"]
    assert len(lines[1].split()) == len(header)
    assert lines[1].split()[0] == "160x120"


def test_main_prints_the_table(monkeypatch: pytest.MonkeyPatch, tmp_path: Path, capsys: pytest.CaptureFixture) -> None:
    path = tmp_path / "b.yaml"
    path.write_text("sizes: [[160, 120]]\nnum_disparities: [16]\nmatchers: [bm]\nthreads: [1]\nruns: 1\n")
    monkeypatch.setenv(bench.CONFIG_ENV_VAR, str(path))
    assert bench.main() == 0
    out = capsys.readouterr().out.splitlines()
    assert out[0].split()[0] == "size" and out[1].split()[1] == "bm"


def test_main_reports_a_config_error(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.setenv(bench.CONFIG_ENV_VAR, str(tmp_path / "missing.yaml"))
    assert bench.main() == 1
