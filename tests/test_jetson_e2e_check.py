"""Unit tests for scripts/jetson_e2e_check.py on synthetic metrics, chrony and health inputs (no network)."""

import math

import pydantic
import pytest
import yaml

from scripts.jetson_e2e_check import (
    DEFAULT_CONFIG,
    PENDING_MESSAGE,
    Observation,
    Settings,
    Snapshot,
    collect,
    evaluate,
    load_settings,
    main,
    p95_from_buckets,
    parse_chrony,
    parse_metrics,
    parse_unit_states,
    rate,
    snapshot_from_text,
    window_p95,
)

BOUNDS = ["0.005", "0.01", "0.025", "0.05", "0.1", "0.2", "0.3", "0.5", "1", "2", "+Inf"]


def metrics_text(frames: dict[str, float], cumulative: dict[str, list[float]], cpu: float) -> str:
    """Render bridge-style exposition text."""
    lines = ["# HELP jetson_bridge_frames_total Frames republished to ROS", "# TYPE jetson_bridge_frames_total counter"]
    for kind, value in frames.items():
        lines.append(f'jetson_bridge_frames_total{{kind="{kind}"}} {value}')
    for kind, counts in cumulative.items():
        for bound, count in zip(BOUNDS, counts, strict=True):
            lines.append(f'jetson_bridge_latency_seconds_bucket{{kind="{kind}",le="{bound}"}} {count}')
        lines.append(f'jetson_bridge_latency_seconds_count{{kind="{kind}"}} {counts[-1]}')
    lines.append(f"process_cpu_seconds_total {cpu}")
    return "\n".join(lines) + "\n"


def fast_buckets(n: float) -> list[float]:
    """All n observations at or below 0.05 s."""
    return [0, 0, 0, n, n, n, n, n, n, n, n]


def slow_buckets(n: float) -> list[float]:
    """All n observations between 0.2 and 0.3 s."""
    return [0, 0, 0, 0, 0, 0, n, n, n, n, n]


def snap(depth: float, fused: float, rgb: float, depth_b=None, rgb_b=None, cpu: float = 0.0) -> Snapshot:
    """Snapshot from counters and cumulative bucket lists."""
    cumulative = {"rgb": rgb_b or fast_buckets(rgb), "depth": depth_b or fast_buckets(depth)}
    return snapshot_from_text(metrics_text({"depth": depth, "fused": fused, "rgb": rgb}, cumulative, cpu))


CHRONY_OK = """Reference ID    : 0A4D0001 (10.77.0.1)
Stratum         : 4
System time     : 0.000120000 seconds slow of NTP time
Last offset     : -0.000100000 seconds
"""


def obs(**kw) -> Observation:
    """Passing calibrated observation, overridden by kw."""
    base = {
        "health": {"state": "ok", "mono_state": "ok"},
        "chrony": parse_chrony(CHRONY_OK),
        "first": snap(0, 0, 0, cpu=10.0),
        "second": snap(330, 66, 330, cpu=14.5),
        "window_s": 30.0,
        "unit_states": {
            "ros2-overview_camera": "not-found",
            "ros2-stereo_depth": "not-found",
            "ros2-stereo_depth_cpp": "not-found",
        },
    }
    base.update(kw)
    return Observation(**base)


def failed(report) -> set[str]:
    """Names of failed checks."""
    return {c.name for c in report.checks if not c.ok}


def test_parse_metrics_reads_labels_and_values() -> None:
    samples = parse_metrics('# c\nfoo{a="x",le="+Inf"} 3\nbar 2.5e-1\nbad line\nbaz{k="v"} NaN\n')
    assert [(s.name, s.labels, s.value) for s in samples[:2]] == [
        ("foo", {"a": "x", "le": "+Inf"}, 3.0),
        ("bar", {}, 0.25),
    ]
    assert math.isnan(samples[2].value)


def test_snapshot_extracts_frames_buckets_cpu() -> None:
    s = snap(10, 2, 20, cpu=1.5)
    assert s.frames == {"depth": 10.0, "fused": 2.0, "rgb": 20.0}
    assert s.cpu_seconds == 1.5
    assert s.buckets["rgb"][0.05] == 20.0 and s.buckets["rgb"][math.inf] == 20.0


def test_snapshot_without_cpu_series_is_an_error() -> None:
    with pytest.raises(ValueError, match="process_cpu_seconds_total"):
        snapshot_from_text('jetson_bridge_frames_total{kind="rgb"} 1\n')


def test_rate_is_delta_over_window() -> None:
    assert rate(10, 40, 30) == 1.0
    assert rate(5, 5, 30) == 0.0


def test_rate_rejects_counter_reset_and_bad_window() -> None:
    with pytest.raises(ValueError):
        rate(10, 5, 30)
    with pytest.raises(ValueError):
        rate(1, 2, 0)


def test_p95_interpolates_inside_bucket() -> None:
    # 100 obs all in (0.2, 0.3]: p95 = 0.2 + 0.1 * 0.95
    buckets = {0.2: 0.0, 0.3: 100.0, math.inf: 100.0}
    assert p95_from_buckets(buckets) == pytest.approx(0.295)


def test_p95_first_bucket_starts_at_zero() -> None:
    assert p95_from_buckets({0.01: 10.0, 0.1: 10.0, math.inf: 10.0}) == pytest.approx(0.0095)


def test_p95_in_inf_bucket_is_inf_and_empty_is_none() -> None:
    assert p95_from_buckets({0.1: 0.0, 2.0: 0.0, math.inf: 10.0}) == math.inf
    assert p95_from_buckets({0.1: 0.0, math.inf: 0.0}) is None


def test_p95_ignores_pre_window_observations_via_delta() -> None:
    first = snap(0, 0, 100, rgb_b=slow_buckets(100))
    second = snap(0, 0, 200, rgb_b=[a + b for a, b in zip(slow_buckets(100), fast_buckets(100), strict=True)])
    assert window_p95(first, second, "rgb") <= 0.05


def test_parse_chrony() -> None:
    c = parse_chrony(CHRONY_OK)
    assert c.reference == "10.77.0.1" and c.offset_s == pytest.approx(0.00012)


def test_parse_chrony_fast_and_garbage() -> None:
    assert (
        parse_chrony("Reference ID : 0A4D0001 (10.77.0.1)\nSystem time : 0.5 seconds fast of NTP time\n").offset_s
        == 0.5
    )
    c = parse_chrony("506 Cannot talk to daemon\n")
    assert c.reference is None and c.offset_s is None


def test_parse_unit_states() -> None:
    assert parse_unit_states("a.service not-found\nb.service loaded\n") == {"a": "not-found", "b": "loaded"}


def test_calibrated_pass() -> None:
    r = evaluate(obs(), Settings())
    assert r.passed and not failed(r)
    assert "depth pending" not in r.summary


def test_uncalibrated_pass_has_no_depth_and_says_pending() -> None:
    r = evaluate(
        obs(health={"state": "uncalibrated", "mono_state": "ok"}, second=snap(0, 0, 330, cpu=14.5)), Settings()
    )
    assert r.passed
    assert PENDING_MESSAGE == "depth pending USER STEP calibration" and PENDING_MESSAGE in r.summary


def test_uncalibrated_placeholder_depth_fails() -> None:
    r = evaluate(
        obs(health={"state": "uncalibrated", "mono_state": "ok"}, second=snap(5, 0, 330, cpu=14.5)), Settings()
    )
    assert "no_placeholder" in failed(r)
    r = evaluate(
        obs(health={"state": "uncalibrated", "mono_state": "ok"}, second=snap(0, 3, 330, cpu=14.5)), Settings()
    )
    assert "no_placeholder" in failed(r)


@pytest.mark.parametrize("state", ["camera_error", "starting", "weird"])
def test_bad_health_state_fails(state: str) -> None:
    assert "jetson_health" in failed(evaluate(obs(health={"state": state, "mono_state": "ok"}), Settings()))


def test_chrony_wrong_reference_and_big_offset_fail() -> None:
    other = parse_chrony(CHRONY_OK.replace("10.77.0.1", "10.0.0.9"))
    assert "chrony" in failed(evaluate(obs(chrony=other), Settings()))
    big = parse_chrony(CHRONY_OK.replace("0.000120000", "0.002"))
    assert "chrony" in failed(evaluate(obs(chrony=big), Settings()))
    assert "chrony" in failed(evaluate(obs(chrony=parse_chrony("garbage")), Settings()))


def test_rgb_rate_and_latency_branches() -> None:
    assert "rgb_rate" in failed(evaluate(obs(second=snap(330, 66, 100, cpu=14.5)), Settings()))
    slow = obs(second=snap(330, 66, 330, rgb_b=slow_buckets(330), cpu=14.5))
    assert "rgb_latency" in failed(evaluate(slow, Settings()))


def test_depth_branches_when_calibrated() -> None:
    assert "depth_rate" in failed(evaluate(obs(second=snap(100, 66, 330, cpu=14.5)), Settings()))
    slow = obs(second=snap(330, 66, 330, depth_b=slow_buckets(330), cpu=14.5))
    assert "depth_latency" in failed(evaluate(slow, Settings()))


def test_fused_rate_required_only_when_mono_ok() -> None:
    low = snap(330, 10, 330, cpu=14.5)
    assert "fused_rate" in failed(evaluate(obs(second=low), Settings()))
    r = evaluate(obs(second=low, health={"state": "ok", "mono_state": "failed"}), Settings())
    assert "fused_rate" not in failed(r)


def test_bridge_cpu_ceiling() -> None:
    # 4.5 cpu-s over 30 s = 0.15 core passes; 12 cpu-s = 0.4 core fails
    assert "bridge_cpu" not in failed(evaluate(obs(), Settings()))
    assert "bridge_cpu" in failed(evaluate(obs(second=snap(330, 66, 330, cpu=22.0)), Settings()))


def test_retired_units_must_be_absent() -> None:
    states = {"ros2-overview_camera": "not-found", "ros2-stereo_depth": "loaded", "ros2-stereo_depth_cpp": "not-found"}
    r = evaluate(obs(unit_states=states), Settings())
    assert "retired_units" in failed(r)
    assert "ros2-stereo_depth" in next(c.detail for c in r.checks if c.name == "retired_units")


def test_settings_defaults_and_shipped_yaml() -> None:
    s = Settings()
    assert (
        s.window_s == 30
        and s.metrics_port == 19114
        and s.jetson_ip == "10.77.0.2"
        and s.chrony_reference == "10.77.0.1"
    )
    loaded = load_settings(DEFAULT_CONFIG)
    assert loaded == s
    assert set(yaml.safe_load(DEFAULT_CONFIG.read_text())) <= set(Settings.model_fields)


def test_settings_reject_bad_values() -> None:
    with pytest.raises(pydantic.ValidationError):
        Settings(window_s=0)
    with pytest.raises(pydantic.ValidationError):
        Settings(bogus=1)


def fake_runner(calls: list[tuple[str, str]], texts: list[str], health: str = '{"state": "ok", "mono_state": "ok"}'):
    """Runner returning canned output by command; metrics reads pop from texts."""

    def run(host: str, command: str) -> str:
        calls.append((host, command))
        if "/metrics" in command:
            return texts.pop(0)
        if "/health" in command:
            return health
        if "chronyc" in command:
            return CHRONY_OK
        return "".join(f"{u}.service not-found\n" for u in Settings().retired_units)

    return run


def good_texts() -> list[str]:
    """Two metrics reads 30 s apart that satisfy every threshold."""
    return [
        metrics_text({"depth": 0, "fused": 0, "rgb": 0}, {"rgb": fast_buckets(0), "depth": fast_buckets(0)}, 10.0),
        metrics_text(
            {"depth": 330, "fused": 66, "rgb": 330}, {"rgb": fast_buckets(330), "depth": fast_buckets(330)}, 14.5
        ),
    ]


def test_collect_reads_metrics_twice_window_apart_with_the_documented_commands() -> None:
    calls: list[tuple[str, str]] = []
    now = [0.0]
    slept: list[float] = []

    def sleep(seconds: float) -> None:
        slept.append(seconds)
        now[0] += seconds

    cfg = Settings()
    o = collect(cfg, fake_runner(calls, good_texts()), sleep, lambda: now[0])
    assert slept == [30.0] and o.window_s == 30.0
    assert [c for c in calls if "/metrics" in c[1]] == [
        ("client", "curl -sf --max-time 5 http://127.0.0.1:19114/metrics")
    ] * 2
    assert ("client", "curl -sf --max-time 5 http://10.77.0.2:8090/health") in calls
    assert ("jetson", "chronyc -n tracking") in calls
    assert evaluate(o, cfg).passed


def test_main_exit_codes(monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]) -> None:
    texts = good_texts()
    monkeypatch.setattr("scripts.jetson_e2e_check.ssh_run", fake_runner([], texts))
    monkeypatch.setattr("scripts.jetson_e2e_check.time.sleep", lambda s: None)
    ticks = iter(range(0, 1000, 30))
    monkeypatch.setattr("scripts.jetson_e2e_check.time.monotonic", lambda: float(next(ticks)))
    assert main([]) == 0
    assert "PASS" in capsys.readouterr().out


def test_main_returns_2_when_collection_fails(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    def boom(host: str, command: str) -> str:
        raise RuntimeError("ssh client failed (255)")

    monkeypatch.setattr("scripts.jetson_e2e_check.ssh_run", boom)
    assert main([]) == 2
    assert "ERROR" in capsys.readouterr().out
