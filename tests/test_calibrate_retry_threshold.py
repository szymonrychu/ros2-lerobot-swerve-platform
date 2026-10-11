"""Unit tests for scripts/calibrate_retry_threshold.py with fixture retry records (no network)."""

import json
from datetime import date
from pathlib import Path

import httpx
import numpy as np
import pydantic
import pytest
import yaml
from scipy.stats import binomtest

from scripts.calibrate_retry_threshold import (
    DEFAULT_CONFIG,
    FETCH_TIMEOUT_S,
    RetryRecord,
    Settings,
    analyze,
    bin_index,
    brier_score,
    build_snippet,
    count_bins,
    eai_breakdown,
    ece,
    fetch_records,
    filter_records,
    load_settings,
    main,
    parse_records,
    propose_threshold,
    reliability_bins,
    render_report,
    wilson_interval,
)


def rec(p: float, outcome: str = "done", **kw: object) -> dict:
    """Build a schema_version 1 record dict."""
    base = {
        "schema_version": 1,
        "ts": 1000.0,
        "instruction_id": "i1",
        "phase_id": "p1",
        "phase_name": "grasp",
        "attempt": 1,
        "iteration": 1,
        "p_retry_success": p,
        "threshold": 0.5,
        "retried": p >= 0.5,
        "refusal": None,
        "failure_type": "execution_failure",
        "eai_error_type": "affordance",
        "next_outcome": outcome,
        "next_via": "none" if outcome == "unobserved" else "auto_retry",
        "model": "m1",
    }
    base.update(kw)
    return base


def records(raw: list[dict]) -> list[RetryRecord]:
    """Validate raw dicts."""
    return [RetryRecord(**r) for r in raw]


def calibrated_set() -> list[dict]:
    """100 observed records: p=0.2 (20 records, 20% success), p=0.9 (80 records, 90% success)."""
    out = [rec(0.2, "done") for _ in range(4)] + [rec(0.2, "failed") for _ in range(16)]
    out += [rec(0.9, "done") for _ in range(72)] + [rec(0.9, "failed") for _ in range(8)]
    return out


def make_settings(**kw: object) -> Settings:
    """Settings with a file source and small defaults."""
    base = {"source_file": "x.jsonl"}
    base.update(kw)
    return Settings(**base)


def test_settings_defaults_and_shipped_yaml() -> None:
    s = load_settings(DEFAULT_CONFIG)
    assert s.bins == 10
    assert s.target_success_rate == 0.5
    assert s.confidence == 0.95
    assert s.min_samples == 30
    assert s.min_bin_samples == 5
    assert s.source_url == "http://client.ros2.lan/api/agent/retry_outcomes"
    assert s.source_file is None


@pytest.mark.parametrize(
    "bad",
    [
        {},
        {"source_url": "http://a", "source_file": "b"},
        {"source_file": "b", "bins": 1},
        {"source_file": "b", "target_success_rate": 1.5},
        {"source_file": "b", "confidence": 1.0},
        {"source_file": "b", "min_samples": 0},
        {"source_file": "b", "min_bin_samples": 0},
        {"source_file": "b", "unknown_key": 1},
    ],
)
def test_settings_validation_errors(bad: dict) -> None:
    with pytest.raises(pydantic.ValidationError):
        Settings(**bad)


def test_load_settings_from_file(tmp_path: Path) -> None:
    path = tmp_path / "c.yaml"
    path.write_text(yaml.safe_dump({"source_file": "r.jsonl", "model": "m1", "since_ts": 5}))
    s = load_settings(path)
    assert (s.source_file, s.model, s.since_ts, s.source_url) == ("r.jsonl", "m1", 5, None)


def test_parse_records_skips_unknown_versions_and_invalid() -> None:
    raw = [rec(0.5), rec(0.6, schema_version=2), {"schema_version": 1, "ts": "bad"}, rec(0.7)]
    parsed, unknown, invalid = parse_records(raw)
    assert [r.p_retry_success for r in parsed] == [0.5, 0.7]
    assert (unknown, invalid) == (1, 1)


def test_fetch_records_from_file_skips_blank_and_corrupt_lines(tmp_path: Path) -> None:
    path = tmp_path / "r.jsonl"
    path.write_text(json.dumps(rec(0.4)) + "\n\nnot json\n" + json.dumps(rec(0.6)) + "\n")
    out = fetch_records(make_settings(source_file=str(path)))
    assert [r["p_retry_success"] for r in out] == [0.4, 0.6]


def test_fetch_records_from_url_uses_timeout(monkeypatch: pytest.MonkeyPatch) -> None:
    seen: dict = {}

    def handler(request: httpx.Request) -> httpx.Response:
        seen["url"] = str(request.url)
        return httpx.Response(200, json={"records": [rec(0.4)], "path_ok": True})

    real = httpx.Client

    def fake_client(**kw: object) -> httpx.Client:
        seen["timeout"] = kw.get("timeout")
        return real(transport=httpx.MockTransport(handler))

    monkeypatch.setattr(httpx, "Client", fake_client)
    s = Settings(source_url="http://robot/api/agent/retry_outcomes")
    out = fetch_records(s)
    assert seen["url"] == "http://robot/api/agent/retry_outcomes"
    assert seen["timeout"] == FETCH_TIMEOUT_S == 10
    assert len(out) == 1


def test_filter_by_model_and_since_ts() -> None:
    recs = records([rec(0.5, ts=1.0, model="a"), rec(0.5, ts=5.0, model="a"), rec(0.5, ts=9.0, model="b")])
    assert len(filter_records(recs, None, None)) == 3
    assert [r.ts for r in filter_records(recs, 5.0, None)] == [5.0, 9.0]
    assert [r.ts for r in filter_records(recs, None, "a")] == [1.0, 5.0]
    assert [r.ts for r in filter_records(recs, 2.0, "a")] == [5.0]


def test_bin_index_puts_one_in_last_bin() -> None:
    assert bin_index(0.0, 10) == 0
    assert bin_index(0.09, 10) == 0
    assert bin_index(0.1, 10) == 1
    assert bin_index(1.0, 10) == 9
    assert bin_index(0.5, 4) == 2


def test_count_bins_total_observed_unobserved() -> None:
    recs = records([rec(0.05), rec(0.05, "unobserved"), rec(0.95, "failed"), rec(1.0, "unobserved")])
    counts = count_bins(recs, 10)
    assert counts[0] == {"total": 2, "observed": 1, "unobserved": 1}
    assert counts[9] == {"total": 2, "observed": 1, "unobserved": 1}
    assert counts[5] == {"total": 0, "observed": 0, "unobserved": 0}


def test_brier_matches_hand_computed_and_excludes_unobserved() -> None:
    recs = records([rec(0.8, "done"), rec(0.6, "failed"), rec(0.5, "skipped"), rec(0.99, "unobserved")])
    expected = ((0.8 - 1) ** 2 + (0.6 - 0) ** 2 + (0.5 - 0) ** 2) / 3
    brier, base = brier_score(recs)
    assert brier == pytest.approx(expected)
    rate = 1 / 3
    assert base == pytest.approx(((1 - rate) ** 2 + 2 * rate**2) / 3)


def test_brier_empty_is_none() -> None:
    assert brier_score(records([rec(0.9, "unobserved")])) == (None, None)


def test_wilson_interval_matches_scipy() -> None:
    low, high = wilson_interval(7, 10, 0.95)
    ci = binomtest(7, 10).proportion_ci(confidence_level=0.95, method="wilson")
    assert (low, high) == (ci.low, ci.high)


def test_reliability_bins_and_ece() -> None:
    recs = records(calibrated_set() + [rec(0.2, "unobserved")])
    bins = reliability_bins(recs, 10, 0.95, 5)
    assert len(bins) == 10
    b2, b9 = bins[2], bins[9]
    assert (b2["n"], b9["n"]) == (20, 80)
    assert b2["mean_p"] == pytest.approx(0.2)
    assert b2["success_rate"] == pytest.approx(0.2)
    ci = binomtest(72, 80).proportion_ci(confidence_level=0.95, method="wilson")
    assert (b9["ci_low"], b9["ci_high"]) == (ci.low, ci.high)
    assert bins[0]["n"] == 0 and bins[0]["success_rate"] is None and bins[0]["ci_low"] is None
    assert b9["low_n"] is False
    expected_ece = (20 * abs(0.2 - 0.2) + 80 * abs(0.9 - 0.9)) / 100
    assert ece(bins) == pytest.approx(expected_ece)


def test_reliability_bin_flags_low_n() -> None:
    bins = reliability_bins(records([rec(0.3, "done")] * 2), 10, 0.95, 5)
    assert bins[3]["low_n"] is True


def test_ece_nonzero_for_miscalibration() -> None:
    recs = records([rec(0.9, "failed")] * 4 + [rec(0.9, "done")])
    assert ece(reliability_bins(recs, 10, 0.95, 1)) == pytest.approx(abs(0.9 - 0.2))


def test_eai_breakdown_success_rates() -> None:
    recs = records(
        [
            rec(0.5, "done", eai_error_type="affordance"),
            rec(0.5, "failed", eai_error_type="affordance"),
            rec(0.5, "done", eai_error_type="parsing"),
            rec(0.5, "unobserved", eai_error_type="parsing"),
        ]
    )
    out = eai_breakdown(recs)
    assert out["affordance"] == {"n": 2, "success_rate": 0.5}
    assert out["parsing"] == {"n": 1, "success_rate": 1.0}


def test_proposal_on_calibrated_set() -> None:
    recs = records(calibrated_set())
    t = propose_threshold(recs, target=0.5, confidence=0.95, min_samples=30)
    # p>=0.2 subset: 76/100 done, lower bound well above 0.5, and 0.2 is the minimum observed p.
    assert t == pytest.approx(0.2)
    # Raising the target moves the proposal past the p=0.2 cluster.
    t2 = propose_threshold(recs, target=0.8, confidence=0.95, min_samples=30)
    assert t2 == pytest.approx(0.25)


def test_proposal_never_below_min_observed_p() -> None:
    recs = records([rec(0.7, "done")] * 40)
    t = propose_threshold(recs, target=0.5, confidence=0.95, min_samples=30)
    assert t == pytest.approx(0.7)


def test_proposal_insufficient_data() -> None:
    assert propose_threshold(records([rec(0.9, "done")] * 10), 0.5, 0.95, 30) is None
    assert propose_threshold(records([rec(0.9, "failed")] * 50), 0.5, 0.95, 30) is None
    assert propose_threshold([], 0.5, 0.95, 30) is None


def test_proposal_requires_min_samples_in_subset() -> None:
    recs = records([rec(0.9, "done")] * 20 + [rec(0.1, "failed")] * 20)
    # Only 20 records have p>=0.9 and the subset p>=0.1 has 50% success: nothing qualifies.
    assert propose_threshold(recs, 0.6, 0.95, 30) is None


def test_analyze_result_shape_and_current_threshold() -> None:
    raw = calibrated_set() + [rec(0.4, "unobserved", ts=2000.0, threshold=0.62)]
    result = analyze(records(raw), make_settings(), skipped_unknown=3, skipped_invalid=1)
    assert result["counts"]["total"] == 101
    assert result["counts"]["observed"] == 100
    assert result["counts"]["unobserved"] == 1
    assert result["skipped_unknown_schema"] == 3
    assert result["skipped_invalid"] == 1
    assert result["current_threshold"] == 0.62
    assert result["proposal"] == pytest.approx(0.2)
    assert result["brier"] == pytest.approx(
        float(np.mean([(r["p_retry_success"] - (r["next_outcome"] == "done")) ** 2 for r in calibrated_set()]))
    )
    assert len(result["bins"]) == 10
    json.dumps(result)


def test_analyze_insufficient_keeps_current() -> None:
    result = analyze(records([rec(0.9, "done", threshold=0.5)]), make_settings(), 0, 0)
    assert result["proposal"] is None
    assert result["current_threshold"] == 0.5


def test_build_snippet_format() -> None:
    result = analyze(records(calibrated_set()), make_settings(), 0, 0)
    line = build_snippet(result, date(2026, 10, 20))
    assert (
        line
        == f"retry_success_threshold: 0.2  # calibrate_retry_threshold.py 2026-10-20 n=100 Brier {result['brier']:.2f}"
    )


def test_build_snippet_insufficient() -> None:
    result = analyze(records([rec(0.9, "done", threshold=0.5)]), make_settings(), 0, 0)
    assert build_snippet(result, date(2026, 10, 20)) == "insufficient data, keep 0.5"


def test_render_report_mentions_sections_and_censoring() -> None:
    result = analyze(records(calibrated_set()), make_settings(), 2, 0)
    text = render_report(result, date(2026, 10, 20))
    for needle in (
        "Brier",
        "ECE",
        "affordance",
        "retry_success_threshold: 0.2",
        "skipped 2",
        "set retry_success_threshold to 0.0",
    ):
        assert needle in text


def test_main_prints_and_writes_only_json(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    data = tmp_path / "r.jsonl"
    data.write_text("\n".join(json.dumps(r) for r in calibrated_set()) + "\n")
    cfg = tmp_path / "c.yaml"
    cfg.write_text(yaml.safe_dump({"source_file": str(data)}))
    out_json = tmp_path / "out.json"
    before = sorted(p.name for p in tmp_path.iterdir())
    assert main([str(cfg), "--json", str(out_json)]) == 0
    assert "retry_success_threshold: 0.2" in capsys.readouterr().out
    assert json.loads(out_json.read_text())["proposal"] == pytest.approx(0.2)
    assert sorted(p.name for p in tmp_path.iterdir()) == sorted([*before, "out.json"])


def test_main_insufficient_data_message(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    data = tmp_path / "r.jsonl"
    data.write_text(json.dumps(rec(0.9, "done", threshold=0.5)) + "\n")
    cfg = tmp_path / "c.yaml"
    cfg.write_text(yaml.safe_dump({"source_file": str(data)}))
    assert main([str(cfg)]) == 0
    assert "insufficient data, keep 0.5" in capsys.readouterr().out


def test_main_empty_source(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    data = tmp_path / "r.jsonl"
    data.write_text("")
    cfg = tmp_path / "c.yaml"
    cfg.write_text(yaml.safe_dump({"source_file": str(data)}))
    assert main([str(cfg)]) == 0
    assert "insufficient data, keep unknown" in capsys.readouterr().out
