"""Calibrate retry_success_threshold from the claude_agent retry ledger.

The agent rates each failed phase with p_retry_success; the ledger (nodes/claude_agent, GET /api/retry_outcomes) pairs
that rating with the next outcome of the same phase. This script reads those records (HTTP or a JSONL file), reports
how well the ratings are calibrated (Brier score against the base-rate reference, reliability bins with Wilson
intervals, ECE, per-eai_error_type success rates) and proposes the smallest threshold on a 0.05 grid whose observed
p >= t subset has enough samples and a Wilson lower bound at or above target_success_rate.

Reads scripts/calibrate_retry_threshold.yaml (pydantic-validated). It prints a report and a paste-ready YAML line for
ansible/group_vars/client.yml; it never writes any repo file (the only file output is the optional --json path).

Usage: python scripts/calibrate_retry_threshold.py [config.yaml] [--json <path>]
"""

import argparse
import json
import sys
from datetime import date
from pathlib import Path
from typing import Any, Literal

import httpx
import numpy as np
import yaml
from pydantic import BaseModel, Field, ValidationError, model_validator
from scipy.stats import binomtest

DEFAULT_CONFIG = Path(__file__).resolve().with_suffix(".yaml")
FETCH_TIMEOUT_S = 10
SCHEMA_VERSION = 1
GRID_STEP = 0.05
GRID_POINTS = 19
OBSERVED_SUCCESS = "done"
UNOBSERVED = "unobserved"
SCRIPT_NAME = "calibrate_retry_threshold.py"
UNKNOWN_THRESHOLD = "unknown"
CENSORING_NOTE = (
    "Censoring: thresholds below the minimum observed p ({min_p:.2f}) are never proposed. Records below the current "
    "threshold are only observed through loop re-entries; to explore lower p, set retry_success_threshold to 0.0 for "
    "an evaluation session (scripts/README.md)."
)


class Settings(BaseModel):
    """Validated contents of calibrate_retry_threshold.yaml."""

    model_config = {"extra": "forbid"}

    # Exactly one of source_url (GET returning {records: [...]}) and source_file (JSONL) is set.
    source_url: str | None = None
    source_file: str | None = None
    since_ts: float | None = None
    model: str | None = None
    bins: int = Field(default=10, ge=2)
    target_success_rate: float = Field(default=0.5, gt=0, lt=1)
    confidence: float = Field(default=0.95, gt=0, lt=1)
    min_samples: int = Field(default=30, ge=1)
    min_bin_samples: int = Field(default=5, ge=1)

    @model_validator(mode="after")
    def exactly_one_source(self) -> "Settings":
        """Require exactly one of source_url and source_file.

        Returns:
            Settings: The validated settings.
        """
        if (self.source_url is None) == (self.source_file is None):
            raise ValueError("set exactly one of source_url and source_file")
        return self


class RetryRecord(BaseModel):
    """One ledger record, mirrors claude_agent retry_ledger.RetryRecord (schema_version 1)."""

    schema_version: Literal[1] = 1
    ts: float
    instruction_id: str
    phase_id: str
    phase_name: str
    attempt: int
    iteration: int
    p_retry_success: float = Field(ge=0, le=1)
    threshold: float
    retried: bool
    refusal: str | None = None
    failure_type: str
    eai_error_type: str
    next_outcome: Literal["done", "failed", "skipped", "unobserved"]
    next_via: Literal["auto_retry", "loop", "none"]
    model: str


def load_settings(path: Path) -> Settings:
    """Load and validate the config file.

    Args:
        path (Path): YAML config file.

    Returns:
        Settings: Validated settings.
    """
    return Settings(**(yaml.safe_load(path.read_text()) or {}))


def fetch_records(settings: Settings) -> list[dict[str, Any]]:
    """Read raw record dicts from the HTTP endpoint or the JSONL file (corrupt lines skipped).

    Args:
        settings (Settings): Source configuration.

    Returns:
        list[dict[str, Any]]: Raw records, oldest first.
    """
    if settings.source_url is not None:
        with httpx.Client(timeout=FETCH_TIMEOUT_S) as client:
            response = client.get(settings.source_url)
            response.raise_for_status()
            return list(response.json().get("records", []))
    out: list[dict[str, Any]] = []
    assert settings.source_file is not None
    for line in Path(settings.source_file).read_text().splitlines():
        if not line.strip():
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            out.append(value)
    return out


def parse_records(raw: list[dict[str, Any]]) -> tuple[list[RetryRecord], int, int]:
    """Validate raw records; unknown schema versions and invalid records are skipped and counted.

    Args:
        raw (list[dict[str, Any]]): Raw record dicts.

    Returns:
        tuple[list[RetryRecord], int, int]: Valid records, count with an unknown schema_version, count invalid.
    """
    records: list[RetryRecord] = []
    unknown = invalid = 0
    for item in raw:
        if item.get("schema_version", SCHEMA_VERSION) != SCHEMA_VERSION:
            unknown += 1
            continue
        try:
            records.append(RetryRecord(**item))
        except ValidationError:
            invalid += 1
    return records, unknown, invalid


def filter_records(records: list[RetryRecord], since_ts: float | None, model: str | None) -> list[RetryRecord]:
    """Keep records at or after since_ts and (when given) of one model.

    Args:
        records (list[RetryRecord]): Records.
        since_ts (float | None): Minimum ts, or None.
        model (str | None): Model name to keep, or None for all.

    Returns:
        list[RetryRecord]: Filtered records.
    """
    return [r for r in records if (since_ts is None or r.ts >= since_ts) and (model is None or r.model == model)]


def observed_records(records: list[RetryRecord]) -> list[RetryRecord]:
    """Records whose next outcome was observed (anything but unobserved).

    Args:
        records (list[RetryRecord]): Records.

    Returns:
        list[RetryRecord]: Observed records.
    """
    return [r for r in records if r.next_outcome != UNOBSERVED]


def bin_index(p: float, bins: int) -> int:
    """Bin of a probability; p = 1.0 falls into the last bin.

    Args:
        p (float): Probability in [0, 1].
        bins (int): Number of equal-width bins.

    Returns:
        int: Bin index 0..bins-1.
    """
    return min(int(p * bins), bins - 1)


def count_bins(records: list[RetryRecord], bins: int) -> list[dict[str, int]]:
    """Total, observed and unobserved record counts per bin.

    Args:
        records (list[RetryRecord]): Records.
        bins (int): Number of bins.

    Returns:
        list[dict[str, int]]: One {total, observed, unobserved} dict per bin.
    """
    counts = [{"total": 0, "observed": 0, "unobserved": 0} for _ in range(bins)]
    for r in records:
        entry = counts[bin_index(r.p_retry_success, bins)]
        entry["total"] += 1
        entry["unobserved" if r.next_outcome == UNOBSERVED else "observed"] += 1
    return counts


def outcomes(records: list[RetryRecord]) -> tuple[np.ndarray, np.ndarray]:
    """Predictions and 0/1 outcomes of the observed records (success = next outcome done).

    Args:
        records (list[RetryRecord]): Records.

    Returns:
        tuple[np.ndarray, np.ndarray]: p array and y array of the observed records.
    """
    obs = observed_records(records)
    p = np.array([r.p_retry_success for r in obs], dtype=float)
    y = np.array([r.next_outcome == OBSERVED_SUCCESS for r in obs], dtype=float)
    return p, y


def brier_score(records: list[RetryRecord]) -> tuple[float | None, float | None]:
    """Brier score of the observed records and the Brier score of always predicting the base rate.

    Args:
        records (list[RetryRecord]): Records (unobserved ones are ignored).

    Returns:
        tuple[float | None, float | None]: (Brier, base-rate reference Brier), None without observed records.
    """
    p, y = outcomes(records)
    if p.size == 0:
        return None, None
    return float(np.mean((p - y) ** 2)), float(np.mean((y.mean() - y) ** 2))


def wilson_interval(successes: int, n: int, confidence: float) -> tuple[float, float]:
    """Wilson score interval of a binomial proportion.

    Args:
        successes (int): Number of successes.
        n (int): Number of trials, > 0.
        confidence (float): Confidence level.

    Returns:
        tuple[float, float]: (low, high).
    """
    ci = binomtest(successes, n).proportion_ci(confidence_level=confidence, method="wilson")
    return float(ci.low), float(ci.high)


def reliability_bins(
    records: list[RetryRecord], bins: int, confidence: float, min_bin_samples: int
) -> list[dict[str, Any]]:
    """Reliability table over the observed records.

    Args:
        records (list[RetryRecord]): Records (unobserved ones are ignored).
        bins (int): Number of bins.
        confidence (float): Wilson confidence level.
        min_bin_samples (int): Bins with fewer observed records are flagged low_n.

    Returns:
        list[dict[str, Any]]: Per bin lo, hi, n, mean_p, success_rate, ci_low, ci_high, low_n (None values when n=0).
    """
    p, y = outcomes(records)
    idx = np.array([bin_index(v, bins) for v in p], dtype=int)
    table: list[dict[str, Any]] = []
    for b in range(bins):
        mask = idx == b
        n = int(mask.sum())
        row: dict[str, Any] = {
            "lo": b / bins,
            "hi": (b + 1) / bins,
            "n": n,
            "mean_p": None,
            "success_rate": None,
            "ci_low": None,
            "ci_high": None,
            "low_n": n < min_bin_samples,
        }
        if n:
            wins = int(y[mask].sum())
            row["mean_p"] = float(p[mask].mean())
            row["success_rate"] = wins / n
            row["ci_low"], row["ci_high"] = wilson_interval(wins, n, confidence)
        table.append(row)
    return table


def ece(table: list[dict[str, Any]]) -> float | None:
    """Expected calibration error: sample-weighted mean |mean p - success rate| over the bins.

    Args:
        table (list[dict[str, Any]]): Output of reliability_bins.

    Returns:
        float | None: ECE, None without observed records.
    """
    total = sum(row["n"] for row in table)
    if not total:
        return None
    return sum(row["n"] * abs(row["mean_p"] - row["success_rate"]) for row in table if row["n"]) / total


def eai_breakdown(records: list[RetryRecord]) -> dict[str, dict[str, float | int]]:
    """Observed count and success rate per eai_error_type.

    Args:
        records (list[RetryRecord]): Records (unobserved ones are ignored).

    Returns:
        dict[str, dict[str, float | int]]: eai_error_type -> {n, success_rate}.
    """
    groups: dict[str, list[bool]] = {}
    for r in observed_records(records):
        groups.setdefault(r.eai_error_type, []).append(r.next_outcome == OBSERVED_SUCCESS)
    return {k: {"n": len(v), "success_rate": sum(v) / len(v)} for k, v in sorted(groups.items())}


def propose_threshold(records: list[RetryRecord], target: float, confidence: float, min_samples: int) -> float | None:
    """Smallest grid threshold (0.05 step) whose observed p >= t subset is large and reliably successful enough.

    A threshold qualifies when the subset has n >= min_samples and a Wilson lower bound >= target. Thresholds below
    the minimum observed p are never proposed (no data there). At least min_samples observed records are required.

    Args:
        records (list[RetryRecord]): Records (unobserved ones are ignored).
        target (float): Required success rate lower bound.
        confidence (float): Wilson confidence level.
        min_samples (int): Minimum observed records overall and in the subset.

    Returns:
        float | None: The proposed threshold, None for insufficient data.
    """
    p, y = outcomes(records)
    if p.size < min_samples:
        return None
    min_p = float(p.min())
    for i in range(1, GRID_POINTS + 1):
        t = round(i * GRID_STEP, 2)
        if t < min_p - 1e-9:
            continue
        mask = p >= t - 1e-9
        n = int(mask.sum())
        if n < min_samples:
            break
        if wilson_interval(int(y[mask].sum()), n, confidence)[0] >= target:
            return t
    return None


def analyze(
    records: list[RetryRecord], settings: Settings, skipped_unknown: int, skipped_invalid: int
) -> dict[str, Any]:
    """Compute every statistic of the report.

    Args:
        records (list[RetryRecord]): Filtered records.
        settings (Settings): Thresholds and bin settings.
        skipped_unknown (int): Records skipped for an unknown schema_version.
        skipped_invalid (int): Records skipped as invalid.

    Returns:
        dict[str, Any]: JSON-serializable result.
    """
    brier, base_brier = brier_score(records)
    table = reliability_bins(records, settings.bins, settings.confidence, settings.min_bin_samples)
    observed = observed_records(records)
    latest = max(records, key=lambda r: r.ts) if records else None
    p, _ = outcomes(records)
    return {
        "counts": {
            "total": len(records),
            "observed": len(observed),
            "unobserved": len(records) - len(observed),
            "per_bin": count_bins(records, settings.bins),
        },
        "skipped_unknown_schema": skipped_unknown,
        "skipped_invalid": skipped_invalid,
        "brier": brier,
        "base_rate_brier": base_brier,
        "ece": ece(table),
        "bins": table,
        "by_eai_error_type": eai_breakdown(records),
        "min_observed_p": float(p.min()) if p.size else None,
        "current_threshold": latest.threshold if latest else None,
        "proposal": propose_threshold(records, settings.target_success_rate, settings.confidence, settings.min_samples),
        "target_success_rate": settings.target_success_rate,
        "confidence": settings.confidence,
        "min_samples": settings.min_samples,
    }


def fmt(value: float | None, digits: int = 2) -> str:
    """Format an optional float.

    Args:
        value (float | None): Value.
        digits (int): Decimal places.

    Returns:
        str: Formatted value or "-".
    """
    return "-" if value is None else f"{value:.{digits}f}"


def build_snippet(result: dict[str, Any], today: date) -> str:
    """Paste-ready group_vars line with provenance, or the insufficient-data message.

    Args:
        result (dict[str, Any]): Output of analyze.
        today (date): Date for the provenance comment.

    Returns:
        str: The line to print.
    """
    if result["proposal"] is None:
        current = result["current_threshold"]
        return f"insufficient data, keep {UNKNOWN_THRESHOLD if current is None else f'{current:g}'}"
    return (
        f"retry_success_threshold: {result['proposal']:g}  # {SCRIPT_NAME} {today.isoformat()} "
        f"n={result['counts']['observed']} Brier {result['brier']:.2f}"
    )


def render_report(result: dict[str, Any], today: date) -> str:
    """Human-readable report ending in the snippet.

    Args:
        result (dict[str, Any]): Output of analyze.
        today (date): Date for the provenance comment.

    Returns:
        str: Report text.
    """
    counts = result["counts"]
    lines = [
        f"records: total {counts['total']}, observed {counts['observed']}, unobserved {counts['unobserved']}"
        f" (skipped {result['skipped_unknown_schema']} unknown schema_version, {result['skipped_invalid']} invalid)",
        f"Brier {fmt(result['brier'], 3)} (base-rate reference {fmt(result['base_rate_brier'], 3)}), "
        f"ECE {fmt(result['ece'], 3)}",
        "",
        "bin         total obs unobs  n  mean_p  success  wilson CI",
    ]
    for row, cnt in zip(result["bins"], counts["per_bin"], strict=True):
        ci = "-" if row["ci_low"] is None else f"[{row['ci_low']:.2f}, {row['ci_high']:.2f}]"
        flag = "  (low n)" if row["n"] and row["low_n"] else ""
        lines.append(
            f"{row['lo']:.2f}-{row['hi']:.2f}  {cnt['total']:5d} {cnt['observed']:3d} {cnt['unobserved']:5d} "
            f"{row['n']:3d}  {fmt(row['mean_p'])}    {fmt(row['success_rate'])}    {ci}{flag}"
        )
    lines += ["", "by eai_error_type (observed):"]
    lines += [f"  {k}: n={v['n']} success {v['success_rate']:.2f}" for k, v in result["by_eai_error_type"].items()]
    if result["min_observed_p"] is not None:
        lines += ["", CENSORING_NOTE.format(min_p=result["min_observed_p"])]
    lines += [
        "",
        f"target: Wilson lower bound ({result['confidence']:.0%}) >= {result['target_success_rate']}, "
        f"n >= {result['min_samples']}",
        build_snippet(result, today),
    ]
    return "\n".join(lines)


def main(argv: list[str]) -> int:
    """Read the records, print the calibration report and optionally write the JSON result.

    Args:
        argv (list[str]): Config file path (default calibrate_retry_threshold.yaml) and optional --json <path>.

    Returns:
        int: Process exit code.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("config", nargs="?", type=Path, default=DEFAULT_CONFIG, help="YAML config file")
    parser.add_argument("--json", type=Path, default=None, help="write the machine-readable result here")
    args = parser.parse_args(argv)
    settings = load_settings(args.config)
    records, unknown, invalid = parse_records(fetch_records(settings))
    records = filter_records(records, settings.since_ts, settings.model)
    result = analyze(records, settings, unknown, invalid)
    print(render_report(result, date.today()))
    if args.json is not None:
        args.json.write_text(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
