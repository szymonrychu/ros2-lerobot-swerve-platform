#!/usr/bin/env python3
"""Turn a topic_scraper capture of /ground/state into the ground-noise runbook numbers.

Input is one NDJSON file: records of scripts/topic_scraper_collect.py (`{topic, value, ...}`) or web_ui /ws
capture lines (`{t, topic, data}`); `value` / `data` is the /ground/state JSON as an object or a JSON string.
Prints one JSON summary and exits 0 when every metric of the selected scene passes, 1 otherwise.

Usage:
    python scripts/ground_noise_report.py --config scripts/ground_noise_report.yaml \\
        --capture capture.ndjson --scene box
"""

import argparse
import json
import logging
import sys
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from pydantic import BaseModel

LOG = logging.getLogger("ground_noise_report")

STATE_TOPIC = "/ground/state"
SCENES = ("box", "ramp", "board", "empty")
CLASS_OBSTACLE = "obstacle"
CLASS_WARNING = "warning"
KIND_RAMP = "ramp"
POLYGON_ROUND_DECIMALS = 2  # 1 cm
P50 = 50.0
P90 = 90.0
P95 = 95.0
EXIT_PASS = 0
EXIT_FAIL = 1


class Roi(BaseModel):
    """Axis-aligned rectangle in base_link metres."""

    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def contains(self, point: Sequence[float]) -> bool:
        """Return True when the (x, y) point lies inside the rectangle (edges included)."""
        return self.x_min <= point[0] <= self.x_max and self.y_min <= point[1] <= self.y_max


class BoxScene(BaseModel):
    """Box object: ROI and nominal height."""

    roi: Roi
    height_m: float


class RampScene(BaseModel):
    """Ramp object: ROI and true slope."""

    roi: Roi
    slope_deg: float


class BoardScene(BaseModel):
    """Colour board lying flat: ROI only."""

    roi: Roi


class SceneConfig(BaseModel):
    """The physical scene."""

    box: BoxScene
    ramp: RampScene
    board: BoardScene


class Thresholds(BaseModel):
    """Pass limits; *_min must be reached, *_max must not be exceeded, churn must stay strictly below."""

    box_obstacle_share_min: float
    ramp_warning_share_min: float
    ramp_slope_abs_error_p50_max: float
    board_lethal_states_max: int
    empty_lethal_points_p95_max: float
    warning_churn_share_max: float


class ReportConfig(BaseModel):
    """Contents of ground_noise_report.yaml."""

    scene: SceneConfig
    thresholds: Thresholds


def load_config(path: Path) -> ReportConfig:
    """Load and validate the YAML config.

    Args:
        path: YAML file path.

    Returns:
        Validated ReportConfig.
    """
    return ReportConfig.model_validate(yaml.safe_load(path.read_text()))


def parse_payload(value: Any) -> dict[str, Any] | None:
    """Return the state object from a dict or a JSON string, None when it is neither."""
    if isinstance(value, str):
        try:
            value = json.loads(value)
        except ValueError:
            return None
    return value if isinstance(value, dict) else None


def read_capture(path: Path) -> tuple[list[dict[str, Any]], int, int]:
    """Read /ground/state objects from an NDJSON capture.

    Args:
        path: capture file.

    Returns:
        (states in file order, lines of other topics skipped, malformed lines).
    """
    states: list[dict[str, Any]] = []
    skipped = 0
    malformed = 0
    for raw in path.read_text(errors="replace").splitlines():
        if not raw.strip():
            continue
        try:
            record = json.loads(raw)
        except ValueError:
            malformed += 1
            continue
        if not isinstance(record, dict):
            malformed += 1
            continue
        if record.get("topic") != STATE_TOPIC:
            skipped += 1
            continue
        payload = parse_payload(record["value"] if "value" in record else record.get("data"))
        if payload is None:
            malformed += 1
            continue
        states.append(payload)
    return states, skipped, malformed


def with_plane(states: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Keep the states that carry a plane."""
    return [s for s in states if s.get("plane")]


def edge_point(region: dict[str, Any]) -> Sequence[float]:
    """Edge point (x, y) of a region."""
    return region["edge"]["point"]


def centroid(area: dict[str, Any]) -> tuple[float, float]:
    """Vertex mean of a warning-area polygon."""
    polygon = area["polygon"]
    return (sum(p[0] for p in polygon) / len(polygon), sum(p[1] for p in polygon) / len(polygon))


def region_in(state: dict[str, Any], roi: Roi, cls: str) -> bool:
    """True when a region of class `cls` has its edge point in the ROI."""
    return any(r.get("class") == cls and roi.contains(edge_point(r)) for r in state.get("regions", []))


def area_in(state: dict[str, Any], roi: Roi, kind: str | None = None) -> bool:
    """True when a warning area (of `kind`, any when None) has its centroid in the ROI."""
    return any((kind is None or a.get("kind") == kind) and roi.contains(centroid(a)) for a in state.get("warnings", []))


def share(states: list[dict[str, Any]], predicate: Callable[[dict[str, Any]], bool]) -> float | None:
    """Share of states satisfying predicate, None without states."""
    return sum(1 for s in states if predicate(s)) / len(states) if states else None


def percentile(values: list[float], q: float) -> float | None:
    """Percentile q of values, None when empty."""
    return float(np.percentile(values, q)) if values else None


def box_hit(state: dict[str, Any], roi: Roi) -> bool:
    """Box counts as an obstacle: obstacle region at the box, or obstacle points with no warning there."""
    if region_in(state, roi, CLASS_OBSTACLE):
        return True
    warned = region_in(state, roi, CLASS_WARNING) or area_in(state, roi)
    return state.get("obstacle_points", 0) > 0 and not warned


def ramp_hit(state: dict[str, Any], roi: Roi) -> bool:
    """Ramp counts as a warning: ramp area at the ramp and no obstacle region there."""
    return area_in(state, roi, KIND_RAMP) and not region_in(state, roi, CLASS_OBSTACLE)


def ramp_slope_errors(states: list[dict[str, Any]], roi: Roi, slope_deg: float) -> list[float]:
    """Absolute slope error of every ramp area whose centroid is in the ROI."""
    return [
        abs(a["slope_deg"] - slope_deg)
        for s in states
        for a in s.get("warnings", [])
        if a.get("kind") == KIND_RAMP and roi.contains(centroid(a))
    ]


def warning_set(state: dict[str, Any]) -> frozenset[tuple[Any, ...]]:
    """Warning set of a state: kind plus polygon rounded to 1 cm."""
    return frozenset(
        (
            a["kind"],
            tuple((round(p[0], POLYGON_ROUND_DECIMALS), round(p[1], POLYGON_ROUND_DECIMALS)) for p in a["polygon"]),
        )
        for a in state.get("warnings", [])
    )


def churn_share(states: list[dict[str, Any]]) -> float | None:
    """Share of consecutive state pairs whose warning set differs, None under two states."""
    sets = [warning_set(s) for s in states]
    pairs = list(zip(sets, sets[1:], strict=False))
    return sum(1 for a, b in pairs if a != b) / len(pairs) if pairs else None


def lethal_points(state: dict[str, Any]) -> int:
    """Obstacle plus drop points of a state."""
    return int(state.get("obstacle_points", 0)) + int(state.get("drop_points", 0))


def scene_metrics(scene: str, states: list[dict[str, Any]], config: ReportConfig) -> dict[str, float | int | None]:
    """Compute the metrics of one scene over states that carry a plane.

    Args:
        scene: one of SCENES.
        states: states with a plane.
        config: scene geometry.

    Returns:
        Metric name to value (None where there was no data).
    """
    metrics: dict[str, float | int | None] = {}
    if scene == "box":
        roi = config.scene.box.roi
        metrics["box_obstacle_share"] = share(states, lambda s: box_hit(s, roi))
    elif scene == "ramp":
        ramp = config.scene.ramp
        errors = ramp_slope_errors(states, ramp.roi, ramp.slope_deg)
        metrics["ramp_warning_share"] = share(states, lambda s: ramp_hit(s, ramp.roi))
        metrics["ramp_slope_abs_error_p50"] = percentile(errors, P50)
        metrics["ramp_slope_abs_error_p90"] = percentile(errors, P90)
    elif scene == "board":
        roi = config.scene.board.roi
        metrics["board_lethal_states"] = sum(1 for s in states if region_in(s, roi, CLASS_OBSTACLE)) if states else None
    else:
        metrics["empty_lethal_points_p95"] = percentile([lethal_points(s) for s in states], P95)
    metrics["warning_churn_share"] = churn_share(states)
    return metrics


def check_thresholds(
    scene: str, metrics: dict[str, float | int | None], limits: Thresholds
) -> dict[str, dict[str, Any]]:
    """Judge each gated metric against its limit; missing data fails.

    Args:
        scene: one of SCENES.
        metrics: output of scene_metrics.
        limits: configured thresholds.

    Returns:
        Metric name to {value, limit, op, passed}.
    """
    rules: dict[str, list[tuple[str, str, float, Callable[[float, float], bool]]]] = {
        "box": [("box_obstacle_share", ">=", limits.box_obstacle_share_min, lambda v, lim: v >= lim)],
        "ramp": [
            ("ramp_warning_share", ">=", limits.ramp_warning_share_min, lambda v, lim: v >= lim),
            ("ramp_slope_abs_error_p50", "<=", limits.ramp_slope_abs_error_p50_max, lambda v, lim: v <= lim),
        ],
        "board": [("board_lethal_states", "<=", limits.board_lethal_states_max, lambda v, lim: v <= lim)],
        "empty": [("empty_lethal_points_p95", "<=", limits.empty_lethal_points_p95_max, lambda v, lim: v <= lim)],
    }
    gated = [*rules[scene], ("warning_churn_share", "<", limits.warning_churn_share_max, lambda v, lim: v < lim)]
    out: dict[str, dict[str, Any]] = {}
    for name, op, limit, ok in gated:
        value = metrics.get(name)
        out[name] = {"value": value, "limit": limit, "op": op, "passed": value is not None and ok(value, limit)}
    return out


def build_report(scene: str, capture: Path, config: ReportConfig) -> dict[str, Any]:
    """Read the capture and build the JSON summary.

    Args:
        scene: one of SCENES.
        capture: NDJSON capture path.
        config: loaded config.

    Returns:
        Summary dict with `passed`.
    """
    states, skipped, malformed = read_capture(capture)
    planed = with_plane(states)
    metrics = scene_metrics(scene, planed, config)
    thresholds = check_thresholds(scene, metrics, config.thresholds)
    return {
        "scene": scene,
        "passed": all(t["passed"] for t in thresholds.values()),
        "states": len(states),
        "states_with_plane": len(planed),
        "skipped_lines": skipped,
        "malformed_lines": malformed,
        "metrics": metrics,
        "thresholds": thresholds,
    }


def main(argv: Sequence[str] | None = None) -> int:
    """CLI entry point.

    Args:
        argv: arguments, sys.argv[1:] when None.

    Returns:
        0 when every metric passes, 1 otherwise.
    """
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, required=True, help="YAML config (scene and thresholds)")
    parser.add_argument("--capture", type=Path, required=True, help="NDJSON capture of /ground/state")
    parser.add_argument("--scene", choices=SCENES, required=True)
    args = parser.parse_args(argv)
    report = build_report(args.scene, args.capture, load_config(args.config))
    if report["malformed_lines"]:
        LOG.warning("%d malformed lines ignored", report["malformed_lines"])
    print(json.dumps(report, indent=2))
    return EXIT_PASS if report["passed"] else EXIT_FAIL


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    sys.exit(main())
