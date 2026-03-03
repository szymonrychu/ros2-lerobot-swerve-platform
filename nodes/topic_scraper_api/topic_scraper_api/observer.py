"""Observation rules runner: compare topics and detect oscillations from scraper samples."""

import time
from collections import deque
from dataclasses import dataclass
from typing import Any

from .config import RULE_TYPE_OSCILLATION, ObservationRule
from .paths import normalize_topic


def _joint_state_positions_by_name(payload: dict[str, Any]) -> dict[str, float] | None:
    """Extract joint name -> position from a scraper payload (JointState message)."""
    if not payload or "message" not in payload:
        return None
    msg = payload["message"]
    if not isinstance(msg, dict):
        return None
    names = msg.get("name")
    positions = msg.get("position")
    if not isinstance(names, list) or not isinstance(positions, list) or len(names) != len(positions):
        return None
    return dict(zip(names, [float(p) for p in positions]))


def _compare_positions(
    payload_a: dict[str, Any],
    payload_b: dict[str, Any],
    joint_names: list[str],
) -> dict[str, float] | None:
    """Compute per-joint position delta (a - b). Returns None if data missing."""
    pos_a = _joint_state_positions_by_name(payload_a)
    pos_b = _joint_state_positions_by_name(payload_b)
    if pos_a is None or pos_b is None:
        return None
    joints = joint_names if joint_names else sorted(set(pos_a.keys()) & set(pos_b.keys()))
    if not joints:
        return None
    out: dict[str, float] = {}
    for j in joints:
        if j in pos_a and j in pos_b:
            out[j] = pos_a[j] - pos_b[j]
    return out if out else None


@dataclass
class RuleResult:
    """Last evaluation result for one rule."""

    rule_name: str
    comparison: dict[str, Any] | None = None
    oscillation_detected: bool = False
    oscillation_score: float = 0.0
    last_updated_ns: int = 0


class RulesObserver:
    """Evaluates observation rules from scraper payloads; holds sliding windows for oscillation."""

    def __init__(self, rules: list[ObservationRule]) -> None:
        self._rules = {r.name: r for r in rules}
        self._results: dict[str, RuleResult] = {name: RuleResult(rule_name=name) for name in self._rules}
        # Per-rule sliding window: list of (time_ns, delta_dict)
        self._windows: dict[str, deque[tuple[int, dict[str, float]]]] = {
            name: deque(maxlen=500) for name in self._rules
        }

    def get_rules_summary(self) -> list[dict[str, Any]]:
        """Return list of rule names with last result summary."""
        out = []
        for name in sorted(self._results.keys()):
            r = self._results[name]
            out.append(
                {
                    "name": name,
                    "has_result": r.last_updated_ns > 0,
                    "oscillation_detected": r.oscillation_detected,
                    "last_updated_ns": r.last_updated_ns,
                }
            )
        return out

    def get_rule_result(self, rule_name: str) -> RuleResult | None:
        """Return full result for a rule."""
        return self._results.get(rule_name)

    def tick(self, get_topic_payload: Any) -> None:
        """Evaluate all rules using get_topic_payload(topic) -> payload or None."""
        now_ns = time.time_ns()
        for name, rule in self._rules.items():
            payloads = []
            for topic in rule.topics:
                norm = normalize_topic(topic)
                payload = get_topic_payload(norm) if norm else None
                payloads.append(payload)

            comparison: dict[str, float] | None = None
            if len(payloads) >= 2 and payloads[0] and payloads[1]:
                comparison = _compare_positions(
                    payloads[0],
                    payloads[1],
                    rule.joint_names,
                )

            oscillation_detected = False
            oscillation_score = 0.0
            if rule.rule_type == RULE_TYPE_OSCILLATION and comparison is not None:
                window_s = rule.window_s
                window_ns = int(window_s * 1e9)
                q = self._windows[name]
                q.append((now_ns, comparison))
                while q and (now_ns - q[0][0]) > window_ns:
                    q.popleft()
                if len(q) >= 3:
                    deltas_list = [d for _, d in q]
                    # Variance of first joint's delta over time (or mean variance across joints)
                    joint_names = list(deltas_list[0].keys()) if deltas_list[0] else []
                    if joint_names:
                        series = [d.get(joint_names[0], 0.0) for d in deltas_list]
                        n = len(series)
                        mean = sum(series) / n
                        variance = sum((x - mean) ** 2 for x in series) / n
                        oscillation_score = variance
                        if variance >= rule.variance_threshold:
                            oscillation_detected = True
                        # Sign changes per second
                        sign_changes = 0
                        for i in range(1, len(series)):
                            if (series[i] - series[i - 1]) != 0 and ((series[i] - mean) * (series[i - 1] - mean) <= 0):
                                sign_changes += 1
                        elapsed_s = (q[-1][0] - q[0][0]) / 1e9
                        if elapsed_s > 0 and (sign_changes / elapsed_s) >= rule.sign_change_min_hz:
                            oscillation_detected = True

            self._results[name] = RuleResult(
                rule_name=name,
                comparison=comparison,
                oscillation_detected=oscillation_detected,
                oscillation_score=oscillation_score,
                last_updated_ns=now_ns,
            )
