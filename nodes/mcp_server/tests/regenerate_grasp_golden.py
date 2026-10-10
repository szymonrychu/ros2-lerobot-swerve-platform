"""Regenerate tests/data/grasp_golden.json after a planner change that alters the golden plans by design.

Re-plans every golden case (same scenario fields) with the current planner and the deployed client.yml config, the
way tests/test_grasp_speed.py runs them, and writes the plans as the new golden plans. The baseline of each case is
set equal to the new plans (the change is intended, so the old plans are no longer the reference). A case that is no
longer feasible stops the regeneration with its reasons: decide about it before writing anything.

    cd nodes/mcp_server && uv run python -m tests.regenerate_grasp_golden
"""

import json
import sys
from typing import Any

from tests.test_grasp_speed import GOLDEN, run

JSON_SEPARATORS = (",", ":")


def plan_record(case: dict[str, Any]) -> dict[str, Any]:
    """Golden plan fields of one case planned with the current planner.

    Args:
        case (dict[str, Any]): Golden case (scenario fields x, y, support_z, surface_z, size, gap, strategy, pitch).

    Returns:
        dict[str, Any]: chosen, pitch_rad, roll, waypoints (label, joints) and segments (label -> joint samples).

    Raises:
        ValueError: When the case is no longer feasible.
    """
    plan = run(
        case["x"],
        case["y"],
        case["support_z"],
        tuple(case["size"]),
        case["gap"],
        case["strategy"],
        case["pitch"],
        surface_z=case["surface_z"],
    )
    if not plan.feasible:
        raise ValueError(f"{case['key']} is no longer feasible: {plan.reasons}")
    return {
        "chosen": plan.strategy,
        "pitch_rad": plan.approach_pitch_rad,
        "roll": plan.wrist_roll_rad,
        "waypoints": [{"label": w.label, "joints": w.joints} for w in plan.waypoints],
        "segments": plan.segments,
    }


def regenerate(cases: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """New golden cases: scenario fields kept, plan and baseline replaced by the current plans.

    Args:
        cases (list[dict[str, Any]]): Current golden cases.

    Returns:
        list[dict[str, Any]]: Regenerated cases in the same order.
    """
    out: list[dict[str, Any]] = []
    for case in cases:
        record = plan_record(case)
        baseline = {k: record[k] for k in ("pitch_rad", "roll", "waypoints", "segments")}
        out.append(case | record | {"baseline": baseline})
    return out


def main() -> int:
    """Rewrite the golden file.

    Returns:
        int: 0 on success, 1 when a case became infeasible (nothing written).
    """
    cases = json.loads(GOLDEN.read_text())
    try:
        new = regenerate(cases)
    except ValueError as exc:
        print(exc)
        return 1
    GOLDEN.write_text(json.dumps(new, separators=JSON_SEPARATORS))
    print(f"wrote {len(new)} golden plans to {GOLDEN}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
