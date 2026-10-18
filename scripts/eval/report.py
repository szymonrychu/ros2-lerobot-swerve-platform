"""Evaluation report: TrialRecord (one labelled trial) and the metrics over a run.

build(trials, k) returns a JSON-ready dict; render_text turns it into the plain-text table printed by the driver.
Trial success and stage outcomes come from the human labels; the agent's claims are only compared against them.
"""

from collections import Counter
from typing import Any

from pydantic import BaseModel, Field, field_validator

from scripts.eval.stages import (
    STAGES,
    STATUS_FAIL,
    STATUS_NA,
    STATUS_OK,
    Derivation,
    PhaseStage,
    StageStatus,
)
from scripts.eval.testset import TOTAL_CASES, Case, Mode
from scripts.eval.workflow_checks import WorkflowDerivation

REACHED = (STATUS_OK, STATUS_FAIL)
UNMAPPED = "unmapped"
UNTYPED = "untyped"


class TrialRecord(BaseModel):
    """One labelled trial, one line of trials.jsonl."""

    model_config = {"extra": "forbid"}

    run_id: str
    testset_version: int
    mode: Mode
    case: Case
    trial: int = Field(ge=1)
    instruction_id: str | None
    started_ts: float
    ended_ts: float
    status: str | None = Field(description="turn_end status; None when no turn_end arrived")
    timed_out: bool = Field(default=False, description="The harness hit trial_timeout_s and POSTed /api/stop")
    cost_usd: float = 0.0
    num_turns: int = 0
    effector_calls: int = 0
    claimed: dict[str, StageStatus]
    labels: dict[str, StageStatus]
    success: bool = Field(description="Human label: the object is in the target at the end")
    first_failed_stage: str | None = Field(description="Human label: the first stage that failed")
    phases: list[PhaseStage] = Field(default_factory=list)
    lookup_ids: list[str] = Field(default_factory=list)
    workflow: WorkflowDerivation | None = Field(default=None, description="Workflow adherence; absent in old lines")
    note: str = ""

    @field_validator("claimed", "labels")
    @classmethod
    def known_stages(cls, value: dict[str, str]) -> dict[str, str]:
        """Reject unknown stage names.

        Args:
            value (dict[str, str]): Stage to status.

        Returns:
            dict[str, str]: The value.
        """
        unknown = sorted(set(value) - set(STAGES))
        if unknown:
            raise ValueError(f"unknown stage(s) {unknown}; expected {list(STAGES)}")
        return value

    @field_validator("first_failed_stage")
    @classmethod
    def known_first_failed(cls, value: str | None) -> str | None:
        """Reject an unknown first failed stage.

        Args:
            value (str | None): Stage name or None.

        Returns:
            str | None: The value.
        """
        if value is not None and value not in STAGES:
            raise ValueError(f"unknown stage {value!r}; expected one of {list(STAGES)}")
        return value


def trial_record_from_derivation(
    derivation: Derivation,
    *,
    run_id: str,
    testset_version: int,
    mode: Mode,
    case: Case,
    trial: int,
    started_ts: float,
    ended_ts: float,
    labels: dict[str, StageStatus],
    success: bool,
    first_failed_stage: str | None,
    note: str = "",
    timed_out: bool = False,
    workflow: WorkflowDerivation | None = None,
) -> TrialRecord:
    """Build a TrialRecord from a stage derivation plus the run context and the human labels.

    Args:
        derivation (Derivation): stages.derive result of the trial's events.
        run_id (str): Run id.
        testset_version (int): Test set version.
        mode (Mode): arm_only or full.
        case (Case): The case.
        trial (int): Trial number (1-based).
        started_ts (float): Trial start (epoch s).
        ended_ts (float): Trial end (epoch s).
        labels (dict[str, StageStatus]): Human label per stage.
        success (bool): Human trial success.
        first_failed_stage (str | None): Human first failed stage.
        note (str): Operator note.
        timed_out (bool): The harness stopped the instruction on trial_timeout_s.
        workflow (WorkflowDerivation | None): workflow_checks.derive_workflow result of the trial's events.

    Returns:
        TrialRecord: The record.
    """
    turn_end = derivation.turn_end or {}
    return TrialRecord(
        run_id=run_id,
        testset_version=testset_version,
        mode=mode,
        case=case,
        trial=trial,
        instruction_id=derivation.instruction_id,
        started_ts=started_ts,
        ended_ts=ended_ts,
        status=turn_end.get("status"),
        timed_out=timed_out,
        cost_usd=float(turn_end.get("cost_usd") or 0.0),
        num_turns=int(turn_end.get("num_turns") or 0),
        effector_calls=int(turn_end.get("effector_calls") or 0),
        claimed=derivation.claimed,
        labels=labels,
        success=success,
        first_failed_stage=first_failed_stage,
        phases=derivation.phases,
        lookup_ids=derivation.lookup_ids,
        workflow=workflow,
        note=note,
    )


def ratio(num: float, den: float) -> float | None:
    """Divide, None for an empty denominator.

    Args:
        num (float): Numerator.
        den (float): Denominator.

    Returns:
        float | None: num / den, None when den is 0.
    """
    return num / den if den else None


def mean(values: list[float]) -> float | None:
    """Arithmetic mean, None for no values.

    Args:
        values (list[float]): Values.

    Returns:
        float | None: The mean.
    """
    return ratio(sum(values), len(values))


def by_case(trials: list[TrialRecord]) -> dict[Case, dict[int, TrialRecord]]:
    """Group trials by case and trial number (a later record of the same trial wins).

    Args:
        trials (list[TrialRecord]): Trials.

    Returns:
        dict[Case, dict[int, TrialRecord]]: Case to {trial number: record}.
    """
    grouped: dict[Case, dict[int, TrialRecord]] = {}
    for record in trials:
        grouped.setdefault(record.case, {})[record.trial] = record
    return grouped


def success_at_1(cases: dict[Case, dict[int, TrialRecord]]) -> dict[str, Any]:
    """Share of cases whose trial 1 is a human success.

    Args:
        cases (dict[Case, dict[int, TrialRecord]]): Trials by case.

    Returns:
        dict[str, Any]: {n, successes, rate}.
    """
    firsts = [t[1] for t in cases.values() if 1 in t]
    successes = sum(r.success for r in firsts)
    return {"n": len(firsts), "successes": successes, "rate": ratio(successes, len(firsts))}


def success_at_k(cases: dict[Case, dict[int, TrialRecord]], k: int) -> dict[str, Any]:
    """Share of cases with a human success in trials 1..k (stop_on_success semantics).

    A case with fewer than k trials counts only when one of them was a success; otherwise it is incomplete and left
    out, so stopping after the first success does not lower the rate.

    Args:
        cases (dict[Case, dict[int, TrialRecord]]): Trials by case.
        k (int): Trials per case.

    Returns:
        dict[str, Any]: {k, n, successes, rate, incomplete}.
    """
    n = successes = incomplete = 0
    for trials in cases.values():
        within = [r for no, r in trials.items() if no <= k]
        if any(r.success for r in within):
            n += 1
            successes += 1
        elif len(within) >= k:
            n += 1
        else:
            incomplete += 1
    return {"k": k, "n": n, "successes": successes, "rate": ratio(successes, n), "incomplete": incomplete}


def stage_rates(trials: list[TrialRecord]) -> dict[str, dict[str, Any]]:
    """Per stage (human labels): attempted, success conditional on reaching it, share of first failures.

    Args:
        trials (list[TrialRecord]): Trials.

    Returns:
        dict[str, dict[str, Any]]: Stage to {attempted, succeeded, conditional_success, first_failures,
        first_failure_share}.
    """
    firsts = Counter(r.first_failed_stage for r in trials if not r.success and r.first_failed_stage)
    total_firsts = sum(firsts.values())
    out = {}
    for stage in STAGES:
        attempted = sum(r.labels.get(stage) in REACHED for r in trials)
        succeeded = sum(r.labels.get(stage) == STATUS_OK for r in trials)
        out[stage] = {
            "attempted": attempted,
            "succeeded": succeeded,
            "conditional_success": ratio(succeeded, attempted),
            "first_failures": firsts[stage],
            "first_failure_share": ratio(firsts[stage], total_firsts),
        }
    return out


def agreement(trials: list[TrialRecord]) -> dict[str, dict[str, Any]]:
    """Agent claim vs human label per stage: accuracy and false "ok" rate.

    Trials where either side is missing or the human labelled the stage n/a are not compared. A false ok is a claimed
    ok that the human did not label ok.

    Args:
        trials (list[TrialRecord]): Trials.

    Returns:
        dict[str, dict[str, Any]]: Stage to {n, agree, accuracy, claimed_ok, false_ok, false_ok_rate}.
    """
    out = {}
    for stage in STAGES:
        pairs = [
            (r.claimed[stage], r.labels[stage])
            for r in trials
            if stage in r.claimed and stage in r.labels and r.labels[stage] != STATUS_NA
        ]
        agree = sum(c == h for c, h in pairs)
        claimed_ok = [h for c, h in pairs if c == STATUS_OK]
        false_ok = sum(h != STATUS_OK for h in claimed_ok)
        out[stage] = {
            "n": len(pairs),
            "agree": agree,
            "accuracy": ratio(agree, len(pairs)),
            "claimed_ok": len(claimed_ok),
            "false_ok": false_ok,
            "false_ok_rate": ratio(false_ok, len(claimed_ok)),
        }
    return out


def attempts(trials: list[TrialRecord]) -> dict[str, Any]:
    """Mean attempts per phase, overall and per mapped stage (phases without a stage under "unmapped").

    Args:
        trials (list[TrialRecord]): Trials.

    Returns:
        dict[str, Any]: {phases, mean_per_phase, per_stage: {stage: {phases, mean}}}.
    """
    phases = [p for r in trials for p in r.phases]
    per_stage: dict[str, list[float]] = {}
    for phase in phases:
        per_stage.setdefault(phase.stage or UNMAPPED, []).append(phase.attempts)
    order = [*STAGES, UNMAPPED]
    return {
        "phases": len(phases),
        "mean_per_phase": mean([p.attempts for p in phases]),
        "per_stage": {s: {"phases": len(per_stage[s]), "mean": mean(per_stage[s])} for s in order if s in per_stage},
    }


def failure_histograms(trials: list[TrialRecord]) -> tuple[dict[str, int], dict[str, int], int]:
    """Histogram of failure_type and eai_error_type over every failed phase close.

    Args:
        trials (list[TrialRecord]): Trials.

    Returns:
        tuple[dict[str, int], dict[str, int], int]: failure_type counts, eai_error_type counts, failed closes
        without a failure record.
    """
    records = [f for r in trials for p in r.phases for f in p.failures]
    typed = [f for f in records if f is not None]
    types = Counter(str(f.get("type", UNTYPED)) for f in typed)
    eai = Counter(str(f.get("eai_error_type", UNTYPED)) for f in typed)
    return dict(sorted(types.items())), dict(sorted(eai.items())), len(records) - len(typed)


def cost_time(trials: list[TrialRecord]) -> dict[str, Any]:
    """Cost, duration and turns per trial.

    Args:
        trials (list[TrialRecord]): Trials.

    Returns:
        dict[str, Any]: {cost_usd_total, cost_usd_mean, duration_s_mean, duration_s_max, num_turns_mean}.
    """
    durations = [r.ended_ts - r.started_ts for r in trials]
    return {
        "cost_usd_total": sum(r.cost_usd for r in trials),
        "cost_usd_mean": mean([r.cost_usd for r in trials]),
        "duration_s_mean": mean(durations),
        "duration_s_max": max(durations) if durations else None,
        "num_turns_mean": mean([float(r.num_turns) for r in trials]),
    }


def workflow_section(trials: list[TrialRecord]) -> dict[str, Any]:
    """Workflow adherence over the trials that carry a workflow derivation.

    Args:
        trials (list[TrialRecord]): Trials.

    Returns:
        dict[str, Any]: trials (with a derivation), per_workflow (trial count by workflow, "none" for free jobs),
        no_skipped_mandatory_share (of trials with a workflow report), agent_retrospective_share and
        mean_survey_poi_writes (of all trials with a derivation); shares and means are None without trials.
    """
    derived = [r.workflow for r in trials if r.workflow is not None]
    reported = [d for d in derived if d.skipped_mandatory is not None]
    return {
        "trials": len(derived),
        "per_workflow": dict(sorted(Counter(d.workflow or "none" for d in derived).items())),
        "no_skipped_mandatory_share": ratio(sum(not d.skipped_mandatory for d in reported), len(reported)),
        "agent_retrospective_share": ratio(sum(d.retrospective == "agent" for d in derived), len(derived)),
        "mean_survey_poi_writes": mean([float(d.survey_poi_writes) for d in derived]),
    }


def build(trials: list[TrialRecord], k: int, cases_planned: int | None = None) -> dict[str, Any]:
    """Compute the evaluation report.

    Args:
        trials (list[TrialRecord]): Labelled trials of the run.
        k (int): Trials per case for success@k.
        cases_planned (int | None): Cases the run was set up for (20 in arm_only), shown in the coverage header.

    Returns:
        dict[str, Any]: JSON-ready report: coverage, success_at_1, success_at_k, stages, attempts,
        mean_auto_retries_per_trial, agreement, failure_types, eai_error_types, untyped_failures, cost_time,
        statuses, timeouts, workflow.

    Raises:
        ValueError: When k < 1.
    """
    if k < 1:
        raise ValueError(f"k must be >= 1, got {k}")
    cases = by_case(trials)
    failure_types, eai_types, untyped = failure_histograms(trials)
    return {
        "coverage": {
            "cases_total": TOTAL_CASES,
            "cases_run": len(cases),
            "cases_planned": cases_planned,
            "trials": len(trials),
            "modes": sorted({r.mode for r in trials}),
            "testset_versions": sorted({r.testset_version for r in trials}),
        },
        "success_at_1": success_at_1(cases),
        "success_at_k": success_at_k(cases, k),
        "stages": stage_rates(trials),
        "failed_trials": sum(not r.success for r in trials),
        "attempts": attempts(trials),
        "mean_auto_retries_per_trial": mean([float(sum(p.attempts - 1 for p in r.phases)) for r in trials]),
        "agreement": agreement(trials),
        "failure_types": failure_types,
        "eai_error_types": eai_types,
        "untyped_failures": untyped,
        "cost_time": cost_time(trials),
        "statuses": dict(sorted(Counter(str(r.status) for r in trials).items())),
        "timeouts": sum(r.timed_out for r in trials),
        "workflow": workflow_section(trials),
    }


def fmt(value: float | None, pattern: str = "{:.2f}") -> str:
    """Format a number, "-" for None.

    Args:
        value (float | None): Value.
        pattern (str): Format pattern.

    Returns:
        str: Formatted value.
    """
    return "-" if value is None else pattern.format(value)


def pad_row(cells: list[str], widths: list[int]) -> str:
    """Join cells padded to the column widths.

    Args:
        cells (list[str]): Cells.
        widths (list[int]): Column widths.

    Returns:
        str: The line without trailing spaces.
    """
    return "  ".join(str(c).ljust(w) for c, w in zip(cells, widths, strict=False)).rstrip()


def table(header: list[str], rows: list[list[str]]) -> list[str]:
    """Render a left-aligned plain-text table.

    Args:
        header (list[str]): Column titles.
        rows (list[list[str]]): Cells.

    Returns:
        list[str]: Lines.
    """
    widths = [max(len(str(c)) for c in col) for col in zip(header, *rows, strict=False)]
    return [pad_row(header, widths), pad_row(["-" * w for w in widths], widths), *[pad_row(r, widths) for r in rows]]


def histogram_line(title: str, counts: dict[str, int]) -> str:
    """One histogram line.

    Args:
        title (str): Label.
        counts (dict[str, int]): Value counts.

    Returns:
        str: ``title: a=1, b=2`` or ``title: none``.
    """
    return f"{title}: " + (", ".join(f"{k}={v}" for k, v in counts.items()) or "none")


def render_text(report: dict[str, Any]) -> str:
    """Render a build() report as plain text; the first line states the coverage.

    Args:
        report (dict[str, Any]): build() result.

    Returns:
        str: The report text.
    """
    cov = report["coverage"]
    planned = f", planned {cov['cases_planned']}" if cov["cases_planned"] is not None else ""
    modes = ", ".join(cov["modes"]) or "-"
    versions = ", ".join(str(v) for v in cov["testset_versions"]) or "-"
    s1, sk = report["success_at_1"], report["success_at_k"]
    lines = [
        f"Coverage: {cov['cases_run']} of {cov['cases_total']} cases run{planned} "
        f"(mode {modes}, {cov['trials']} trials, test set version {versions})",
        f"success@1: {fmt(s1['rate'], '{:.0%}')} ({s1['successes']}/{s1['n']})",
        f"success@{sk['k']} (k={sk['k']}): {fmt(sk['rate'], '{:.0%}')} ({sk['successes']}/{sk['n']}, "
        f"{sk['incomplete']} incomplete case(s) left out)",
        "",
        "Stages (human labels)",
    ]
    lines += table(
        ["Stage", "attempted", "succeeded", "cond. success", "first failure", "share"],
        [
            [
                stage,
                str(v["attempted"]),
                str(v["succeeded"]),
                fmt(v["conditional_success"], "{:.0%}"),
                str(v["first_failures"]),
                fmt(v["first_failure_share"], "{:.0%}"),
            ]
            for stage, v in report["stages"].items()
        ],
    )
    lines += ["", "Agreement (agent claim vs human)"]
    lines += table(
        ["Stage", "n", "accuracy", "claimed ok", "false ok", "false ok rate"],
        [
            [
                stage,
                str(v["n"]),
                fmt(v["accuracy"], "{:.0%}"),
                str(v["claimed_ok"]),
                str(v["false_ok"]),
                fmt(v["false_ok_rate"], "{:.0%}"),
            ]
            for stage, v in report["agreement"].items()
        ],
    )
    att = report["attempts"]
    lines += [
        "",
        f"Mean attempts per phase: {fmt(att['mean_per_phase'])} over {att['phases']} phase(s)",
        *[f"  {stage}: {fmt(v['mean'])} ({v['phases']} phase(s))" for stage, v in att["per_stage"].items()],
        f"Mean automatic retries per trial: {fmt(report['mean_auto_retries_per_trial'])}",
        "",
        histogram_line("failure_type", report["failure_types"]),
        histogram_line("eai_error_type", report["eai_error_types"]),
        f"untyped failed closes: {report['untyped_failures']}",
        histogram_line("turn_end status", report["statuses"]),
        f"harness timeouts: {report['timeouts']}",
    ]
    wf = report["workflow"]
    lines += [
        "",
        "Workflows",
        histogram_line("trials per workflow", wf["per_workflow"]),
        f"no skipped mandatory step: {fmt(wf['no_skipped_mandatory_share'], '{:.0%}')}; "
        f"agent retrospective: {fmt(wf['agent_retrospective_share'], '{:.0%}')}; "
        f"mean survey POI writes: {fmt(wf['mean_survey_poi_writes'])} ({wf['trials']} trial(s))",
    ]
    ct = report["cost_time"]
    lines += [
        "",
        f"Cost per trial: mean ${fmt(ct['cost_usd_mean'], '{:.3f}')}, total ${ct['cost_usd_total']:.3f}; "
        f"time per trial: mean {fmt(ct['duration_s_mean'], '{:.0f}')} s, max {fmt(ct['duration_s_max'], '{:.0f}')} s; "
        f"turns mean {fmt(ct['num_turns_mean'], '{:.1f}')}",
    ]
    return "\n".join(lines)
