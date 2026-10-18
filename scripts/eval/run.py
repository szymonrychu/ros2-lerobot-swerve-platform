"""Evaluation driver: runs the fixed test set against the claude_agent trial by trial with a human labelling step.

Usage: python -m scripts.eval.run <run.yaml> [--resume <run_id>]

Per trial: setup prompt (Enter, s skip, q quit), idle check, optional reset, POST the instruction (deny_tools only in
arm_only mode), poll the history to the trial's turn_end (on timeout POST stop and wait 60 s more), save the raw events,
print the claimed stages and lookup ids, ask the labels and append one TrialRecord line to
<output_dir>/<run_id>/trials.jsonl. The report is printed and written to report.json at the end, on q and on abort.

A workflow test set (``kind: workflows``, scripts/eval/testset_workflows_v1.yaml) runs in mode full only: per case the
setup prompt, the case prompt, the derived workflow against expected_workflow, the human success label, and one
WorkflowTrialRecord line in trials.jsonl.
"""

import argparse
import json
import sys
import time
from collections.abc import Callable
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml
from pydantic import BaseModel, Field

from scripts.eval import report as report_module
from scripts.eval.client import AgentClient, AgentError
from scripts.eval.labels import SUCCESS_KEYS, ask, ask_key
from scripts.eval.report import TrialRecord, trial_record_from_derivation
from scripts.eval.stages import derive
from scripts.eval.testset import (
    MODE_ARM_ONLY,
    MODE_FULL,
    Case,
    Mode,
    TestSet,
    WorkflowCase,
    WorkflowTestSet,
    load_any_testset,
)
from scripts.eval.workflow_checks import WorkflowDerivation, derive_workflow

STOP_GRACE_S = 60.0
SETUP_KEYS = {"s": "skip this trial", "q": "quit and report"}
TRIALS_FILE = "trials.jsonl"
REPORT_FILE = "report.json"
EVENTS_DIR = "events"
RUN_ID_FORMAT = "%Y%m%d-%H%M%S"
EXIT_OK = 0
EXIT_ABORTED = 1
EXIT_CONFIG = 2
DEFAULT_CONFIG_HELP = "run config YAML (scripts/eval/run.yaml)"


class RunConfig(BaseModel):
    """Validated contents of run.yaml."""

    model_config = {"extra": "forbid"}

    base_url: str
    testset: str = "scripts/eval/testset_v1.yaml"
    mode: Mode = "arm_only"
    arm_only_deny_tools: list[str] = Field(
        default_factory=lambda: ["navigate_to_pose", "move_relative", "drive", "look_around", "take_control:base"]
    )
    k: int = Field(default=3, ge=1)
    stop_on_success: bool = True
    reset_between_trials: bool = True
    poll_interval_s: float = Field(default=2.0, gt=0)
    trial_timeout_s: float = Field(default=1020, gt=0)
    output_dir: str = ".logs/eval"
    cases: list[str] | None = None


class WorkflowTrialRecord(BaseModel):
    """One labelled trial of a workflow test set, one line of trials.jsonl."""

    model_config = {"extra": "forbid"}

    run_id: str
    testset_version: int
    mode: Mode
    case: str
    trial: int = Field(ge=1)
    prompt: str
    instruction_id: str | None
    started_ts: float
    ended_ts: float
    status: str | None = Field(description="turn_end status; None when no turn_end arrived")
    timed_out: bool = False
    cost_usd: float = 0.0
    num_turns: int = 0
    workflow: WorkflowDerivation
    expected_workflow: str
    workflow_match: bool = Field(description="The agent started expected_workflow")
    success: bool = Field(description="Human label: the job was done")
    note: str = ""


def load_config(path: str | Path) -> RunConfig:
    """Load and validate a run config.

    Args:
        path (str | Path): YAML file.

    Returns:
        RunConfig: The config.
    """
    return RunConfig.model_validate(yaml.safe_load(Path(path).read_text()) or {})


def check_workflow_mode(config: RunConfig) -> None:
    """Refuse every mode but full for a workflow test set.

    Args:
        config (RunConfig): Run config.

    Raises:
        ValueError: When the mode is arm_only (the arm-only deny list would block driving and looking around).
    """
    if config.mode != MODE_FULL:
        raise ValueError(
            f"workflow test sets need mode: full in the run config, got mode: {config.mode} "
            "(arm_only denies driving and looking around, which these jobs need)"
        )


def select_cases(testset: TestSet | WorkflowTestSet, config: RunConfig) -> list[Case] | list[WorkflowCase]:
    """Expand the cases of the mode and apply the optional subset.

    Args:
        testset (TestSet | WorkflowTestSet): The test set.
        config (RunConfig): Run config (``cases`` subset of ``<object_id>/<placement_id>`` or workflow case ids).

    Returns:
        list[Case] | list[WorkflowCase]: Cases in deterministic order.

    Raises:
        ValueError: When the subset names a case that the mode does not run.
    """
    cases = testset.cases if isinstance(testset, WorkflowTestSet) else testset.cases(config.mode)
    if config.cases is None:
        return cases
    known = {c.key for c in cases}
    unknown = [c for c in config.cases if c not in known]
    if unknown:
        raise ValueError(f"unknown case(s) for mode {config.mode}: {', '.join(unknown)}")
    return [c for c in cases if c.key in set(config.cases)]  # type: ignore[return-value]


def load_trials(path: Path) -> list[TrialRecord]:
    """Read the recorded trials of a run.

    Args:
        path (Path): trials.jsonl (may not exist).

    Returns:
        list[TrialRecord]: Recorded trials in file order.
    """
    if not path.exists():
        return []
    return [TrialRecord.model_validate_json(line) for line in path.read_text().splitlines() if line.strip()]


def load_workflow_trials(path: Path) -> list[WorkflowTrialRecord]:
    """Read the recorded trials of a workflow run.

    Args:
        path (Path): trials.jsonl (may not exist).

    Returns:
        list[WorkflowTrialRecord]: Recorded trials in file order.
    """
    if not path.exists():
        return []
    return [WorkflowTrialRecord.model_validate_json(line) for line in path.read_text().splitlines() if line.strip()]


def append_trial(path: Path, record: TrialRecord | WorkflowTrialRecord) -> None:
    """Append one record to trials.jsonl and flush it to disk.

    Args:
        path (Path): trials.jsonl.
        record (TrialRecord | WorkflowTrialRecord): The record.
    """
    with path.open("a") as handle:
        handle.write(record.model_dump_json() + "\n")
        handle.flush()


def wait_idle(
    client: AgentClient,
    config: RunConfig,
    print_fn: Callable[..., None],
    clock: Callable[[], float],
    sleep: Callable[[float], None],
) -> None:
    """Wait until the agent is not busy; never resets a busy agent.

    Args:
        client (AgentClient): Agent client.
        config (RunConfig): Run config (trial_timeout_s bounds the wait, poll_interval_s paces it).
        print_fn (Callable[..., None]): Output.
        clock (Callable[[], float]): Time source.
        sleep (Callable[[float], None]): Sleep.

    Raises:
        AgentError: When the agent is still busy after trial_timeout_s.
    """
    deadline = clock() + config.trial_timeout_s
    announced = False
    while client.state().get("busy"):
        if clock() >= deadline:
            raise AgentError(
                f"the agent is still busy after {config.trial_timeout_s:.0f} s (is someone using the chat?)"
            )
        if not announced:
            print_fn("agent is busy, waiting for it to become idle ...")
            announced = True
        sleep(config.poll_interval_s)


def poll_trial(
    client: AgentClient,
    config: RunConfig,
    baseline_seq: int,
    clock: Callable[[], float],
    sleep: Callable[[float], None],
    print_fn: Callable[..., None],
) -> tuple[list[dict[str, Any]], bool]:
    """Poll the history until the trial's turn_end; stop the instruction on timeout and wait 60 s more.

    Args:
        client (AgentClient): Agent client.
        config (RunConfig): Run config.
        baseline_seq (int): Newest seq before the message was posted.
        clock (Callable[[], float]): Time source.
        sleep (Callable[[float], None]): Sleep.
        print_fn (Callable[..., None]): Output.

    Returns:
        tuple[list[dict[str, Any]], bool]: The trial's events, from its user_message through its turn_end (through the
        newest event when no turn_end arrived), and whether the harness hit trial_timeout_s.
    """
    deadline = clock() + config.trial_timeout_s
    stopped = False
    while True:
        events = client.history_since(baseline_seq)
        user = next((e for e in events if e.get("type") == "user_message"), None)
        if user is not None:
            end = next(
                (
                    e
                    for e in events
                    if e.get("type") == "turn_end" and e.get("instruction_id") == user.get("instruction_id")
                ),
                None,
            )
            if end is not None:
                return [e for e in events if user["seq"] <= e["seq"] <= end["seq"]], stopped
        if clock() >= deadline:
            if stopped:
                print_fn("no turn_end 60 s after stop; recording the trial without one")
                return [e for e in events if user is not None and e["seq"] >= user["seq"]], True
            print_fn(f"trial timeout after {config.trial_timeout_s:.0f} s: stopping the instruction")
            client.stop()
            stopped = True
            deadline = clock() + STOP_GRACE_S
        sleep(config.poll_interval_s)


def write_report(
    trials: list[TrialRecord], config: RunConfig, planned: int, run_dir: Path, print_fn: Callable[..., None]
) -> None:
    """Print the report and write report.json.

    Args:
        trials (list[TrialRecord]): All recorded trials of the run.
        config (RunConfig): Run config (k).
        planned (int): Cases the run was set up for.
        run_dir (Path): Run directory.
        print_fn (Callable[..., None]): Output.
    """
    result = report_module.build(trials, config.k, cases_planned=planned)
    (run_dir / REPORT_FILE).write_text(json.dumps(result, indent=2))
    print_fn(report_module.render_text(result))


def run(
    config: RunConfig,
    client: AgentClient,
    input_fn: Callable[[str], str],
    *,
    run_id: str | None = None,
    resume: bool = False,
    print_fn: Callable[..., None] = print,
    clock: Callable[[], float] = time.time,
    sleep: Callable[[float], None] = time.sleep,
) -> int:
    """Run the evaluation.

    Args:
        config (RunConfig): Run config.
        client (AgentClient): Agent client.
        input_fn (Callable[[str], str]): Operator input.
        run_id (str | None): Run id (default: the current timestamp); required with resume.
        resume (bool): Continue an existing run, skipping recorded trials.
        print_fn (Callable[..., None]): Output.
        clock (Callable[[], float]): Time source.
        sleep (Callable[[float], None]): Sleep.

    Returns:
        int: 0 on a finished or quit run, 1 when aborted (busy agent, refused reset, battery block, agent error).

    Raises:
        FileNotFoundError: When resuming a run directory that does not exist.
        ValueError: For a case subset that the mode does not run.
    """
    testset = load_any_testset(config.testset)
    if isinstance(testset, WorkflowTestSet):
        check_workflow_mode(config)
    cases = select_cases(testset, config)
    run_id = run_id or datetime.fromtimestamp(clock()).strftime(RUN_ID_FORMAT)
    run_dir = Path(config.output_dir) / run_id
    if resume and not run_dir.is_dir():
        raise FileNotFoundError(f"cannot resume: {run_dir} does not exist")
    (run_dir / EVENTS_DIR).mkdir(parents=True, exist_ok=True)
    trials_path = run_dir / TRIALS_FILE
    trials: list[Any] = (
        load_workflow_trials(trials_path) if isinstance(testset, WorkflowTestSet) else load_trials(trials_path)
    )
    done = {(t.case, t.trial) for t in trials}
    case_ref = (lambda c: c.key) if isinstance(testset, WorkflowTestSet) else (lambda c: c)
    print_fn(f"run {run_id}: {len(cases)} case(s), mode {config.mode}, k={config.k}, output {run_dir}")
    code = EXIT_OK
    try:
        quit_run = False
        for case in cases:
            for trial in range(1, config.k + 1):
                if (case_ref(case), trial) in done:
                    continue
                if config.stop_on_success and any(t.success for t in trials if t.case == case_ref(case)):
                    break
                outcome = run_trial(
                    config, testset, client, input_fn, case, trial, run_id, run_dir, print_fn, clock, sleep
                )
                if outcome == "quit":
                    quit_run = True
                    break
                if isinstance(outcome, TrialRecord | WorkflowTrialRecord):
                    append_trial(trials_path, outcome)
                    trials.append(outcome)
            if quit_run:
                break
    except AgentError as exc:
        print_fn(f"ABORTED: {exc}")
        code = EXIT_ABORTED
    if isinstance(testset, WorkflowTestSet):
        write_workflow_report(trials, config, len(cases), run_dir, print_fn)
    else:
        write_report(trials, config, len(cases), run_dir, print_fn)
    return code


def build_workflow_report(trials: list[WorkflowTrialRecord], planned: int) -> dict[str, Any]:
    """Aggregate workflow trials per case.

    Args:
        trials (list[WorkflowTrialRecord]): Recorded trials.
        planned (int): Cases the run was set up for.

    Returns:
        dict[str, Any]: trials, cases_planned, workflow_match_rate (None without trials) and per case trials,
        successes and workflow_matches.
    """
    cases: dict[str, dict[str, int]] = {}
    for t in trials:
        row = cases.setdefault(t.case, {"trials": 0, "successes": 0, "workflow_matches": 0})
        row["trials"] += 1
        row["successes"] += int(t.success)
        row["workflow_matches"] += int(t.workflow_match)
    matches = sum(t.workflow_match for t in trials)
    return {
        "trials": len(trials),
        "cases_planned": planned,
        "workflow_match_rate": matches / len(trials) if trials else None,
        "cases": cases,
    }


def write_workflow_report(
    trials: list[WorkflowTrialRecord], config: RunConfig, planned: int, run_dir: Path, print_fn: Callable[..., None]
) -> None:
    """Print the workflow report and write report.json.

    Args:
        trials (list[WorkflowTrialRecord]): All recorded trials of the run.
        config (RunConfig): Run config (unused fields kept for symmetry with write_report).
        planned (int): Cases the run was set up for.
        run_dir (Path): Run directory.
        print_fn (Callable[..., None]): Output.
    """
    result = build_workflow_report(trials, planned)
    (run_dir / REPORT_FILE).write_text(json.dumps(result, indent=2))
    rate = result["workflow_match_rate"]
    print_fn(
        f"workflow report: {result['trials']} trial(s), workflow match rate {'n/a' if rate is None else f'{rate:.0%}'}"
    )
    for case_id, row in result["cases"].items():
        print_fn(
            f"  {case_id}: {row['trials']} trial(s), {row['successes']} success, "
            f"{row['workflow_matches']} with the expected workflow"
        )


def run_workflow_trial(
    config: RunConfig,
    testset: WorkflowTestSet,
    client: AgentClient,
    input_fn: Callable[[str], str],
    case: WorkflowCase,
    trial: int,
    run_id: str,
    run_dir: Path,
    print_fn: Callable[..., None],
    clock: Callable[[], float],
    sleep: Callable[[float], None],
) -> WorkflowTrialRecord | str:
    """Run and label one trial of a workflow case (mode full, no deny list).

    Args:
        config (RunConfig): Run config (mode must be full).
        testset (WorkflowTestSet): The test set.
        client (AgentClient): Agent client.
        input_fn (Callable[[str], str]): Operator input.
        case (WorkflowCase): The case.
        trial (int): Trial number (1-based).
        run_id (str): Run id.
        run_dir (Path): Run directory.
        print_fn (Callable[..., None]): Output.
        clock (Callable[[], float]): Time source.
        sleep (Callable[[float], None]): Sleep.

    Returns:
        WorkflowTrialRecord | str: The record, ``"skip"`` or ``"quit"``.

    Raises:
        ValueError: When the mode is not full.
        AgentError: When the agent cannot be used.
    """
    check_workflow_mode(config)
    print_fn(f"\n=== {case.key} trial {trial}/{config.k} ===")
    answer = ask_key(f"Setup: {case.setup} Enter to start, s to skip, q to quit: ", SETUP_KEYS, input_fn, default="")
    if answer in SETUP_KEYS:
        return "skip" if answer == "s" else "quit"
    wait_idle(client, config, print_fn, clock, sleep)
    if config.reset_between_trials:
        client.reset()
    baseline = client.latest_seq()
    started = clock()
    client.message(case.prompt)
    print_fn("instruction sent, waiting for the agent ...")
    events, timed_out = poll_trial(client, config, baseline, clock, sleep, print_fn)
    ended = clock()
    (run_dir / EVENTS_DIR / f"{case.key}_{trial}.json").write_text(json.dumps(events, indent=2))
    if timed_out:
        print_fn("TIMEOUT: the harness stopped this trial; it is recorded as timed_out")
    derivation = derive(events)
    workflow = derive_workflow(events)
    match = workflow.workflow == case.expected_workflow
    print_fn(
        f"workflow: {workflow.workflow or 'none'}, expected {case.expected_workflow}: "
        f"{'match' if match else 'MISMATCH'}; skipped mandatory: "
        f"{'n/a' if workflow.skipped_mandatory is None else ', '.join(workflow.skipped_mandatory) or 'none'}; "
        f"retrospective: {workflow.retrospective or 'none'}; skills: {', '.join(workflow.skills_loaded) or 'none'}"
    )
    success = SUCCESS_KEYS[ask_key("Was the job done? [y/n]: ", SUCCESS_KEYS, input_fn)]
    note = input_fn("Note (optional, Enter for none): ").strip()
    turn_end = derivation.turn_end or {}
    return WorkflowTrialRecord(
        run_id=run_id,
        testset_version=testset.version,
        mode=config.mode,
        case=case.key,
        trial=trial,
        prompt=case.prompt,
        instruction_id=derivation.instruction_id,
        started_ts=started,
        ended_ts=ended,
        status=turn_end.get("status"),
        timed_out=timed_out,
        cost_usd=float(turn_end.get("cost_usd") or 0.0),
        num_turns=int(turn_end.get("num_turns") or 0),
        workflow=workflow,
        expected_workflow=case.expected_workflow,
        workflow_match=match,
        success=success,
        note=note,
    )


def run_trial(
    config: RunConfig,
    testset: TestSet | WorkflowTestSet,
    client: AgentClient,
    input_fn: Callable[[str], str],
    case: Case | WorkflowCase,
    trial: int,
    run_id: str,
    run_dir: Path,
    print_fn: Callable[..., None],
    clock: Callable[[], float],
    sleep: Callable[[float], None],
) -> TrialRecord | WorkflowTrialRecord | str:
    """Run and label one trial; a workflow test set goes to run_workflow_trial.

    Args:
        config (RunConfig): Run config.
        testset (TestSet | WorkflowTestSet): The test set.
        client (AgentClient): Agent client.
        input_fn (Callable[[str], str]): Operator input.
        case (Case | WorkflowCase): The case.
        trial (int): Trial number (1-based).
        run_id (str): Run id.
        run_dir (Path): Run directory.
        print_fn (Callable[..., None]): Output.
        clock (Callable[[], float]): Time source.
        sleep (Callable[[float], None]): Sleep.

    Returns:
        TrialRecord | WorkflowTrialRecord | str: The record, ``"skip"`` or ``"quit"``.

    Raises:
        ValueError: For a workflow test set outside mode full.
        AgentError: When the agent cannot be used (busy, refused reset or message, unreachable, battery block).
    """
    if isinstance(testset, WorkflowTestSet) and isinstance(case, WorkflowCase):
        return run_workflow_trial(
            config, testset, client, input_fn, case, trial, run_id, run_dir, print_fn, clock, sleep
        )
    obj = next(o for o in testset.objects if o.id == case.object_id)
    placement = next(p for p in testset.placements if p.id == case.placement_id)
    print_fn(f"\n=== {case.key} trial {trial}/{config.k} ===")
    answer = ask_key(
        f"Place the {obj.name} ({obj.description}) {placement.description}. Enter to start, s to skip, q to quit: ",
        SETUP_KEYS,
        input_fn,
        default="",
    )
    if answer in SETUP_KEYS:
        return "skip" if answer == "s" else "quit"
    wait_idle(client, config, print_fn, clock, sleep)
    if config.reset_between_trials:
        client.reset()
    baseline = client.latest_seq()
    started = clock()
    deny = config.arm_only_deny_tools if config.mode == MODE_ARM_ONLY else None
    client.message(testset.render_prompt(case, config.mode), deny_tools=deny)
    print_fn("instruction sent, waiting for the agent ...")
    events, timed_out = poll_trial(client, config, baseline, clock, sleep, print_fn)
    ended = clock()
    (run_dir / EVENTS_DIR / f"{case.key.replace('/', '_')}_{trial}.json").write_text(json.dumps(events, indent=2))
    if timed_out:
        print_fn("TIMEOUT: the harness stopped this trial; it is recorded as timed_out")
    derivation = derive(events)
    print_fn("agent-claimed stages: " + ", ".join(f"{s}={v}" for s, v in derivation.claimed.items()))
    print_fn("lookup ids: " + (", ".join(derivation.lookup_ids) or "none"))
    workflow = derive_workflow(events)
    print_fn(
        f"workflow: {workflow.workflow or 'none'}; skipped mandatory: "
        f"{'n/a' if workflow.skipped_mandatory is None else ', '.join(workflow.skipped_mandatory) or 'none'}; "
        f"retrospective: {workflow.retrospective or 'none'}; survey POI writes: {workflow.survey_poi_writes}; "
        f"skills: {', '.join(workflow.skills_loaded) or 'none'}"
    )
    labelled = ask(derivation.claimed, config.mode, input_fn)
    return trial_record_from_derivation(
        derivation,
        run_id=run_id,
        testset_version=testset.version,
        mode=config.mode,
        case=case,
        trial=trial,
        started_ts=started,
        ended_ts=ended,
        labels=labelled.labels,
        success=labelled.success,
        first_failed_stage=labelled.first_failed_stage,
        note=labelled.note,
        timed_out=timed_out,
        workflow=workflow,
    )


def main(argv: list[str]) -> int:
    """CLI entry.

    Args:
        argv (list[str]): Config path and optional ``--resume <run_id>``.

    Returns:
        int: Process exit code.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("config", type=Path, help=DEFAULT_CONFIG_HELP)
    parser.add_argument("--resume", metavar="RUN_ID", default=None, help="continue this run, skipping recorded trials")
    args = parser.parse_args(argv)
    config = load_config(args.config)
    try:
        return run(config, AgentClient(config.base_url), input, run_id=args.resume, resume=args.resume is not None)
    except ValueError as exc:
        print(f"error: {exc}")
        return EXIT_CONFIG


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
