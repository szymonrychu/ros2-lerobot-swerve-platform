"""Evaluation driver: runs the fixed test set against the claude_agent trial by trial with a human labelling step.

Usage: python -m scripts.eval.run <run.yaml> [--resume <run_id>]

Per trial: setup prompt (Enter, s skip, q quit), idle check, optional reset, POST the instruction (deny_tools only in
arm_only mode), poll the history to the trial's turn_end (on timeout POST stop and wait 60 s more), save the raw events,
print the claimed stages and lookup ids, ask the labels and append one TrialRecord line to
<output_dir>/<run_id>/trials.jsonl. The report is printed and written to report.json at the end, on q and on abort.
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
from scripts.eval.labels import ask, ask_key
from scripts.eval.report import TrialRecord, trial_record_from_derivation
from scripts.eval.stages import derive
from scripts.eval.testset import MODE_ARM_ONLY, Case, Mode, TestSet, load_testset

STOP_GRACE_S = 60.0
SETUP_KEYS = {"s": "skip this trial", "q": "quit and report"}
TRIALS_FILE = "trials.jsonl"
REPORT_FILE = "report.json"
EVENTS_DIR = "events"
RUN_ID_FORMAT = "%Y%m%d-%H%M%S"
EXIT_OK = 0
EXIT_ABORTED = 1
DEFAULT_CONFIG_HELP = "run config YAML (scripts/eval/run.yaml)"


class RunConfig(BaseModel):
    """Validated contents of run.yaml."""

    model_config = {"extra": "forbid"}

    base_url: str
    testset: str = "scripts/eval/testset_v1.yaml"
    mode: Mode = "arm_only"
    arm_only_deny_tools: list[str] = Field(default_factory=lambda: ["navigate_to_pose", "move_relative", "drive"])
    k: int = Field(default=3, ge=1)
    stop_on_success: bool = True
    reset_between_trials: bool = True
    poll_interval_s: float = Field(default=2.0, gt=0)
    trial_timeout_s: float = Field(default=1020, gt=0)
    output_dir: str = ".logs/eval"
    cases: list[str] | None = None


def load_config(path: str | Path) -> RunConfig:
    """Load and validate a run config.

    Args:
        path (str | Path): YAML file.

    Returns:
        RunConfig: The config.
    """
    return RunConfig.model_validate(yaml.safe_load(Path(path).read_text()) or {})


def select_cases(testset: TestSet, config: RunConfig) -> list[Case]:
    """Expand the cases of the mode and apply the optional subset.

    Args:
        testset (TestSet): The test set.
        config (RunConfig): Run config (``cases`` subset of ``<object_id>/<placement_id>``).

    Returns:
        list[Case]: Cases in deterministic order.

    Raises:
        ValueError: When the subset names a case that the mode does not run.
    """
    cases = testset.cases(config.mode)
    if config.cases is None:
        return cases
    known = {c.key for c in cases}
    unknown = [c for c in config.cases if c not in known]
    if unknown:
        raise ValueError(f"unknown case(s) for mode {config.mode}: {', '.join(unknown)}")
    return [c for c in cases if c.key in set(config.cases)]


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


def append_trial(path: Path, record: TrialRecord) -> None:
    """Append one record to trials.jsonl and flush it to disk.

    Args:
        path (Path): trials.jsonl.
        record (TrialRecord): The record.
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
    testset = load_testset(config.testset)
    cases = select_cases(testset, config)
    run_id = run_id or datetime.fromtimestamp(clock()).strftime(RUN_ID_FORMAT)
    run_dir = Path(config.output_dir) / run_id
    if resume and not run_dir.is_dir():
        raise FileNotFoundError(f"cannot resume: {run_dir} does not exist")
    (run_dir / EVENTS_DIR).mkdir(parents=True, exist_ok=True)
    trials_path = run_dir / TRIALS_FILE
    trials = load_trials(trials_path)
    done = {(t.case, t.trial) for t in trials}
    print_fn(f"run {run_id}: {len(cases)} case(s), mode {config.mode}, k={config.k}, output {run_dir}")
    code = EXIT_OK
    try:
        quit_run = False
        for case in cases:
            for trial in range(1, config.k + 1):
                if (case, trial) in done:
                    continue
                if config.stop_on_success and any(t.success for t in trials if t.case == case):
                    break
                outcome = run_trial(
                    config, testset, client, input_fn, case, trial, run_id, run_dir, print_fn, clock, sleep
                )
                if outcome == "quit":
                    quit_run = True
                    break
                if isinstance(outcome, TrialRecord):
                    append_trial(trials_path, outcome)
                    trials.append(outcome)
            if quit_run:
                break
    except AgentError as exc:
        print_fn(f"ABORTED: {exc}")
        code = EXIT_ABORTED
    write_report(trials, config, len(cases), run_dir, print_fn)
    return code


def run_trial(
    config: RunConfig,
    testset: TestSet,
    client: AgentClient,
    input_fn: Callable[[str], str],
    case: Case,
    trial: int,
    run_id: str,
    run_dir: Path,
    print_fn: Callable[..., None],
    clock: Callable[[], float],
    sleep: Callable[[float], None],
) -> TrialRecord | str:
    """Run and label one trial.

    Args:
        config (RunConfig): Run config.
        testset (TestSet): The test set.
        client (AgentClient): Agent client.
        input_fn (Callable[[str], str]): Operator input.
        case (Case): The case.
        trial (int): Trial number (1-based).
        run_id (str): Run id.
        run_dir (Path): Run directory.
        print_fn (Callable[..., None]): Output.
        clock (Callable[[], float]): Time source.
        sleep (Callable[[float], None]): Sleep.

    Returns:
        TrialRecord | str: The record, ``"skip"`` or ``"quit"``.

    Raises:
        AgentError: When the agent cannot be used (busy, refused reset or message, unreachable, battery block).
    """
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
    return run(config, AgentClient(config.base_url), input, run_id=args.resume, resume=args.resume is not None)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
