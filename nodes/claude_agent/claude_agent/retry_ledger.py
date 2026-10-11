"""Retry calibration ledger: each p_retry_success prediction paired with the outcome of the next attempt.

The ledger lives in ``<state_dir>/calibration/retry_outcomes.jsonl`` (one RetryRecord per line, created on demand). It is
not part of the session log, so /api/reset never touches it. It is bounded by ``retry_ledger_max_records``: past the
bound the oldest half of the records is dropped with an atomic rewrite. A write failure is logged once and never raised;
the Prometheus metrics move either way.
"""

import json
import logging
import math
import time
from pathlib import Path
from typing import Any, Literal

from pydantic import BaseModel, ValidationError

from .metrics import RETRY_PREDICTIONS, RETRY_SQUARED_ERROR

RETRY_LEDGER_RELPATH = Path("calibration") / "retry_outcomes.jsonl"
SCHEMA_VERSION = 1
OUTCOME_DONE = "done"
OUTCOME_FAILED = "failed"
OUTCOME_UNOBSERVED = "unobserved"
VIA_AUTO_RETRY = "auto_retry"
VIA_LOOP = "loop"
VIA_NONE = "none"
METRIC_SUCCESS = "success"
METRIC_FAILURE = "failure"
BIN_COUNT = 10
BIN_EPSILON = 1e-9
WRITE_FAILED_MESSAGE = "retry ledger: cannot write {path} ({error}); records are kept in metrics only"


class RetryRecord(BaseModel):
    """One retry prediction and the outcome of the next attempt of the same phase.

    Attributes:
        schema_version (int): Always 1.
        ts (float): Unix close time of the predicting attempt.
        instruction_id (str): Instruction the phase ran in.
        phase_id (str): Plan-tree phase id.
        phase_name (str): Phase name.
        attempt (int): Attempt that made the prediction.
        iteration (int): Loop iteration of that attempt.
        p_retry_success (float): The model's estimate that another attempt succeeds.
        threshold (float): retry_success_threshold at decision time.
        retried (bool): Whether the automatic retry was granted.
        refusal (str | None): Why it was refused (low_p, attempts, cap, budget, plan_failure), None when retried.
        failure_type (str): FailureRecord type.
        eai_error_type (str): FailureRecord eai_error_type.
        next_outcome (str): Status of the next close of the phase, or unobserved when none came in the instruction.
        next_via (str): auto_retry, loop (re-entered later) or none (unobserved).
        model (str): Configured Claude model.
    """

    schema_version: Literal[1] = SCHEMA_VERSION
    ts: float
    instruction_id: str
    phase_id: str
    phase_name: str
    attempt: int
    iteration: int
    p_retry_success: float
    threshold: float
    retried: bool
    refusal: str | None
    failure_type: str
    eai_error_type: str
    next_outcome: Literal["done", "failed", "skipped", "unobserved"]
    next_via: Literal["auto_retry", "loop", "none"]
    model: str


def probability_bin(p: float) -> str:
    """Label of the 0.1-wide bin of a probability (1.0 falls into "0.9").

    Args:
        p (float): Probability in [0, 1].

    Returns:
        str: "0.0" .. "0.9".
    """
    index = min(max(math.floor(p * BIN_COUNT + BIN_EPSILON), 0), BIN_COUNT - 1)
    return f"{index / BIN_COUNT:.1f}"


def observe_record(record: RetryRecord) -> None:
    """Move the calibration metrics for one written record.

    Args:
        record (RetryRecord): The resolved record.
    """
    if record.next_outcome == OUTCOME_UNOBSERVED:
        outcome = OUTCOME_UNOBSERVED
    else:
        outcome = METRIC_SUCCESS if record.next_outcome == OUTCOME_DONE else METRIC_FAILURE
        y = 1.0 if outcome == METRIC_SUCCESS else 0.0
        RETRY_SQUARED_ERROR.observe((record.p_retry_success - y) ** 2)
    RETRY_PREDICTIONS.labels(probability_bin(record.p_retry_success), outcome).inc()


def opens_prediction(phase: dict[str, Any]) -> bool:
    """Whether a closed phase is a failed close by the model carrying p_retry_success and a failure record.

    Args:
        phase (dict[str, Any]): Phase payload (Phase.as_dict).

    Returns:
        bool: True for a model-judged failed close; False for done/skipped and revise_plan's implicit close.
    """
    return (
        phase["status"] == OUTCOME_FAILED
        and phase.get("p_retry_success") is not None
        and phase.get("failure") is not None
    )


class RetryLedger:
    """Pairs failed-phase predictions with the next close of the same phase and appends them to a bounded JSONL file.

    Attributes:
        path (Path | None): Ledger file (None keeps records in metrics only).
        max_records (int): Bound of the file; past it the oldest half is dropped.
        model (str): Model name written into each record.
        path_ok (bool): False once a write failed.
    """

    def __init__(self, path: Path | None, max_records: int, model: str, logger: Any = None) -> None:
        """Create the ledger (the file is created at the first record).

        Args:
            path (Path | None): Ledger file, or None for no file.
            max_records (int): Most records kept in the file.
            model (str): Model name for the records.
            logger (Any): Object with warning(str); defaults to the stdlib "claude_agent" logger.
        """
        self.path = path
        self.max_records = max_records
        self.model = model
        self.logger = logger or logging.getLogger("claude_agent")
        self.path_ok = True
        self.count: int | None = None
        self.pending: dict[tuple[str, str], dict[str, Any]] = {}

    def on_phase_started(self, instruction_id: str, phase: dict[str, Any]) -> None:
        """Mark a pending prediction as re-entered (auto_retry when the retry was granted, else loop).

        Args:
            instruction_id (str): Current instruction.
            phase (dict[str, Any]): The started phase payload.
        """
        entry = self.pending.get((instruction_id, phase["id"]))
        if entry is not None and entry["next_via"] == VIA_NONE:
            entry["next_via"] = VIA_AUTO_RETRY if entry["retried"] else VIA_LOOP

    def on_phase_completed(
        self,
        instruction_id: str,
        phase: dict[str, Any],
        retried: bool,
        refusal: str | None,
        threshold: float,
        ts: float | None = None,
    ) -> None:
        """Resolve the pending prediction of this phase and open a new one for a model-judged failed close.

        A close without a model judgement (revise_plan's implicit close) neither resolves nor opens a prediction.

        Args:
            instruction_id (str): Current instruction.
            phase (dict[str, Any]): The closed phase payload.
            retried (bool): Whether the automatic retry was granted for this close.
            refusal (str | None): Refusal code of this close, None when retried or not failed.
            threshold (float): retry_success_threshold at decision time.
            ts (float | None): Close time (defaults to now).
        """
        key = (instruction_id, phase["id"])
        if phase["status"] == OUTCOME_FAILED and not opens_prediction(phase):
            return
        entry = self.pending.pop(key, None)
        if entry is not None:
            if entry["next_via"] == VIA_NONE:
                entry["next_via"] = VIA_AUTO_RETRY if entry["retried"] else VIA_LOOP
            self.write(RetryRecord(**entry, next_outcome=phase["status"]))
        if opens_prediction(phase):
            self.pending[key] = {
                "ts": time.time() if ts is None else ts,
                "instruction_id": instruction_id,
                "phase_id": phase["id"],
                "phase_name": phase["name"],
                "attempt": phase["attempt"],
                "iteration": phase["iteration"],
                "p_retry_success": phase["p_retry_success"],
                "threshold": threshold,
                "retried": retried,
                "refusal": refusal,
                "failure_type": phase["failure"]["type"],
                "eai_error_type": phase["failure"]["eai_error_type"],
                "next_via": VIA_NONE,
                "model": self.model,
            }

    def end_instruction(self, instruction_id: str) -> None:
        """Write every prediction of the instruction still pending as unobserved.

        Args:
            instruction_id (str): The instruction that ended.
        """
        for key in [key for key in self.pending if key[0] == instruction_id]:
            entry = self.pending.pop(key)
            entry["next_via"] = VIA_NONE
            self.write(RetryRecord(**entry, next_outcome=OUTCOME_UNOBSERVED))

    def write(self, record: RetryRecord) -> None:
        """Count the record in the metrics and append it to the file (rotating past max_records).

        Args:
            record (RetryRecord): The resolved record.
        """
        observe_record(record)
        if self.path is None or not self.path_ok:
            return
        try:
            if self.count is None:
                self.count = len(self.read_lines())
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open("a", encoding="utf-8") as handle:
                handle.write(record.model_dump_json() + "\n")
            self.count += 1
            if self.count > self.max_records:
                self.rotate()
        except OSError as error:
            self.path_ok = False
            self.logger.warning(WRITE_FAILED_MESSAGE.format(path=self.path, error=error))

    def rotate(self) -> None:
        """Drop the oldest half of the records (atomic rewrite)."""
        assert self.path is not None
        lines = self.read_lines()
        kept = lines[len(lines) // 2 :]
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text("".join(line + "\n" for line in kept), encoding="utf-8")
        tmp.replace(self.path)
        self.count = len(kept)

    def read_lines(self) -> list[str]:
        """Return the valid record lines of the file, oldest first (corrupt lines skipped).

        Returns:
            list[str]: JSON lines that parse as a RetryRecord; empty when the file is missing.
        """
        if self.path is None or not self.path.is_file():
            return []
        lines = []
        for line in self.path.read_text(encoding="utf-8").splitlines():
            try:
                RetryRecord.model_validate(json.loads(line))
            except (ValueError, ValidationError):
                continue
            lines.append(line)
        return lines

    def read(self, limit: int | None = None) -> list[dict[str, Any]]:
        """Return the stored records, newest last (corrupt lines skipped).

        Args:
            limit (int | None): Return only the newest ``limit`` records.

        Returns:
            list[dict[str, Any]]: The records; empty when the file is missing or unreadable.
        """
        try:
            records = [json.loads(line) for line in self.read_lines()]
        except OSError:
            return []
        return records if limit is None else records[-limit:] if limit > 0 else []
