"""Retrospective store: one structured, bounded record per job, kept on the robot.

The store lives in ``<state_dir>/retrospectives/retrospectives.jsonl`` (one Retrospective per line, created on demand).
It is not part of the session log, so /api/reset never touches it. It is bounded by ``retro_max_records``: past the
bound the oldest half of the records is dropped with an atomic rewrite. Text fields are truncated, never rejected.
"""

import builtins
import json
import logging
import time
import uuid
from datetime import UTC, datetime
from pathlib import Path
from typing import Annotated, Any, Literal

from pydantic import BaseModel, BeforeValidator, Field, ValidationError

RETRO_RELPATH = Path("retrospectives") / "retrospectives.jsonl"
INSTRUCTION_MAX_CHARS = 500
SUMMARY_MAX_CHARS = 500
LIST_MAX_ITEMS = 10
ITEM_MAX_CHARS = 300
FREE_WORKFLOW = "free"
FEED_HEADER = "RECENT RETROSPECTIVES (newest first; lessons from earlier jobs on this robot):"
FEED_DATE_FORMAT = "%Y-%m-%d %H:%M"
CORRUPT_MESSAGE = "retrospectives: skipping corrupt line {number} of {path}"
WRITE_FAILED_MESSAGE = "retrospectives: cannot write {path} ({error})"

Outcome = Literal["done", "partial", "failed", "interrupted", "timeout", "error"]


def truncate_text(limit: int) -> BeforeValidator:
    """Validator that cuts a string to ``limit`` characters (non-strings pass through to normal validation).

    Args:
        limit (int): Most characters kept.

    Returns:
        BeforeValidator: The validator.
    """
    return BeforeValidator(lambda value: value[:limit] if isinstance(value, str) else value)


def truncate_list(value: Any) -> Any:
    """Cut a list to LIST_MAX_ITEMS items of ITEM_MAX_CHARS characters (anything else passes through).

    Args:
        value (Any): Candidate list of strings.

    Returns:
        Any: The truncated list, or the value unchanged when it is not a list.
    """
    if not isinstance(value, list):
        return value
    return [item[:ITEM_MAX_CHARS] if isinstance(item, str) else item for item in value[:LIST_MAX_ITEMS]]


Instruction = Annotated[str, truncate_text(INSTRUCTION_MAX_CHARS)]
Summary = Annotated[str, truncate_text(SUMMARY_MAX_CHARS)]
Notes = Annotated[list[str], BeforeValidator(truncate_list)]


class RetroPhase(BaseModel):
    """One plan phase of the job.

    Attributes:
        name (str): Phase name.
        outcome (str): Phase outcome (done, failed, skipped, ...).
    """

    name: str
    outcome: str


class Retrospective(BaseModel):
    """One job's retrospective.

    Attributes:
        id (str): uuid4 hex.
        created_at (float): Unix seconds.
        instruction (str): The instruction text, cut to 500 characters.
        workflow (str | None): Workflow template name, None for a free job.
        outcome (str): done, partial, failed, interrupted, timeout or error.
        trivial (bool): Short form (one-line summary) for a trivial job.
        author (str): agent (written by the model) or runner (synthesised by the runner).
        summary (str): One to three sentences, cut to 500 characters.
        went_well (list[str]): Up to 10 items of 300 characters.
        went_wrong (list[str]): Same limits.
        improvements (list[str]): Same limits.
        notes_for_fixers (list[str]): Same limits; notes for the developers who fix the robot.
        skipped_mandatory (list[str]): Mandatory workflow step ids skipped.
        phases (list[RetroPhase]): Plan phases with their outcomes.
        cost_usd (float | None): Job cost.
        num_turns (int | None): Model turns.
        effector_calls (int | None): Motion tool calls.
        duration_s (float | None): Job duration.
    """

    id: str = Field(default_factory=lambda: uuid.uuid4().hex)
    created_at: float = Field(default_factory=time.time)
    instruction: Instruction
    workflow: str | None = None
    outcome: Outcome
    trivial: bool = False
    author: Literal["agent", "runner"] = "agent"
    summary: Summary
    went_well: Notes = Field(default_factory=list)
    went_wrong: Notes = Field(default_factory=list)
    improvements: Notes = Field(default_factory=list)
    notes_for_fixers: Notes = Field(default_factory=list)
    skipped_mandatory: list[str] = Field(default_factory=list)
    phases: list[RetroPhase] = Field(default_factory=list)
    cost_usd: float | None = None
    num_turns: int | None = None
    effector_calls: int | None = None
    duration_s: float | None = None


class RetroStore:
    """Bounded JSONL store of retrospectives.

    Attributes:
        path (Path): Store file (created at the first append).
        max_records (int): Bound; past it the oldest half is dropped.
    """

    def __init__(self, path: Path, max_records: int, logger: Any = None) -> None:
        """Create the store handle.

        Args:
            path (Path): Store file.
            max_records (int): Most records kept.
            logger (Any): Object with warning(str); defaults to the stdlib "claude_agent" logger.
        """
        self.path = path
        self.max_records = max_records
        self.logger = logger or logging.getLogger("claude_agent")

    def read_all(self) -> list[Retrospective]:
        """Read the valid records, oldest first; each corrupt line is skipped with one warning.

        Returns:
            list[Retrospective]: The records; empty when the file is missing.
        """
        if not self.path.is_file():
            return []
        records = []
        for number, line in enumerate(self.path.read_text(encoding="utf-8").splitlines(), 1):
            if not line.strip():
                continue
            try:
                records.append(Retrospective.model_validate(json.loads(line)))
            except (ValueError, ValidationError):
                self.logger.warning(CORRUPT_MESSAGE.format(number=number, path=self.path))
        return records

    def append(self, record: Retrospective) -> Retrospective:
        """Append one record, rotating past max_records (oldest half dropped, atomic rewrite).

        Args:
            record (Retrospective): The record to store.

        Returns:
            Retrospective: The stored record.
        """
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.path.open("a", encoding="utf-8") as handle:
            handle.write(record.model_dump_json() + "\n")
        lines = [line for line in self.path.read_text(encoding="utf-8").splitlines() if line.strip()]
        if len(lines) > self.max_records:
            records = self.read_all()
            kept = records[len(records) // 2 :]
            tmp = self.path.with_suffix(".tmp")
            tmp.write_text("".join(r.model_dump_json() + "\n" for r in kept), encoding="utf-8")
            tmp.replace(self.path)
        return record

    def replace(self, record: Retrospective) -> Retrospective:
        """Replace the stored record with the same id (atomic rewrite, position kept); append it when there is none.

        Args:
            record (Retrospective): The new version of the record.

        Returns:
            Retrospective: The stored record.
        """
        records = self.read_all()
        if not any(r.id == record.id for r in records):
            return self.append(record)
        updated = [record if r.id == record.id else r for r in records]
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text("".join(r.model_dump_json() + "\n" for r in updated), encoding="utf-8")
        tmp.replace(self.path)
        return record

    def list(self, limit: int, before: float | None = None) -> list[Retrospective]:
        """Return records newest first.

        Args:
            limit (int): Most records returned.
            before (float | None): Only records with created_at < before.

        Returns:
            list[Retrospective]: Newest first.
        """
        records = [r for r in self.read_all() if before is None or r.created_at < before]
        records.sort(key=lambda r: r.created_at, reverse=True)
        return records[: max(limit, 0)]

    def recent(self, n: int) -> builtins.list[Retrospective]:
        """Return the newest n records, newest first.

        Args:
            n (int): Count.

        Returns:
            list[Retrospective]: Newest first.
        """
        return self.list(n, None)

    def count(self) -> int:
        """Return the number of valid stored records.

        Returns:
            int: Record count.
        """
        return len(self.read_all())


def feed_text(records: list[Retrospective], max_chars: int) -> str:
    """Build the RECENT RETROSPECTIVES prompt block.

    Args:
        records (list[Retrospective]): Records, newest first.
        max_chars (int): Cut the block at a record boundary so it is at most this long.

    Returns:
        str: The block, or "" when there are no records (or none fits).
    """
    text = FEED_HEADER
    added = 0
    for record in records:
        date = datetime.fromtimestamp(record.created_at, tz=UTC).strftime(FEED_DATE_FORMAT)
        lines = [f"- {date} {record.workflow or FREE_WORKFLOW} [{record.outcome}]: {record.summary}"]
        if record.went_wrong:
            lines.append("  went wrong: " + "; ".join(record.went_wrong))
        if record.improvements:
            lines.append("  improve: " + "; ".join(record.improvements))
        candidate = text + "\n" + "\n".join(lines)
        if len(candidate) > max_chars:
            break
        text = candidate
        added += 1
    return text if added else ""
