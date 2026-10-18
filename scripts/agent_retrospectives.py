#!/usr/bin/env python3
"""Pull the driving agent's retrospectives into the repo's logs.

Usage: uv run python scripts/agent_retrospectives.py [config.yaml]

Fetches ``GET <base_url>/retrospectives?limit=`` and writes ``<output_dir>/retrospectives-<YYYYmmdd-HHMMSS>.md``
(one section per record) plus the raw ``.jsonl`` beside it. Exit 0 on success, 1 on an HTTP or connection error,
2 on a bad config.
"""

import json
import logging
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

import httpx
import yaml
from pydantic import BaseModel, Field, ValidationError

EXIT_OK = 0
EXIT_FETCH_ERROR = 1
EXIT_BAD_CONFIG = 2
DEFAULT_CONFIG = Path(__file__).with_name("agent_retrospectives.yaml")
FILE_STAMP_FORMAT = "%Y%m%d-%H%M%S"
DATE_FORMAT = "%Y-%m-%d %H:%M"
FILE_PREFIX = "retrospectives-"
FREE_WORKFLOW = "free"
LIST_SECTIONS = (
    ("went_well", "Went well"),
    ("went_wrong", "Went wrong"),
    ("improvements", "Improvements"),
    ("notes_for_fixers", "Notes for fixers"),
    ("skipped_mandatory", "Skipped mandatory steps"),
)

logger = logging.getLogger("agent_retrospectives")


class PullConfig(BaseModel):
    """Validated contents of agent_retrospectives.yaml.

    Attributes:
        base_url (str): Agent API root (through the web_ui proxy).
        limit (int): Most records requested.
        output_dir (str): Directory for the Markdown and JSONL files.
        timeout_s (float): HTTP timeout in seconds.
    """

    model_config = {"extra": "forbid"}

    base_url: str
    limit: int = Field(default=50, ge=1, le=500)
    output_dir: str = ".logs/retrospectives"
    timeout_s: float = Field(default=10.0, gt=0)


def load_config(path: str | Path) -> PullConfig:
    """Load and validate the config.

    Args:
        path (str | Path): YAML file.

    Returns:
        PullConfig: The config.
    """
    return PullConfig.model_validate(yaml.safe_load(Path(path).read_text()) or {})


def fetch_retrospectives(config: PullConfig, transport: httpx.BaseTransport | None = None) -> list[dict[str, Any]]:
    """GET the retrospectives (newest first).

    Args:
        config (PullConfig): The config.
        transport (httpx.BaseTransport | None): Optional transport (tests).

    Returns:
        list[dict[str, Any]]: The raw records.

    Raises:
        httpx.HTTPError: On a connection error or a non-2xx status.
        ValueError: When the body is not the expected JSON.
    """
    url = config.base_url.rstrip("/") + "/retrospectives"
    with httpx.Client(timeout=config.timeout_s, transport=transport) as client:
        response = client.get(url, params={"limit": config.limit})
        response.raise_for_status()
        body = response.json()
    records = body.get("retrospectives") if isinstance(body, dict) else None
    if not isinstance(records, list):
        raise ValueError("response has no 'retrospectives' list")
    return records


def render_markdown(records: list[dict[str, Any]]) -> str:
    """Render the records, one section each.

    Args:
        records (list[dict[str, Any]]): Retrospective records.

    Returns:
        str: Markdown text.
    """
    lines = [f"# Agent retrospectives ({len(records)})", ""]
    for record in records:
        date = datetime.fromtimestamp(float(record.get("created_at", 0))).strftime(DATE_FORMAT)
        workflow = record.get("workflow") or FREE_WORKFLOW
        lines += [
            f"## {date} - {workflow} - {record.get('outcome', '?')}",
            "",
            f"- Author: {record.get('author', '?')}",
            f"- Instruction: {record.get('instruction', '')}",
            "",
            str(record.get("summary", "")),
            "",
        ]
        for key, title in LIST_SECTIONS:
            items = record.get(key) or []
            if items:
                lines += [f"### {title}", ""] + [f"- {item}" for item in items] + [""]
        phases = record.get("phases") or []
        if phases:
            lines += ["### Phases", ""] + [f"- {phase['name']}: {phase['outcome']}" for phase in phases] + [""]
    return "\n".join(lines)


def write_outputs(records: list[dict[str, Any]], output_dir: Path, now: datetime) -> tuple[Path, Path]:
    """Write the Markdown report and the raw JSONL.

    Args:
        records (list[dict[str, Any]]): Retrospective records.
        output_dir (Path): Target directory (created).
        now (datetime): Timestamp for the file names.

    Returns:
        tuple[Path, Path]: Markdown path and JSONL path.
    """
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = FILE_PREFIX + now.strftime(FILE_STAMP_FORMAT)
    md_path = output_dir / f"{stem}.md"
    jsonl_path = output_dir / f"{stem}.jsonl"
    md_path.write_text(render_markdown(records))
    jsonl_path.write_text("".join(json.dumps(record) + "\n" for record in records))
    return md_path, jsonl_path


def main(argv: list[str] | None = None, transport: httpx.BaseTransport | None = None) -> int:
    """Run the pull.

    Args:
        argv (list[str] | None): Arguments after the program name (optional config path).
        transport (httpx.BaseTransport | None): Optional transport (tests).

    Returns:
        int: Exit code (0 ok, 1 fetch error, 2 bad config).
    """
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    args = sys.argv[1:] if argv is None else argv
    config_path = Path(args[0]) if args else DEFAULT_CONFIG
    try:
        config = load_config(config_path)
    except (OSError, yaml.YAMLError, ValidationError) as error:
        print(f"bad config {config_path}: {error}", file=sys.stderr)
        return EXIT_BAD_CONFIG
    try:
        records = fetch_retrospectives(config, transport)
    except httpx.HTTPStatusError as error:
        response = error.response
        print(f"HTTP {response.status_code} from {error.request.url}: {response.text[:200]}", file=sys.stderr)
        return EXIT_FETCH_ERROR
    except (httpx.HTTPError, ValueError) as error:
        print(f"cannot fetch retrospectives from {config.base_url}: {error}", file=sys.stderr)
        return EXIT_FETCH_ERROR
    md_path, jsonl_path = write_outputs(records, Path(config.output_dir), datetime.now())
    logger.info("wrote %d retrospectives", len(records))
    print(f"{len(records)} retrospectives\n{md_path}\n{jsonl_path}")
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
