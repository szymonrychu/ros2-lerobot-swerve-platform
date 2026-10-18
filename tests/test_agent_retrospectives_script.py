"""Unit tests for scripts/agent_retrospectives.py: config, Markdown rendering, files written, exit codes."""

import json
from datetime import datetime
from pathlib import Path
from typing import Any

import httpx
import pydantic
import pytest
import yaml

from scripts.agent_retrospectives import (
    EXIT_BAD_CONFIG,
    EXIT_FETCH_ERROR,
    EXIT_OK,
    PullConfig,
    load_config,
    main,
    render_markdown,
)

SAMPLE: dict[str, Any] = {
    "id": "abc",
    "created_at": 1760000000.0,
    "instruction": "bring the plush",
    "workflow": "pick_place",
    "outcome": "partial",
    "author": "runner",
    "summary": "Picked but not placed.",
    "went_well": ["grasp worked"],
    "went_wrong": ["lost the object"],
    "improvements": ["slow down near the table"],
    "notes_for_fixers": ["gripper effort saturates"],
    "skipped_mandatory": ["verify_grasp"],
    "phases": [{"name": "pick", "outcome": "done"}, {"name": "place", "outcome": "failed"}],
}


def write_config(tmp_path: Path, **overrides: Any) -> Path:
    data = {"base_url": "http://robot/api/agent", "limit": 7, "output_dir": str(tmp_path / "out"), "timeout_s": 3}
    data.update(overrides)
    path = tmp_path / "c.yaml"
    path.write_text(yaml.safe_dump(data))
    return path


def transport(records: list[dict[str, Any]], seen: list[httpx.Request] | None = None) -> httpx.MockTransport:
    def handler(request: httpx.Request) -> httpx.Response:
        if seen is not None:
            seen.append(request)
        return httpx.Response(200, json={"retrospectives": records, "total": len(records)})

    return httpx.MockTransport(handler)


def test_shipped_config_validates() -> None:
    config = load_config(Path("scripts/agent_retrospectives.yaml"))
    assert config.limit == 50
    assert config.output_dir == ".logs/retrospectives"


def test_config_rejects_unknown_key_and_bad_limit(tmp_path: Path) -> None:
    with pytest.raises(pydantic.ValidationError):
        PullConfig.model_validate({"base_url": "x", "limit": 5, "output_dir": "o", "timeout_s": 1, "extra": 1})
    with pytest.raises(pydantic.ValidationError):
        PullConfig.model_validate({"base_url": "x", "limit": 0, "output_dir": "o", "timeout_s": 1})


def test_render_markdown_includes_all_sections() -> None:
    text = render_markdown([SAMPLE])
    for part in (
        "pick_place",
        "partial",
        "runner",
        "Picked but not placed.",
        "grasp worked",
        "lost the object",
        "slow down near the table",
        "Notes for fixers",
        "gripper effort saturates",
        "Skipped mandatory steps",
        "verify_grasp",
        "place: failed",
        datetime.fromtimestamp(1760000000.0).strftime("%Y-%m-%d %H:%M"),
    ):
        assert part in text


def test_render_markdown_free_job_without_lists() -> None:
    record = {"created_at": 1.0, "workflow": None, "outcome": "done", "author": "agent", "summary": "ok"}
    text = render_markdown([record])
    assert "free" in text
    assert "Skipped mandatory steps" not in text


def test_main_writes_timestamped_files(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    seen: list[httpx.Request] = []
    code = main([str(write_config(tmp_path))], transport=transport([SAMPLE], seen))
    assert code == EXIT_OK
    assert seen[0].url.path == "/api/agent/retrospectives"
    assert seen[0].url.params["limit"] == "7"
    out = tmp_path / "out"
    md = list(out.glob("retrospectives-*.md"))
    jsonl = list(out.glob("retrospectives-*.jsonl"))
    assert len(md) == 1 and len(jsonl) == 1
    assert md[0].stem == jsonl[0].stem
    datetime.strptime(md[0].stem, "retrospectives-%Y%m%d-%H%M%S")
    assert json.loads(jsonl[0].read_text().splitlines()[0])["id"] == "abc"
    printed = capsys.readouterr().out
    assert str(md[0]) in printed and str(jsonl[0]) in printed and "1" in printed


def test_main_exit_1_on_http_error(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    failing = httpx.MockTransport(lambda request: httpx.Response(502, text="bad gateway"))
    assert main([str(write_config(tmp_path))], transport=failing) == EXIT_FETCH_ERROR
    assert "502" in capsys.readouterr().err


def test_main_exit_1_on_connection_error(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    def refuse(request: httpx.Request) -> httpx.Response:
        raise httpx.ConnectError("refused", request=request)

    assert main([str(write_config(tmp_path))], transport=httpx.MockTransport(refuse)) == EXIT_FETCH_ERROR
    assert "refused" in capsys.readouterr().err


def test_main_exit_2_on_bad_config(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    assert main([str(write_config(tmp_path, limit=0))], transport=transport([])) == EXIT_BAD_CONFIG
    assert main([str(tmp_path / "missing.yaml")], transport=transport([])) == EXIT_BAD_CONFIG
    assert capsys.readouterr().err
