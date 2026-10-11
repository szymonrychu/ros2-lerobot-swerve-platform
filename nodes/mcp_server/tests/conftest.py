"""Shared pytest fixtures of the mcp_server tests."""

from pathlib import Path

import pytest

from mcp_server import config


@pytest.fixture(autouse=True)
def lookups_in_tmp(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    """Point the default camera lookup store at the test's tmp dir, so no test writes to /var/lib/ros2/poi/lookups.

    Args:
        tmp_path (Path): Per-test temporary directory.
        monkeypatch (pytest.MonkeyPatch): Patcher.

    Returns:
        Path: The lookup store directory every default McpServerConfig of this test uses.
    """
    store = tmp_path / "lookups"
    monkeypatch.setattr(config, "DEFAULT_LOOKUPS_DIR", store)
    return store
