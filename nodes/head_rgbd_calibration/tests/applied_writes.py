"""Record the order in which the node's writers land files in applied/."""

import os
from pathlib import Path
from typing import Any

import pytest


def record_applied_writes(monkeypatch: pytest.MonkeyPatch, applied: Path) -> list[str]:
    """Record the names each os.replace lands in applied/, in order (every node writer goes through os.replace).

    Args:
        monkeypatch: pytest monkeypatch.
        applied: The applied/ directory.

    Returns:
        list[str]: Filled as the writes happen.
    """
    landed: list[str] = []
    replace = os.replace

    def recording(src: Any, dst: Any) -> None:
        if Path(dst).parent == applied:
            landed.append(Path(dst).name)
        replace(src, dst)

    monkeypatch.setattr(os, "replace", recording)
    return landed
