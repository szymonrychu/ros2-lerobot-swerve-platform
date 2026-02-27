"""Unit tests for shared library (extend tests as project grows)."""

import sys
from pathlib import Path

_repo_root = Path(__file__).resolve().parent.parent
if (_repo_root / "shared" / "ros2_common").exists():
    sys.path.insert(0, str(_repo_root))

from shared.ros2_common._utils import clamp  # noqa: E402


def test_clamp_within() -> None:
    """clamp returns value when within range."""
    assert clamp(0.5, 0.0, 1.0) == 0.5


def test_clamp_low() -> None:
    """clamp returns low when value below range."""
    assert clamp(-0.5, 0.0, 1.0) == 0.0


def test_clamp_high() -> None:
    """clamp returns high when value above range."""
    assert clamp(1.5, 0.0, 1.0) == 1.0


def test_clamp_equal_bounds() -> None:
    """clamp with low == high returns that bound."""
    assert clamp(0.5, 1.0, 1.0) == 1.0
    assert clamp(1.0, 1.0, 1.0) == 1.0


def test_clamp_at_bounds() -> None:
    """clamp returns value when value equals low or high."""
    assert clamp(0.0, 0.0, 1.0) == 0.0
    assert clamp(1.0, 0.0, 1.0) == 1.0
