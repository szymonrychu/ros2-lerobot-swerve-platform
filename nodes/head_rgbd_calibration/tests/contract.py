"""Structural match of a real API body against a literal JSON fixture under tests/fixtures/api/.

Keys must be equal at every dict level; values must have the fixture value's type (bool, int, float, str, None and
list/dict recursively). Values themselves are never compared, so stamps and measurements may differ. A float
fixture value accepts an int. A null fixture value requires None. Lists: every element must match the first fixture
element; an empty fixture list requires an empty list.
"""

import json
from pathlib import Path
from typing import Any

FIXTURE_DIR = Path(__file__).parent / "fixtures" / "api"


def load_fixture(name: str) -> Any:
    """Read tests/fixtures/api/<name>.json."""
    return json.loads((FIXTURE_DIR / f"{name}.json").read_text())


def structure_diff(actual: Any, expected: Any, path: str = "$") -> list[str]:
    """Differences in keys or value types between a body and a fixture.

    Args:
        actual: The real response body.
        expected: The fixture content.
        path: JSON path prefix used in the messages.

    Returns:
        One message per difference; empty when the structures match.
    """
    if expected is None:
        return [] if actual is None else [f"{path}: expected null, got {type(actual).__name__}"]
    if isinstance(expected, dict):
        if not isinstance(actual, dict):
            return [f"{path}: expected object, got {type(actual).__name__}"]
        diffs = [f"{path}: missing key {k}" for k in expected.keys() - actual.keys()]
        diffs += [f"{path}: unexpected key {k}" for k in actual.keys() - expected.keys()]
        for key in expected.keys() & actual.keys():
            diffs += structure_diff(actual[key], expected[key], f"{path}.{key}")
        return diffs
    if isinstance(expected, list):
        if not isinstance(actual, list):
            return [f"{path}: expected array, got {type(actual).__name__}"]
        if not expected:
            return [] if not actual else [f"{path}: expected empty array"]
        return [d for i, item in enumerate(actual) for d in structure_diff(item, expected[0], f"{path}[{i}]")]
    if isinstance(expected, bool):
        ok = isinstance(actual, bool)
    elif isinstance(expected, float):
        ok = isinstance(actual, int | float) and not isinstance(actual, bool)
    elif isinstance(expected, int):
        ok = isinstance(actual, int) and not isinstance(actual, bool)
    else:
        ok = isinstance(actual, type(expected))
    return [] if ok else [f"{path}: expected {type(expected).__name__}, got {type(actual).__name__}"]


def assert_matches_fixture(actual: Any, name: str) -> None:
    """Fail with every structural difference against fixtures/api/<name>.json."""
    diffs = structure_diff(actual, load_fixture(name))
    assert not diffs, f"{name}.json mismatch:\n" + "\n".join(diffs)
