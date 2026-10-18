"""Retirement pin for the Jetson coprocessor: removed paths stay gone and removed tokens stay absent.

Later tasks of the retirement epic extend only REMOVED_PATHS and ABSENT_TOKENS.
"""

from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent

REMOVED_PATHS: list[str] = [
    "jetson",
    "scripts/jetson_e2e_check.py",
    "scripts/jetson_e2e_check.yaml",
    "tests/test_jetson_e2e_check.py",
]

ABSENT_TOKENS: list[tuple[str, str]] = [
    ("scripts/README.md", "jetson_e2e_check"),
    ("ansible/playbooks/optimize.yml", "jetson"),
]


@pytest.mark.parametrize("path", REMOVED_PATHS)
def test_removed_path_is_gone(path: str) -> None:
    """Assert a retired repo-relative path does not exist.

    Args:
        path: Repo-relative path that must not exist.
    """
    assert not (REPO_ROOT / path).exists(), f"{path} must stay deleted"


@pytest.mark.parametrize(("path", "token"), ABSENT_TOKENS)
def test_removed_token_is_absent(path: str, token: str) -> None:
    """Assert a retired token does not appear in a file (case-sensitive).

    Args:
        path: Repo-relative file to scan.
        token: Text that must not occur in it.
    """
    assert token not in (REPO_ROOT / path).read_text(), f"{path} still mentions {token!r}"
