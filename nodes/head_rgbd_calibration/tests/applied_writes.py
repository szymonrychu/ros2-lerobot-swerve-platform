"""Record the order in which the node's writers land files in applied/, and interleave two applies."""

import os
import threading
from collections.abc import Callable
from pathlib import Path
from types import ModuleType
from typing import Any

import pytest

from head_rgbd_calibration import apply

# Bound of every wait of interleave_apply; reached only when an apply hangs (the test then fails, never hangs).
INTERLEAVE_TIMEOUT_S = 60.0
# How long interleave_apply lets the concurrent apply run before the paused one resumes. Only an implementation whose
# check is outside the applied lock lets it land in that time; a correct one passes whatever this is.
SECOND_LANDS_S = 0.5


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


class WatchedLock:
    """A lock standing in for apply.APPLIED_LOCK that reports when the thread named `watched` starts to acquire it."""

    def __init__(self, watched: str) -> None:
        self.lock = threading.Lock()
        self.watched = watched
        self.waiting = threading.Event()

    def __enter__(self) -> bool:
        if threading.current_thread().name == self.watched:
            self.waiting.set()
        return self.lock.__enter__()

    def __exit__(self, *exc: object) -> None:
        self.lock.__exit__(*exc)


def interleave_apply(
    monkeypatch: pytest.MonkeyPatch, module: ModuleType, check: str, first: Callable[[], Any], second: Callable[[], Any]
) -> list[Any]:
    """Run `second` (an apply) while `first` (an apply) is paused right after its applied-state check `module.check`.

    `second` starts once `first` has made its check, and `first` resumes once `second` is trying to take the applied
    lock (plus SECOND_LANDS_S for an implementation whose check is outside the lock to let `second` land in between).
    With the check under the applied lock `second` waits for `first`, so `first` lands first whatever the timing.

    Args:
        monkeypatch: pytest monkeypatch (replaces apply.APPLIED_LOCK and module.check).
        module: Module whose `check` function `first` calls for its applied-state check.
        check: Name of that function.
        first: The apply that is paused after its check.
        second: The concurrent apply.

    Returns:
        list[Any]: [first's outcome, second's outcome]: the return value or the raised exception.
    """
    lock = WatchedLock("second")
    monkeypatch.setattr(apply, "APPLIED_LOCK", lock)
    checked, resume = threading.Event(), threading.Event()
    original = getattr(module, check)

    def paused(*args: Any, **kwargs: Any) -> Any:
        value = original(*args, **kwargs)
        if threading.current_thread().name == "first":
            checked.set()
            assert resume.wait(INTERLEAVE_TIMEOUT_S)
        return value

    monkeypatch.setattr(module, check, paused)
    outcomes: list[Any] = [None, None]

    def run(index: int, call: Callable[[], Any]) -> None:
        try:
            outcomes[index] = call()
        except Exception as exc:  # noqa: BLE001 - the outcome is asserted by the test
            outcomes[index] = exc

    threads = [threading.Thread(target=run, args=(i, call), name=name) for i, (name, call) in enumerate(
        (("first", first), ("second", second))
    )]  # fmt: skip
    threads[0].start()
    assert checked.wait(INTERLEAVE_TIMEOUT_S), f"{check} was not called"
    threads[1].start()
    assert lock.waiting.wait(INTERLEAVE_TIMEOUT_S), "the concurrent apply never took the applied lock"
    threads[1].join(SECOND_LANDS_S)
    resume.set()
    for thread in threads:
        thread.join(INTERLEAVE_TIMEOUT_S)
        assert not thread.is_alive()
    return outcomes
