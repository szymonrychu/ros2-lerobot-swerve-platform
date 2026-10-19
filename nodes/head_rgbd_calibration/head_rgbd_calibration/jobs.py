"""One compute child at a time: niced, time limited, with its stderr tail reported on failure."""

import logging
import subprocess
import sys
import tempfile
import threading
from collections.abc import Callable
from datetime import datetime, timedelta
from typing import IO, Any

from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.store import RESULT_FILE, SessionStore

LOGGER = logging.getLogger(__name__)

STDERR_TAIL_LINES = 20
WATCH_INTERVAL_S = 0.2
KILL_WAIT_S = 5.0
STOP_JOIN_S = 10.0
MODULE = "head_rgbd_calibration.compute"


class Busy(RuntimeError):
    """A computation is already running (maps to 409)."""


class ComputeJob:
    """Runs `python -m head_rgbd_calibration.compute` as a niced child, one at a time."""

    def __init__(
        self,
        cfg: NodeConfig,
        store: SessionStore,
        popen: Callable[..., subprocess.Popen],
        clock: Callable[[], datetime],
        config_path: str,
    ) -> None:
        """Create a job runner.

        Args:
            cfg: Node config (compute_timeout_s, compute_nice).
            store: Session store (result directories).
            popen: subprocess.Popen or a stand-in with the same call signature.
            clock: Returns the current time.
            config_path: Node config path handed to the child (HEAD_RGBD_CALIBRATION_CONFIG).
        """
        self.cfg = cfg
        self.store = store
        self.popen = popen
        self.clock = clock
        self.config_path = config_path
        self.lock = threading.RLock()
        self.child: subprocess.Popen | None = None
        self.stderr_file: IO[bytes] | None = None
        self.session_id: str | None = None
        self.result_id: str | None = None
        self.started_at: datetime | None = None
        self.error: str | None = None
        self.watcher: threading.Thread | None = None

    def start(self, sid: str) -> str:
        """Start computing a session.

        Args:
            sid: Session id.

        Returns:
            The new result id.

        Raises:
            Busy: a computation is running.
            NotFound / InvalidId: the session does not exist or the id is invalid.
        """
        with self.lock:
            self.refresh()
            if self.child is not None:
                raise Busy("a calibration is already being computed")
            self.store.existing_session_dir(sid)
            started = self.clock()
            rid = self.store.new_result_id(started)
            while (self.store.result_dir(sid, rid)).exists():
                started += timedelta(seconds=1)
                rid = self.store.new_result_id(started)
            command = [
                "nice",
                "-n",
                str(self.cfg.compute_nice),
                sys.executable,
                "-m",
                MODULE,
                "--config",
                self.config_path,
                "--session",
                sid,
                "--result",
                rid,
            ]
            self.stderr_file = tempfile.TemporaryFile()
            self.child = self.popen(command, stdout=subprocess.DEVNULL, stderr=self.stderr_file)
            self.session_id, self.result_id, self.started_at, self.error = sid, rid, self.clock(), None
            self.watcher = threading.Thread(target=self.watch, name="compute-watch", daemon=True)
            self.watcher.start()
            return rid

    def watch(self) -> None:
        """Poll the child until it is gone."""
        while True:
            with self.lock:
                self.refresh()
                if self.child is None:
                    return
            threading.Event().wait(WATCH_INTERVAL_S)

    def refresh(self) -> None:
        """Reap a finished child, or kill one past compute_timeout_s, and record the outcome."""
        with self.lock:
            child = self.child
            if child is None:
                return
            code = child.poll()
            if code is None:
                elapsed = (self.clock() - self.started_at).total_seconds()
                if elapsed <= self.cfg.compute_timeout_s:
                    return
                child.kill()
                child.wait(KILL_WAIT_S)
                self.finish(f"timed out after {self.cfg.compute_timeout_s:g} s")
            elif (self.store.result_dir(self.session_id, self.result_id) / RESULT_FILE).is_file():
                self.finish(None)
            else:
                self.finish(self.stderr_tail() or f"compute exited with code {code} and wrote no result")

    def stderr_tail(self) -> str:
        """The last STDERR_TAIL_LINES lines the child wrote to stderr."""
        self.stderr_file.seek(0)
        lines = self.stderr_file.read().decode(errors="replace").splitlines()
        return "\n".join(lines[-STDERR_TAIL_LINES:])

    def finish(self, error: str | None) -> None:
        """Record the end of the child."""
        self.error = error
        self.child = None
        self.stderr_file.close()
        self.stderr_file = None
        if error:
            LOGGER.warning("compute of %s failed: %s", self.session_id, error)

    def state(self) -> dict[str, Any]:
        """{running, session_id, result_id, started_at, error} of the current or last computation."""
        with self.lock:
            self.refresh()
            return {
                "running": self.child is not None,
                "session_id": self.session_id,
                "result_id": self.result_id,
                "started_at": None if self.started_at is None else self.started_at.isoformat(),
                "error": self.error,
            }

    def stop(self) -> None:
        """Kill a running child (node shutdown)."""
        with self.lock:
            if self.child is not None:
                self.child.kill()
                self.child.wait(KILL_WAIT_S)
                self.finish("stopped")
        if self.watcher is not None:
            self.watcher.join(STOP_JOIN_S)
