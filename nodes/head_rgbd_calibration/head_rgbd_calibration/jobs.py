"""One compute child at a time: niced, time limited, with its stderr tail reported on failure.

Telemetry: the child logs to its stderr in CHILD_LOG_FORMAT (compute.py); when it ends, every such line is re-logged in
this process under its logger and level, so the child's calib_event lines reach the journal with its run id (the
result id), and the other lines (a traceback) at WARNING. The job logs `child` start and end events of flow
`d435_compute` (exit code, duration, error).
"""

import logging
import subprocess
import sys
import tempfile
import threading
import time
from collections.abc import Callable
from datetime import datetime, timedelta
from typing import IO, Any

from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.store import RESULT_FILE, SessionStore
from head_rgbd_calibration.telemetry import calib_event, forward_child_log

LOGGER = logging.getLogger(__name__)

STDERR_TAIL_LINES = 20
WATCH_INTERVAL_S = 0.2
KILL_WAIT_S = 5.0
STOP_JOIN_S = 10.0
MODULE = "head_rgbd_calibration.compute"
FLOW = "d435_compute"
CHILD_STEP = "child"


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
        self.began_s = 0.0

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
            started_info = {
                "session_id": sid,
                "poses": len(self.store.pose_ids(sid)),
                "command": command,
                "timeout_s": self.cfg.compute_timeout_s,
            }
            try:
                self.child = self.popen(command, stdout=subprocess.DEVNULL, stderr=self.stderr_file)
            except Exception as exc:
                self.stderr_file.close()
                self.stderr_file = None
                calib_event(LOGGER, FLOW, rid, CHILD_STEP, "error", logging.ERROR, str(exc), **started_info)
                raise
            self.began_s = time.monotonic()
            calib_event(LOGGER, FLOW, rid, CHILD_STEP, "start", pid=self.child.pid, **started_info)
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
                self.finish(f"timed out after {self.cfg.compute_timeout_s:g} s", child.returncode)
            elif (self.store.result_dir(self.session_id, self.result_id) / RESULT_FILE).is_file():
                self.finish(None, code)
            else:
                self.finish(None, code, crashed=True)

    def finish(self, error: str | None, code: int | None, crashed: bool = False) -> None:
        """Record the end of the child: forward its stderr to the log and log the end event.

        Args:
            error (str | None): Why it failed (timeout, stop), None when it ended on its own.
            code (int | None): Its exit code, None when unknown.
            crashed (bool): It exited without a result.json: the error is the last STDERR_TAIL_LINES stderr lines
                that are not log lines, else the exit code and the last log line.
        """
        self.stderr_file.seek(0)
        text = self.stderr_file.read().decode(errors="replace")
        other = forward_child_log(text, LOGGER, f"compute {self.result_id}")
        if crashed:
            last = text.strip().splitlines()[-1:] if text.strip() else []
            ended = f"compute exited with code {code} and wrote no result"
            error = "\n".join(other[-STDERR_TAIL_LINES:]) or "; ".join([ended, *last])
        self.error = error
        self.child = None
        self.stderr_file.close()
        self.stderr_file = None
        calib_event(
            LOGGER,
            FLOW,
            self.result_id,
            CHILD_STEP,
            "error" if error else "ok",
            logging.ERROR if error else logging.INFO,
            error,
            time.monotonic() - self.began_s,
            session_id=self.session_id,
            exit_code=code,
        )

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
                self.finish("stopped", self.child.returncode)
        if self.watcher is not None:
            self.watcher.join(STOP_JOIN_S)
