"""Localhost HTTP API of the head_rgbd_calibration node (design D1; web_ui proxies /api/calibration/rgbd/* to it).

The /api/capture, /api/sessions and /api/applied routes serve the D435 intrinsics check. Every error answers
{ok: false, message}: InvalidId and NotFound give 404, Busy and NotApplicable give 409, a session of another kind 400.
"""

import json
import subprocess
from collections.abc import Callable
from datetime import datetime
from typing import Any

from fastapi import FastAPI, Request
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse, Response
from pydantic import BaseModel
from starlette.exceptions import HTTPException as StarletteHTTPException

from head_rgbd_calibration.apply import (
    APPLIED_DIR,
    APPLIED_MARKER,
    NotApplicable,
    applied_state,
    apply_result,
    download_zip,
)
from head_rgbd_calibration.capture import Busy as CaptureBusy
from head_rgbd_calibration.capture import CaptureController, CaptureError
from head_rgbd_calibration.compute import REPORT_FILE
from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.jobs import Busy as JobBusy
from head_rgbd_calibration.jobs import ComputeJob
from head_rgbd_calibration.store import KIND_INTRINSICS, POSE_FILE, RESULT_FILE, InvalidId, NotFound, SessionStore

JPEG_MEDIA_TYPE = "image/jpeg"
ZIP_MEDIA_TYPE = "application/zip"
HTTP_BAD_REQUEST = 400
HTTP_NOT_FOUND = 404
HTTP_CONFLICT = 409
HTTP_ACCEPTED = 202
HTTP_UNPROCESSABLE = 422

RunFn = Callable[..., subprocess.CompletedProcess]


class StartRequest(BaseModel):
    """Body of POST /api/capture/start."""

    session_id: str | None = None


def error_response(status: int, message: str) -> JSONResponse:
    """The common error body.

    Args:
        status: HTTP status code.
        message: Operator-readable reason.

    Returns:
        A JSON response {ok: false, message}.
    """
    return JSONResponse(status_code=status, content={"ok": False, "message": message})


def read_applied_marker(store: SessionStore) -> dict[str, Any] | None:
    """applied.json content, None when nothing is applied (no systemctl call)."""
    marker = store.state_dir / APPLIED_DIR / APPLIED_MARKER
    return json.loads(marker.read_text()) if marker.is_file() else None


def build_app(capture: CaptureController, job: ComputeJob, store: SessionStore, cfg: NodeConfig, run: RunFn) -> FastAPI:
    """Create the node API.

    Args:
        capture: Capture controller (state, start, stop).
        job: Compute job runner (state, start).
        store: Session store.
        cfg: Node config (limits.min_views, restart_units).
        run: subprocess.run-compatible callable used for systemctl show on GET /api/applied only.

    Returns:
        The FastAPI app with the routes of the design's Node HTTP API table.
    """
    app = FastAPI(title="head_rgbd_calibration")

    @app.exception_handler(InvalidId)
    @app.exception_handler(NotFound)
    async def not_found(_: Request, exc: Exception) -> JSONResponse:
        return error_response(HTTP_NOT_FOUND, str(exc))

    @app.exception_handler(CaptureBusy)
    @app.exception_handler(JobBusy)
    @app.exception_handler(NotApplicable)
    async def conflict(_: Request, exc: Exception) -> JSONResponse:
        return error_response(HTTP_CONFLICT, str(exc))

    @app.exception_handler(CaptureError)
    async def capture_failed(_: Request, exc: Exception) -> JSONResponse:
        return error_response(HTTP_BAD_REQUEST, str(exc))

    @app.exception_handler(StarletteHTTPException)
    async def http_error(_: Request, exc: StarletteHTTPException) -> JSONResponse:
        return error_response(exc.status_code, str(exc.detail))

    @app.exception_handler(RequestValidationError)
    async def invalid_request(_: Request, exc: RequestValidationError) -> JSONResponse:
        problems = "; ".join(f"{'.'.join(str(p) for p in e['loc'])}: {e['msg']}" for e in exc.errors())
        return error_response(HTTP_UNPROCESSABLE, problems)

    def full_state() -> dict[str, Any]:
        return {"capture": capture.state(), "compute": job.state(), "applied": read_applied_marker(store)}

    @app.get("/api/state")
    def state() -> dict[str, Any]:
        return full_state()

    @app.post("/api/capture/start")
    def capture_start(body: StartRequest | None = None) -> Any:
        if job.state()["running"]:
            return error_response(HTTP_CONFLICT, "a compute is running; wait for it to finish")
        capture.start(body.session_id if body else None)
        return full_state()

    @app.post("/api/capture/stop")
    def capture_stop() -> dict[str, Any]:
        capture.stop()
        return full_state()

    @app.get("/api/sessions")
    def sessions() -> list[dict[str, Any]]:
        return [
            {key: s[key] for key in ("id", "created", "pose_count", "result_count")}
            for s in store.list_sessions()
            if s["kind"] == KIND_INTRINSICS
        ]

    @app.get("/api/sessions/{sid}")
    def session(sid: str) -> Any:
        data = store.load_session(sid)
        if data["kind"] != KIND_INTRINSICS:
            return error_response(HTTP_BAD_REQUEST, f"{sid} is a {data['kind']} session, not {KIND_INTRINSICS}")
        poses = []
        for pid in data["pose_ids"]:
            meta = json.loads((store.pose_dir(sid, pid) / POSE_FILE).read_text())
            poses.append(
                {
                    "id": pid,
                    "rgb_corners": len(meta["rgb"]["points"]),
                    "plane_bias_samples": meta["plane_bias_samples"],
                    "hints": store.pose_hints(sid, pid),
                }
            )
        return {
            "id": data["id"],
            "created": data["created"],
            "poses": poses,
            "coverage": store.coverage(sid),
            "results": store.list_results(sid),
        }

    @app.delete("/api/sessions/{sid}/poses/{pid}")
    def delete_pose(sid: str, pid: str) -> dict[str, Any]:
        store.delete_pose(sid, pid)
        return {"ok": True}

    @app.get("/api/sessions/{sid}/poses/{pid}/{kind}.jpg")
    def thumbnail(sid: str, pid: str, kind: str) -> Response:
        return Response(content=store.thumbnail_path(sid, pid, kind).read_bytes(), media_type=JPEG_MEDIA_TYPE)

    @app.post("/api/sessions/{sid}/compute", status_code=HTTP_ACCEPTED)
    def compute(sid: str) -> Any:
        store.existing_session_dir(sid)
        if capture.state()["running"]:
            return error_response(HTTP_CONFLICT, "a capture is running; stop it before computing")
        have, need = len(store.pose_ids(sid)), cfg.limits.min_views
        if have < need:
            return error_response(HTTP_CONFLICT, f"session has {have} poses, at least {need} are required")
        return {"result_id": job.start(sid)}

    @app.get("/api/sessions/{sid}/results/{rid}")
    def result(sid: str, rid: str) -> dict[str, Any]:
        path = store.result_dir(sid, rid)
        if not (path / RESULT_FILE).is_file():
            raise NotFound(f"no such result: {rid}")
        report = path / REPORT_FILE
        return {
            **json.loads((path / RESULT_FILE).read_text()),
            "report": report.read_text() if report.is_file() else None,
        }

    @app.get("/api/sessions/{sid}/results/{rid}/download")
    def download(sid: str, rid: str) -> Response:
        if not (store.result_dir(sid, rid) / RESULT_FILE).is_file():
            raise NotFound(f"no such result: {rid}")
        name, data = download_zip(store, sid, rid)
        return Response(
            content=data, media_type=ZIP_MEDIA_TYPE, headers={"Content-Disposition": f'attachment; filename="{name}"'}
        )

    @app.post("/api/sessions/{sid}/results/{rid}/apply")
    def apply(sid: str, rid: str) -> Any:
        if job.state()["running"]:
            return error_response(HTTP_CONFLICT, "a compute is running; wait for it to finish")
        if not (store.result_dir(sid, rid) / RESULT_FILE).is_file():
            raise NotFound(f"no such result: {rid}")
        apply_result(store, sid, rid, datetime.now().astimezone())
        return applied_state(store, cfg.restart_units, run)

    @app.get("/api/applied")
    def applied() -> Any:
        found = applied_state(store, cfg.restart_units, run)
        if found is None:
            return error_response(HTTP_NOT_FOUND, "no calibration is applied")
        return found

    return app
