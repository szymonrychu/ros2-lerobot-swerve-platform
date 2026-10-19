"""Static downloads of printable model files: GET /api/models lists them, GET /models/{name} serves one.

Only the PDF and 3MF files directly inside config.models_dir are offered (the repo's models/ dir on the robot); the
files are served as they are, nothing is generated. Without models_dir both routes answer 404.
"""

from __future__ import annotations

from pathlib import Path

from fastapi import FastAPI
from fastapi.responses import FileResponse, JSONResponse, Response

from .config import AppConfig

MODEL_MEDIA_TYPES = {".pdf": "application/pdf", ".3mf": "model/3mf"}
MODELS_URL_PREFIX = "/models"
NOT_FOUND_STATUS = 404


def model_files(models_dir: Path) -> dict[str, Path]:
    """Offered model files by name: the PDF and 3MF files directly inside models_dir.

    Args:
        models_dir: Directory holding the model files.

    Returns:
        dict[str, Path]: File name -> path, sorted by name; empty when the directory is missing.
    """
    if not models_dir.is_dir():
        return {}
    files = (p for p in models_dir.iterdir() if p.is_file() and p.suffix.lower() in MODEL_MEDIA_TYPES)
    return {p.name: p for p in sorted(files, key=lambda p: p.name)}


def register_model_routes(app: FastAPI, config: AppConfig) -> None:
    """Add GET /api/models and GET /models/{name} (404 for anything not offered, or when models_dir is unset).

    Args:
        app: FastAPI application (routes must be added before the SPA mount).
        config: Application config; models_dir enables the routes.
    """
    models_dir = Path(config.models_dir) if config.models_dir else None

    def index() -> Response:
        if models_dir is None:
            return JSONResponse({"ok": False, "message": "models not configured"}, status_code=NOT_FOUND_STATUS)
        files = [
            {"name": name, "size": path.stat().st_size, "url": f"{MODELS_URL_PREFIX}/{name}"}
            for name, path in model_files(models_dir).items()
        ]
        return JSONResponse({"files": files})

    def download(name: str) -> Response:
        path = model_files(models_dir).get(name) if models_dir is not None else None
        if path is None:
            return JSONResponse({"ok": False, "message": "no such model file"}, status_code=NOT_FOUND_STATUS)
        return FileResponse(path, media_type=MODEL_MEDIA_TYPES[path.suffix.lower()], filename=name)

    app.add_api_route("/api/models", index, methods=["GET"], include_in_schema=False)
    app.add_api_route(f"{MODELS_URL_PREFIX}/{{name}}", download, methods=["GET"], include_in_schema=False)
