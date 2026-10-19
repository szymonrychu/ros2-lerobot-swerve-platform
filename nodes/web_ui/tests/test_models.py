"""Tests for web_ui/models.py: the static download endpoint for printable model files (PDF, 3MF)."""

from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from web_ui.config import AppConfig
from web_ui.server import build_app

PDF = b"%PDF-1.4 board"
THREE_MF = b"PK\x03\x04 3mf"


@pytest.fixture
def models_dir(tmp_path: Path) -> Path:
    root = tmp_path / "models"
    root.mkdir()
    (root / "charuco-6x5-40mm.pdf").write_bytes(PDF)
    (root / "charuco-6x5-40mm.3mf").write_bytes(THREE_MF)
    (root / "charuco-6x5-40mm-base.stl").write_bytes(b"solid")
    (root / "notes.txt").write_text("not served")
    return root


def make_client(tmp_path: Path, models_dir: Path | None) -> TestClient:
    config = AppConfig() if models_dir is None else AppConfig(models_dir=str(models_dir))
    return TestClient(build_app(config, tmp_path / "urdf", tmp_path / "none"))


def test_models_index_lists_only_pdf_and_3mf(tmp_path: Path, models_dir: Path) -> None:
    response = make_client(tmp_path, models_dir).get("/api/models")
    assert response.status_code == 200
    assert response.json() == {
        "files": [
            {"name": "charuco-6x5-40mm.3mf", "size": len(THREE_MF), "url": "/models/charuco-6x5-40mm.3mf"},
            {"name": "charuco-6x5-40mm.pdf", "size": len(PDF), "url": "/models/charuco-6x5-40mm.pdf"},
        ]
    }


@pytest.mark.parametrize(
    ("name", "body", "media_type"),
    [("charuco-6x5-40mm.pdf", PDF, "application/pdf"), ("charuco-6x5-40mm.3mf", THREE_MF, "model/3mf")],
)
def test_model_file_is_served_as_a_download(
    tmp_path: Path, models_dir: Path, name: str, body: bytes, media_type: str
) -> None:
    response = make_client(tmp_path, models_dir).get(f"/models/{name}")
    assert response.status_code == 200
    assert response.content == body
    assert response.headers["content-type"].startswith(media_type)
    assert f'filename="{name}"' in response.headers["content-disposition"]


@pytest.mark.parametrize("name", ["charuco-6x5-40mm-base.stl", "notes.txt", "missing.pdf", "..%2Fsecret.pdf"])
def test_other_files_are_not_served(tmp_path: Path, models_dir: Path, name: str) -> None:
    (tmp_path / "secret.pdf").write_bytes(PDF)
    assert make_client(tmp_path, models_dir).get(f"/models/{name}").status_code == 404


def test_without_models_dir_the_endpoints_answer_404(tmp_path: Path) -> None:
    client = make_client(tmp_path, None)
    assert client.get("/api/models").status_code == 404
    assert client.get("/models/charuco-6x5-40mm.pdf").status_code == 404
