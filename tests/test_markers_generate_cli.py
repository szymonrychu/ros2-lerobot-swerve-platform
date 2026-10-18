"""Tests for scripts/markers_generate.py: file sets, error exits and the shipped config end to end."""

from pathlib import Path

import numpy as np
import pypdfium2 as pdfium
import pytest
import yaml
import zxingcpp

from scripts.markers_generate import main
from shared.ros2_common.marker_payload import decode_payload

REPO_ROOT = Path(__file__).resolve().parent.parent
SHIPPED_YAML = REPO_ROOT / "scripts" / "markers_generate.yaml"
DPI = 300
MARKER_SUFFIXES = (".pdf", "-base.stl", "-modules.stl", ".3mf")


def write_config(tmp_path: Path, config: dict) -> Path:
    """Write config as YAML into tmp_path and return its path."""
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(config))
    return path


def names(directory: Path) -> set[str]:
    """File names in directory."""
    return {p.name for p in directory.iterdir()}


def test_two_markers_and_charuco_write_expected_files(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    out = tmp_path / "out"
    config = {
        "output_dir": str(out),
        "markers": [
            {"kind_code": "CHARGER", "name": "DOCK-1", "size_mm": 100},
            {"kind_code": "BASE", "name": "HOME", "size_mm": 100, "outputs": ["pdf"]},
        ],
        "charuco": {},
    }
    assert main([str(write_config(tmp_path, config))]) == 0
    expected = {f"charger-dock-1{s}" for s in MARKER_SUFFIXES} | {"base-home.pdf"}
    expected |= {f"charuco-6x5-40mm{s}" for s in MARKER_SUFFIXES}
    assert names(out) == expected
    printed = capsys.readouterr().out
    assert "charger-dock-1.pdf" in printed and "charuco-6x5-40mm.3mf" in printed


def test_unknown_key_returns_1_and_writes_nothing(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    out = tmp_path / "out"
    config = {"output_dir": str(out), "markers": [{"kind_code": "BASE", "name": "HOME", "size_mm": 100, "bogus": 1}]}
    assert main([str(write_config(tmp_path, config))]) == 1
    assert "bogus" in capsys.readouterr().err
    assert not out.exists() or not list(out.iterdir())


def test_too_small_stl_marker_fails_others_written(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    out = tmp_path / "out"
    config = {
        "output_dir": str(out),
        "markers": [
            {"kind_code": "BUTTON", "name": "TINY", "size_mm": 20, "outputs": ["stl"]},
            {"kind_code": "CHARGER", "name": "DOCK-1", "size_mm": 100},
        ],
    }
    assert main([str(write_config(tmp_path, config))]) == 1
    assert "TINY" in capsys.readouterr().err
    assert names(out) == {f"charger-dock-1{s}" for s in MARKER_SUFFIXES}


def test_relative_output_dir_resolves_against_repo_root(tmp_path: Path) -> None:
    rel = f".logs/test-markers-{tmp_path.name}"
    config = {"output_dir": rel, "markers": [{"kind_code": "BASE", "name": "HOME", "size_mm": 100, "outputs": ["pdf"]}]}
    target = REPO_ROOT / rel
    try:
        assert main([str(write_config(tmp_path, config))]) == 0
        assert (target / "base-home.pdf").exists()
    finally:
        for child in target.glob("*"):
            child.unlink()
        target.rmdir()


def test_shipped_config_runs_and_pdfs_decode(tmp_path: Path) -> None:
    config = yaml.safe_load(SHIPPED_YAML.read_text())
    out = tmp_path / "out"
    config["output_dir"] = str(out)
    assert main([str(write_config(tmp_path, config))]) == 0
    pdfs = sorted(p for p in out.glob("*.pdf") if not p.name.startswith("charuco"))
    assert len(pdfs) == len(config["markers"])
    for pdf_path in pdfs:
        page = pdfium.PdfDocument(str(pdf_path))[0]
        image = np.array(page.render(scale=DPI / 72).to_pil().convert("L"))
        texts = {r.text for r in zxingcpp.read_barcodes(image)}
        assert len(texts) == 1, pdf_path.name
        decoded = decode_payload(texts.pop())
        assert f"{decoded.kind_code}-{decoded.name}".lower() == pdf_path.stem
        stem = pdf_path.stem
        assert (out / f"{stem}-base.stl").exists() and (out / f"{stem}.3mf").exists()
    assert (out / "charuco-6x5-40mm.pdf").exists()
