"""The retired stereo colour path and stale names are gone from the frontend and the docs."""

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STALE = re.compile(r"color_small|colorSmall|ColorSmall|image_rect")


def test_frontend_sources_have_no_color_small() -> None:
    hits = [
        str(p.relative_to(ROOT))
        for p in (ROOT / "frontend" / "src").rglob("*")
        if p.is_file() and p.suffix in {".ts", ".tsx"} and STALE.search(p.read_text())
    ]
    assert hits == []


def test_depth_mesh_component_removed() -> None:
    assert not (ROOT / "frontend" / "src" / "components3d" / "DepthMesh.tsx").exists()


def test_readmes_have_no_stale_names_or_typos() -> None:
    text = (ROOT / "README.md").read_text()
    assert "[...]]" not in text
    assert "every topic Envelope" not in text
    assert "color_small" not in text
    tests_readme = (ROOT.parents[1] / "tests" / "README.md").read_text()
    assert "color_small_b64" not in tests_readme
    assert "image_rect" not in tests_readme


def test_msg_serializer_has_no_double_period() -> None:
    assert "dict.." not in (ROOT / "web_ui" / "msg_serializer.py").read_text()
