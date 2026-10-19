"""The web_ui model downloads in Ansible group_vars point at the repo's models/ dir, which holds the printable files."""

from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
MODELS_DIR = REPO_ROOT / "models"


def web_ui_config() -> dict:
    """Parse the `config: |` block of the client web_ui entry."""
    nodes = yaml.safe_load(CLIENT_VARS.read_text())["ros2_nodes"]
    return yaml.safe_load(next(n for n in nodes if n["name"] == "web_ui")["config"])


def test_web_ui_serves_the_repo_models_dir() -> None:
    assert web_ui_config()["models_dir"] == "{{ ros2_repo_dest }}/models"


def test_models_dir_holds_a_pdf_and_3mf_per_model() -> None:
    pdfs = {p.stem for p in MODELS_DIR.glob("*.pdf")}
    three_mfs = {p.stem for p in MODELS_DIR.glob("*.3mf")}
    assert "charuco-6x5-40mm" in pdfs
    assert pdfs == three_mfs
    assert {p.suffix for p in MODELS_DIR.iterdir()} == {".pdf", ".3mf"}
