from pathlib import Path

import pytest

from grasp_sim.config import BoxObjectConfig, SceneConfig

REPO_ROOT = Path(__file__).resolve().parents[3]
URDF_PATH = REPO_ROOT / "nodes" / "web_ui" / "urdf" / "so101_arm.urdf"


@pytest.fixture
def urdf_path() -> Path:
    return URDF_PATH


@pytest.fixture
def floor_scene() -> SceneConfig:
    return SceneConfig(object=BoxObjectConfig(size_m=(0.03, 0.03, 0.04), x_m=0.2))
