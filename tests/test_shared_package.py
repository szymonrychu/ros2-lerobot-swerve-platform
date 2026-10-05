"""shared/ is an installable Poetry package (ros2-common) that nodes consume as a relative path dependency."""

import tomllib
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SHARED_DIR = REPO_ROOT / "shared"
PACKAGE_NAME = "ros2-common"
MODULE_NAME = "ros2_common"
CONSUMER_NODES = ("mcp_server",)


def load_pyproject(path: Path) -> dict:
    """Parse a pyproject.toml.

    Args:
        path: Directory holding pyproject.toml.

    Returns:
        dict: Parsed TOML.
    """
    return tomllib.loads((path / "pyproject.toml").read_text())


def test_shared_pyproject_defines_ros2_common() -> None:
    """shared/pyproject.toml names the package ros2-common and ships the ros2_common module."""
    poetry = load_pyproject(SHARED_DIR)["tool"]["poetry"]
    assert poetry["name"] == PACKAGE_NAME
    assert {"include": MODULE_NAME} in poetry["packages"]
    assert (SHARED_DIR / MODULE_NAME / "__init__.py").is_file()


def test_nodes_depend_on_shared_by_relative_develop_path() -> None:
    """Each consumer node points at the shared dir relative to itself; the layout is the same on the Pi.

    Ansible syncs the whole repo to ros2_repo_dest and runs poetry inside <repo>/nodes/<node>, so the same
    relative path resolves there as in the checkout.
    """
    for node in CONSUMER_NODES:
        node_dir = REPO_ROOT / "nodes" / node
        dep = load_pyproject(node_dir)["tool"]["poetry"]["dependencies"][PACKAGE_NAME]
        assert dep["develop"] is True
        assert (node_dir / dep["path"]).resolve() == SHARED_DIR.resolve()
