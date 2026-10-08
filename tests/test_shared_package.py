"""shared/ is an installable PEP 621 package (ros2-common) that nodes consume as a relative path dependency."""

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
    pyproject = load_pyproject(SHARED_DIR)
    assert pyproject["project"]["name"] == PACKAGE_NAME
    assert pyproject["build-system"]["build-backend"] == "hatchling.build"
    assert pyproject["tool"]["hatch"]["build"]["targets"]["wheel"]["packages"] == [MODULE_NAME]
    assert (SHARED_DIR / MODULE_NAME / "__init__.py").is_file()


def test_nodes_depend_on_shared_by_relative_develop_path() -> None:
    """Each consumer node points at the shared dir relative to itself; the layout is the same on the Pi.

    Ansible syncs the whole repo to ros2_repo_dest and installs inside <repo>/nodes/<node>, so the same
    relative path resolves there as in the checkout. Nodes declare it as a uv editable path source, or
    (until migrated) as a Poetry develop path dependency.
    """
    for node in CONSUMER_NODES:
        node_dir = REPO_ROOT / "nodes" / node
        tool = load_pyproject(node_dir)["tool"]
        if "poetry" in tool:
            dep = tool["poetry"]["dependencies"][PACKAGE_NAME]
            assert dep["develop"] is True
        else:
            dep = tool["uv"]["sources"][PACKAGE_NAME]
            assert dep["editable"] is True
        assert (node_dir / dep["path"]).resolve() == SHARED_DIR.resolve()
