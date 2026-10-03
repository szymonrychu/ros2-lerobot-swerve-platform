"""Guard: pure logic stays rclpy-free; only ros_iface.py and __main__.py import ROS packages."""

import ast
from pathlib import Path

PACKAGE = Path(__file__).resolve().parent.parent / "mcp_server"
ROS_ROOTS = {
    "rclpy",
    "tf2_ros",
    "sensor_msgs",
    "nav_msgs",
    "geometry_msgs",
    "std_msgs",
    "std_srvs",
    "nav2_msgs",
    "action_msgs",
}
ROS_MODULES = {"ros_iface.py", "__main__.py"}


def imported_roots(path: Path) -> set[str]:
    """Top-level package names imported by a module.

    Args:
        path (Path): Python file.

    Returns:
        set[str]: Imported root packages (absolute imports only).
    """
    roots: set[str] = set()
    for node in ast.walk(ast.parse(path.read_text())):
        if isinstance(node, ast.Import):
            roots |= {alias.name.split(".")[0] for alias in node.names}
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            roots.add(node.module.split(".")[0])
    return roots


def test_pure_modules_do_not_import_ros() -> None:
    for path in PACKAGE.glob("*.py"):
        if path.name not in ROS_MODULES:
            assert not imported_roots(path) & ROS_ROOTS, path.name


def test_ros_modules_parse_and_use_rclpy() -> None:
    for name in ROS_MODULES:
        assert "rclpy" in imported_roots(PACKAGE / name), name
