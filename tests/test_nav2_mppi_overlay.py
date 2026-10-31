"""Static checks of the Nav2 MPPI overlay wiring (patched nav2_mppi_controller built into /opt/ros2-ws).

No ROS or Ansible run is needed: YAML via yaml.safe_load, the patch and probe script as text. They pin the overlay
source, the gate and rollback flag, the apt build dependencies and the generic colcon_source fields the overlay needs
(stamp_apt_packages, jobs) without changing the rf2o/libcamera/camera_ros stamp keys.
"""

import re
from pathlib import Path
from typing import Any

import jinja2
import yaml

from tests.test_cpp_node_deploy import package_xml_apt_names

REPO_ROOT = Path(__file__).resolve().parent.parent
ANSIBLE_DIR = REPO_ROOT / "ansible"
CLIENT_VARS = ANSIBLE_DIR / "group_vars" / "client.yml"
PACKAGE_TASKS = ANSIBLE_DIR / "roles" / "ros2_node_deploy" / "tasks" / "colcon_source_package.yml"
LAUNCHER = ANSIBLE_DIR / "roles" / "ros2_node_deploy" / "templates" / "ros2-node-launcher.j2"
PATCH = REPO_ROOT / "nodes" / "nav2_bringup" / "patches" / "nav2_mppi_controller-0001-model-delay-pr6154.patch"
PROBE = REPO_ROOT / "scripts" / "probe-mppi-overlay-docker.sh"
ROLE_MAIN = ANSIBLE_DIR / "roles" / "ros2_node_deploy" / "tasks" / "main.yml"
ANSIBLE_README = ANSIBLE_DIR / "README.md"
WORKSPACE = "/opt/ros2-ws"
ARM_WAIT_DIR = "nodes/arm_drive_wait_bt_cpp"
ARM_WAIT_SOURCE = {
    "local": ARM_WAIT_DIR,
    "package": "arm_drive_wait_bt_cpp",
    "workspace": WORKSPACE,
    "cmake_args": ["-DCMAKE_BUILD_TYPE=Release", "-DBUILD_TESTING=OFF"],
    "base_paths": ["."],
    "jobs": 2,
    "stamp_apt_packages": [
        "ros-jazzy-nav2-behavior-tree",
        "ros-jazzy-behaviortree-cpp",
        "ros-jazzy-rclcpp",
        "ros-jazzy-std-msgs",
        "nlohmann-json3-dev",
    ],
}
MPPI_PURGE = [{"package": "nav2_mppi_controller", "workspace": WORKSPACE}]

PIN = "f4108e5b1c2bce804a1aa0c7be6673a8eb4a1501"
REQUIRED_APT = {
    "git",
    "build-essential",
    "cmake",
    "python3-colcon-common-extensions",
    "ros-jazzy-nav2-mppi-controller",
    "libxtensor-dev",
    "libxsimd-dev",
    "libbenchmark-dev",
    "libomp-dev",
    "ros-jazzy-ament-cmake-ros",
    "ros-jazzy-nav2-common",
    "ros-jazzy-tf2-eigen",
}


def client_vars() -> dict[str, Any]:
    """Load group_vars/client.yml.

    Returns:
        dict[str, Any]: Parsed YAML.
    """
    return yaml.safe_load(CLIENT_VARS.read_text())


def package_tasks() -> list[dict[str, Any]]:
    """Load the per-source colcon build tasks.

    Returns:
        list[dict[str, Any]]: Parsed task list.
    """
    return yaml.safe_load(PACKAGE_TASKS.read_text())


def render(expression: str, overlay: bool) -> Any:
    """Evaluate a group_vars Jinja expression the way Ansible does, with the rollback flag set.

    Args:
        expression (str): A "{{ ... }}" value from client.yml.
        overlay (bool): The nav2_mppi_overlay value.

    Returns:
        Any: The evaluated value (lists and mappings stay native).
    """
    env = jinja2.Environment()
    env.filters["bool"] = lambda value: value if isinstance(value, bool) else str(value).lower() in ("true", "yes", "1")
    inner = expression.strip().removeprefix("{{").removesuffix("}}")
    return env.compile_expression(inner)(**{**client_vars(), "nav2_mppi_overlay": overlay})


def nav2_type() -> dict[str, Any]:
    """The nav2_bringup node type of client.yml.

    Returns:
        dict[str, Any]: The node type.
    """
    return client_vars()["ros2_node_type_defaults"]["nav2_bringup"]


def test_overlay_source_is_pinned_to_the_apt_tag_with_the_patch() -> None:
    assert client_vars()["nav2_mppi_overlay"] is True
    source = client_vars()["nav2_mppi_overlay_source"]
    assert source == {
        "repo": "https://github.com/ros-navigation/navigation2.git",
        "commit": PIN,
        "package": "nav2_mppi_controller",
        "workspace": "/opt/ros2-ws",
        "base_paths": ["nav2_mppi_controller"],
        "patches": [
            "{{ ros2_repo_dest }}/nodes/nav2_bringup/patches/nav2_mppi_controller-0001-model-delay-pr6154.patch"
        ],
        "cmake_args": ["-DCMAKE_BUILD_TYPE=Release", "-DBUILD_TESTING=OFF"],
        "stamp_apt_packages": ["ros-jazzy-nav2-mppi-controller"],
        "jobs": 2,
    }
    assert re.fullmatch(r"[0-9a-f]{40}", source["commit"])
    assert "tag 1.3.13 = apt version" in CLIENT_VARS.read_text()


def test_overlay_patch_exists_and_only_touches_the_mppi_package() -> None:
    assert PATCH.is_file()
    files = re.findall(r"^(?:---|\+\+\+) [ab]/(\S+)", PATCH.read_text(), re.MULTILINE)
    assert files
    assert all(f.startswith("nav2_mppi_controller/") for f in files)


def test_probe_pin_equals_the_client_commit() -> None:
    pin = re.search(r'^PIN="([0-9a-f]{40})"', PROBE.read_text(), re.MULTILINE)
    assert pin is not None
    assert pin.group(1) == client_vars()["nav2_mppi_overlay_source"]["commit"]


def test_nav2_colcon_source_builds_the_plugin_whatever_the_rollback_flag() -> None:
    """The WaitForArmDriveReady plugin builds whatever the flag; the MPPI overlay only with nav2_mppi_overlay. Both
    sources use /opt/ros2-ws, the workspace the launcher sources (the first source's). The overlay comes first: a
    later source's stamp chains the earlier ones (_colcon_chain_key), so a plugin change rebuilds only the small
    plugin, never the xtensor MPPI build that stops every node."""
    expression = nav2_type()["colcon_source"]
    overlay = client_vars()["nav2_mppi_overlay_source"]
    assert render(expression, True) == [overlay, ARM_WAIT_SOURCE]
    assert render(expression, False) == [ARM_WAIT_SOURCE]
    for flag in (True, False):
        assert render(expression, flag)[0]["workspace"] == WORKSPACE


def test_arm_drive_wait_bt_source_is_the_local_plugin_package() -> None:
    assert client_vars()["arm_drive_wait_bt_source"] == ARM_WAIT_SOURCE
    package_xml = REPO_ROOT / ARM_WAIT_DIR / "package.xml"
    assert f"<name>{ARM_WAIT_SOURCE['package']}</name>" in package_xml.read_text()
    assert set(ARM_WAIT_SOURCE["stamp_apt_packages"]) <= set(nav2_type()["apt_packages"])


def test_nav2_apt_packages_cover_the_plugin_package_xml() -> None:
    """apt_packages installs ros-jazzy-nav2-behavior-tree, ros-jazzy-behaviortree-cpp and every other build or run
    dependency of the plugin's package.xml."""
    apt = set(nav2_type()["apt_packages"])
    assert {"ros-jazzy-nav2-behavior-tree", "ros-jazzy-behaviortree-cpp", "rsync"} <= apt
    needed = package_xml_apt_names(REPO_ROOT / ARM_WAIT_DIR / "package.xml", {ARM_WAIT_SOURCE["package"]})
    assert needed <= apt, sorted(needed - apt)


def test_nav2_rollback_purges_the_overlay() -> None:
    """With the flag off the launcher still sources /opt/ros2-ws (the plugin), so the overlay is purged from it."""
    expression = nav2_type()["colcon_purge"]
    assert render(expression, True) == []
    assert render(expression, False) == MPPI_PURGE


def test_role_runs_colcon_purge_on_the_deploy_path_before_the_build() -> None:
    tasks = yaml.safe_load(ROLE_MAIN.read_text())
    block = next(t for t in tasks if t.get("name") == "Deploy ROS2 node natively (deploy_mode == native)")["block"]
    names = [t.get("name") for t in block]
    include = next(t for t in block if "colcon_purge.yml" in str(t.get("ansible.builtin.include_tasks", "")))
    assert include["ansible.builtin.include_tasks"]["apply"]["become"] is True
    assert "node_colcon_purge" in include["when"] and "length > 0" in include["when"]
    assert names.index(include["name"]) < names.index("Build the source workspace (colcon, pinned or local)")


def test_ansible_readme_describes_the_plugin_build_and_the_rollback_purge() -> None:
    text = ANSIBLE_README.read_text()
    for needle in ("arm_drive_wait_bt_source", "arm_drive_wait_bt_cpp", "colcon_purge", "nav2_mppi_overlay"):
        assert needle in text, needle


def test_nav2_apt_packages_carry_the_overlay_build_dependencies() -> None:
    nav2 = client_vars()["ros2_node_type_defaults"]["nav2_bringup"]
    assert REQUIRED_APT <= set(nav2["apt_packages"])


def test_apt_version_feeds_the_stamp_key_only_for_a_non_empty_list() -> None:
    tasks = package_tasks()
    query = next(t for t in tasks if "dpkg-query" in str(t.get("ansible.builtin.command", "")))
    assert "dpkg-query -W -f='${Version}' {{ item }}" in query["ansible.builtin.command"]
    assert query["changed_when"] is False
    assert query["check_mode"] is False
    assert query["loop"] == "{{ colcon_src.stamp_apt_packages | default([]) }}"
    key = next(t for t in tasks if "_colcon_patch_key" in str(t.get("ansible.builtin.set_fact", "")))
    expr = key["ansible.builtin.set_fact"]["_colcon_patch_key"]
    assert "stamp_apt_packages" in expr and "length > 0" in expr
    # The original formula is untouched, so rf2o/libcamera/camera_ros stamps stay byte-identical.
    assert (
        "(colcon_src.commit | default('')) ~ (_colcon_patch_stats.results | map(attribute='stat.checksum') | join(''))"
        in expr
    )
    assert "~ (_colcon_chain_key | default(''))" in expr


def test_build_jobs_default_to_ros2_build_jobs() -> None:
    build = next(t for t in package_tasks() if "colcon build" in str(t.get("ansible.builtin.command", "")))
    assert build["environment"]["MAKEFLAGS"] == "-j{{ colcon_src.jobs | default(ros2_build_jobs) }}"


def test_overlay_build_is_limited_to_the_mppi_subdirectory() -> None:
    source = client_vars()["nav2_mppi_overlay_source"]
    assert source["base_paths"] == ["nav2_mppi_controller"]
    build = next(t for t in package_tasks() if "colcon build" in str(t.get("ansible.builtin.command", "")))
    cmd = build["ansible.builtin.command"]["cmd"]
    # Without --base-paths colcon discovers the unbuilt nav2_* siblings of the navigation2 checkout and they shadow apt.
    assert "--base-paths" in cmd and "colcon_src.base_paths | default([])" in cmd
    assert cmd.index("--base-paths") < cmd.index("--packages-select")
    # Only sources that set base_paths get the flag; rf2o/libcamera/camera_ros build command is unchanged.
    assert "if colcon_src.base_paths is defined" in cmd


def test_launcher_still_sources_the_first_source_workspace() -> None:
    template = LAUNCHER.read_text()
    first = "(node_colcon_source if node_colcon_source is mapping else node_colcon_source[0]).workspace"
    assert "source {{ " + first + " }}/install/setup.bash" in template
    assert template.index("source /opt/ros/jazzy/setup.bash") < template.index("node_colcon_source")
