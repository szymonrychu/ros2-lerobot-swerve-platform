"""Contract of the C++ scaffolding: .clang-format, scripts/cpp-node-test.sh and its Dockerfile, the poe tasks and the
lint setup of the shared ament package ros2_common_cpp."""

import os
import re
import subprocess
import tomllib
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CLANG_FORMAT = REPO_ROOT / ".clang-format"
SCRIPT = REPO_ROOT / "scripts" / "cpp-node-test.sh"
DOCKERFILE = REPO_ROOT / "scripts" / "cpp-node-test.Dockerfile"
ROOT_PYPROJECT = REPO_ROOT / "pyproject.toml"
COMMON_CPP_DIR = REPO_ROOT / "shared" / "ros2_common_cpp"
SCRIPT_REL = "scripts/cpp-node-test.sh"
LINT_TEST_DEPENDS = ("ament_lint_auto", "ament_cmake_clang_format", "ament_cmake_cppcheck", "ament_cmake_xmllint")
COLUMN_LIMIT = 120
EXIT_USAGE = 2


def discover_cpp_package_dirs() -> list[Path]:
    """Find every in-repo C++ (ament_cmake) package: shared/ros2_common_cpp plus nodes/*_cpp.

    Returns:
        list[Path]: Package directories holding a package.xml, sorted by package name.
    """
    candidates = [COMMON_CPP_DIR, *(REPO_ROOT / "nodes").glob("*_cpp")]
    return sorted((d for d in candidates if (d / "package.xml").is_file()), key=lambda d: d.name)


def discover_cpp_packages() -> list[str]:
    """Names of every in-repo C++ package (the directory name, which equals the package.xml <name>).

    Returns:
        list[str]: Sorted package names.
    """
    return [d.name for d in discover_cpp_package_dirs()]


def poe_task_body(name: str) -> str:
    """Return the shell body of a root poe task.

    Args:
        name (str): Task name.

    Returns:
        str: The task's shell script.
    """
    task = tomllib.loads(ROOT_PYPROJECT.read_text())["tool"]["poe"]["tasks"][name]
    return task["shell"] if isinstance(task, dict) else task


def run_script(*args: str) -> subprocess.CompletedProcess[str]:
    """Run cpp-node-test.sh on paths that exit before docker is touched.

    Args:
        *args (str): Script arguments.

    Returns:
        subprocess.CompletedProcess[str]: The finished process with captured output.
    """
    return subprocess.run(
        ["bash", str(SCRIPT), *args], capture_output=True, text=True, timeout=30, cwd=REPO_ROOT, check=False
    )


def test_cpp_package_discovery_finds_ros2_common_cpp() -> None:
    """Guards the discovery itself: an empty list would make the coverage tests pass vacuously."""
    assert "ros2_common_cpp" in discover_cpp_packages()


def test_cpp_package_dir_name_equals_package_name() -> None:
    """The script, the poe tasks and the CI matrix address a package by its directory name."""
    for pkg_dir in discover_cpp_package_dirs():
        name = ET.parse(pkg_dir / "package.xml").getroot().findtext("name")
        assert name == pkg_dir.name


def test_clang_format_is_ament_style_at_120_columns() -> None:
    """Root .clang-format: the ament_clang_format style (Google base, ament overrides) with ColumnLimit 120 (ruff)."""
    style = yaml.safe_load(CLANG_FORMAT.read_text())
    assert style["ColumnLimit"] == COLUMN_LIMIT
    assert style["BasedOnStyle"] == "Google"
    assert style["Language"] == "Cpp"
    assert style["AccessModifierOffset"] == -2
    assert style["PointerAlignment"] == "Middle"
    assert style["BreakBeforeBraces"] == "Custom"


def test_script_usage_and_result_line_contract() -> None:
    """Static contract: strict bash, usage line, and exactly one final PASS/FAIL line per run."""
    text = SCRIPT.read_text()
    assert os.access(SCRIPT, os.X_OK)
    assert text.startswith("#!/usr/bin/env bash")
    assert "set -euo pipefail" in text
    assert "Usage: scripts/cpp-node-test.sh <pkg> [--lint-only]" in text
    assert "cpp-node-test: ${PKG} PASS" in text
    assert "cpp-node-test: ${PKG} FAIL" in text
    for step in (
        "rosdep install",
        "colcon build --packages-up-to",
        "-DBUILD_TESTING=ON",
        "-DCMAKE_BUILD_TYPE=Release",
        "colcon test --packages-select",
        "colcon test-result --verbose",
    ):
        assert step in text, step
    assert ":ro" in text, "the repo must be mounted read-only"
    assert "--platform" not in text, "the image runs on the native platform (arm64 Mac, amd64 CI)"


def test_script_without_arguments_prints_usage_and_exits_2() -> None:
    result = run_script()
    assert result.returncode == EXIT_USAGE
    assert "Usage: scripts/cpp-node-test.sh <pkg> [--lint-only]" in result.stderr


def test_script_unknown_package_exits_2_with_known_list() -> None:
    result = run_script("no_such_pkg_cpp")
    assert result.returncode == EXIT_USAGE
    for name in discover_cpp_packages():
        assert name in result.stderr
    assert result.stdout.strip().splitlines()[-1] == "cpp-node-test: no_such_pkg_cpp FAIL"


def test_dockerfile_base_image_and_rosdep_preinstall() -> None:
    """FROM ros:jazzy-ros-base on the native platform, with every package's rosdep keys installed at build time."""
    lines = [ln.strip() for ln in DOCKERFILE.read_text().splitlines() if ln.strip() and not ln.startswith("#")]
    from_lines = [ln for ln in lines if ln.upper().startswith("FROM ")]
    assert from_lines == ["FROM ros:jazzy-ros-base"]
    text = DOCKERFILE.read_text()
    assert "rosdep install" in text
    assert "python3-colcon-common-extensions" in text


def test_poe_tasks_run_the_script_for_every_cpp_package() -> None:
    """test-cpp runs build+test, lint-cpp the --lint-only variant, over shared/ros2_common_cpp and nodes/*_cpp."""
    for task, flag in (("test-cpp", None), ("lint-cpp", "--lint-only")):
        body = poe_task_body(task)
        assert "set -e" in body
        assert SCRIPT_REL in body
        assert "shared/ros2_common_cpp" in body and "nodes/*_cpp" in body
        if flag:
            assert flag in body
        else:
            assert "--lint-only" not in body


def test_ros2_common_cpp_declares_the_lint_packages() -> None:
    """ament_lint_auto runs clang-format, cppcheck and xmllint; the copyright lint stays off."""
    root = ET.parse(COMMON_CPP_DIR / "package.xml").getroot()
    test_depends = {e.text for e in root.findall("test_depend")}
    assert set(LINT_TEST_DEPENDS) <= test_depends
    assert "ament_cmake_gtest" in test_depends
    assert "ament_cmake_copyright" not in test_depends
    assert root.findtext("export/build_type") == "ament_cmake"
    cmake = (COMMON_CPP_DIR / "CMakeLists.txt").read_text()
    assert "ament_lint_auto_find_test_dependencies()" in cmake
    assert re.search(
        r"ament_cmake_clang_format_CONFIG_FILE\s+\"?\$\{CMAKE_CURRENT_SOURCE_DIR\}/\.\./\.\./\.clang-format", cmake
    )


def test_ros2_common_cpp_libraries_do_not_link_rclcpp() -> None:
    """config and metrics are plain libraries, usable from a library target without rclcpp."""
    cmake = (COMMON_CPP_DIR / "CMakeLists.txt").read_text()
    xml = (COMMON_CPP_DIR / "package.xml").read_text()
    assert "rclcpp" not in cmake and "rclcpp" not in xml
    assert re.search(r"add_library\(\s*config\b", cmake)
    assert re.search(r"add_library\(\s*metrics\b", cmake)
