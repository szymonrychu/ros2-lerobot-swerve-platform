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
DEFAULT_UBUNTU_MIRROR = "http://archive.ubuntu.com/ubuntu"
DEFAULT_UBUNTU_SECURITY_MIRROR = "http://security.ubuntu.com/ubuntu"
GITHUB_UBUNTU_MIRROR = "http://azure.archive.ubuntu.com/ubuntu"
APT_RETRIES_CONF = "/etc/apt/apt.conf.d/80-cpp-node-test-retries"
# Stand-in docker for run_script_with_fake_docker: `info` succeeds, `image inspect` misses (forcing a build), `build`
# appends its arguments one per line to $FAKE_DOCKER_LOG and fails, so the script stops before `docker run`.
FAKE_DOCKER = """#!/usr/bin/env bash
case "$1" in
  info) exit 0 ;;
  image) exit 1 ;;
  build) printf '%s\\n' "$@" >> "$FAKE_DOCKER_LOG"; exit 1 ;;
  *) exit 99 ;;
esac
"""


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


def dockerfile_instructions() -> str:
    """Return the Dockerfile without its comment lines, so ordering checks see instructions only.

    Returns:
        str: The non-comment lines of scripts/cpp-node-test.Dockerfile, newline-joined.
    """
    return "\n".join(ln for ln in DOCKERFILE.read_text().splitlines() if not ln.lstrip().startswith("#"))


def run_script_with_fake_docker(tmp_path: Path, github_actions: str | None) -> tuple[int, list[str]]:
    """Run cpp-node-test.sh for ros2_common_cpp with FAKE_DOCKER first on PATH and capture its docker build arguments.

    Args:
        tmp_path (Path): Directory for the fake docker binary and its argument log.
        github_actions (str | None): Value of GITHUB_ACTIONS for the run; None removes the variable.

    Returns:
        tuple[int, list[str]]: The script's exit code and the arguments of its `docker build` call.
    """
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    docker = bin_dir / "docker"
    docker.write_text(FAKE_DOCKER)
    docker.chmod(0o755)
    log = tmp_path / "docker-build-args.txt"
    env = {k: v for k, v in os.environ.items() if k != "GITHUB_ACTIONS"}
    env["PATH"] = f"{bin_dir}{os.pathsep}{env['PATH']}"
    env["FAKE_DOCKER_LOG"] = str(log)
    if github_actions is not None:
        env["GITHUB_ACTIONS"] = github_actions
    result = subprocess.run(
        ["bash", str(SCRIPT), "ros2_common_cpp"],
        capture_output=True,
        text=True,
        timeout=60,
        cwd=REPO_ROOT,
        env=env,
        check=False,
    )
    return result.returncode, log.read_text().splitlines() if log.exists() else []


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
    assert 'docker build --progress=plain ${BUILD_ARGS[@]+"${BUILD_ARGS[@]}"} -t "$IMAGE" "$CONTEXT" \\\n' in text, (
        "the image build streams plain progress (not redirected) and passes the optional mirror build args"
    )


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


def test_dockerfile_rewrites_ubuntu_mirrors_before_the_first_apt_get() -> None:
    """ARG UBUNTU_MIRROR / UBUNTU_SECURITY_MIRROR (defaults: the base image's archive URIs) rewrite the deb822
    ubuntu.sources before any apt-get, and an amd64 build fails when the rewritten URIs are missing."""
    text = DOCKERFILE.read_text()
    assert f"ARG UBUNTU_MIRROR={DEFAULT_UBUNTU_MIRROR}\n" in text
    assert f"ARG UBUNTU_SECURITY_MIRROR={DEFAULT_UBUNTU_SECURITY_MIRROR}\n" in text
    assert '"s|^URIs: http://archive\\.ubuntu\\.com/ubuntu/\\?$|URIs: ${UBUNTU_MIRROR}/|"' in text
    assert '"s|^URIs: http://security\\.ubuntu\\.com/ubuntu/\\?$|URIs: ${UBUNTU_SECURITY_MIRROR}/|"' in text
    assert "/etc/apt/sources.list.d/ubuntu.sources" in text
    assert 'grep -qxF "URIs: ${UBUNTU_MIRROR}/"' in text
    assert 'grep -qxF "URIs: ${UBUNTU_SECURITY_MIRROR}/"' in text
    code = dockerfile_instructions()
    rewrite = code.index("/etc/apt/sources.list.d/ubuntu.sources")
    assert code.index("ARG UBUNTU_MIRROR=") < rewrite < code.index("apt-get")


def test_dockerfile_configures_apt_retries_and_timeouts_before_the_first_apt_get() -> None:
    """An apt.conf.d snippet retries each download 5 times with 30 s http/https timeouts, before any apt-get."""
    text = DOCKERFILE.read_text()
    for setting in ('Acquire::Retries "5";', 'Acquire::http::Timeout "30";', 'Acquire::https::Timeout "30";'):
        assert f"'{setting}'" in text, setting
    assert f"> {APT_RETRIES_CONF}" in text
    code = dockerfile_instructions()
    assert code.index(APT_RETRIES_CONF) < code.index("apt-get")


def test_script_passes_the_azure_mirror_on_github_actions(tmp_path: Path) -> None:
    """GITHUB_ACTIONS=true: docker build gets the runners' Azure mirror for both the archive and security."""
    code, args = run_script_with_fake_docker(tmp_path, "true")
    assert code == 1
    assert args[0] == "build"
    assert "--progress=plain" in args
    pairs = list(zip(args, args[1:], strict=False))
    assert ("--build-arg", f"UBUNTU_MIRROR={GITHUB_UBUNTU_MIRROR}") in pairs
    assert ("--build-arg", f"UBUNTU_SECURITY_MIRROR={GITHUB_UBUNTU_MIRROR}") in pairs


def test_script_keeps_the_default_mirrors_outside_github_actions(tmp_path: Path) -> None:
    """No GITHUB_ACTIONS (local and Mac builds) or any other value: docker build gets no build arg, so the
    Dockerfile defaults apply."""
    for github_actions in (None, "false"):
        run_dir = tmp_path / str(github_actions)
        run_dir.mkdir()
        code, args = run_script_with_fake_docker(run_dir, github_actions)
        assert code == 1
        assert args[0] == "build"
        assert "--progress=plain" in args
        assert "--build-arg" not in args
