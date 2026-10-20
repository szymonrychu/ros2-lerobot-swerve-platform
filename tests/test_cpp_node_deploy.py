"""In-repo C++ node deploy: the local-source colcon build of ros2_node_deploy and the _cpp node types.

A local source (colcon_source item with `local: <repo path>`) is synced from the repo checkout into
<workspace>/src/<package>; its build stamp key is a hash of the synced tree (paths and file contents) plus the stamped
apt versions, so an unchanged deploy builds nothing. It builds in the low-priority scope and never stops the running
nodes (stop_for_build.yml is for the pinned upstream builds only); only the node's own unit restarts.
"""

import hashlib
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import jinja2
import pytest
import yaml
from ansible.plugins.filter.core import FilterModule

from tests.test_cpp_scaffolding import discover_cpp_package_dirs

REPO_ROOT = Path(__file__).resolve().parent.parent
ANSIBLE_DIR = REPO_ROOT / "ansible"
ROLE_TASKS = ANSIBLE_DIR / "roles" / "ros2_node_deploy" / "tasks"
PACKAGE_TASKS = ROLE_TASKS / "colcon_source_package.yml"
CLIENT_VARS = ANSIBLE_DIR / "group_vars" / "client.yml"
ALL_VARS = ANSIBLE_DIR / "group_vars" / "all.yml"
CLIENT_PLAYBOOK = ANSIBLE_DIR / "playbooks" / "deploy_nodes_client.yml"
ANSIBLE_README = ANSIBLE_DIR / "README.md"
NODES_README = REPO_ROOT / "nodes" / "README.md"
DEPLOY_SKILL = REPO_ROOT / ".claude" / "skills" / "ansible-deploy" / "SKILL.md"

SYNC_TASK = "Sync the local source tree into the workspace"
HASH_TASK = "Hash the synced local source tree"
KEY_TASK = "Derive the build stamp key from the source, the patch contents and the apt versions"
BUILD_TASK = "Build the workspace for the pinned commit, patches and build args"
STOP_TASK = "Stop the running nodes before the colcon build"
WORKSPACE = "/opt/ros2-ws"
COMMON_SOURCE = {"local": "shared/ros2_common_cpp", "package": "ros2_common_cpp", "workspace": WORKSPACE}
# _cpp type -> (Python counterpart, env var of its config path, launch command).
CPP_TYPES = {
    "map_guard_cpp": ("map_guard", "MAP_GUARD_CONFIG", "ros2 run map_guard_cpp map_guard_node"),
}
METRICS_PORTS = {"map_guard_cpp": 19110}
# _cpp types without a Python counterpart: head_camera_cpp builds the pinned libcamera first, then the two local trees.
STANDALONE_CPP_TYPES = ("head_camera_cpp", "head_rgbd_cpp", "ground_estimator_cpp")
ALL_CPP_TYPES = tuple(sorted([*CPP_TYPES, *STANDALONE_CPP_TYPES]))
# In-repo C++ node types of the server (ansible/group_vars/server.yml), same two local sources.
SERVER_VARS = ANSIBLE_DIR / "group_vars" / "server.yml"
SERVER_CPP_TYPES = ("map3d_manager_cpp",)
# rosdep keys of system (non-ROS) dependencies -> their Ubuntu 24.04 apt package.
ROSDEP_APT = {
    "yaml-cpp": "libyaml-cpp-dev",
    "prometheus-cpp": "prometheus-cpp-dev",
    "opencv": "libopencv-dev",
    "nlohmann-json-dev": "nlohmann-json3-dev",
    "cpp-httplib": "libcpp-httplib-dev",
    "eigen": "libeigen3-dev",
    "libpcl-all-dev": "libpcl-dev",
}
# package.xml dependency tags that must be installed on the Pi to build and run (test_depend is not built there).
BUILD_DEPEND_TAGS = ("buildtool_depend", "depend", "build_depend", "build_export_depend", "exec_depend")
TIER_KEYS = ("memory_low", "memory_max", "cpu_weight", "io_weight", "oom_score_adjust")
LOW_PRIORITY_FLAGS = (
    "systemd-run --quiet --scope",
    "-p CPUWeight=20",
    "-p IOWeight=10",
    "nice -n 19",
    "ionice -c 3",
    "--parallel-workers 1",
    "--cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF",
)


def load(path: Path) -> Any:
    """Parse a YAML file.

    Args:
        path (Path): YAML file.

    Returns:
        Any: Parsed document.
    """
    return yaml.safe_load(path.read_text())


def task(name: str) -> dict:
    """Return one task of colcon_source_package.yml by name.

    Args:
        name (str): Task name.

    Returns:
        dict: The task.
    """
    return next(t for t in load(PACKAGE_TASKS) if t.get("name") == name)


def jinja_env() -> jinja2.Environment:
    """A Jinja environment with Ansible's core filters (hash, regex_replace, ...).

    Returns:
        jinja2.Environment: The environment.
    """
    env = jinja2.Environment(undefined=jinja2.ChainableUndefined)
    # Only the filters Jinja lacks: Ansible's own default/map/select need its templar.
    env.filters.update({k: v for k, v in FilterModule().filters().items() if k not in env.filters})
    return env


def render(template: str, **context: object) -> str:
    """Render one task field the way Ansible does (with the group_vars/all.yml build limits in scope).

    Args:
        template (str): Jinja template text.
        **context (object): Task variables.

    Returns:
        str: Rendered, whitespace-normalised text.
    """
    variables = {**load(ALL_VARS), **context}
    return " ".join(jinja_env().from_string(template).render(**variables).split())


def when_holds(conditions: str | list[str], **context: object) -> bool:
    """Evaluate a task's `when` (a string or a list of conditions, all must hold).

    Args:
        conditions (str | list[str]): The task's when.
        **context (object): Task variables.

    Returns:
        bool: True when the task runs.
    """
    items = [conditions] if isinstance(conditions, str) else conditions
    return all(jinja_env().compile_expression(c)(**context) for c in items)


def bash(script: str) -> str:
    """Run a rendered shell script with bash.

    Args:
        script (str): Script text.

    Returns:
        str: Stripped stdout.
    """
    return subprocess.run(["bash", "-c", script], check=True, capture_output=True, text=True).stdout.strip()


def tree_key(workspace: Path, source: dict, apt_versions: list[str], chain_key: str = "") -> str:
    """Hash the synced tree with the role's script and render the stamp key expression for it.

    Args:
        workspace (Path): Workspace directory (holds src/<package>).
        source (dict): The colcon source item.
        apt_versions (list[str]): dpkg-query output of each stamp_apt_packages entry.
        chain_key (str): Key of the earlier sources of the node type.

    Returns:
        str: The rendered stamp key.
    """
    src = {**source, "workspace": str(workspace)}
    script = task(HASH_TASK)["ansible.builtin.shell"]
    tree_hash = bash(jinja_env().from_string(script).render(colcon_src=src))
    expression = task(KEY_TASK)["ansible.builtin.set_fact"]["_colcon_patch_key"]
    return render(
        expression,
        colcon_src=src,
        _colcon_tree_hash={"stdout": tree_hash},
        _colcon_patch_stats={"results": []},
        _colcon_apt_versions={"results": [{"stdout": v} for v in apt_versions]},
        _colcon_chain_key=chain_key,
    )


def require_rsync() -> None:
    """Fail (never skip) when rsync is missing: the role syncs every local source with it.

    Raises:
        pytest.fail.Exception: rsync is not on PATH.
    """
    if shutil.which("rsync") is None:
        pytest.fail("rsync not installed: it is an apt dependency of every local colcon source; install it")


@pytest.fixture
def synced(tmp_path: Path) -> tuple[Path, Path, dict]:
    """A repo checkout with a local package, synced into a workspace with the role's rsync command.

    Args:
        tmp_path (Path): pytest temporary directory.

    Returns:
        tuple[Path, Path, dict]: (repo package dir, workspace, source item).
    """
    require_rsync()
    repo, workspace = tmp_path / "repo", tmp_path / "ws"
    package = repo / "nodes" / "demo_cpp"
    (package / "src").mkdir(parents=True)
    (package / "package.xml").write_text("<package/>\n")
    (package / "src" / "node.cpp").write_text("int main() { return 0; }\n")
    (workspace / "src").mkdir(parents=True)
    source = {
        "local": "nodes/demo_cpp",
        "package": "demo_cpp",
        "workspace": str(workspace),
        "stamp_apt_packages": ["ros-jazzy-rclcpp"],
    }
    sync(repo, source)
    return package, workspace, source


def test_missing_rsync_fails_instead_of_skipping(monkeypatch: pytest.MonkeyPatch) -> None:
    """rsync is an apt dependency of every local source; without it the stamp tests must fail, not go quietly green."""
    monkeypatch.setattr(shutil, "which", lambda _name: None)
    with pytest.raises(pytest.fail.Exception, match="rsync"):
        require_rsync()


def sync(repo: Path, source: dict) -> None:
    """Run the role's sync command for a local source.

    Args:
        repo (Path): Repo checkout (repo_dest).
        source (dict): The colcon source item.
    """
    cmd = task(SYNC_TASK)["ansible.builtin.command"]["cmd"]
    bash(jinja_env().from_string(cmd).render(colcon_src=source, repo_dest=str(repo)))


# --- stamp key --------------------------------------------------------------------------------------------------------


def test_stamp_key_is_stable_for_an_unchanged_tree(synced: tuple[Path, Path, dict]) -> None:
    """An unchanged deploy (sync again, hash again) derives the same key: the stamp exists and nothing builds."""
    package, workspace, source = synced
    first = tree_key(workspace, source, ["4.0.0"])
    sync(package.parent.parent, source)
    assert tree_key(workspace, source, ["4.0.0"]) == first
    assert len(first) == 40


def test_stamp_key_changes_when_a_source_file_changes(synced: tuple[Path, Path, dict]) -> None:
    package, workspace, source = synced
    before = tree_key(workspace, source, ["4.0.0"])
    (package / "src" / "node.cpp").write_text("int main() { return 1; }\n")
    sync(package.parent.parent, source)
    assert tree_key(workspace, source, ["4.0.0"]) != before


def test_stamp_key_covers_paths_deletions_and_apt_versions(synced: tuple[Path, Path, dict]) -> None:
    """A rename (same contents), a deleted file (rsync --delete) and an apt upgrade each change the key."""
    package, workspace, source = synced
    repo = package.parent.parent
    base = tree_key(workspace, source, ["4.0.0"])
    assert tree_key(workspace, source, ["4.0.1"]) != base
    assert tree_key(workspace, source, ["4.0.0"], chain_key="abc") != base
    (package / "src" / "node.cpp").rename(package / "src" / "main.cpp")
    sync(repo, source)
    renamed = tree_key(workspace, source, ["4.0.0"])
    assert renamed != base
    assert not (workspace / "src" / "demo_cpp" / "src" / "node.cpp").exists()
    (package / "src" / "main.cpp").rename(package / "src" / "node.cpp")
    sync(repo, source)
    assert tree_key(workspace, source, ["4.0.0"]) == base


def test_python_caches_do_not_reach_the_synced_tree(synced: tuple[Path, Path, dict]) -> None:
    """A __pycache__ left by a fixture generator run on the Pi neither syncs nor changes the key."""
    package, workspace, source = synced
    base = tree_key(workspace, source, ["4.0.0"])
    (package / "test" / "__pycache__").mkdir(parents=True)
    (package / "test" / "__pycache__" / "gen.cpython-312.pyc").write_bytes(b"\0")
    sync(package.parent.parent, source)
    assert not (workspace / "src" / "demo_cpp" / "test" / "__pycache__").exists()
    assert tree_key(workspace, source, ["4.0.0"]) == base


def test_pinned_sources_keep_their_key_inputs() -> None:
    """A pinned upstream source has no synced tree: its key is the old one (commit, patches, args, apt versions)."""
    expression = task(KEY_TASK)["ansible.builtin.set_fact"]["_colcon_patch_key"]
    source = {"repo": "r", "commit": "abc", "package": "rf2o", "workspace": WORKSPACE}
    skipped = {"skipped": True, "changed": False}
    key = render(
        expression,
        colcon_src=source,
        _colcon_tree_hash=skipped,
        _colcon_patch_stats={"results": [{"stat": {"checksum": "p1"}}]},
        _colcon_apt_versions={"results": []},
        _colcon_chain_key="",
    )
    assert key == hashlib.sha1(b"abcp1").hexdigest()


def test_local_only_tasks_run_only_for_local_sources() -> None:
    local, pinned = {**COMMON_SOURCE}, {"repo": "r", "commit": "c", "package": "p", "workspace": WORKSPACE}
    for name in (SYNC_TASK, HASH_TASK):
        assert when_holds(task(name)["when"], colcon_src=local), name
        assert not when_holds(task(name)["when"], colcon_src=pinned), name
    tasks = load(PACKAGE_TASKS)
    names = [t["name"] for t in tasks]
    assert (
        names.index(SYNC_TASK)
        < names.index(HASH_TASK)
        < names.index(KEY_TASK)
        < names.index("Look for the build stamp")
    )
    for t in tasks:
        module = next(k for k in t if k.startswith("ansible.builtin."))
        if module in ("ansible.builtin.git", "ansible.builtin.patch"):
            missing = {"stat": {"exists": False}}
            assert not when_holds(t["when"], colcon_src=local, _colcon_stamp_stat=missing), t["name"]
            assert when_holds(t["when"], colcon_src=pinned, _colcon_stamp_stat=missing), t["name"]


# --- no stop-all, own restart -----------------------------------------------------------------------------------------


def test_local_builds_never_include_stop_for_build() -> None:
    """D4: an in-repo package builds next to the running robot; pinned upstream builds still stop the nodes first."""
    stop = task(STOP_TASK)
    assert stop["ansible.builtin.include_tasks"] == "stop_for_build.yml"
    missing = {"stat": {"exists": False}}
    assert not when_holds(stop["when"], colcon_src=COMMON_SOURCE, _colcon_stamp_stat=missing)
    pinned = {"repo": "r", "commit": "c", "package": "p", "workspace": WORKSPACE}
    assert when_holds(stop["when"], colcon_src=pinned, _colcon_stamp_stat=missing)
    assert not when_holds(stop["when"], colcon_src=pinned, _colcon_stamp_stat={"stat": {"exists": True}})
    includes = [t for t in load(PACKAGE_TASKS) if "include_tasks" in str(t)]
    assert includes == [stop], "no other way into stop_for_build"
    assert "systemctl" not in PACKAGE_TASKS.read_text()


def test_a_local_build_restarts_only_its_own_node() -> None:
    build = task(BUILD_TASK)
    assert build["notify"] == "Restart ROS2 node"
    tasks = load(PACKAGE_TASKS)
    queue = tasks[tasks.index(build) + 1]["ansible.builtin.lineinfile"]
    assert queue["line"] == "{{ node_name }}"


# --- low-priority scope -----------------------------------------------------------------------------------------------


def build_context(source: dict) -> dict:
    """Variables the build task sees for one source.

    Args:
        source (dict): The colcon source item.

    Returns:
        dict: Render context.
    """
    return {"colcon_src": source, "_colcon_chain_key": "", "_colcon_patch_key": "k"}


def configured_local_sources() -> list[dict]:
    """Every local colcon source of the client node types, as configured.

    Returns:
        list[dict]: Source items with a `local` key.
    """
    types = client()["ros2_node_type_defaults"]
    return [s for t in ALL_CPP_TYPES for s in types[t]["colcon_source"] if "local" in s]


def test_local_build_runs_in_the_low_priority_scope() -> None:
    """Every configured local source renders the low-priority scope, its own discovery root and make -j2."""
    build = task(BUILD_TASK)
    sources = configured_local_sources()
    assert len(sources) == 2 * len(ALL_CPP_TYPES)
    for source in sources:
        cmd = render(build["ansible.builtin.command"]["cmd"], **build_context(source))
        for flag in LOW_PRIORITY_FLAGS:
            assert flag in cmd, (source["package"], flag)
        package = source["package"]
        assert f"--base-paths {WORKSPACE}/src/{package}/. --packages-select {package}" in cmd, "only its own tree"
        assert render(build["environment"]["MAKEFLAGS"], **build_context(source)) == "-j2"
        chained = render(build["ansible.builtin.command"]["cmd"], **{**build_context(source), "_colcon_chain_key": "x"})
        assert f"source {WORKSPACE}/install/setup.bash" in chained, "a chained package finds ros2_common_cpp"


def test_pinned_build_command_is_unchanged() -> None:
    """rf2o, camera_ros and MPPI build exactly as before: Release only, make -j ros2_build_jobs, no --base-paths."""
    build = task(BUILD_TASK)
    source = {"repo": "r", "commit": "c", "package": "rf2o_laser_odometry", "workspace": WORKSPACE}
    cmd = render(build["ansible.builtin.command"]["cmd"], **build_context(source))
    assert "--cmake-args -DCMAKE_BUILD_TYPE=Release &&" in cmd
    assert "--base-paths" not in cmd
    assert render(build["environment"]["MAKEFLAGS"], **build_context(source)) == "-j4"


# --- node types and entries -------------------------------------------------------------------------------------------


def client() -> dict:
    """Parse ansible/group_vars/client.yml.

    Returns:
        dict: Group variables.
    """
    return load(CLIENT_VARS)


@pytest.mark.parametrize("cpp_type", sorted(CPP_TYPES))
def test_cpp_node_type_is_a_local_colcon_build(cpp_type: str) -> None:
    python_type, env_var, launch = CPP_TYPES[cpp_type]
    types = client()["ros2_node_type_defaults"]
    entry, python = types[cpp_type], types[python_type]
    assert entry["deploy_mode"] == "native"
    assert entry["node_src_dir"] == ""
    assert entry["node_launch_command"] == launch
    assert entry["src_paths"] == [], "the build stamp restarts it; it runs no repo file"
    identity = [{k: s[k] for k in ("local", "package", "workspace")} for s in entry["colcon_source"]]
    assert identity == [COMMON_SOURCE, {"local": f"nodes/{cpp_type}", "package": cpp_type, "workspace": WORKSPACE}]
    for source in entry["colcon_source"]:
        assert source["stamp_apt_packages"], "apt versions are part of the key"
        assert set(source["stamp_apt_packages"]) <= set(entry["apt_packages"]), source["package"]
    for package in (
        "build-essential",
        "cmake",
        "rsync",
        "python3-colcon-common-extensions",
        "ros-jazzy-ament-cmake",
        "ros-jazzy-rclcpp",
        "ros-jazzy-rclcpp-components",
        "libyaml-cpp-dev",
        "prometheus-cpp-dev",
    ):
        assert package in entry["apt_packages"], package  # fmt: skip
    assert entry["config_path"] == f"/etc/ros2/{cpp_type}"
    assert entry["env"] == [f"{env_var}=/etc/ros2/{cpp_type}/config.yaml"]
    assert {k: entry[k] for k in TIER_KEYS} == {k: python[k] for k in TIER_KEYS}, "same tier as the Python node"


def package_xml_apt_names(package_xml: Path, local_packages: set[str]) -> set[str]:
    """Map the build/run dependencies of a package.xml to the apt packages that provide them.

    Args:
        package_xml (Path): The package's package.xml.
        local_packages (set[str]): Package names built from the repo tree (no apt package).

    Returns:
        set[str]: Apt package names (ros-jazzy-<dashed> for ROS keys, ROSDEP_APT for system keys).
    """
    root = ET.parse(package_xml).getroot()
    keys = {e.text.strip() for tag in BUILD_DEPEND_TAGS for e in root.iter(tag) if e.text}
    names: set[str] = set()
    for key in keys - local_packages:
        if key in ROSDEP_APT:
            names.add(ROSDEP_APT[key])
        elif key.startswith("lib") and key.endswith("-dev"):
            names.add(key)
        else:
            names.add("ros-jazzy-" + key.replace("_", "-"))
    return names


def test_package_xml_apt_names_maps_ros_system_and_local_keys(tmp_path: Path) -> None:
    package_xml = tmp_path / "package.xml"
    package_xml.write_text(
        "<package format='3'><name>x</name><buildtool_depend>ament_cmake</buildtool_depend>"
        "<depend>rclcpp</depend><depend>yaml-cpp</depend><depend>ros2_common_cpp</depend>"
        "<exec_depend>libopencv-dev</exec_depend><test_depend>ament_lint_auto</test_depend></package>"
    )
    assert package_xml_apt_names(package_xml, {"ros2_common_cpp"}) == {
        "ros-jazzy-ament-cmake",
        "ros-jazzy-rclcpp",
        "libyaml-cpp-dev",
        "libopencv-dev",
    }


@pytest.mark.parametrize("cpp_type", ALL_CPP_TYPES)
def test_cpp_node_apt_packages_cover_every_in_tree_package_xml(cpp_type: str) -> None:
    """apt_packages installs every dependency the package.xml of each of its local sources declares.

    A source whose package.xml is not in the tree yet (built by a parallel task) is checked as soon as it lands.
    """
    entry = client()["ros2_node_type_defaults"][cpp_type]
    local_packages = {s["package"] for s in entry["colcon_source"]}
    present = [REPO_ROOT / s["local"] / "package.xml" for s in entry["colcon_source"] if "local" in s]
    present = [p for p in present if p.is_file()]
    assert REPO_ROOT / COMMON_SOURCE["local"] / "package.xml" in present
    for package_xml in present:
        missing = package_xml_apt_names(package_xml, local_packages) - set(entry["apt_packages"])
        assert not missing, f"{package_xml.relative_to(REPO_ROOT)}: {sorted(missing)} not in {cpp_type} apt_packages"


def test_ros2_common_cpp_is_a_discovered_cpp_package() -> None:
    assert REPO_ROOT / COMMON_SOURCE["local"] in discover_cpp_package_dirs()


def test_head_camera_cpp_identity_is_pinned_libcamera_then_the_two_local_trees() -> None:
    """The standalone type's sources: pinned libcamera, then ros2_common_cpp and the node as local sources."""
    entry = client()["ros2_node_type_defaults"]["head_camera_cpp"]
    assert entry["node_launch_command"] == "ros2 run head_camera_cpp head_camera_node"
    identity = [(s.get("local"), s["package"], s["workspace"]) for s in entry["colcon_source"]]
    assert identity == [
        (None, "libcamera", WORKSPACE),
        (COMMON_SOURCE["local"], "ros2_common_cpp", WORKSPACE),
        ("nodes/head_camera_cpp", "head_camera_cpp", WORKSPACE),
    ]
    assert entry["env"] == ["HEAD_CAMERA_CONFIG=/etc/ros2/head_camera_cpp/config.yaml"]
    assert entry["config_path"] == "/etc/ros2/head_camera_cpp"


@pytest.mark.parametrize("cpp_type", sorted(CPP_TYPES))
def test_cpp_node_entry_is_live_with_the_python_config(cpp_type: str) -> None:
    """Side by side: the _cpp entry (live since the cut-over) carries the Python node's config."""
    nodes = {n["name"]: n for n in client()["ros2_nodes"]}
    entry, python = nodes[cpp_type], nodes[CPP_TYPES[cpp_type][0]]
    assert entry["node_type"] == cpp_type
    assert entry["present"] is True and entry["enabled"] is True
    assert entry["config"] == python["config"]
    assert yaml.safe_load(entry["config"])["metrics_port"] == METRICS_PORTS[cpp_type]
    names = list(nodes)
    assert names.index(cpp_type) == names.index(CPP_TYPES[cpp_type][0]) + 1


@pytest.mark.parametrize("cpp_type", sorted(CPP_TYPES))
def test_client_playbook_deploys_the_cpp_node_after_its_python_counterpart(cpp_type: str) -> None:
    tasks = [c for t in load(CLIENT_PLAYBOOK)[0]["tasks"] for c in (t["block"] if "block" in t else [t])]
    deploys = [t["vars"]["_deploy_node_name"] for t in tasks if "_deploy_node_name" in t.get("vars", {})]
    assert deploys.index(cpp_type) == deploys.index(CPP_TYPES[cpp_type][0]) + 1
    step = next(t for t in tasks if t.get("vars", {}).get("_deploy_node_name") == cpp_type)
    assert {cpp_type, "build", "config"} <= set(step["tags"])
    assert step["ansible.builtin.include_tasks"]["apply"]["tags"] == [cpp_type]


def test_docs_describe_the_local_source_build_and_the_cpp_nodes() -> None:
    ansible_readme = ANSIBLE_README.read_text()
    for phrase in ("colcon_source.local", "stop_for_build.yml", "map_guard_cpp"):
        assert phrase in ansible_readme, phrase
    for path in (NODES_README, DEPLOY_SKILL):
        text = path.read_text()
        assert "map_guard_cpp" in text, path.name


@pytest.mark.parametrize("cpp_type", SERVER_CPP_TYPES)
def test_server_cpp_node_builds_two_local_sources_and_covers_their_package_xml(cpp_type: str) -> None:
    """The server's in-repo C++ node: ros2_common_cpp then the node, apt covers both package.xml files."""
    entry = load(SERVER_VARS)["ros2_node_type_defaults"][cpp_type]
    sources = entry["colcon_source"]
    assert [s["local"] for s in sources] == [COMMON_SOURCE["local"], f"nodes/{cpp_type}"]
    local_packages = {s["package"] for s in sources}
    for source in sources:
        missing = package_xml_apt_names(REPO_ROOT / source["local"] / "package.xml", local_packages)
        assert not missing - set(entry["apt_packages"]), (source["package"], sorted(missing))
