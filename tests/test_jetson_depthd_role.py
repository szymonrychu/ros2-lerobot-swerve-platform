"""The jetson_depthd role, part 1: build dependencies on the Jetson. Defaults and tasks are read as data; nothing here
runs ansible."""

import re
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
ANSIBLE_DIR = REPO_ROOT / "ansible"
ROLE_DIR = ANSIBLE_DIR / "roles" / "jetson_depthd"
PLAYBOOK = ANSIBLE_DIR / "playbooks" / "deploy_nodes_jetson.yml"

BUILD_PACKAGES = {
    "libzmq3-dev",
    "libzstd-dev",
    "nlohmann-json3-dev",
    "libyaml-cpp-dev",
    "libgtest-dev",
    "pkg-config",
}
NVIDIA_PACKAGES = {
    "cuda-compiler-10-2",
    "cuda-cudart-dev-10-2",
    "cuda-libraries-dev-10-2",
    "libcudnn8-dev",
    "libnvinfer-dev",
    "libnvonnxparsers-dev",
    "libnvinfer-plugin-dev",
    "vpi1-dev",
}


def load(path: Path) -> Any:
    """Parse a YAML file.

    Args:
        path: File to parse.

    Returns:
        The parsed document.
    """
    return yaml.safe_load(path.read_text())


def defaults() -> dict[str, Any]:
    """The role's defaults.

    Returns:
        The parsed defaults/main.yml.
    """
    return load(ROLE_DIR / "defaults" / "main.yml")


def deps_tasks() -> list[dict[str, Any]]:
    """The tasks of deps.yml.

    Returns:
        The parsed tasks/deps.yml.
    """
    return load(ROLE_DIR / "tasks" / "deps.yml")


def task_named(fragment: str) -> dict[str, Any]:
    """The single deps task whose name contains a fragment.

    Args:
        fragment: Substring of the task name (case-insensitive).

    Returns:
        The task.
    """
    found = [t for t in deps_tasks() if fragment.lower() in t["name"].lower()]
    assert len(found) == 1, f"{fragment!r} matched {[t['name'] for t in found]}"
    return found[0]


def test_build_packages_are_pinned() -> None:
    assert set(defaults()["jetson_depthd_build_packages"]) == BUILD_PACKAGES
    install = task_named("Install the build packages")["ansible.builtin.apt"]
    assert install["name"] == "{{ jetson_depthd_build_packages }}"
    assert install["state"] == "present"


def test_cmake_comes_from_kitware_at_least_3_20() -> None:
    d = defaults()
    assert d["jetson_depthd_cmake_min_version"] == "3.20"
    assert "https://apt.kitware.com/ubuntu/" in d["jetson_depthd_kitware_repo_url"]
    assert d["jetson_depthd_kitware_suite"] == "focal"
    assert d["jetson_depthd_cmake_packages"] == ["cmake", "cmake-data"]
    cmake = task_named("Install cmake from Kitware")["ansible.builtin.apt"]
    assert cmake["name"] == "{{ jetson_depthd_cmake_packages }}" and cmake["state"] == "latest"
    key = task_named("Kitware signing key")["ansible.builtin.get_url"]
    assert key["dest"] == "{{ jetson_depthd_kitware_keyring }}"
    assert d["jetson_depthd_kitware_keyring"].startswith("/usr/share/keyrings/")
    repo = task_named("Kitware APT repository")["ansible.builtin.apt_repository"]
    assert "signed-by={{ jetson_depthd_kitware_keyring }}" in repo["repo"]
    assert "{{ jetson_depthd_kitware_suite }}" in repo["repo"]
    assert repo["state"] == "present"
    assert (
        repo.get("update_cache") is True
        or "update_cache" in task_named("Install the build packages")["ansible.builtin.apt"]
    )
    check = task_named("Assert cmake")["ansible.builtin.assert"]
    assert "jetson_depthd_cmake_min_version" in str(check["that"])


def test_nvidia_dev_packages_come_from_the_l4t_repo() -> None:
    d = defaults()
    assert set(d["jetson_depthd_nvidia_packages"]) == NVIDIA_PACKAGES
    assert d["jetson_depthd_nvidia_repo_hosts"] == [
        "repo.download.nvidia.com/jetson/common",
        "repo.download.nvidia.com/jetson/t210",
    ]
    assert d["jetson_depthd_l4t_release"] == "r32.6"
    probe = task_named("Look for the L4T r32.6 APT repositories")["ansible.builtin.shell"]
    assert "{{ jetson_depthd_l4t_release }}" in probe["cmd"] and "/etc/apt/sources.list.d/" in probe["cmd"]
    repos = task_named("Add the missing L4T r32.6 APT repositories")["ansible.builtin.apt_repository"]
    assert "{{ item.item }}" in repos["repo"] and "{{ jetson_depthd_l4t_release }}" in repos["repo"]
    install = task_named("Install the CUDA, cuDNN, TensorRT and VPI dev packages")["ansible.builtin.apt"]
    assert install["name"] == "{{ jetson_depthd_nvidia_packages }}"
    assert install["state"] == "present"


def test_usr_local_cuda_resolves_to_cuda_10_2() -> None:
    d = defaults()
    assert d["jetson_depthd_cuda_home"] == "/usr/local/cuda-10.2"
    query = task_named("Read the /usr/local/cuda link")["ansible.builtin.command"]
    assert "readlink -f /usr/local/cuda" in query
    setter = task_named("Point /usr/local/cuda")
    assert (
        "update-alternatives --install /usr/local/cuda cuda {{ jetson_depthd_cuda_home }}"
        in setter["ansible.builtin.command"]
    )
    assert "jetson_depthd_cuda_home" in setter["when"]
    assert task_named("Read the /usr/local/cuda link")["changed_when"] is False
    assert task_named("Read the /usr/local/cuda link")["check_mode"] is False


def test_no_opencv_cuda_or_tensorrt_runtime_package_is_touched() -> None:
    d = defaults()
    assert d["jetson_depthd_forbidden_packages"] == [
        "libopencv-dev",
        "python3-opencv",
        "libopencv-core-dev",
        "cuda-toolkit-10-2",
        "tensorrt",
    ]
    for name in (
        set(d["jetson_depthd_build_packages"])
        | set(d["jetson_depthd_cmake_packages"])
        | set(d["jetson_depthd_nvidia_packages"])
    ):
        assert "opencv" not in name
        assert name not in d["jetson_depthd_forbidden_packages"]
    for task in deps_tasks():
        apt = task.get("ansible.builtin.apt")
        if apt:
            assert apt.get("state") == ("latest" if apt["name"] == "{{ jetson_depthd_cmake_packages }}" else "present")
            assert (
                not apt.get("purge") and not apt.get("autoremove") and not apt.get("upgrade") and not apt.get("force")
            )
    text = (ROLE_DIR / "tasks" / "deps.yml").read_text()
    assert not re.search(r"apt(-get)?\s+(remove|purge|autoremove|upgrade|dist-upgrade)|dpkg\s+(-P|--purge|-r)", text)


def test_directories_are_created_with_owners() -> None:
    d = defaults()
    assert d["jetson_depthd_data_dir"] == "/var/lib/depthd"
    assert d["jetson_depthd_config_dir"] == "/etc/depthd"
    assert d["jetson_depthd_user"] == "jetson"
    data = task_named("data directory")["ansible.builtin.file"]
    assert (data["path"], data["owner"], data["group"], data["mode"], data["state"]) == (
        "{{ jetson_depthd_data_dir }}",
        "{{ jetson_depthd_user }}",
        "{{ jetson_depthd_user }}",
        "0755",
        "directory",
    )
    conf = task_named("config directory")["ansible.builtin.file"]
    assert (conf["path"], conf["owner"], conf["group"], conf["mode"], conf["state"]) == (
        "{{ jetson_depthd_config_dir }}",
        "root",
        "root",
        "0755",
        "directory",
    )


def test_main_includes_deps_under_the_build_deps_tag() -> None:
    tasks = load(ROLE_DIR / "tasks" / "main.yml")
    include = next(
        t
        for t in tasks
        if "deps.yml" in str(t.get("ansible.builtin.import_tasks") or t.get("ansible.builtin.include_tasks"))
    )
    assert "build_deps" in include["tags"]


def test_playbook_applies_the_role_with_build_deps_and_depthd_tags() -> None:
    plays = load(PLAYBOOK)
    roles = [r for r in plays[0]["roles"] if r["role"] == "jetson_depthd"]
    assert len(roles) == 1
    assert roles[0]["tags"] == ["build_deps", "depthd"]
    order = [r["role"] for r in plays[0]["roles"]]
    assert order.index("jetson_base") < order.index("jetson_depthd")


# ---- part 2: sync, build, config, service, calibration tool ----

DEPTHD_CONFIG = REPO_ROOT / "jetson" / "depthd" / "config" / "depthd.yaml"


def build_tasks() -> list[dict[str, Any]]:
    """The tasks of build.yml.

    Returns:
        The parsed tasks/build.yml.
    """
    return load(ROLE_DIR / "tasks" / "build.yml")


def build_task(fragment: str) -> dict[str, Any]:
    """The single build task whose name contains a fragment.

    Args:
        fragment: Substring of the task name (case-insensitive).

    Returns:
        The task.
    """
    found = [t for t in build_tasks() if fragment.lower() in t["name"].lower()]
    assert len(found) == 1, f"{fragment!r} matched {[t['name'] for t in found]}"
    return found[0]


def flat_keys(tree: Any, prefix: str = "") -> set[str]:
    """Dotted paths of every mapping key in a nested document.

    Args:
        tree: Parsed YAML.
        prefix: Path of the parent.

    Returns:
        The dotted key paths of all mappings.
    """
    if not isinstance(tree, dict):
        return set()
    keys: set[str] = set()
    for key, value in tree.items():
        path = f"{prefix}.{key}" if prefix else str(key)
        keys.add(path)
        keys |= flat_keys(value, path)
    return keys


def test_sources_sync_from_the_controller_checkout_without_build_dirs() -> None:
    for fragment, src, dest in (
        ("Sync the depthd source", "jetson/depthd/", "/opt/depthd/src/jetson/depthd/"),
        ("Sync the shared C++ library", "shared/ros2_common_cpp/", "/opt/depthd/src/shared/ros2_common_cpp/"),
    ):
        sync = build_task(fragment)["ansible.posix.synchronize"]
        assert sync["src"] == "{{ ros2_repo_root }}/" + src
        assert sync["dest"] == dest
        assert "--exclude=build/" in " ".join(sync["rsync_opts"]) or "build/" in sync.get("rsync_opts", [])
        assert sync.get("delete") in (None, False)
    assert not re.search(r"ansible.builtin.git|git clone", (ROLE_DIR / "tasks" / "build.yml").read_text())


def test_build_runs_only_when_the_stamp_is_missing() -> None:
    text = (ROLE_DIR / "tasks" / "build.yml").read_text()
    digest = build_task("Hash the synced")["ansible.builtin.shell"]["cmd"]
    assert "sha256sum" in digest and "sort" in digest
    stamp = build_task("Look for the build stamp")["ansible.builtin.stat"]
    assert stamp["path"] == "{{ jetson_depthd_prefix }}/.built-{{ jetson_depthd_tree_hash.stdout }}"
    build = build_task("Build depthd")
    assert "not jetson_depthd_stamp.stat.exists" in str(build["when"])
    assert "/usr/local/bin/depthd" in str(build["when"]) or "jetson_depthd_binary_stat" in str(build["when"])
    cmd = build["ansible.builtin.shell"]["cmd"]
    assert "nice -n 10" in cmd and "-j3" in cmd and "--target depthd" in cmd
    assert "CMAKE_BUILD_TYPE=Release" in cmd
    assert ".built-" in text and "state: touch" in text or "touch" in text


def test_binary_is_installed_to_usr_local_bin_and_reports_change() -> None:
    install = build_task("Install the depthd binary")["ansible.builtin.copy"]
    assert install["dest"] == "/usr/local/bin/depthd"
    assert install["remote_src"] is True and install["mode"] == "0755"
    assert build_task("Install the depthd binary")["register"] == "jetson_depthd_binary"


def test_engine_step_copies_onnx_and_builds_fp16_only_on_sha_change() -> None:
    text = (ROLE_DIR / "tasks" / "engines.yml").read_text()
    tasks = load(ROLE_DIR / "tasks" / "engines.yml")
    names = [t["name"] for t in tasks]
    assert defaults()["jetson_depthd_engine_models"] == ["dav2_small_266", "midas_small_128"]
    assert "jetson/models/build/{{ item }}.onnx" in text
    assert "T17 export" in text and "ansible.builtin.assert" in text
    copy = next(t for t in tasks if "ansible.builtin.copy" in t)
    assert copy["ansible.builtin.copy"]["dest"].startswith("{{ jetson_depthd_data_dir }}/models/")
    build = next(t for t in tasks if "trtexec" in str(t.get("ansible.builtin.shell", "")))
    assert "--fp16" in build["ansible.builtin.shell"]["cmd"] and "--saveEngine" in build["ansible.builtin.shell"]["cmd"]
    assert "checksum" in build["when"] and "!=" in build["when"]
    # a rebuilt engine is loaded only by a restarted depthd (service.yml restarts on this register)
    assert build["register"] == "jetson_depthd_engine_build"
    assert any("sha256" in n.lower() for n in names)
    main = load(ROLE_DIR / "tasks" / "main.yml")
    assert any(t.get("ansible.builtin.import_tasks") == "engines.yml" and t["tags"] == ["depthd"] for t in main)


def test_calibrate_tool_is_synced_and_runs_on_system_python() -> None:
    sync = build_task("Sync the calibration tool")["ansible.posix.synchronize"]
    assert sync["src"] == "{{ ros2_repo_root }}/jetson/calibrate/"
    assert sync["dest"] == "{{ jetson_depthd_prefix }}/calibrate/"
    opts = " ".join(sync["rsync_opts"])
    assert "--exclude=.venv" in opts and "--exclude=__pycache__" in opts
    check = build_task("Check the calibration tool imports")["ansible.builtin.command"]
    assert check.startswith("python3 -c") and "stereo_calibrate" in check
    assert "venv" not in (ROLE_DIR / "tasks" / "build.yml").read_text().replace("--exclude=.venv", "")


def test_template_keys_equal_the_depthd_yaml_keys() -> None:
    config = defaults()["depthd_config"]
    assert flat_keys(config) == flat_keys(load(DEPTHD_CONFIG))
    assert config["processing"]["matcher"] == "cuda_sgm"
    # mono fill is on: the engine step builds the engine the config names, under /var/lib/depthd/models
    assert config["mono"]["enabled"] is True
    engine = config["mono"]["engine_path"]
    assert engine.startswith("{{ jetson_depthd_data_dir }}/models/") or engine.startswith("/var/lib/depthd/models/")
    assert engine.rsplit("/", 1)[1].removesuffix(".engine") in defaults()["jetson_depthd_engine_models"]
    assert engine.rsplit("/", 1)[1].startswith(config["mono"]["model"].replace("_small", "_small_"))
    assert 2.0 <= config["mono"]["rate_hz"] <= 4.0
    # jetson/models/RESULTS.md: MiDaS v2.1 small at 128x128 is the model that keeps stereo near 12 fps
    assert (config["mono"]["model"], engine.rsplit("/", 1)[1]) == ("midas_small", "midas_small_128.engine")
    template = (ROLE_DIR / "templates" / "depthd.yaml.j2").read_text()
    assert "depthd_config" in template


def test_config_is_rendered_and_validated_before_restart() -> None:
    tasks = load(ROLE_DIR / "tasks" / "service.yml")
    names = [t["name"] for t in tasks]
    conf = next(
        t for t in tasks if "ansible.builtin.template" in t and "depthd.yaml" in t["ansible.builtin.template"]["src"]
    )
    assert conf["ansible.builtin.template"]["dest"] == "{{ jetson_depthd_config_dir }}/depthd.yaml"
    assert conf["ansible.builtin.template"]["validate"] == "/usr/local/bin/depthd --check-config %s"
    assert names.index(conf["name"]) < names.index(next(n for n in names if "Restart depthd" in n))


def test_unit_directives() -> None:
    unit = (ROLE_DIR / "templates" / "depthd.service.j2").read_text()
    for line in (
        "User=jetson",
        "Group=video",
        "After=nvargus-daemon.service network-online.target jetson-clocks.service",
        "Restart=always",
        "RestartSec=2",
        "CPUWeight=500",
        "IOWeight=500",
        "Nice=-5",
        "LimitMEMLOCK=infinity",
        "ExecStart=/usr/local/bin/depthd --config /etc/depthd/depthd.yaml",
    ):
        assert line in unit.splitlines(), line
    assert "CPUQuota" not in unit and "MemoryMax" not in unit


def test_service_restarts_only_on_binary_config_unit_or_engine_change() -> None:
    tasks = load(ROLE_DIR / "tasks" / "service.yml")
    restart = next(t for t in tasks if "Restart depthd" in t["name"])
    cond = str(restart["when"])
    for register in ("jetson_depthd_binary", "jetson_depthd_conf", "jetson_depthd_unit", "jetson_depthd_engine_build"):
        assert f"{register} is changed" in cond
    assert restart["ansible.builtin.systemd"]["state"] == "restarted"
    enable = next(t for t in tasks if "Enable depthd" in t["name"])["ansible.builtin.systemd"]
    assert enable["enabled"] is True and enable["state"] == "started"


def test_main_imports_build_and_service_under_the_depthd_tag() -> None:
    tasks = load(ROLE_DIR / "tasks" / "main.yml")
    for name in ("build.yml", "service.yml"):
        item = next(t for t in tasks if name in str(t.get("ansible.builtin.import_tasks")))
        assert "depthd" in item["tags"]
