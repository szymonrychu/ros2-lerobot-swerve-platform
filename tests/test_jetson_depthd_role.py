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
