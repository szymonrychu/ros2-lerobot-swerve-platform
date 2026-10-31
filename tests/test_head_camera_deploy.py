"""Static invariants of the head camera deploy: the head_camera_cpp node type in client.yml.

The ros2_nodes entry is retired (present: false, tests/test_old_sensors_retired.py); the node type and
nodes/head_camera_cpp stay for front-and-back-camera-module (plan Q8).

head_camera_cpp (the Camera Module 3 on CSI cam0) builds against the same pinned libcamera as pi_cameras: the libcamera
source dict is identical, so the build stamp matches and nothing rebuilds. The patch is a byte-identical copy that
outlives the retirement of pi_cameras.
"""

import hashlib
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_PLAYBOOK = REPO_ROOT / "ansible" / "playbooks" / "deploy_nodes_client.yml"
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
NODE_DIR = REPO_ROOT / "nodes" / "head_camera_cpp"
PATCH = NODE_DIR / "patches" / "libcamera-0001-add-package-xml.patch"
WORKSPACE = "/opt/ros2-ws"
# sha1 of nodes/pi_cameras/patches/libcamera-0001-add-package-xml.patch: the patch contents are part of the build stamp.
PATCH_SHA1 = "422bd0e3b23dbbbdbd09435ab90cac84ded67166"
# The libcamera colcon source of pi_cameras as recorded: any difference changes the build stamp and rebuilds libcamera.
# Only the patch path may differ (it does not enter the stamp key).
LIBCAMERA_SOURCE = {
    "repo": "https://github.com/raspberrypi/libcamera.git",
    "commit": "v0.7.2+rpt20260817",
    "package": "libcamera",
    "workspace": WORKSPACE,
    "patches": ["{{ ros2_repo_dest }}/nodes/head_camera_cpp/patches/libcamera-0001-add-package-xml.patch"],
    "meson_args": [
        "-Dpipelines=rpi/pisp",
        "-Dipas=rpi/pisp",
        "-Dcam=enabled",
        "-Dqcam=disabled",
        "-Dgstreamer=disabled",
        "-Dv4l2=false",
        "-Dtest=false",
        "-Ddocumentation=disabled",
        "-Dpycamera=disabled",
    ],
}
LOCAL_BUILD = {
    "workspace": WORKSPACE,
    "cmake_args": ["-DCMAKE_BUILD_TYPE=Release", "-DBUILD_TESTING=OFF"],
    "base_paths": ["."],
    "jobs": 2,
}
TIER = {
    "memory_low": "128M",
    "memory_max": "384M",
    "cpu_weight": 100,
    "io_weight": 100,
    "oom_score_adjust": 0,
    "nice": 10,
}
FORBIDDEN_APT_PACKAGES = ("ros-jazzy-camera-ros", "ros-jazzy-libcamera")
REQUIRED_APT_PACKAGES = (
    "git",
    "build-essential",
    "cmake",
    "pkg-config",
    "python3-colcon-common-extensions",
    "python3-colcon-meson",
    "meson",
    "ninja-build",
    "python3-ply",
    "python3-jinja2",
    "rsync",
    "ros-jazzy-ament-cmake",
    "ros-jazzy-rclcpp",
    "ros-jazzy-rclcpp-components",
    "ros-jazzy-sensor-msgs",
    "ros-jazzy-std-msgs",
    "libopencv-dev",
    "nlohmann-json3-dev",
    "libyaml-cpp-dev",
    "prometheus-cpp-dev",
)


def client_vars() -> dict:
    """Load group_vars/client.yml.

    Returns:
        dict: Parsed YAML.
    """
    return yaml.safe_load(CLIENT_VARS.read_text())


def node_type() -> dict:
    """Return the head_camera_cpp node type defaults.

    Returns:
        dict: ros2_node_type_defaults["head_camera_cpp"].
    """
    return client_vars()["ros2_node_type_defaults"]["head_camera_cpp"]


def test_node_type_fields() -> None:
    entry = node_type()
    assert entry["deploy_mode"] == "native"
    assert entry["node_launch_command"] == "ros2 run head_camera_cpp head_camera_node"
    assert entry["node_src_dir"] == ""
    assert entry["src_paths"] == []


def test_libcamera_source_is_the_pinned_dict_of_pi_cameras() -> None:
    source = node_type()["colcon_source"][0]
    assert source == LIBCAMERA_SOURCE


def test_patch_is_a_byte_identical_copy_of_the_pinned_one() -> None:
    assert hashlib.sha1(PATCH.read_bytes()).hexdigest() == PATCH_SHA1


def test_sources_build_libcamera_then_common_then_the_node() -> None:
    sources = node_type()["colcon_source"]
    assert [s["package"] for s in sources] == ["libcamera", "ros2_common_cpp", "head_camera_cpp"]
    common, node = sources[1], sources[2]
    assert common["local"] == "shared/ros2_common_cpp"
    assert node["local"] == "nodes/head_camera_cpp"
    for source in (common, node):
        assert {k: source[k] for k in LOCAL_BUILD} == LOCAL_BUILD
        assert source["stamp_apt_packages"]
        assert set(source["stamp_apt_packages"]) <= set(node_type()["apt_packages"])


def test_no_apt_camera_stack_and_all_build_packages() -> None:
    apt = node_type()["apt_packages"]
    for package in FORBIDDEN_APT_PACKAGES:
        assert package not in apt, package
    for package in REQUIRED_APT_PACKAGES:
        assert package in apt, package


def test_background_tier() -> None:
    entry = node_type()
    assert {k: entry[k] for k in TIER} == TIER


def test_config_path_and_env() -> None:
    entry = node_type()
    assert entry["config_path"] == "/etc/ros2/head_camera_cpp"
    assert entry["env"] == ["HEAD_CAMERA_CONFIG=/etc/ros2/head_camera_cpp/config.yaml"]


def test_client_playbook_deploys_head_camera_cpp_right_after_pi_cameras() -> None:
    play = yaml.safe_load(CLIENT_PLAYBOOK.read_text())[0]
    blocks = [t for t in play["tasks"] if "block" in t]
    steps = next(b["block"] for b in blocks if any(s["name"] == "Deploy pi_cameras" for s in b["block"]))
    names = [s["name"] for s in steps]
    assert names.index("Deploy head_camera_cpp") == names.index("Deploy pi_cameras") + 1
    step = steps[names.index("Deploy head_camera_cpp")]
    include = step["ansible.builtin.include_tasks"]
    assert include["file"] == "tasks/resolve_and_deploy.yml"
    assert include["apply"]["tags"] == ["head_camera_cpp"]
    assert step["vars"] == {"_deploy_node_name": "head_camera_cpp"}
    assert step["tags"] == ["head_camera_cpp", "apt", "python", "build", "config"]


def test_docs_describe_head_camera_cpp() -> None:
    node_readme = (NODE_DIR / "README.md").read_text()
    assert "## Zoom interface" in node_readme
    assert "## Configuration" in node_readme
    for doc in (
        REPO_ROOT / "nodes" / "README.md",
        REPO_ROOT / "ansible" / "README.md",
        REPO_ROOT / ".claude" / "skills" / "ansible-deploy" / "SKILL.md",
        REPO_ROOT / "MEMORY.md",
        NODE_DIR / "README.md",
    ):
        assert "head_camera_cpp" in doc.read_text(), doc
    for doc in (REPO_ROOT / "nodes" / "README.md", REPO_ROOT / ".claude" / "skills" / "ansible-deploy" / "SKILL.md"):
        lines = [line for line in doc.read_text().splitlines() if "pi_cameras" in line]
        assert any("etired" in line for line in lines), doc
