"""Static invariants of the CSI Camera Module 3 deploy: the front_camera node type and entry in client.yml.

front_camera runs the head_camera_cpp binary on CSI cam0 as the body-fixed front camera (front-and-back-camera-module).
It builds against the same pinned libcamera as the retired pi_cameras: the libcamera source dict is identical, so the
build stamp matches and nothing rebuilds. The patch is a byte-identical copy that outlives the retirement of
pi_cameras. The head_camera_cpp type only uninstalls the retired head camera unit. The static TF of the camera comes
only from the user's tape measurement (`front_camera_mount`, null until entered): with null no camera frame is
published.
"""

import hashlib
import math
from pathlib import Path

import jinja2
import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_PLAYBOOK = REPO_ROOT / "ansible" / "playbooks" / "deploy_nodes_client.yml"
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
NODE_DIR = REPO_ROOT / "nodes" / "head_camera_cpp"
PATCH = NODE_DIR / "patches" / "libcamera-0001-add-package-xml.patch"
WORKSPACE = "/opt/ros2-ws"
FRONT = "front_camera"
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
CAM0_ID = "/base/axi/pcie@120000/rp1/i2c@88000/imx708@1a"
FRONT_CONFIG = {
    "camera_id": CAM0_ID,
    "frame_id": "front_camera_optical_frame",
    "topic_namespace": "/front_camera",
    "width": 1152,
    "height": 648,
    "frame_duration_us": 66666,
    "af_mode": "continuous",
    "jpeg_quality": 70,
    "camera_info_file": "/var/lib/ros2/head_rgbd_calibration/applied/front_camera_calibration.yaml",
    "metrics_port": 19121,
    "zoom": {"enabled": False},
    "mapping": {"enabled": False},
}
# A mount only for this test: the real values come from the user's tape measurement.
TEST_MOUNT = {"x": 0.21, "y": -0.03, "z": 0.34, "roll": 0.0, "pitch": 0.1, "yaw": 0.05}
# REP-103 optical frame: z forward, x right, y down, relative to the x-forward camera link.
OPTICAL_RPY = (-math.pi / 2, 0.0, -math.pi / 2)


def client_vars() -> dict:
    """Load group_vars/client.yml.

    Returns:
        dict: Parsed YAML.
    """
    return yaml.safe_load(CLIENT_VARS.read_text())


def node_type(name: str = FRONT) -> dict:
    """Return a node type's defaults.

    Args:
        name (str): Node type.

    Returns:
        dict: ros2_node_type_defaults[name].
    """
    return client_vars()["ros2_node_type_defaults"][name]


def node_entry(name: str) -> dict:
    """Return a ros2_nodes entry.

    Args:
        name (str): Node name.

    Returns:
        dict: The entry.
    """
    return next(n for n in client_vars()["ros2_nodes"] if n["name"] == name)


def rendered_static_frames(**mounts: object) -> list[dict]:
    """Render the static_tf_publisher config the way Ansible does (Jinja, trim_blocks) and return its frames.

    Args:
        **mounts (object): front_camera_mount (and any other mount variable) to render with.

    Returns:
        list[dict]: The frames of the rendered config.
    """
    env = jinja2.Environment(trim_blocks=True, undefined=jinja2.StrictUndefined)
    text = env.from_string(node_entry("static_tf_publisher")["config"]).render(**mounts)
    return yaml.safe_load(text)["frames"]


def test_node_type_fields() -> None:
    entry = node_type()
    assert entry["deploy_mode"] == "native"
    assert (
        entry["node_launch_command"] == "ros2 run head_camera_cpp head_camera_node --ros-args -r __node:=front_camera"
    )
    assert entry["node_src_dir"] == ""
    assert entry["src_paths"] == []


def test_libcamera_source_is_the_pinned_dict_of_pi_cameras() -> None:
    assert node_type()["colcon_source"][0] == LIBCAMERA_SOURCE


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
    assert entry["config_path"] == "/etc/ros2/front_camera"
    assert entry["env"] == ["HEAD_CAMERA_CONFIG=/etc/ros2/front_camera/config.yaml"]


def test_head_camera_cpp_type_keeps_only_the_uninstall_path() -> None:
    entry = node_type("head_camera_cpp")
    assert entry == {"deploy_mode": "native", "node_launch_command": "ros2 run head_camera_cpp head_camera_node"}


def test_front_camera_entry_is_present_enabled_with_the_design_config() -> None:
    entry = node_entry(FRONT)
    assert entry["node_type"] == FRONT
    assert entry["present"] is True
    assert entry["enabled"] is True
    assert yaml.safe_load(entry["config"]) == FRONT_CONFIG


def test_front_camera_is_scraped_on_19121() -> None:
    targets = client_vars()["monitoring_agent_node_targets"]
    assert {"node": FRONT, "address": "127.0.0.1:19121", "path": "/metrics"} in targets
    assert sum(t["address"].endswith(":19121") for t in targets) == 1


def test_client_playbook_deploys_front_camera_after_head_camera_cpp() -> None:
    play = yaml.safe_load(CLIENT_PLAYBOOK.read_text())[0]
    blocks = [t for t in play["tasks"] if "block" in t]
    steps = next(b["block"] for b in blocks if any(s["name"] == "Deploy head_camera_cpp" for s in b["block"]))
    names = [s["name"] for s in steps]
    assert names.index("Deploy front_camera") == names.index("Deploy head_camera_cpp") + 1
    step = steps[names.index("Deploy front_camera")]
    include = step["ansible.builtin.include_tasks"]
    assert include["file"] == "tasks/resolve_and_deploy.yml"
    assert include["apply"]["tags"] == [FRONT]
    assert step["vars"] == {"_deploy_node_name": FRONT}
    assert step["tags"] == [FRONT, "apt", "python", "build", "config"]


def test_boot_config_include_carries_the_front_camera_tag() -> None:
    play = yaml.safe_load(CLIENT_PLAYBOOK.read_text())[0]
    include = next(
        t for t in play["pre_tasks"] if "pi_camera_boot_config.yml" in str(t.get("ansible.builtin.include_tasks"))
    )
    assert FRONT in include["tags"]
    assert FRONT in include["ansible.builtin.include_tasks"]["apply"]["tags"]


def test_front_camera_mount_is_null_until_measured() -> None:
    variables = client_vars()
    assert "front_camera_mount" in variables
    assert variables["front_camera_mount"] is None


def test_no_front_camera_frame_without_a_measured_mount() -> None:
    frames = rendered_static_frames(front_camera_mount=None)
    assert not [f for f in frames if "front_camera" in f["child"] or "front_camera" in f.get("parent", "")]
    assert {"imu_link", "laser_frame"} <= {f["child"] for f in frames}


def test_measured_mount_publishes_link_and_optical_frame() -> None:
    frames = {f["child"]: f for f in rendered_static_frames(front_camera_mount=TEST_MOUNT)}
    link = frames["front_camera_link"]
    assert link["parent"] == "base_link"
    assert {k: float(link[k]) for k in TEST_MOUNT} == TEST_MOUNT
    optical = frames["front_camera_optical_frame"]
    assert optical["parent"] == "front_camera_link"
    assert (float(optical["x"]), float(optical["y"]), float(optical["z"])) == (0.0, 0.0, 0.0)
    rpy = (float(optical["roll"]), float(optical["pitch"]), float(optical["yaw"]))
    assert rpy == pytest.approx(OPTICAL_RPY, abs=1e-9)


def test_partial_mount_fails_to_render() -> None:
    """A mount missing a key is a render error (Ansible's undefined variable), never a defaulted 0."""
    partial = {k: v for k, v in TEST_MOUNT.items() if k != "z"}
    with pytest.raises(jinja2.UndefinedError):
        rendered_static_frames(front_camera_mount=partial)


def test_docs_describe_head_camera_cpp() -> None:
    node_readme = (NODE_DIR / "README.md").read_text()
    assert "## Zoom interface" in node_readme
    assert "## Configuration" in node_readme
    assert "## Instances: front_camera and back_camera" in node_readme
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
