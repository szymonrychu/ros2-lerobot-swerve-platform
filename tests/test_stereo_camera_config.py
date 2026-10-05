"""Static invariants of the stereo_camera node: boot overlays, source build, Ansible wiring, launch and params.

No ROS or Raspberry Pi is available on the dev machine, so these tests read the repo files directly (YAML via
yaml.safe_load, launch files via ast) and pin the pipeline of two Arducam B0184 (IMX219) cameras on CSI cam0/cam1
published through a libcamera fork + camera_ros built from source.
"""

import ast
import re
from pathlib import Path
from typing import Any

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
ANSIBLE_DIR = REPO_ROOT / "ansible"
CLIENT_VARS = ANSIBLE_DIR / "group_vars" / "client.yml"
PLAYBOOKS_DIR = ANSIBLE_DIR / "playbooks"
DEPLOY_CLIENT = PLAYBOOKS_DIR / "deploy_nodes_client.yml"
NODE_PLAYBOOK = PLAYBOOKS_DIR / "nodes" / "client" / "stereo_camera.yml"
BOOT_TASKS = PLAYBOOKS_DIR / "tasks" / "stereo_camera_boot_config.yml"
RF2O_PLAYBOOK = PLAYBOOKS_DIR / "nodes" / "client" / "rf2o_laser_odometry.yml"
COLCON_TASKS = ANSIBLE_DIR / "roles" / "ros2_node_deploy" / "tasks" / "colcon_source_build.yml"
COLCON_ONE_TASKS = ANSIBLE_DIR / "roles" / "ros2_node_deploy" / "tasks" / "colcon_source_package.yml"
LAUNCHER_TEMPLATE = ANSIBLE_DIR / "roles" / "ros2_node_deploy" / "templates" / "ros2-node-launcher.j2"
NODE_DIR = REPO_ROOT / "nodes" / "stereo_camera"
LAUNCH_FILE = NODE_DIR / "launch" / "stereo_camera.launch.py"
PARAMS_FILE = NODE_DIR / "config" / "params.yaml"
PATCHES_DIR = NODE_DIR / "patches"
CALIBRATION_DIR = NODE_DIR / "calibration"
README = NODE_DIR / "README.md"
TESTS_README = REPO_ROOT / "tests" / "README.md"

BOOT_CONFIG_PATH = "/boot/firmware/config.txt"
LIBCAMERA_TAG = "v0.7.2+rpt20260817"
CAMERA_ROS_COMMIT = "8f792e27a6dbc81e4943a75765fc1b7b7d37b301"
LIBCAMERA_MESON_ARGS = [
    "-Dpipelines=rpi/pisp",
    "-Dipas=rpi/pisp",
    "-Dcam=enabled",
    "-Dqcam=disabled",
    "-Dgstreamer=disabled",
    "-Dv4l2=false",
    "-Dtest=false",
    "-Ddocumentation=disabled",
    "-Dpycamera=disabled",
]
STEREO_APT_PACKAGES = [
    "python3-colcon-meson",
    "meson",
    "ninja-build",
    "python3-ply",
    "python3-jinja2",
    "libyaml-dev",
    "libgnutls28-dev",
    "ros-jazzy-image-proc",
    "ros-jazzy-stereo-image-proc",
    "ros-jazzy-camera-calibration",
]
FORBIDDEN_APT_PACKAGES = ("ros-jazzy-camera-ros", "ros-jazzy-libcamera", "libcamera-dev", "libcamera0")


def client_vars() -> dict:
    """Load group_vars/client.yml.

    Returns:
        dict: Parsed YAML.
    """
    return yaml.safe_load(CLIENT_VARS.read_text())


def node_entry(name: str) -> dict:
    """Return one ros2_nodes entry of the client by name.

    Args:
        name: Node name.

    Returns:
        dict: The ros2_nodes entry.
    """
    return next(n for n in client_vars()["ros2_nodes"] if n["name"] == name)


def type_defaults(name: str) -> dict:
    """Return one ros2_node_type_defaults entry of the client.

    Args:
        name: Node type.

    Returns:
        dict: The node type defaults.
    """
    return client_vars()["ros2_node_type_defaults"][name]


def load_yaml(path: Path) -> Any:
    """Parse a YAML file.

    Args:
        path: YAML file.

    Returns:
        Any: Parsed document.
    """
    return yaml.safe_load(path.read_text())


def boot_tasks() -> list[dict]:
    """Tasks of the shared stereo_camera boot-config task file.

    Returns:
        list[dict]: Parsed tasks.
    """
    return load_yaml(BOOT_TASKS)


# --- boot overlays -------------------------------------------------------------------------------------------------


def test_boot_tasks_set_camera_overlays_in_firmware_config() -> None:
    lineinfile = [t["ansible.builtin.lineinfile"] for t in boot_tasks() if "ansible.builtin.lineinfile" in t]
    assert {task["path"] for task in lineinfile} == {BOOT_CONFIG_PATH}
    assert {task["line"] for task in lineinfile} == {
        "camera_auto_detect=0",
        "dtoverlay=imx219,cam0",
        "dtoverlay=imx219,cam1",
    }
    for task in lineinfile:
        # Each line is replaced in place (regexp) so a rerun never appends a duplicate.
        assert re.search(task["regexp"], task["line"]), task


def test_boot_tasks_register_results_and_reboot_only_when_changed() -> None:
    tasks = boot_tasks()
    registered = [t["register"] for t in tasks if "ansible.builtin.lineinfile" in t]
    assert len(registered) == 3 and len(set(registered)) == 3
    reboot = [t for t in tasks if "ansible.builtin.reboot" in t]
    assert len(reboot) == 1
    condition = reboot[0]["when"]
    assert isinstance(condition, str)
    for name in registered:
        assert f"{name}.changed" in condition
    assert reboot[0]["ansible.builtin.reboot"]["reboot_timeout"] == 120
    assert tasks[-1] is reboot[0], "reboot comes after every overlay task"


def test_boot_overlays_use_the_same_path_as_the_uart_task() -> None:
    uart = [
        t["ansible.builtin.lineinfile"]
        for t in load_yaml(DEPLOY_CLIENT)[0]["pre_tasks"]
        if "ansible.builtin.lineinfile" in t and t["ansible.builtin.lineinfile"]["line"] == "enable_uart=1"
    ]
    assert uart and uart[0]["path"] == BOOT_CONFIG_PATH


def include_names(tasks: list[dict]) -> list[str]:
    """Included task files of a task list.

    Args:
        tasks: Parsed tasks.

    Returns:
        list[str]: The include_tasks targets.
    """
    return [t["ansible.builtin.include_tasks"] for t in tasks if isinstance(t.get("ansible.builtin.include_tasks"), str)]


def test_both_client_playbooks_apply_the_boot_overlays_in_pre_tasks() -> None:
    full = load_yaml(DEPLOY_CLIENT)[0]["pre_tasks"]
    single = load_yaml(NODE_PLAYBOOK)[0]["pre_tasks"]
    assert "tasks/stereo_camera_boot_config.yml" in include_names(full)
    assert "../../tasks/stereo_camera_boot_config.yml" in include_names(single)
    # Boot config first after the stop-all, before the repo sync, in both.
    assert include_names(full).index("tasks/stop_ros_nodes.yml") < include_names(full).index(
        "tasks/stereo_camera_boot_config.yml"
    )
    assert include_names(single).index("../../tasks/stop_ros_nodes.yml") < include_names(single).index(
        "../../tasks/stereo_camera_boot_config.yml"
    )


def test_boot_overlays_are_only_written_on_aarch64_hosts() -> None:
    for task in boot_tasks():
        if "ansible.builtin.lineinfile" in task:
            assert task["when"] == 'ansible_machine == "aarch64"'


# --- per-node playbook ---------------------------------------------------------------------------------------------


def test_per_node_playbook_stops_syncs_deploys_and_starts_gradually() -> None:
    play = load_yaml(NODE_PLAYBOOK)[0]
    assert play["hosts"] == "client" and play["become"] is True
    pre = include_names(play["pre_tasks"])
    assert pre[0] == "../../tasks/stop_ros_nodes.yml"
    assert "../../tasks/repo_sync.yml" in pre
    deployed = [t["vars"]["_deploy_node_name"] for t in play["tasks"]]
    assert deployed == ["stereo_camera"]
    assert include_names(play["post_tasks"]) == ["../../tasks/start_ros_nodes.yml"]


def test_full_client_playbook_deploys_stereo_camera_and_not_realsense_before_it() -> None:
    tasks = load_yaml(DEPLOY_CLIENT)[0]["tasks"]
    deployed = [t.get("vars", {}).get("_deploy_node_name") for t in tasks]
    assert "stereo_camera" in deployed
    # Uninstalling the RealSense unit must happen before the new camera starts.
    assert deployed.index("realsense_d435i") < deployed.index("stereo_camera")


# --- colcon multi-source build ---------------------------------------------------------------------------------------


def test_rf2o_build_inputs_are_unchanged() -> None:
    source = type_defaults("rf2o_laser_odometry")["colcon_source"]
    assert isinstance(source, dict), "rf2o keeps the single-dict form"
    assert source == {
        "repo": "https://github.com/MAPIRlab/rf2o_laser_odometry.git",
        "commit": "b38c68e46387b98845ecbfeb6660292f967a00d3",
        "package": "rf2o_laser_odometry",
        "workspace": "/opt/ros2-ws",
        "patches": ["{{ ros2_repo_dest }}/nodes/rf2o_laser_odometry/patches/0001-retry-laser-tf.patch"],
    }


def test_colcon_build_accepts_one_dict_or_a_list_of_sources() -> None:
    text = COLCON_TASKS.read_text()
    assert "node_colcon_source is mapping" in text, "a single dict is wrapped into a one-element list"
    assert "colcon_source_package.yml" in text
    tasks = load_yaml(COLCON_TASKS)
    loops = [t for t in tasks if t.get("ansible.builtin.include_tasks") == "colcon_source_package.yml"]
    assert len(loops) == 1 and loops[0]["loop_control"]["loop_var"] == "colcon_src"
    assert "_colcon_sources" in str(loops[0]["loop"]), "sources are built in list order"


def test_colcon_package_build_keeps_the_limits_and_stamp_logic() -> None:
    text = COLCON_ONE_TASKS.read_text()
    for needle in (
        "systemd-run --quiet --scope -p CPUQuota=100% -p IOWeight=10 nice -n 19 ionice -c 3",
        "colcon build --merge-install --parallel-workers 1",
        "--packages-select {{ colcon_src.package }}",
        "MAKEFLAGS: \"-j2\"",
        "source /opt/ros/jazzy/setup.bash",
        "install/setup.bash",
        "creates:",
        "Restart ROS2 node",
        "force: true",
    ):
        assert needle in text, needle
    # The rf2o default (no cmake_args given) stays Release.
    assert "-DCMAKE_BUILD_TYPE=Release" in text
    assert "--meson-args" in text and "--cmake-args" in text


def test_colcon_stamp_key_is_unchanged_for_the_first_source_and_chained_after() -> None:
    text = COLCON_ONE_TASKS.read_text()
    assert "colcon_src.commit ~ (_colcon_patch_stats" in text
    # Later sources are rebuilt when an earlier one (their dependency) changes.
    assert "_colcon_chain_key" in text


def test_stereo_camera_node_type_builds_libcamera_then_camera_ros() -> None:
    sources = type_defaults("stereo_camera")["colcon_source"]
    assert isinstance(sources, list) and [s["package"] for s in sources] == ["libcamera", "camera_ros"]
    libcamera, camera_ros = sources
    assert libcamera["repo"] == "https://github.com/raspberrypi/libcamera.git"
    assert libcamera["commit"] == LIBCAMERA_TAG
    for arg in LIBCAMERA_MESON_ARGS:
        assert arg in libcamera["meson_args"], arg
    assert "cmake_args" not in libcamera
    assert camera_ros["repo"] == "https://github.com/christianrauch/camera_ros.git"
    assert camera_ros["commit"] == CAMERA_ROS_COMMIT
    assert "meson_args" not in camera_ros
    for source in sources:
        assert source["workspace"] == "/opt/ros2-ws"


def test_source_patches_are_listed_and_exist_in_the_repo() -> None:
    libcamera, camera_ros = type_defaults("stereo_camera")["colcon_source"]
    assert libcamera["patches"] == ["{{ ros2_repo_dest }}/nodes/stereo_camera/patches/libcamera-0001-add-package-xml.patch"]
    assert camera_ros["patches"] == [
        "{{ ros2_repo_dest }}/nodes/stereo_camera/patches/camera_ros-0001-expose-sync-controls.patch"
    ]
    for patch in (*libcamera["patches"], *camera_ros["patches"]):
        assert (PATCHES_DIR / Path(patch).name).is_file()


def test_camera_ros_patch_exposes_the_sync_controls() -> None:
    patch = (PATCHES_DIR / "camera_ros-0001-expose-sync-controls.patch").read_text()
    assert "+++ b/src/type_extent.cpp" in patch
    assert "+  IF(rpi::SyncMode)" in patch
    assert "+  IF(rpi::SyncFrames)" in patch


def test_libcamera_patch_adds_a_meson_package_xml() -> None:
    patch = (PATCHES_DIR / "libcamera-0001-add-package-xml.patch").read_text()
    assert "+++ b/package.xml" in patch
    assert "+    <build_type>meson</build_type>" in patch
    assert "+  <name>libcamera</name>" in patch


def test_apt_packages_carry_the_build_deps_and_never_the_apt_camera_stack() -> None:
    packages = type_defaults("stereo_camera")["apt_packages"]
    for package in STEREO_APT_PACKAGES:
        assert package in packages, package
    for package in FORBIDDEN_APT_PACKAGES:
        assert package not in packages, f"{package} would shadow the source-built fork"
    for name, defaults in client_vars()["ros2_node_type_defaults"].items():
        assert not set(defaults.get("apt_packages", [])) & set(FORBIDDEN_APT_PACKAGES), name


def test_launcher_template_sources_the_first_source_workspace_for_a_list() -> None:
    text = LAUNCHER_TEMPLATE.read_text()
    assert "node_colcon_source is mapping" in text
    assert "install/setup.bash" in text


# --- node type, node entry, realsense, tf --------------------------------------------------------------------------------


def test_stereo_camera_node_type_resources_and_launch_command() -> None:
    defaults = type_defaults("stereo_camera")
    assert defaults["deploy_mode"] == "native"
    assert defaults["node_src_dir"] == ""
    assert defaults["cpu_quota"] == "150%"
    assert defaults["memory_max"] == "384M"
    assert defaults["nice"] == 5
    assert defaults["config_path"] == "/etc/ros2/stereo_camera"
    assert "STEREO_CAMERA_CONFIG=/etc/ros2/stereo_camera/config.yaml" in defaults["env"]
    assert defaults["node_launch_command"].strip() == (
        "ros2 launch {{ ros2_repo_dest }}/nodes/stereo_camera/launch/stereo_camera.launch.py"
    )


def test_stereo_camera_node_entry_is_present_and_enabled_after_the_lidar() -> None:
    entry = node_entry("stereo_camera")
    assert entry["node_type"] == "stereo_camera"
    assert entry["present"] is True and entry["enabled"] is True
    config = yaml.safe_load(entry["config"])
    # Until the libcamera IDs are read with `cam -l`, the cameras are selected by index (the launch logs a warning).
    assert set(config) <= {"left_camera_id", "right_camera_id", "publish_points"}


def test_realsense_is_uninstalled() -> None:
    entry = node_entry("realsense_d435i")
    assert entry["present"] is False


def test_guessed_camera_link_frame_is_removed_from_the_static_tf_config() -> None:
    config = yaml.safe_load(node_entry("static_tf_publisher")["config"])
    children = [frame["child"] for frame in config["frames"]]
    assert "camera_link" not in children
    assert {"imu_link", "laser_frame"} <= set(children)


# --- node files ------------------------------------------------------------------------------------------------------


def test_node_directory_layout_and_no_fake_calibration_committed() -> None:
    for path in (LAUNCH_FILE, PARAMS_FILE, README, CALIBRATION_DIR / "README.md"):
        assert path.is_file(), path
    committed = sorted(p.name for p in CALIBRATION_DIR.iterdir())
    assert "left.yaml" not in committed and "right.yaml" not in committed
    calibration_readme = (CALIBRATION_DIR / "README.md").read_text()
    assert "left.yaml" in calibration_readme and "right.yaml" in calibration_readme


def params() -> dict:
    """Parse nodes/stereo_camera/config/params.yaml.

    Returns:
        dict: Parsed defaults.
    """
    return load_yaml(PARAMS_FILE)


def test_camera_defaults_match_the_requirements() -> None:
    camera = params()["camera"]
    assert camera["sensor_mode"] == "1640:1232"
    assert camera["width"] == 320 and camera["height"] == 240
    assert camera["role"] == "video"
    assert camera["FrameDurationLimits"] == [66666, 66666]
    assert camera["AeExposureMode"] == "short"
    assert params()["left"]["sync_mode"] == "server"
    assert params()["right"]["sync_mode"] == "client"
    assert params()["left"]["frame_id"] == "stereo_left_optical_frame"
    assert params()["right"]["frame_id"] == "stereo_right_optical_frame"


def test_disparity_defaults_match_the_requirements() -> None:
    assert params()["disparity"] == {
        "stereo_algorithm": 1,
        "sgbm_mode": 2,
        "correlation_window_size": 5,
        "disparity_range": 64,
        "min_disparity": 0,
        "P1": 200.0,
        "P2": 800.0,
        "uniqueness_ratio": 10.0,
        "speckle_size": 50,
        "speckle_range": 2,
        "disp12_max_diff": 1,
        "prefilter_cap": 31,
        "approximate_sync": True,
        "approximate_sync_tolerance_seconds": 0.002,
    }


def test_launch_settings_default_to_no_ids_and_no_point_cloud() -> None:
    launch = params()["launch"]
    assert launch["left_camera_id"] == "" and launch["right_camera_id"] == ""
    assert launch["publish_points"] is False


def launch_source() -> str:
    """Text of the launch file.

    Returns:
        str: Python source.
    """
    return LAUNCH_FILE.read_text()


def test_launch_file_is_valid_python_with_a_launch_description() -> None:
    tree = ast.parse(launch_source())
    functions = {n.name for n in tree.body if isinstance(n, ast.FunctionDef)}
    assert "generate_launch_description" in functions


@pytest.mark.parametrize(
    "needle",
    [
        "component_container_mt",
        "use_intra_process_comms",
        "camera::CameraNode",
        "image_proc::RectifyNode",
        "stereo_image_proc::DisparityNode",
        "stereo_image_proc::PointCloudNode",
        "/stereo/left",
        "/stereo/right",
    ],
)
def test_launch_file_builds_the_composable_pipeline(needle: str) -> None:
    assert needle in launch_source()


def test_launch_file_uses_the_gating_helper() -> None:
    source = launch_source()
    assert "from stereo_camera_config import" in source
    assert "calibration_ready" in source and "publish_points" in source


def test_every_new_root_test_file_is_documented() -> None:
    readme = TESTS_README.read_text()
    for path in sorted((REPO_ROOT / "tests").glob("test_stereo_camera*.py")):
        assert f"### `{path.name}`" in readme or f"### {path.name}" in readme, path.name
        for name in re.findall(r"^def (test_\w+)", path.read_text(), flags=re.M):
            assert f"`{name}`" in readme, f"{path.name}::{name} is not documented in tests/README.md"
