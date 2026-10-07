"""Static invariants of the overview_camera node: boot overlay, source build, Ansible wiring, launch and params.

No ROS or Raspberry Pi is available on the dev machine, so these tests read the repo files directly (YAML via
yaml.safe_load, launch files via ast) and pin the pipeline of one Raspberry Pi Camera Module 3 (IMX708) on CSI cam0
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
NODE_PLAYBOOK = PLAYBOOKS_DIR / "nodes" / "client" / "overview_camera.yml"
BOOT_TASKS = PLAYBOOKS_DIR / "tasks" / "overview_camera_boot_config.yml"
RF2O_PLAYBOOK = PLAYBOOKS_DIR / "nodes" / "client" / "rf2o_laser_odometry.yml"
COLCON_TASKS = ANSIBLE_DIR / "roles" / "ros2_node_deploy" / "tasks" / "colcon_source_build.yml"
COLCON_ONE_TASKS = ANSIBLE_DIR / "roles" / "ros2_node_deploy" / "tasks" / "colcon_source_package.yml"
LAUNCHER_TEMPLATE = ANSIBLE_DIR / "roles" / "ros2_node_deploy" / "templates" / "ros2-node-launcher.j2"
NODE_DIR = REPO_ROOT / "nodes" / "overview_camera"
LAUNCH_FILE = NODE_DIR / "launch" / "overview_camera.launch.py"
PARAMS_FILE = NODE_DIR / "config" / "params.yaml"
PATCHES_DIR = NODE_DIR / "patches"
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
CAMERA_APT_PACKAGES = [
    "python3-colcon-meson",
    "meson",
    "ninja-build",
    "python3-ply",
    "python3-jinja2",
    "libyaml-dev",
    "libgnutls28-dev",
    "ros-jazzy-image-transport",
    "ros-jazzy-image-transport-plugins",
    "ros-jazzy-compressed-image-transport",
]
REMOVED_APT_PACKAGES = ("ros-jazzy-stereo-image-proc", "ros-jazzy-image-proc", "ros-jazzy-camera-calibration")
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
    """Tasks of the shared overview_camera boot-config task file.

    Returns:
        list[dict]: Parsed tasks.
    """
    return load_yaml(BOOT_TASKS)


# --- boot overlays -------------------------------------------------------------------------------------------------


def test_boot_tasks_set_the_imx708_overlay_in_firmware_config() -> None:
    present = [
        t["ansible.builtin.lineinfile"]
        for t in boot_tasks()
        if "ansible.builtin.lineinfile" in t and t["ansible.builtin.lineinfile"].get("state") != "absent"
    ]
    assert {task["path"] for task in present} == {BOOT_CONFIG_PATH}
    assert {task["line"] for task in present} == {"camera_auto_detect=0", "dtoverlay=imx708,cam0"}
    for task in present:
        # Each line is replaced in place (regexp) so a rerun never appends a duplicate.
        assert re.search(task["regexp"], task["line"]), task


def test_boot_tasks_remove_stale_imx219_overlays_idempotently() -> None:
    absent = [
        t["ansible.builtin.lineinfile"]
        for t in boot_tasks()
        if "ansible.builtin.lineinfile" in t and t["ansible.builtin.lineinfile"].get("state") == "absent"
    ]
    assert len(absent) == 1
    task = absent[0]
    assert task["path"] == BOOT_CONFIG_PATH and "line" not in task
    for stale in ("dtoverlay=imx219,cam0", "dtoverlay=imx219,cam1", "dtoverlay=imx219"):
        assert re.search(task["regexp"], stale), stale
    for kept in ("dtoverlay=imx708,cam0", "camera_auto_detect=0", "# dtoverlay=imx219,cam0"):
        assert not re.search(task["regexp"], kept), kept
    assert boot_tasks()[0]["ansible.builtin.lineinfile"] == task, "stale lines go before the new ones are added"


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
    assert "tasks/overview_camera_boot_config.yml" in include_names(full)
    assert "../../tasks/overview_camera_boot_config.yml" in include_names(single)
    # Boot config first after the stop-all, before the repo sync, in both.
    assert include_names(full).index("tasks/stop_ros_nodes.yml") < include_names(full).index(
        "tasks/overview_camera_boot_config.yml"
    )
    assert include_names(single).index("../../tasks/stop_ros_nodes.yml") < include_names(single).index(
        "../../tasks/overview_camera_boot_config.yml"
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
    assert deployed == ["overview_camera"]
    assert include_names(play["post_tasks"]) == ["../../tasks/start_ros_nodes.yml"]


def test_full_client_playbook_deploys_overview_camera_and_not_realsense_before_it() -> None:
    tasks = load_yaml(DEPLOY_CLIENT)[0]["tasks"]
    deployed = [t.get("vars", {}).get("_deploy_node_name") for t in tasks]
    assert "overview_camera" in deployed
    # Uninstalling the RealSense unit must happen before the new camera starts.
    assert deployed.index("realsense_d435i") < deployed.index("overview_camera")


def test_retired_stereo_nodes_are_gone_from_the_client_wiring() -> None:
    text = DEPLOY_CLIENT.read_text() + CLIENT_VARS.read_text()
    for retired in ("stereo_depth", "stereo_camera", "STEREO_"):
        assert retired not in text, retired
    assert not (REPO_ROOT / "nodes" / "stereo_depth").exists()
    assert not (REPO_ROOT / "nodes" / "stereo_camera").exists()
    assert not (PLAYBOOKS_DIR / "nodes" / "client" / "stereo_depth.yml").exists()
    assert not (PLAYBOOKS_DIR / "nodes" / "client" / "stereo_camera.yml").exists()
    assert "stereo_depth" not in (REPO_ROOT / "scripts" / "lint-all-nodes.sh").read_text()


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
        "systemd-run --quiet --scope -p CPUQuota={{ ros2_build_cpu_quota }} -p IOWeight=10 nice -n 19 ionice -c 3",
        "colcon build --merge-install --parallel-workers 1",
        "--packages-select {{ colcon_src.package }}",
        "MAKEFLAGS: \"-j{{ ros2_build_jobs }}\"",
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


def test_overview_camera_node_type_builds_libcamera_then_camera_ros() -> None:
    sources = type_defaults("overview_camera")["colcon_source"]
    assert isinstance(sources, list) and [s["package"] for s in sources] == ["libcamera", "camera_ros"]
    libcamera, camera_ros = sources
    assert libcamera["repo"] == "https://github.com/raspberrypi/libcamera.git"
    assert libcamera["commit"] == LIBCAMERA_TAG
    for arg in LIBCAMERA_MESON_ARGS:
        assert arg in libcamera["meson_args"], arg
    assert "cmake_args" not in libcamera
    # colcon-meson passes --prefix and --libdir itself; meson rejects the same option given twice.
    assert not [a for a in libcamera["meson_args"] if a.startswith(("-Dlibdir", "-Dprefix"))], libcamera["meson_args"]
    assert camera_ros["repo"] == "https://github.com/christianrauch/camera_ros.git"
    assert camera_ros["commit"] == CAMERA_ROS_COMMIT
    assert "meson_args" not in camera_ros
    for source in sources:
        assert source["workspace"] == "/opt/ros2-ws"


def test_only_the_libcamera_patch_is_listed_and_it_exists_in_the_repo() -> None:
    libcamera, camera_ros = type_defaults("overview_camera")["colcon_source"]
    assert libcamera["patches"] == [
        "{{ ros2_repo_dest }}/nodes/overview_camera/patches/libcamera-0001-add-package-xml.patch"
    ]
    assert "patches" not in camera_ros, "the SyncMode patch is gone together with the stereo pair"
    assert [p.name for p in PATCHES_DIR.iterdir()] == ["libcamera-0001-add-package-xml.patch"]


def test_libcamera_patch_adds_a_meson_package_xml() -> None:
    patch = (PATCHES_DIR / "libcamera-0001-add-package-xml.patch").read_text()
    assert "+++ b/package.xml" in patch
    assert "+    <build_type>meson</build_type>" in patch
    assert "+  <name>libcamera</name>" in patch


def test_apt_packages_carry_the_build_deps_and_never_the_apt_camera_stack() -> None:
    packages = type_defaults("overview_camera")["apt_packages"]
    for package in CAMERA_APT_PACKAGES:
        assert package in packages, package
    for package in REMOVED_APT_PACKAGES:
        assert package not in packages, package
    for package in FORBIDDEN_APT_PACKAGES:
        assert package not in packages, f"{package} would shadow the source-built fork"
    for name, defaults in client_vars()["ros2_node_type_defaults"].items():
        assert not set(defaults.get("apt_packages", [])) & set(FORBIDDEN_APT_PACKAGES), name


def test_launcher_template_sources_the_first_source_workspace_for_a_list() -> None:
    text = LAUNCHER_TEMPLATE.read_text()
    assert "node_colcon_source is mapping" in text
    assert "install/setup.bash" in text


# --- node type, node entry, realsense, tf --------------------------------------------------------------------------------


def test_overview_camera_node_type_resources_and_launch_command() -> None:
    defaults = type_defaults("overview_camera")
    assert defaults["deploy_mode"] == "native"
    assert defaults["node_src_dir"] == ""
    assert defaults["cpu_quota"] == "100%"
    assert defaults["memory_max"] == "256M"
    assert defaults["nice"] == 5
    assert defaults["config_path"] == "/etc/ros2/overview_camera"
    assert "OVERVIEW_CAMERA_CONFIG=/etc/ros2/overview_camera/config.yaml" in defaults["env"]
    assert defaults["node_launch_command"].strip() == (
        "ros2 launch {{ ros2_repo_dest }}/nodes/overview_camera/launch/overview_camera.launch.py"
    )


def test_overview_camera_node_entry_is_present_and_enabled() -> None:
    entry = node_entry("overview_camera")
    assert entry["node_type"] == "overview_camera"
    assert entry["present"] is True and entry["enabled"] is True
    config = yaml.safe_load(entry["config"])
    # Until the libcamera ID is read with `cam -l`, the camera is selected by index 0 (the launch logs a warning).
    assert set(config) <= {"camera_id"}


def test_realsense_is_uninstalled() -> None:
    entry = node_entry("realsense_d435i")
    assert entry["present"] is False


def test_guessed_camera_link_frame_is_removed_from_the_static_tf_config() -> None:
    config = yaml.safe_load(node_entry("static_tf_publisher")["config"])
    children = [frame["child"] for frame in config["frames"]]
    assert "camera_link" not in children
    assert {"imu_link", "laser_frame"} <= set(children)


# --- node files ------------------------------------------------------------------------------------------------------


def test_node_directory_layout_without_calibration_or_stereo_leftovers() -> None:
    for path in (LAUNCH_FILE, LAUNCH_FILE.with_name("overview_camera_config.py"), PARAMS_FILE, README):
        assert path.is_file(), path
    assert not (NODE_DIR / "calibration").exists()
    readme = README.read_text()
    for needle in ("cam -l", "AfMode", "IMX708", "dtoverlay=imx708,cam0", "/overview_camera/image_raw/compressed"):
        assert needle in readme, needle


def params() -> dict:
    """Parse nodes/overview_camera/config/params.yaml.

    Returns:
        dict: Parsed defaults.
    """
    return load_yaml(PARAMS_FILE)


def test_camera_defaults_match_the_requirements() -> None:
    camera = params()["camera"]
    assert camera["width"] == 640 and camera["height"] == 480
    assert camera["FrameDurationLimits"] == [66666, 66666]
    assert camera["AfMode"] == "continuous"
    assert camera["frame_id"] == "overview_camera_optical_frame"
    assert camera["jpeg_quality"] == 80
    assert set(params()) == {"launch", "camera"}, "no disparity, sync or calibration sections"
    assert "SyncMode" not in camera and "sync_mode" not in camera


def test_launch_settings_default_to_no_camera_id() -> None:
    assert params()["launch"] == {"camera_id": ""}


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
        "/overview_camera",
        "image_raw/compressed",
        "camera_info",
    ],
)
def test_launch_file_builds_the_single_camera_pipeline(needle: str) -> None:
    assert needle in launch_source()


def test_launch_file_has_no_stereo_or_calibration_stages() -> None:
    source = launch_source()
    for retired in ("RectifyNode", "DisparityNode", "PointCloudNode", "calibration_ready", "CALIBRATION_DIR", "stereo", "SyncMode"):
        assert retired not in source, retired
    assert "from overview_camera_config import" in source


def test_every_new_root_test_file_is_documented() -> None:
    readme = TESTS_README.read_text()
    for path in sorted((REPO_ROOT / "tests").glob("test_overview_camera*.py")):
        assert f"### `{path.name}`" in readme or f"### {path.name}" in readme, path.name
        for name in re.findall(r"^def (test_\w+)", path.read_text(), flags=re.M):
            assert f"`{name}`" in readme, f"{path.name}::{name} is not documented in tests/README.md"


def test_builds_use_all_cores_at_lowest_priority_and_nodes_start_quickly() -> None:
    """Deploy speed: source and frontend builds may use all four cores (still nice 19 / idle IO), and the gradual
    start spaces node starts 2 s apart."""
    all_vars = load_yaml(ANSIBLE_DIR / "group_vars" / "all.yml")
    assert all_vars["ros2_build_jobs"] == 4
    assert all_vars["ros2_build_cpu_quota"] == "400%"
    assert all_vars["ros2_node_start_interval_s"] == 2
