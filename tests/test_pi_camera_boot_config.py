"""Static invariants of the Pi 5 CSI boot config (ansible/playbooks/tasks/pi_camera_boot_config.yml).

CSI cam0 keeps its Camera Module 3 overlay (IMX708) for front-and-back-camera-module (plan Q8); the Arducam ToF on
cam1 is retired with the D435 switchover, so its overlay is removed like the retired IMX219 overlays. Auto detection
stays off and the cam0 overlay is written explicitly; the client reboots only when a line changed.
"""

from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
ANSIBLE = REPO_ROOT / "ansible"
CLIENT_PLAYBOOK = ANSIBLE / "playbooks" / "deploy_nodes_client.yml"
BOOT_TASKS = ANSIBLE / "playbooks" / "tasks" / "pi_camera_boot_config.yml"
ANSIBLE_README = ANSIBLE / "README.md"
TESTS_README = REPO_ROOT / "tests" / "README.md"
CONFIG_TXT = "/boot/firmware/config.txt"
RETIRED_OVERLAYS = ("dtoverlay=imx219,cam0", "dtoverlay=imx219,cam1")
TOF_OVERLAY = "dtoverlay=arducam-pivariety,cam1"
WRITTEN_LINES = ("camera_auto_detect=0", "dtoverlay=imx708,cam0")
INCLUDE_TAGS = ["boot", "pi_cameras", "arducam_tof", "head_camera_cpp", "front_camera"]


def boot_tasks() -> list[dict]:
    """Load the boot config task file.

    Returns:
        list[dict]: Its tasks.
    """
    return yaml.safe_load(BOOT_TASKS.read_text())


def lineinfile_modules() -> list[dict]:
    """Return the lineinfile module arguments of the boot config tasks.

    Returns:
        list[dict]: ansible.builtin.lineinfile arguments, in file order.
    """
    return [t["ansible.builtin.lineinfile"] for t in boot_tasks() if "ansible.builtin.lineinfile" in t]


def test_retired_imx219_overlays_are_removed() -> None:
    absent = {m["regexp"]: m for m in lineinfile_modules() if m.get("state") == "absent"}
    for overlay in RETIRED_OVERLAYS:
        assert absent[f"^{overlay}$"]["path"] == CONFIG_TXT


def test_tof_overlay_is_removed_and_cam0_kept() -> None:
    """The cam1 ToF overlay is absent; auto detection stays off and the cam0 imx708 overlay is still written."""
    absent = {m["regexp"]: m for m in lineinfile_modules() if m.get("state") == "absent"}
    assert absent[f"^{TOF_OVERLAY}$"]["path"] == CONFIG_TXT
    written = {m["line"]: m for m in lineinfile_modules() if m.get("state") != "absent"}
    assert set(written) == set(WRITTEN_LINES)
    assert written["camera_auto_detect=0"]["regexp"] == "^camera_auto_detect="
    for line in WRITTEN_LINES:
        assert written[line]["path"] == CONFIG_TXT


def test_every_line_task_is_aarch64_only_and_registers_its_result() -> None:
    tasks = [t for t in boot_tasks() if "ansible.builtin.lineinfile" in t]
    assert len(tasks) == 5
    for task in tasks:
        assert task["when"] == 'ansible_machine == "aarch64"'
        assert "boot" in task["tags"]
    assert len({t["register"] for t in tasks}) == 5


def test_client_reboots_only_when_a_line_changed() -> None:
    tasks = boot_tasks()
    reboot = tasks[-1]
    assert reboot["ansible.builtin.reboot"]["reboot_timeout"] == 120
    for task in tasks[:-1]:
        assert f"{task['register']}.changed" in reboot["when"], task["register"]


def test_client_playbook_includes_the_boot_config_under_the_camera_node_tags() -> None:
    """Deploying a retired camera node (arducam_tof, head_camera_cpp, pi_cameras) still runs the boot config, so its
    uninstall deploy also cleans config.txt."""
    play = yaml.safe_load(CLIENT_PLAYBOOK.read_text())[0]
    include = next(
        t for t in play["pre_tasks"] if "pi_camera_boot_config.yml" in str(t.get("ansible.builtin.include_tasks"))
    )
    assert include["tags"] == INCLUDE_TAGS
    assert include["ansible.builtin.include_tasks"]["apply"]["tags"] == INCLUDE_TAGS


def test_ansible_readme_describes_the_boot_config() -> None:
    readme = ANSIBLE_README.read_text()
    assert "pi_camera_boot_config.yml" in readme
    for line in (*WRITTEN_LINES, TOF_OVERLAY):
        assert line in readme, line


def test_tests_readme_lists_every_test_of_this_file() -> None:
    readme = TESTS_README.read_text()
    assert "### `test_pi_camera_boot_config.py`" in readme
    for name in [n for n in globals() if n.startswith("test_")]:
        assert f"`{name}`" in readme, name
