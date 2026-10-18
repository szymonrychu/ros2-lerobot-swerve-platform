"""Static invariants of the Pi 5 CSI boot config (ansible/playbooks/tasks/pi_camera_boot_config.yml).

CSI cam0 carries a Camera Module 3 (IMX708, pi_cameras front preview) and cam1 the Arducam ToF camera. The firmware's
auto detection does not know the ToF, so the file turns it off and writes both overlays explicitly, removes the
retired IMX219 overlays, and reboots the client only when a line changed.
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
WRITTEN_LINES = ("camera_auto_detect=0", "dtoverlay=imx708,cam0", "dtoverlay=arducam-pivariety,cam1")
INCLUDE_TAGS = ["boot", "pi_cameras", "arducam_tof"]


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


def test_auto_detect_is_off_and_both_overlays_are_written() -> None:
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


def test_client_playbook_includes_the_boot_config_under_both_camera_node_tags() -> None:
    play = yaml.safe_load(CLIENT_PLAYBOOK.read_text())[0]
    include = next(
        t for t in play["pre_tasks"] if "pi_camera_boot_config.yml" in str(t.get("ansible.builtin.include_tasks"))
    )
    assert include["tags"] == INCLUDE_TAGS
    assert include["ansible.builtin.include_tasks"]["apply"]["tags"] == INCLUDE_TAGS


def test_ansible_readme_describes_the_boot_config() -> None:
    readme = ANSIBLE_README.read_text()
    assert "pi_camera_boot_config.yml" in readme
    for line in WRITTEN_LINES:
        assert line in readme, line


def test_tests_readme_lists_every_test_of_this_file() -> None:
    readme = TESTS_README.read_text()
    assert "### `test_pi_camera_boot_config.py`" in readme
    for name in [n for n in globals() if n.startswith("test_")]:
        assert f"`{name}`" in readme, name
