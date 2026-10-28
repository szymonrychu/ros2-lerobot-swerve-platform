"""The D435 udev rule (PID 0b07) is applied by the realsense_d435 node deploy, not by ros2_base."""

from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
SETUP = REPO_ROOT / "ansible" / "playbooks" / "tasks" / "realsense_d435_setup.yml"
CLIENT_PLAYBOOK = REPO_ROOT / "ansible" / "playbooks" / "deploy_nodes_client.yml"
BASE_TASKS = REPO_ROOT / "ansible" / "roles" / "ros2_base" / "tasks" / "main.yml"
RULE_PATH = "/etc/udev/rules.d/99-realsense-d435.rules"
STALE_PATH = "/etc/udev/rules.d/99-realsense-d435i.rules"
MODPROBE_PATH = "/etc/modprobe.d/99-realsense-uvcvideo.conf"
# Linux drivers/media/usb/uvc/uvcvideo.h quirk bits.
UVC_QUIRK_FIX_BANDWIDTH = 0x80
UVC_QUIRK_RESTRICT_FRAME_RATE = 0x200
UVC_QUIRKS = 128


def setup_tasks() -> list[dict[str, Any]]:
    """Load the realsense_d435 setup task file.

    Returns:
        list[dict[str, Any]]: Parsed task list.
    """
    return yaml.safe_load(SETUP.read_text())


def task_for(dest: str) -> dict[str, Any]:
    """Find the setup task that targets a path.

    Args:
        dest: File path the task writes or removes.

    Returns:
        dict[str, Any]: The task.
    """
    for task in setup_tasks():
        for module in ("ansible.builtin.copy", "ansible.builtin.file"):
            params = task.get(module)
            if params and params.get("dest", params.get("path")) == dest:
                return task
    raise AssertionError(f"no task for {dest}")


def walk(node: Any) -> list[dict[str, Any]]:
    """Collect every mapping nested anywhere in a parsed playbook (plays, blocks, tasks).

    Args:
        node: Parsed YAML value.

    Returns:
        list[dict[str, Any]]: All dicts found.
    """
    if isinstance(node, dict):
        return [node] + [d for v in node.values() for d in walk(v)]
    if isinstance(node, list):
        return [d for v in node for d in walk(v)]
    return []


def test_rule_matches_the_d435_product_id() -> None:
    """The rule matches 8086:0b07 only, opens the device and turns autosuspend off."""
    content = task_for(RULE_PATH)["ansible.builtin.copy"]["content"]
    lines = [x for x in content.splitlines() if x and not x.startswith("#")]
    assert lines
    for line in lines:
        assert 'ATTRS{idVendor}=="8086"' in line
        assert 'ATTRS{idProduct}=="0b07"' in line
    assert "0b3a" not in content
    assert 'MODE="0666", GROUP="video"' in content
    assert 'ATTR{power/autosuspend}="-1"' in content
    assert 'ATTR{power/control}="on"' in content
    assert "iio" not in content


def test_d435i_rules_are_gone() -> None:
    """The stale D435i rule file is removed and ros2_base carries no RealSense rule."""
    stale = task_for(STALE_PATH)["ansible.builtin.file"]
    assert stale["state"] == "absent"
    text = BASE_TASKS.read_text()
    for needle in ("realsense", "8086", "0b3a", "uvcvideo"):
        assert needle not in text
    modprobe = task_for(MODPROBE_PATH)["ansible.builtin.copy"]
    assert modprobe["content"] == f"options uvcvideo quirks={UVC_QUIRKS}\n"
    assert "unverified" in SETUP.read_text().lower()


def test_quirks_keep_every_frame_rate() -> None:
    """The quirks omit UVC_QUIRK_RESTRICT_FRAME_RATE, which hides every frame interval but the default one per mode.

    With quirks=640 (0x280) uvcvideo listed only the highest fps of each mode, so the D2 profile 640x360x15 did not
    exist and realsense2_camera fell back to 1920x1080x30 colour and no depth (bring-up 2026-10-28).
    """
    assert UVC_QUIRKS & UVC_QUIRK_RESTRICT_FRAME_RATE == 0
    assert UVC_QUIRKS == UVC_QUIRK_FIX_BANDWIDTH
    assert "RESTRICT_FRAME_RATE" in SETUP.read_text()


def test_live_quirks_are_applied_and_the_d435_re_enumerated() -> None:
    """The running uvcvideo gets the quirks and the D435 is re-enumerated with the unit stopped, only on a change.

    uvcvideo reads its quirks when it probes a device, so the setup writes the module parameter and re-authorizes the
    8086:0b07 USB device (the unit is stopped first; the deploy's start step starts it again).
    """
    shells = [t for t in setup_tasks() if "ansible.builtin.shell" in t]
    assert len(shells) == 1
    task = shells[0]
    script = task["ansible.builtin.shell"]
    assert f"echo {UVC_QUIRKS} > " in script
    assert "/sys/module/uvcvideo/parameters/quirks" in script
    assert "systemctl stop ros2-realsense_d435" in script
    assert script.index("systemctl stop ros2-realsense_d435") < script.index("/authorized")
    assert "0b07" in script and "8086" in script
    assert task["become"] is True
    assert "changed_when" in task
    assert "setup" in task["tags"]


def test_setup_reloads_udev_on_change() -> None:
    """Udev is reloaded and triggered only when a rule changed."""
    commands = [t["ansible.builtin.command"] for t in setup_tasks() if "ansible.builtin.command" in t]
    assert "udevadm control --reload-rules" in commands
    assert "udevadm trigger" in commands


def test_setup_is_part_of_the_node_deploy() -> None:
    """The Deploy realsense_d435 step is preceded by an include of the setup file tagged realsense_d435 and setup."""
    text = CLIENT_PLAYBOOK.read_text()
    plays = yaml.safe_load(text)
    found = [t for t in walk(plays) if "ansible.builtin.include_tasks" in t]
    includes = [
        t
        for t in found
        if isinstance(t["ansible.builtin.include_tasks"], dict)
        and t["ansible.builtin.include_tasks"].get("file") == "tasks/realsense_d435_setup.yml"
    ]
    assert len(includes) == 1
    assert {"realsense_d435", "setup"} <= set(includes[0]["tags"])
    assert text.index("tasks/realsense_d435_setup.yml") < text.index("_deploy_node_name: realsense_d435\n")
