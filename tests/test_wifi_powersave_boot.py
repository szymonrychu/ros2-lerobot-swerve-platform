"""Wi-Fi power save stays off across reboots on systemd-networkd hosts (no NetworkManager)."""

from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
TASKS = REPO_ROOT / "ansible" / "roles" / "system_optimize" / "tasks" / "main.yml"
GUARD = "rpi_wifi_power_save_off"
UNIT_PATH = "/etc/systemd/system/wifi-powersave-off.service"


def tasks() -> list[dict[str, Any]]:
    """Load the system_optimize tasks.

    Returns:
        list[dict[str, Any]]: Parsed task list.
    """
    return yaml.safe_load(TASKS.read_text())


def guarded(task: dict[str, Any]) -> bool:
    """Check that a task is conditioned on the Wi-Fi power save variable.

    Args:
        task: Task dict.

    Returns:
        bool: True when its `when` includes the variable.
    """
    when = task.get("when", [])
    when = [when] if isinstance(when, str) else when
    return any(GUARD in str(w) for w in when)


def test_unit_installed_with_iw_power_save_off() -> None:
    """A root 0644 unit file runs `iw dev <if> set power_save off`, wanted by multi-user.target."""
    found = [t for t in tasks() if UNIT_PATH in str(t.get("ansible.builtin.copy", {}).get("dest", ""))]
    assert len(found) == 1
    task = found[0]
    assert guarded(task)
    copy = task["ansible.builtin.copy"]
    assert copy["owner"] == "root" and copy["mode"] == "0644"
    content = copy["content"]
    assert "ExecStart=/usr/local/sbin/wifi-powersave-off.sh" in content
    assert "Type=oneshot" in content
    assert "WantedBy=multi-user.target" in content
    assert "systemd-networkd" in content


def test_script_runs_iw_power_save_off_on_each_wlan() -> None:
    """The unit's script is installed root 0755 and runs `iw dev <if> set power_save off` per wlan interface."""
    found = [
        t for t in tasks() if t.get("ansible.builtin.copy", {}).get("dest") == "/usr/local/sbin/wifi-powersave-off.sh"
    ]
    assert len(found) == 1
    assert guarded(found[0])
    copy = found[0]["ansible.builtin.copy"]
    assert copy["mode"] == "0755" and copy["owner"] == "root"
    assert "/usr/sbin/iw" in copy["content"]
    assert "set power_save off" in copy["content"]
    assert "/sys/class/net/wlan" in copy["content"]


def test_unit_enabled_and_started_after_daemon_reload() -> None:
    """The unit is enabled and started with daemon_reload, guarded by the variable."""
    found = [t for t in tasks() if t.get("ansible.builtin.systemd", {}).get("name") == "wifi-powersave-off.service"]
    assert len(found) == 1
    assert guarded(found[0])
    systemd = found[0]["ansible.builtin.systemd"]
    assert systemd["enabled"] is True
    assert systemd["state"] == "started"
    assert systemd["daemon_reload"] is True


def test_not_networkmanager_only() -> None:
    """The immediate iw task stays; the NetworkManager file is only written where its conf dir exists."""
    all_tasks = tasks()
    assert any("set power_save off" in str(t.get("ansible.builtin.shell", "")) and guarded(t) for t in all_tasks)
    nm = [t for t in all_tasks if "/etc/NetworkManager/conf.d/wifi-powersave-off.conf" in str(t)]
    assert nm
    for task in nm:
        assert "NetworkManager" in str(task.get("when")) or "nm_" in str(task.get("when"))
        assert guarded(task)
