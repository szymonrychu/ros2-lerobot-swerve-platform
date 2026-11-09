"""The tracked Ansible inventory is secret-free and parses, and the network role's WiFi credentials come from the
MIKROTIK_WIFI_SSID / MIKROTIK_WIFI_PASSWORD env lookups in group_vars/all.yml, never from the inventory."""

from pathlib import Path
from typing import Any

import pytest
import yaml
from ansible.inventory.manager import InventoryManager
from ansible.parsing.dataloader import DataLoader

REPO_ROOT = Path(__file__).resolve().parent.parent
ANSIBLE_DIR = REPO_ROOT / "ansible"
INVENTORY = ANSIBLE_DIR / "inventory"
ALL_VARS = ANSIBLE_DIR / "group_vars" / "all.yml"
NETWORK_TASKS = ANSIBLE_DIR / "roles" / "network" / "tasks" / "main.yml"
GITIGNORE = REPO_ROOT / ".gitignore"
EXPECTED_HOSTS = {
    "server": "server-rpi4",
    "client": "client-rpi5",
    "controller": "controller-steamdeck",
    "jetson": "jetson-nano",
}
WIFI_ENV = {
    "network_wifi_ssid": "{{ lookup('env', 'MIKROTIK_WIFI_SSID') }}",
    "network_wifi_password": "{{ lookup('env', 'MIKROTIK_WIFI_PASSWORD') }}",
}


def load_inventory() -> InventoryManager:
    """Parse the tracked inventory with Ansible's own inventory loader (no group_vars).

    Returns:
        InventoryManager: The parsed inventory.
    """
    return InventoryManager(loader=DataLoader(), sources=[str(INVENTORY)])


def network_task(name: str) -> dict[str, Any]:
    """One task of the network role by name.

    Args:
        name (str): Task name.

    Returns:
        dict[str, Any]: The parsed task.
    """
    tasks = yaml.safe_load(NETWORK_TASKS.read_text())
    return next(t for t in tasks if t.get("name") == name)


def test_inventory_is_tracked_not_ignored_and_example_is_gone() -> None:
    assert INVENTORY.is_file()
    assert "ansible/inventory" not in GITIGNORE.read_text().splitlines()
    assert not (ANSIBLE_DIR / "inventory.example").exists()


@pytest.mark.parametrize(("group", "host"), sorted(EXPECTED_HOSTS.items()))
def test_inventory_parses_with_the_four_groups(group: str, host: str) -> None:
    inventory = load_inventory()
    assert [h.name for h in inventory.groups[group].get_hosts()] == [host]


def test_inventory_all_vars_are_the_non_secret_ones() -> None:
    all_vars = load_inventory().groups["all"].get_vars()
    assert all_vars == {"ansible_user": "szymonri", "ansible_python_interpreter": "/usr/bin/python3"}


def test_inventory_holds_no_wifi_keys() -> None:
    inventory = load_inventory()
    assert "network_wifi" not in INVENTORY.read_text()
    for host in inventory.get_hosts():
        assert not [k for k in host.get_vars() if k.startswith("network_wifi")]


def test_group_vars_all_reads_wifi_from_mikrotik_env() -> None:
    all_vars = yaml.safe_load(ALL_VARS.read_text())
    for key, value in WIFI_ENV.items():
        assert all_vars[key] == value


def test_network_role_fails_naming_the_ssid_env_var() -> None:
    task = network_task("Require network_wifi_ssid when primary interface is WiFi")
    assert "network_wifi_ssid | length > 0" in task["ansible.builtin.assert"]["that"]
    assert "MIKROTIK_WIFI_SSID" in task["ansible.builtin.assert"]["fail_msg"]
    assert task["when"] == "_network_interface_is_wlan | bool"


def test_netplan_task_with_the_password_is_no_log() -> None:
    assert network_task("Deploy netplan configuration")["no_log"] is True
