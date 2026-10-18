"""Serial device naming and the deploy-time preflight for feetech_servos adapters.

A second USB-serial adapter can take /dev/ttyUSB0 at boot, so every adapter is named by its by-id path, and the
ros2_node_deploy role fails a deploy before any unit restarts when a configured feetech device is missing.
"""

import ast
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
ANSIBLE_DIR = REPO_ROOT / "ansible"
CLIENT_VARS = ANSIBLE_DIR / "group_vars" / "client.yml"
DEPLOY_TASKS = ANSIBLE_DIR / "roles" / "ros2_node_deploy" / "tasks" / "main.yml"
TESTS_README = REPO_ROOT / "tests" / "README.md"

BY_ID_PREFIX = "/dev/serial/by-id/"
SCHEMA_TASK = "Validate feetech_servos node config schema (name+id per joint)"
LIDAR_ENV = "RPLIDAR_SERIAL_PORT"
# design.md "Assumptions and unknowns" A2.
LIDAR_DEVICE = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
UNSTABLE_PREFIXES = ("/dev/ttyUSB", "/dev/ttyACM")


def deploy_tasks() -> list[dict]:
    """Parse the ros2_node_deploy role's main task list.

    Returns:
        list[dict]: Top-level tasks in file order.
    """
    return yaml.safe_load(DEPLOY_TASKS.read_text())


def client_nodes() -> list[dict]:
    """Return the client ros2_nodes entries.

    Returns:
        list[dict]: Entries in file order.
    """
    return yaml.safe_load(CLIENT_VARS.read_text())["ros2_nodes"]


def feetech_devices() -> dict[str, str]:
    """Map each present client feetech_servos node to its configured serial device.

    Returns:
        dict[str, str]: Node name to device path.
    """
    devices = {}
    for node in client_nodes():
        if node.get("node_type") == "feetech_servos" and node.get("present", True):
            devices[node["name"]] = yaml.safe_load(node["config"])["device"]
    return devices


def env_values(node: dict) -> dict[str, str]:
    """Split a node's KEY=VAL env list.

    Args:
        node: A ros2_nodes entry.

    Returns:
        dict[str, str]: Env names to values.
    """
    return dict(item.split("=", 1) for item in node.get("env", []) if "=" in item)


def serial_devices() -> dict[str, str]:
    """Collect every serial device named in client.yml.

    Returns:
        dict[str, str]: Label to path (each feetech device and the lidar port).
    """
    devices = dict(feetech_devices())
    lidar = next(n for n in client_nodes() if n["name"] == "rplidar_a1")
    devices[LIDAR_ENV] = env_values(lidar).get(LIDAR_ENV, "")
    return devices


def test_preflight_tasks_exist_in_the_right_place() -> None:
    """The stat+fail pair sits after the schema assert and before the first include_tasks/block."""
    tasks = deploy_tasks()
    names = [t.get("name") for t in tasks]
    schema_at = names.index(SCHEMA_TASK)
    first_deploy_at = next(i for i, t in enumerate(tasks) if "block" in t or "ansible.builtin.include_tasks" in t)
    between = tasks[schema_at + 1 : first_deploy_at]
    stats = [t for t in between if "ansible.builtin.stat" in t]
    fails = [t for t in between if "ansible.builtin.fail" in t]
    assert stats and fails, "need a stat task and a fail task between the schema assert and the deploy blocks"
    for task in stats + fails:
        conditions = " ".join(str(c) for c in task["when"])
        assert "feetech_servos" in conditions
        assert "node_present" in conditions
        assert "node_enabled" in conditions
        assert "[config]" == str(task["tags"]).replace("'", "")
    assert "device" in str(stats[0]["ansible.builtin.stat"]["path"])
    assert stats[0]["ansible.builtin.stat"].get("follow") is True
    assert BY_ID_PREFIX in str(fails[0]["ansible.builtin.fail"]["msg"])
    listings = [t for t in between if "ansible.builtin.command" in t or "ansible.builtin.find" in t]
    assert listings, "the by-id directory is listed when the device is missing"
    assert listings[0]["changed_when"] is False


def test_every_serial_device_is_a_distinct_by_id_path() -> None:
    """Each feetech device and the lidar port is a by-id path, and no two are the same."""
    devices = serial_devices()
    assert LIDAR_ENV in devices and len(devices) >= 3
    bad = {k: v for k, v in devices.items() if not v.startswith(BY_ID_PREFIX)}
    assert not bad, f"not by-id paths: {bad}"
    assert devices[LIDAR_ENV] == LIDAR_DEVICE
    assert len(set(devices.values())) == len(devices), f"duplicate device paths: {devices}"


def test_no_client_node_names_a_numbered_tty() -> None:
    """No client ros2_nodes config, env or extra_args names /dev/ttyUSB* or /dev/ttyACM*."""
    for node in client_nodes():
        text = " ".join([str(node.get("config", "")), *map(str, node.get("env", [])), str(node.get("extra_args", ""))])
        for prefix in UNSTABLE_PREFIXES:
            assert prefix not in text, f"{node['name']} names {prefix}"


def test_every_test_is_documented_in_tests_readme() -> None:
    """tests/README.md must list every test in this file."""
    section = TESTS_README.read_text().split("### `test_serial_devices.py`", 1)[1].split("\n### ", 1)[0]
    tree = ast.parse(Path(__file__).read_text())
    names = [n.name for n in tree.body if isinstance(n, ast.FunctionDef) and n.name.startswith("test_")]
    missing = [name for name in names if f"`{name}`" not in section]
    assert not missing, f"undocumented in tests/README.md: {missing}"
