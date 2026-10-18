"""Servo bus split in Ansible group_vars: the arm alone on lerobot_follower, the swerve base on base_servos.

The arm (IDs 1-6) keeps its own Feetech bus; the eight swerve servos (IDs 32-39) run in a second feetech_servos
instance `base_servos` on the other USB-serial adapter, and every base consumer (swerve_controller, its tilt guard,
mcp_server) is wired to that bridge's `/swerve_drive/*` topics.
"""

import ast
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
ANSIBLE_DIR = REPO_ROOT / "ansible"
CLIENT_VARS = ANSIBLE_DIR / "group_vars" / "client.yml"
CLIENT_PLAYBOOK = ANSIBLE_DIR / "playbooks" / "deploy_nodes_client.yml"
TESTS_README = REPO_ROOT / "tests" / "README.md"

FEETECH_TYPE = "feetech_servos"
FOLLOWER = "lerobot_follower"
BASE = "base_servos"
RETIRED = "swerve_drive_servos"
BY_ID_PREFIX = "/dev/serial/by-id/"
# design.md "Assumptions and unknowns" A1: the arm moved to the new adapter, the base stays on the original one.
ARM_DEVICE = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AE6081020-if00"
BASE_DEVICE = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A059004-if00"
ARM_IDS = [1, 2, 3, 4, 5, 6]
ARM_RATE_HZ = 100
BASE_RATE_HZ = 50
BASE_METRICS_PORT = 19115
BASE_NAMESPACE = "swerve_drive"
# (name, id, mode, inverted): left wheels 32 and 39 are mounted mirrored (bench-verified 2026-10-03).
BASE_JOINTS = [
    ("fl_drive", 32, "velocity", True),
    ("fl_steer", 33, "position", False),
    ("fr_drive", 35, "velocity", False),
    ("fr_steer", 34, "position", False),
    ("rl_drive", 39, "velocity", True),
    ("rl_steer", 38, "position", False),
    ("rr_drive", 36, "velocity", False),
    ("rr_steer", 37, "position", False),
]
CRITICAL_WEIGHTS = (1000, 1000, -900)
CGROUP_OVERRIDE_KEYS = ("cpu_weight", "io_weight", "oom_score_adjust")
# Deferral wording that must not appear in the bridge entries' comments: they state facts and mechanisms only.
PARKING_WORDS = ("later", "next epic", "follow-up", "yet", "todo", "re-measure")


def client_vars() -> dict:
    """Parse ansible/group_vars/client.yml.

    Returns:
        dict: Group variables.
    """
    return yaml.safe_load(CLIENT_VARS.read_text())


def client_nodes() -> list[dict]:
    """Return the client ros2_nodes entries.

    Returns:
        list[dict]: Entries in file order.
    """
    return client_vars()["ros2_nodes"]


def node_entry(name: str) -> dict:
    """Return one client ros2_nodes entry.

    Args:
        name (str): Node name.

    Returns:
        dict: The entry.
    """
    return next(n for n in client_nodes() if n["name"] == name)


def node_config(name: str) -> dict:
    """Parse the `config: |` block of one client ros2_nodes entry.

    Args:
        name (str): Node name.

    Returns:
        dict: Parsed config YAML.
    """
    return yaml.safe_load(node_entry(name)["config"])


def all_joints(config: dict) -> list[dict]:
    """Return every joint a feetech bridge serves (primary group plus extra groups).

    Args:
        config (dict): Parsed feetech_servos config.

    Returns:
        list[dict]: Joint entries.
    """
    joints = list(config.get("joint_names", []))
    for group in config.get("extra_groups", []) or []:
        joints.extend(group.get("joint_names", []))
    return joints


def present_feetech_entries() -> list[dict]:
    """Return the client's present feetech_servos entries.

    Returns:
        list[dict]: Entries with node_type feetech_servos and present true.
    """
    return [n for n in client_nodes() if n["node_type"] == FEETECH_TYPE and n.get("present", True)]


def playbook_step_names() -> list[str]:
    """Return the task names of every play in the client deploy playbook (blocks included).

    Returns:
        list[str]: Task names.
    """

    def walk(tasks: list[dict]) -> list[str]:
        names: list[str] = []
        for task in tasks:
            if "name" in task:
                names.append(task["name"])
            for key in ("block", "rescue", "always"):
                names.extend(walk(task.get(key, [])))
        return names

    plays = yaml.safe_load(CLIENT_PLAYBOOK.read_text())
    return [name for play in plays for key in ("pre_tasks", "tasks", "post_tasks") for name in walk(play.get(key, []))]


def test_follower_serves_only_the_arm() -> None:
    """lerobot_follower serves exactly IDs 1-6 on its primary group and has no extra groups."""
    cfg = node_config(FOLLOWER)
    assert [j["id"] for j in all_joints(cfg)] == ARM_IDS
    assert "extra_groups" not in cfg
    assert "velocity_command_timeout_s" not in cfg


def test_follower_identity_device_and_rate() -> None:
    """lerobot_follower names itself, drives the arm adapter by-id path and keeps the 100 Hz default rate."""
    cfg = node_config(FOLLOWER)
    assert cfg["node_name"] == FOLLOWER
    assert cfg["device"] == ARM_DEVICE
    assert cfg.get("control_loop_hz", ARM_RATE_HZ) == ARM_RATE_HZ


def test_base_servos_entry_is_present_and_enabled() -> None:
    """base_servos is a present, enabled feetech_servos instance with its own config path."""
    entry = node_entry(BASE)
    assert entry["node_type"] == FEETECH_TYPE
    assert entry["present"] is True
    assert entry["enabled"] is True
    assert entry["env"] == [f"FEETECH_SERVOS_CONFIG=/etc/ros2/{BASE}/config.yaml"]


def test_base_servos_carries_the_eight_swerve_joints() -> None:
    """base_servos serves the eight swerve joints verbatim (ID, mode, inversion) on namespace swerve_drive."""
    cfg = node_config(BASE)
    assert cfg["namespace"] == BASE_NAMESPACE
    joints = [(j["name"], j["id"], j.get("mode", "position"), j.get("inverted", False)) for j in cfg["joint_names"]]
    assert joints == BASE_JOINTS
    assert [g["namespace"] for g in cfg["extra_groups"]] == ["head"], "only the head group shares the base bus"


def test_base_servos_runtime_settings() -> None:
    """base_servos: 50 Hz, battery off, metrics 19115, 0.3 s velocity watchdog, torque on start."""
    cfg = node_config(BASE)
    assert cfg["node_name"] == BASE
    assert cfg["device"] == BASE_DEVICE
    assert cfg["control_loop_hz"] == BASE_RATE_HZ
    assert cfg["battery_interval_s"] == 0
    assert cfg["metrics_port"] == BASE_METRICS_PORT
    assert cfg["velocity_command_timeout_s"] == 0.3
    assert cfg["enable_torque_on_start"] is True
    assert cfg["goal_refresh_interval_s"] == 1.0
    assert cfg["read_failure_limit"] == 3
    assert cfg["read_retry_interval_s"] == 5.0
    assert cfg["register_publish_interval_s"] == 10


def test_retired_swerve_drive_servos_name_is_gone() -> None:
    """No ros2_nodes entry and no client deploy step is named swerve_drive_servos; base_servos has a step."""
    assert RETIRED not in {n["name"] for n in client_nodes()}
    steps = playbook_step_names()
    assert not [s for s in steps if RETIRED in s]
    assert f"Deploy {BASE}" in steps
    assert steps.index(f"Deploy {BASE}") == steps.index(f"Deploy {FOLLOWER}") + 1


def test_both_devices_are_distinct_by_id_paths() -> None:
    """Both bridges open a stable /dev/serial/by-id/ path, and not the same adapter."""
    follower = node_config(FOLLOWER)["device"]
    base = node_config(BASE)["device"]
    assert follower.startswith(BY_ID_PREFIX)
    assert base.startswith(BY_ID_PREFIX)
    assert follower != base


def test_present_feetech_instances_are_exactly_follower_and_base() -> None:
    """The client runs exactly two feetech instances, with distinct names, metrics ports and disjoint servo IDs."""
    entries = present_feetech_entries()
    assert sorted(e["name"] for e in entries) == sorted([FOLLOWER, BASE])
    configs = [yaml.safe_load(e["config"]) for e in entries]
    assert len({c["node_name"] for c in configs}) == len(configs)
    assert len({c["metrics_port"] for c in configs}) == len(configs)
    follower_ids, base_ids = ({j["id"] for j in all_joints(c)} for c in configs)
    assert follower_ids.isdisjoint(base_ids)


def test_swerve_controller_drives_the_base_joints() -> None:
    """swerve_controller joint names equal the base bridge's joints, on the /swerve_drive/* topics."""
    controller = node_config("swerve_controller")
    base_names = [j["name"] for j in node_config(BASE)["joint_names"]]
    assert list(controller["joint_names"]) == base_names
    assert controller["joint_states_topic"] == f"/{BASE_NAMESPACE}/joint_states"
    assert controller["joint_commands_topic"] == f"/{BASE_NAMESPACE}/joint_commands"


def test_tilt_guard_torque_reaches_the_base_bridge() -> None:
    """The tilt guard's torque_off goes to the base bridge's set_register topic."""
    assert node_config("swerve_controller")["tilt_guard"]["torque_topic"] == f"/{BASE_NAMESPACE}/set_register"


def test_mcp_server_register_topics_follow_the_split() -> None:
    """mcp_server reads both bridges' register dumps; gripper set_register stays on the arm."""
    topics = node_config("mcp_server")["topics"]
    assert topics["base_servo_registers"] == f"/{BASE_NAMESPACE}/servo_registers"
    assert topics["servo_registers"] == "/follower/servo_registers"
    assert topics["follower_set_register"] == "/follower/set_register"


def test_base_servos_gets_the_critical_cgroup_tier_from_type_defaults() -> None:
    """The feetech_servos type defaults are the critical tier and base_servos does not override them."""
    defaults = client_vars()["ros2_node_type_defaults"][FEETECH_TYPE]
    assert tuple(defaults[key] for key in CGROUP_OVERRIDE_KEYS) == CRITICAL_WEIGHTS
    entry = node_entry(BASE)
    assert not [key for key in CGROUP_OVERRIDE_KEYS if key in entry]


def test_bridge_entry_comments_carry_no_parking_wording() -> None:
    """Comments from the lerobot_follower entry through base_servos state facts, never deferrals."""
    text = CLIENT_VARS.read_text()
    start = text.index(f"  - name: {FOLLOWER}\n")
    end = text.index("  - name: swerve_controller\n", start)
    comments = [line.split("#", 1)[1].lower() for line in text[start:end].splitlines() if "#" in line]
    found = [(word, c.strip()) for c in comments for word in PARKING_WORDS if word in c]
    assert not found, f"parking wording in bridge entry comments: {found}"


def test_every_test_is_documented_in_tests_readme() -> None:
    """tests/README.md must list every test in this file."""
    section = TESTS_README.read_text().split("### `test_servo_bus_split_config.py`", 1)[1].split("\n### ", 1)[0]
    tree = ast.parse(Path(__file__).read_text())
    names = [n.name for n in tree.body if isinstance(n, ast.FunctionDef) and n.name.startswith("test_")]
    missing = [name for name in names if f"`{name}`" not in section]
    assert not missing, f"undocumented in tests/README.md: {missing}"
