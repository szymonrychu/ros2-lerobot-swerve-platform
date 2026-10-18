"""Pin the arm-alone / base-on-second-bus split: no live text or config puts the swerve servos on the arm bus."""

import fnmatch
import subprocess
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
THIS_FILE = "tests/test_servo_bus_live_refs.py"

STALE_PHRASES = (
    "swerve servos (now )?run inside|shared with the swerve servos|on the arm bus"
    "|share the follower-arm serial bus|group .swerve_drive.|all servos incl. the swerve ones"
)
STALE_PHRASE_EXCLUDES = ("docs/plans/**", "graphify-out/**", THIS_FILE)
RETIRED_NAME = "swerve_drive_servos"
RETIRED_NAME_EXCLUDES = ("docs/plans/**", "graphify-out/**", "tests/**", "MEMORY.md")
SWERVE_IDS = set(range(32, 40))
ARM_IDS = set(range(1, 7))
BASE_NODE = "base_servos"
ARM_NODE = "lerobot_follower"


def grep_hits(pattern: str, excludes: tuple[str, ...]) -> list[str]:
    """Run `git grep -n -I -E` over tracked files and drop hits in excluded globs.

    Args:
        pattern: Extended regular expression.
        excludes: fnmatch globs of repo-relative paths to ignore.

    Returns:
        The remaining `path:line:text` hits.
    """
    result = subprocess.run(
        ["git", "grep", "-n", "-I", "-E", pattern],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode in (0, 1), result.stderr
    return [
        line
        for line in result.stdout.splitlines()
        if not any(fnmatch.fnmatch(line.split(":", 1)[0], glob) for glob in excludes)
    ]


def test_no_live_text_puts_swerve_servos_on_the_arm_bus() -> None:
    """No tracked file outside plans/graph output describes swerve servos on the arm bus."""
    assert grep_hits(STALE_PHRASES, STALE_PHRASE_EXCLUDES) == []


def test_retired_swerve_drive_servos_name_is_gone() -> None:
    """The retired `swerve_drive_servos` name appears only in plans, tests, MEMORY.md and graph output."""
    assert grep_hits(RETIRED_NAME, RETIRED_NAME_EXCLUDES) == []


def test_no_feetech_entry_but_the_base_serves_swerve_ids() -> None:
    """In client.yml only base_servos lists IDs 32-39 and only lerobot_follower lists IDs 1-6."""
    nodes = yaml.safe_load(CLIENT_VARS.read_text())["ros2_nodes"]
    ids_by_node: dict[str, set[int]] = {}
    for node in nodes:
        if node.get("node_type") != "feetech_servos" or not node.get("present", True):
            continue
        config = yaml.safe_load(node.get("config", "")) or {}
        ids_by_node[node["name"]] = {joint["id"] for joint in config.get("joint_names", [])}
    assert ids_by_node, "no feetech_servos entries found"
    for name, ids in ids_by_node.items():
        assert (ids & SWERVE_IDS != set()) == (name == BASE_NODE), name
        assert (ids & ARM_IDS != set()) == (name == ARM_NODE), name
    assert ids_by_node[BASE_NODE] == SWERVE_IDS
    assert ids_by_node[ARM_NODE] == ARM_IDS
