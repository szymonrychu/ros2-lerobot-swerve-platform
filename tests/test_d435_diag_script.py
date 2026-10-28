"""Tests for scripts/d435_diag.sh: syntax, help, read-only SSH commands, --device guard, --topics, --rates.

ssh and curl are stubbed on PATH. The ssh stub logs every remote command to a file and answers from canned output, so
the tests see exactly what the script would run on the client.
"""

import json
import os
import re
import subprocess
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
SCRIPT = REPO_ROOT / "scripts" / "d435_diag.sh"
CLIENT_GROUP_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
# macOS ships bash 3.2 here; the script must run on it.
BASH = "/bin/bash"
UNIT = "ros2-realsense_d435"
EVERY_MODE = ["--topics", "--rates", "--device", "--watch", "--lines", "--since", "--interval", "--help"]
EVERY_ENV = ["D435_CLIENT_HOST", "D435_SSH_USER", "SCRAPER_PORT"]
CAMERA_INFO_TYPE = "sensor_msgs/msg/CameraInfo"
FRAME_PERIOD_NS = 33_333_333
FIRST_STAMP_NS = 1_761_600_000_000_000_000

SSH_STUB = """#!/bin/bash
while [ $# -gt 0 ]; do
  case "$1" in
    -o) shift 2 ;;
    -*) shift ;;
    *) break ;;
  esac
done
shift
cmd="$*"
printf '%s\\n' "$cmd" >> "$SSH_LOG"
case "$cmd" in
  *"systemctl is-active"*)
    echo "$STUB_UNIT_STATE"
    [ "$STUB_UNIT_STATE" = active ] || exit 3 ;;
  *rs-enumerate-devices*) echo "Device Name: Intel RealSense D435" ;;
  *since-ok*) [ -n "${STUB_BAD_SINCE:-}" ] || echo since-ok ;;
  *camera_info*) cat "$STUB_RATES_FILE" ;;
esac
exit 0
"""

CURL_STUB = """#!/bin/bash
[ -n "${STUB_CURL_FAIL:-}" ] && exit 7
cat "$STUB_TOPICS_FILE"
"""

TOPICS = {
    "topics": [
        {"topic": "/head/d435/color/camera_info", "endpoint": "/topics/head/d435/color/camera_info",
         "type": CAMERA_INFO_TYPE, "has_sample": True, "is_image": False},
        {"topic": "/head/d435/color/image_raw/compressed", "endpoint": "/topics/head/d435/color/image_raw/compressed",
         "type": "sensor_msgs/msg/CompressedImage", "has_sample": True, "is_image": True},
        {"topic": "/head/d435/depth/camera_info", "endpoint": "/topics/head/d435/depth/camera_info",
         "type": CAMERA_INFO_TYPE, "has_sample": False, "is_image": False},
        {"topic": "/imu/data", "endpoint": "/topics/imu/data", "type": "sensor_msgs/msg/Imu", "has_sample": True,
         "is_image": False},
    ]
}  # fmt: skip


def rates_lines() -> str:
    """Scraper answers to a dense poll: the colour camera_info at 30 Hz, each frame seen twice; depth has no sample.

    Returns:
        str: NDJSON as curl -w '\\n' prints it.
    """
    lines = []
    for i in range(31):
        payload = {
            "topic": "/head/d435/color/camera_info",
            "received_at_ns": FIRST_STAMP_NS + i * FRAME_PERIOD_NS + 1000,
            "header_stamp_ns": FIRST_STAMP_NS + i * FRAME_PERIOD_NS,
            "sample_seq": 2 * i,
            "message": {"width": 640, "height": 360},
        }
        lines += [json.dumps(payload), json.dumps(payload)]
        lines.append(json.dumps({"error": "topic has no sample yet", "topic": "/head/d435/depth/camera_info"}))
    return "\n".join(lines) + "\n"


@pytest.fixture
def stub_env(tmp_path: Path) -> dict[str, str]:
    """Environment with ssh and curl stubbed on PATH and the canned answers on disk.

    Args:
        tmp_path (Path): pytest temporary directory.

    Returns:
        dict[str, str]: Environment for subprocess.run.
    """
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    for name, body in (("ssh", SSH_STUB), ("curl", CURL_STUB)):
        stub = bin_dir / name
        stub.write_text(body)
        stub.chmod(0o755)
    topics_file = tmp_path / "topics.json"
    topics_file.write_text(json.dumps(TOPICS))
    rates_file = tmp_path / "rates.ndjson"
    rates_file.write_text(rates_lines())
    env = dict(os.environ)
    env.update(
        PATH=f"{bin_dir}{os.pathsep}{env['PATH']}",
        SSH_LOG=str(tmp_path / "ssh.log"),
        STUB_UNIT_STATE="active",
        STUB_TOPICS_FILE=str(topics_file),
        STUB_RATES_FILE=str(rates_file),
        D435_CLIENT_HOST="client.test",
        D435_SSH_USER="tester",
    )
    return env


def run(env: dict[str, str], *args: str) -> subprocess.CompletedProcess[str]:
    """Run the script with the stubbed environment.

    Args:
        env (dict[str, str]): Environment from stub_env.
        *args (str): Script arguments.

    Returns:
        subprocess.CompletedProcess[str]: Finished process with captured text output.
    """
    return subprocess.run([BASH, str(SCRIPT), *args], env=env, capture_output=True, text=True, timeout=60)


def ssh_commands(env: dict[str, str]) -> list[str]:
    """Remote commands the ssh stub saw.

    Args:
        env (dict[str, str]): Environment from stub_env.

    Returns:
        list[str]: One entry per logged line.
    """
    log = Path(env["SSH_LOG"])
    return log.read_text().splitlines() if log.exists() else []


def test_script_parses_with_bash_n() -> None:
    result = subprocess.run([BASH, "-n", str(SCRIPT)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_help_lists_every_mode(stub_env: dict[str, str]) -> None:
    result = run(stub_env, "--help")
    assert result.returncode == 0, result.stderr
    for word in EVERY_MODE + EVERY_ENV:
        assert word in result.stdout, word
    assert ssh_commands(stub_env) == []


def test_unknown_option_fails(stub_env: dict[str, str]) -> None:
    result = run(stub_env, "--bogus")
    assert result.returncode == 1
    assert "Unknown option" in result.stderr


def test_since_with_a_quote_is_rejected(stub_env: dict[str, str]) -> None:
    result = run(stub_env, "--since", "1 hour ago'; reboot; '")
    assert result.returncode == 1
    assert ssh_commands(stub_env) == []


def error_count_commands(env: dict[str, str]) -> list[str]:
    """Remote error-count commands the ssh stub saw.

    Args:
        env (dict[str, str]): Environment from stub_env.

    Returns:
        list[str]: The grep -c commands.
    """
    return [c for c in ssh_commands(env) if "grep -c" in c]


def test_bare_relative_since_becomes_ago(stub_env: dict[str, str]) -> None:
    """A bare span like 30min (not a journalctl time) is read as that long ago."""
    result = run(stub_env, "--since", "30min")
    assert result.returncode == 0, result.stderr
    counts = error_count_commands(stub_env)
    assert len(counts) == 3
    assert all("--since '30min ago'" in c for c in counts)
    assert "Errors since '30min ago'" in result.stdout


@pytest.mark.parametrize("since", ["2 hours ago", "2026-10-28 17:48:34", "-45min", "today"])
def test_journalctl_since_syntax_passes_verbatim(stub_env: dict[str, str], since: str) -> None:
    result = run(stub_env, "--since", since)
    assert result.returncode == 0, result.stderr
    assert all(f"--since '{since}'" in c for c in error_count_commands(stub_env))


def test_rejected_since_fails_instead_of_counting_zero(stub_env: dict[str, str]) -> None:
    """When journalctl rejects the window the script says so and exits 1; it never prints counts for it."""
    stub_env["STUB_BAD_SINCE"] = "1"
    result = run(stub_env, "--since", "yesterdayish")
    assert result.returncode == 1
    assert "journalctl rejected --since 'yesterdayish'" in result.stderr
    assert error_count_commands(stub_env) == []
    assert not re.search(r"VIDIOC_QBUF\s+0", result.stdout)
    assert any("--since 'yesterdayish' -n 0" in c for c in ssh_commands(stub_env))


def test_no_rclpy_or_python_on_the_client(stub_env: dict[str, str]) -> None:
    text = SCRIPT.read_text()
    code = "\n".join(line for line in text.splitlines() if not line.lstrip().startswith("#"))
    assert "python" not in code
    assert not re.search(r"\bros2 ", code)
    for args in ([], ["--topics"], ["--rates", "--interval", "1"]):
        run(stub_env, *args)
    stub_env["STUB_UNIT_STATE"] = "inactive"
    run(stub_env, "--device")
    commands = ssh_commands(stub_env)
    assert any("journalctl" in c for c in commands)
    assert any("rs-enumerate-devices" in c for c in commands)
    assert any("camera_info" in c for c in commands)
    for command in commands:
        assert "python" not in command, command
        assert not re.search(r"\bros2 ", command), command


def test_default_mode_reads_service_usb_firmware_psu_and_errors(stub_env: dict[str, str]) -> None:
    result = run(stub_env, "--since", "2 hours ago", "--lines", "7")
    assert result.returncode == 0, result.stderr
    commands = "\n".join(ssh_commands(stub_env))
    assert f"systemctl is-active {UNIT}" in commands
    assert "lsusb -d 8086:0b07" in commands
    assert "/speed" in commands
    assert "Device FW version" in commands and "Device USB type" in commands
    assert "vcgencmd get_config usb_max_current_enable" in commands
    assert "5A" in commands
    assert "--since '2 hours ago'" in commands
    for pattern in ("VIDIOC_QBUF", "No such device", "Frame didn"):
        assert pattern in commands
    assert "-n 7" in commands
    for heading in ("Service", "USB", "Firmware", "PSU", "Errors"):
        assert heading in result.stdout


def test_device_mode_refuses_a_running_unit(stub_env: dict[str, str]) -> None:
    result = run(stub_env, "--device")
    assert result.returncode != 0
    assert "refus" in (result.stdout + result.stderr).lower()
    assert not any("rs-enumerate-devices" in c for c in ssh_commands(stub_env))


def test_device_mode_enumerates_a_stopped_camera(stub_env: dict[str, str]) -> None:
    stub_env["STUB_UNIT_STATE"] = "inactive"
    result = run(stub_env, "--device")
    assert result.returncode == 0, result.stderr
    commands = ssh_commands(stub_env)
    assert any("rs-enumerate-devices -s" in c for c in commands)
    assert any("rs-enumerate-devices -c" in c for c in commands)
    assert "Intel RealSense D435" in result.stdout


def test_topics_lists_only_d435_topics_with_type(stub_env: dict[str, str]) -> None:
    result = run(stub_env, "--topics")
    assert result.returncode == 0, result.stderr
    assert "/head/d435/color/camera_info" in result.stdout
    assert CAMERA_INFO_TYPE in result.stdout
    assert "/head/d435/color/image_raw/compressed" in result.stdout
    assert "/imu/data" not in result.stdout


@pytest.mark.parametrize("mode", ["--topics", "--rates"])
def test_scraper_unreachable_exits_2_with_a_hint(stub_env: dict[str, str], mode: str) -> None:
    stub_env["STUB_CURL_FAIL"] = "1"
    result = run(stub_env, mode)
    assert result.returncode == 2
    output = result.stdout + result.stderr
    assert "topic_scraper_api" in output
    assert "deploy-nodes.sh client topic_scraper_api" in output


def test_rates_from_header_stamps(stub_env: dict[str, str]) -> None:
    result = run(stub_env, "--rates", "--interval", "3")
    assert result.returncode == 0, result.stderr
    color = next(line for line in result.stdout.splitlines() if "/head/d435/color/camera_info" in line)
    assert "30.0 Hz" in color
    assert "lower bound" not in color
    depth = next(line for line in result.stdout.splitlines() if "/head/d435/depth/camera_info" in line)
    assert "no sample" in depth
    command = next(c for c in ssh_commands(stub_env) if "camera_info" in c)
    assert "http://127.0.0.1:18100/topics/head/d435/color/camera_info" in command
    assert "/head/d435/color/image_raw" not in command
    assert "+ 3" in command


def test_scraper_allows_camera_info() -> None:
    nodes = yaml.safe_load(CLIENT_GROUP_VARS.read_text())["ros2_nodes"]
    scraper = next(node for node in nodes if node["name"] == "topic_scraper_api")
    allowed = yaml.safe_load(scraper["config"])["allowed_types"]
    assert CAMERA_INFO_TYPE in allowed
    assert "sensor_msgs/msg/Image" not in allowed
