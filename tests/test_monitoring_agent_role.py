"""Monitoring agent role (Grafana Alloy on both hosts): per-host Alloy config, slice caps, throttling, robot cleanup."""

import importlib.util
import os
import re
import stat
import subprocess
import sys
from pathlib import Path
from types import ModuleType

import pytest
import yaml

from tests.monitoring_helpers import (
    AGENT_DIR,
    GROUP_VARS_DIR,
    HOSTS,
    group_monitoring_vars,
    host_vars,
    load_yaml,
    render,
    resolve,
    role_defaults,
    role_tasks,
    tasks_using,
    unit_settings,
)

TEMPLATES_DIR = AGENT_DIR / "templates"
FILES_DIR = AGENT_DIR / "files"
THROTTLE_SCRIPT = FILES_DIR / "rpi_throttled_textfile.py"

METRICS_PUSH = {"client": "http://server.ros2.lan:9009/api/v1/push", "server": "http://127.0.0.1:9009/api/v1/push"}
LOGS_PUSH = {
    "client": "http://server.ros2.lan:3100/loki/api/v1/push",
    "server": "http://127.0.0.1:3100/loki/api/v1/push",
}
# monitoring.slice per host: (CPUQuota, MemoryMax, MemoryHigh). The robot runs Alloy only; the server the whole stack.
SLICE_CAPS = {"client": ("40%", "300M", "280M"), "server": ("150%", "2200M", "2000M")}
STACK_TARGETS = {"mimir": "127.0.0.1:9009", "loki": "127.0.0.1:3100", "grafana": "127.0.0.1:3000"}


def agent(template: str, host: str, **extra: object) -> str:
    """Render one agent template for a host.

    Args:
        template (str): Template under roles/monitoring_agent/templates.
        host (str): client or server.
        **extra (object): Extra variables.

    Returns:
        str: Rendered text.
    """
    return render(AGENT_DIR, template, host, **extra)


def alloy_block(config: str, header: str) -> str:
    """Return the body of one top-level Alloy block.

    Args:
        config (str): Rendered config.alloy.
        header (str): Block header, e.g. 'prometheus.scrape "nodes"'.

    Returns:
        str: Text between the header's opening brace and the closing brace at column 0.
    """
    match = re.search(re.escape(header) + r" \{\n(.*?)\n\}", config, re.S)
    assert match, header
    return match.group(1)


def load_throttle_script() -> ModuleType:
    """Import files/rpi_throttled_textfile.py as a module.

    Returns:
        ModuleType: The script module.
    """
    spec = importlib.util.spec_from_file_location("rpi_throttled_textfile", THROTTLE_SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# --- role structure ----------------------------------------------------------------------------------------------


def test_role_files_exist() -> None:
    """The agent role has tasks (main, the shared Grafana apt repo, the robot cleanup), defaults, handlers, meta."""
    for rel in [
        "tasks/main.yml",
        "tasks/grafana_repo.yml",
        "tasks/remove_local_stack.yml",
        "defaults/main.yml",
        "handlers/main.yml",
        "meta/main.yml",
        "templates/config.alloy.j2",
        "templates/monitoring.slice.j2",
        "files/rpi_throttled_textfile.py",
    ]:
        assert (AGENT_DIR / rel).is_file(), rel


@pytest.mark.parametrize("host", HOSTS)
def test_every_template_and_file_the_tasks_name_exists(host: str) -> None:
    """Every src: of a template/copy task names a file shipped with the role."""
    checked = 0
    for task in role_tasks(AGENT_DIR):
        for module, folder in (("ansible.builtin.template", TEMPLATES_DIR), ("ansible.builtin.copy", FILES_DIR)):
            src = (task.get(module) or {}).get("src")
            if not src:
                continue
            loop = task.get("loop", [None])
            items = yaml.safe_load(str(resolve(loop, host))) if isinstance(loop, str) else loop
            for item in items:
                assert (folder / str(resolve(src, host, item))).is_file(), f"{task['name']}: {src}"
                checked += 1
    assert checked >= 6


def test_defaults_are_the_agent_only_host() -> None:
    """Defaults describe a host that only runs Alloy and pushes to the server; group_vars set the rest per host."""
    defaults = role_defaults(AGENT_DIR)
    assert defaults["monitoring_agent_enabled"] is True
    assert defaults["monitoring_agent_alloy_port"] == 12345
    assert defaults["monitoring_agent_interval"] == "5s"
    assert defaults["monitoring_agent_scrape_timeout"] == "4s"
    assert defaults["monitoring_agent_metrics_url"] == METRICS_PUSH["client"]
    assert defaults["monitoring_agent_logs_url"] == LOGS_PUSH["client"]
    assert defaults["monitoring_agent_node_targets"] == []
    assert defaults["monitoring_agent_stack_targets"] == []
    assert defaults["monitoring_agent_remove_local_stack"] is False
    assert {u["name"]: u["memory_max"] for u in defaults["monitoring_agent_units"]} == {"alloy": "250M"}
    assert "monitoring_host" not in defaults, "the host label must be set per host in group_vars"


@pytest.mark.parametrize("host", HOSTS)
def test_group_vars_set_the_host_label(host: str) -> None:
    """group_vars/<host>.yml names the host label every series and log stream carries."""
    assert group_monitoring_vars(host)["monitoring_host"] == host


# --- resource isolation ------------------------------------------------------------------------------------------


@pytest.mark.parametrize("host", HOSTS)
def test_monitoring_slice_caps_per_host(host: str) -> None:
    """The slice caps every monitoring unit of the host: low CPU/IO weight plus a per-host CPU and memory total."""
    settings = unit_settings(agent("monitoring.slice.j2", host))
    quota, memory_max, memory_high = SLICE_CAPS[host]
    assert settings["CPUWeight"] == ["20"]
    assert settings["IOWeight"] == ["20"]
    assert settings["CPUQuota"] == [quota]
    assert settings["MemoryMax"] == [memory_max]
    assert settings["MemoryHigh"] == [memory_high]


def test_server_unit_caps_fit_the_server_slice() -> None:
    """The per-unit caps of the server (Mimir, Loki, Grafana, Alloy) add up to no more than the slice's hard total."""
    variables = host_vars("server")
    units = variables["monitoring_units"] + variables["monitoring_agent_units"]
    assert {u["name"] for u in units} == {"mimir", "loki", "grafana-server", "alloy"}
    total = sum(int(u["memory_max"].rstrip("M")) for u in units)
    assert total <= int(SLICE_CAPS["server"][1].rstrip("M"))


@pytest.mark.parametrize("host", HOSTS)
def test_alloy_dropin_moves_it_into_the_slice(host: str) -> None:
    """Alloy runs in monitoring.slice, niced, capped at 250M, first in line for the OOM killer, as root (cAdvisor)."""
    item = role_defaults(AGENT_DIR)["monitoring_agent_units"][0]
    settings = unit_settings(agent("monitoring-dropin.conf.j2", host, item=item))
    assert settings["Slice"] == ["monitoring.slice"]
    assert settings["Nice"] == ["10"]
    assert settings["MemoryMax"] == ["250M"]
    assert settings["OOMScoreAdjust"] == ["500"]
    assert settings["Restart"] == ["on-failure"]
    assert settings["User"] == ["root"]


def test_dropins_are_installed_for_every_agent_unit() -> None:
    """A template task loops over monitoring_agent_units and writes /etc/systemd/system/<unit>.service.d/."""
    tasks = [
        t
        for t in role_tasks(AGENT_DIR)
        if (t.get("ansible.builtin.template") or {}).get("src") == "monitoring-dropin.conf.j2"
    ]
    assert len(tasks) == 1
    assert tasks[0]["loop"] == "{{ monitoring_agent_units }}"
    assert "/etc/systemd/system/{{ item.name }}.service.d/" in tasks[0]["ansible.builtin.template"]["dest"]


# --- packages ----------------------------------------------------------------------------------------------------


def test_agent_installs_alloy_only() -> None:
    """The agent installs Alloy (and vcgencmd on aarch64) without recommends; Prometheus is never installed."""
    apt = [t["ansible.builtin.apt"] for t in tasks_using(role_tasks(AGENT_DIR, ("main.yml",)), "ansible.builtin.apt")]
    installs = [a for a in apt if a.get("state") == "present" and "alloy" in str(a.get("name"))]
    assert len(installs) == 1
    assert installs[0]["install_recommends"] is False
    names = resolve(installs[0]["name"], "client")
    assert "alloy" in str(names) and "libraspberrypi-bin" in str(names)
    assert "prometheus" not in str(names) and "grafana" not in str(names)


def test_node_exporter_is_masked() -> None:
    """Alloy provides host metrics: a prometheus-node-exporter the system may have is masked."""
    tasks = tasks_using(role_tasks(AGENT_DIR), "ansible.builtin.systemd_service")
    masked = [t["ansible.builtin.systemd_service"] for t in tasks if t["ansible.builtin.systemd_service"].get("masked")]
    assert any(m["name"] == "prometheus-node-exporter" for m in masked)


def test_grafana_apt_repo_signed_by_keyring() -> None:
    """Grafana's stable apt repo (Alloy, Mimir, Loki, Grafana), signed by a key under /etc/apt/keyrings."""
    tasks = role_tasks(AGENT_DIR, ("grafana_repo.yml",))
    repo = [t["ansible.builtin.deb822_repository"] for t in tasks_using(tasks, "ansible.builtin.deb822_repository")]
    assert len(repo) == 1
    assert repo[0]["uris"] == "https://apt.grafana.com"
    assert repo[0]["suites"] == "stable"
    assert repo[0]["components"] == "main"
    assert repo[0]["signed_by"].startswith("/etc/apt/keyrings/")
    keys = [t["ansible.builtin.get_url"] for t in tasks_using(tasks, "ansible.builtin.get_url")]
    assert any(k["url"] == "https://apt.grafana.com/gpg.key" and k["dest"] == repo[0]["signed_by"] for k in keys)
    main = role_tasks(AGENT_DIR, ("main.yml",))
    assert any((t.get("ansible.builtin.include_tasks") or "") == "grafana_repo.yml" for t in main)


# --- alloy: destinations and labels -------------------------------------------------------------------------------


@pytest.mark.parametrize("host", HOSTS)
def test_alloy_remote_writes_to_mimir_with_the_host_label(host: str) -> None:
    """Metrics go to the server's Mimir (the server itself writes to 127.0.0.1) with external label host=<host>."""
    body = alloy_block(agent("config.alloy.j2", host), 'prometheus.remote_write "mimir"')
    assert f'url = "{METRICS_PUSH[host]}"' in body
    assert re.search(r'external_labels\s*=\s*\{\s*host\s*=\s*"' + host + r'"\s*\}', body)


@pytest.mark.parametrize("host", HOSTS)
def test_alloy_pushes_logs_to_loki_with_the_host_label(host: str) -> None:
    """Logs go to the server's Loki (the server itself to 127.0.0.1) with external label host=<host>."""
    body = alloy_block(agent("config.alloy.j2", host), 'loki.write "server"')
    assert f'url = "{LOGS_PUSH[host]}"' in body
    assert re.search(r'external_labels\s*=\s*\{\s*host\s*=\s*"' + host + r'"\s*\}', body)


@pytest.mark.parametrize("host", HOSTS)
def test_alloy_scrapes_every_5s_and_writes_to_mimir(host: str) -> None:
    """Every scrape runs at 5 s with a 4 s timeout and ends in the Mimir remote write (directly or via relabel)."""
    config = agent("config.alloy.j2", host)
    scrapes = re.findall(r'prometheus\.scrape "[^"]+"', config)
    assert len(scrapes) >= (3 if host == "client" else 4)
    assert config.count('scrape_interval = "5s"') == len(scrapes)
    # Alloy rejects a scrape whose timeout (default 10 s) exceeds its interval.
    assert config.count('scrape_timeout  = "4s"') == len(scrapes)
    assert "prometheus.remote_write.local" not in config
    forwards = re.findall(r"forward_to\s*=\s*\[([^\]]*)\]", config)
    metric_forwards = [f for f in forwards if "prometheus." in f]
    assert metric_forwards and all(
        f in ("prometheus.remote_write.mimir.receiver", "prometheus.relabel.cadvisor.receiver") for f in metric_forwards
    )


@pytest.mark.parametrize("host", HOSTS)
def test_alloy_collects_host_cgroup_and_self_metrics(host: str) -> None:
    """unix (textfile dir), cadvisor (docker_only = false) and self exporters run on both hosts."""
    config = agent("config.alloy.j2", host)
    assert 'prometheus.exporter.unix "host"' in config
    assert 'directory = "/var/lib/alloy/textfile"' in config
    assert 'prometheus.exporter.cadvisor "units"' in config
    assert re.search(r"docker_only\s*=\s*false", config)
    assert 'prometheus.exporter.self "alloy"' in config


@pytest.mark.parametrize("host", HOSTS)
def test_alloy_drops_high_cardinality_cgroups_and_veth(host: str) -> None:
    """Only the root, slices and their services are kept; veth/docker interfaces are dropped."""
    config = agent("config.alloy.j2", host)
    keep = re.search(r'source_labels\s*=\s*\["id"\]\s*\n\s*regex\s*=\s*"([^"]+)"\s*\n\s*action\s*=\s*"keep"', config)
    assert keep, "keep rule on id"
    pattern = re.compile(keep.group(1).replace("\\\\", "\\"))
    for kept in ["/", "/system.slice", "/system.slice/ros2-web_ui.service", "/monitoring.slice/mimir.service"]:
        assert pattern.fullmatch(kept), kept
    for dropped in ["/system.slice/docker-abc.scope", "/user.slice/user-1000.slice/session-3.scope", "/init.scope"]:
        assert not pattern.fullmatch(dropped), dropped
    assert re.search(r'source_labels\s*=\s*\["interface"\]\s*\n\s*regex\s*=\s*"[^"]*veth', config)


def test_alloy_listens_on_lan_port() -> None:
    """Alloy's HTTP server (UI, /metrics) is on 0.0.0.0:12345 and the reporting call-home is off."""
    for host in HOSTS:
        args = agent("alloy.default.j2", host)
        assert "--server.http.listen-addr=0.0.0.0:12345" in args
        assert "--disable-reporting" in args
        assert 'CONFIG_FILE="/etc/alloy/config.alloy"' in args


# --- alloy: what each host scrapes --------------------------------------------------------------------------------


def test_robot_alloy_scrapes_every_node_target() -> None:
    """The robot's scrape 'nodes': each target is 127.0.0.1:<port> with a node label, 5 s / 4 s, path /metrics."""
    targets = host_vars("client")["monitoring_agent_node_targets"]
    nodes = load_yaml(GROUP_VARS_DIR / "client.yml")["ros2_nodes"]
    enabled = {n["name"] for n in nodes if n.get("present", True) and n.get("enabled", True)}
    target_nodes = [t["node"] for t in targets]
    assert target_nodes and len(set(target_nodes)) == len(target_nodes)
    # depthd on the Jetson is no ROS node: the Pi's Alloy scrapes it over the direct cable (no route out).
    non_ros = {"jetson_depthd"}
    assert set(target_nodes) - non_ros <= enabled, "scrape target for a node that is not deployed and enabled"
    body = alloy_block(agent("config.alloy.j2", "client"), 'prometheus.scrape "nodes"')
    rendered = re.findall(r'\{"__address__" = "([^"]+)", "node" = "([^"]+)"\}', body)
    assert rendered == [(t["address"], t["node"]) for t in targets]
    assert all(re.fullmatch(r"127\.0\.0\.1:\d+", t["address"]) for t in targets if t["node"] not in non_ros)
    assert all(
        re.fullmatch(r"(\{\{ jetson_link_jetson_ip \}\}|10\.77\.0\.2):8090", t["address"])
        for t in targets
        if t["node"] in non_ros
    )
    assert all(t["path"] == "/metrics" for t in targets)
    assert re.search(r'job_name\s*=\s*"ros2_nodes"', body)
    assert re.search(r'metrics_path\s*=\s*"/metrics"', body)
    # The helper's robot_node_info{node} keeps its own node label instead of becoming exported_node.
    assert re.search(r"honor_labels\s*=\s*true", body)
    assert 'scrape_interval = "5s"' in body and 'scrape_timeout  = "4s"' in body
    assert "forward_to      = [prometheus.remote_write.mimir.receiver]" in body


def test_robot_alloy_scrapes_no_stack() -> None:
    """The robot no longer runs Prometheus or Grafana: nothing scrapes them."""
    config = agent("config.alloy.j2", "client")
    assert 'prometheus.scrape "stack"' not in config
    assert "127.0.0.1:9090" not in config and "127.0.0.1:3000" not in config


def test_server_alloy_scrapes_the_stack_and_no_nodes() -> None:
    """The server's Alloy scrapes Mimir, Loki and Grafana (job per service) and no ROS node /metrics."""
    config = agent("config.alloy.j2", "server")
    assert 'prometheus.scrape "nodes"' not in config
    body = alloy_block(config, 'prometheus.scrape "stack"')
    rendered = dict(
        (job, address) for address, job in re.findall(r'"__address__" = "([^"]+)", "job" = "([^"]+)"', body)
    )
    assert rendered == STACK_TARGETS
    assert host_vars("server")["monitoring_agent_node_targets"] == []


@pytest.mark.parametrize("host", HOSTS)
def test_alloy_systemd_collector_limited_to_ros_and_stack_units(host: str) -> None:
    """The unix exporter adds the systemd collector with restart metrics for ros2-* and the monitoring units only."""
    body = alloy_block(agent("config.alloy.j2", host), 'prometheus.exporter.unix "host"')
    assert re.search(r'enable_collectors\s*=\s*\["systemd"\]', body)
    systemd = re.search(r"systemd \{(.*?)\}", body, re.S)
    assert systemd
    assert re.search(r"enable_restarts\s*=\s*true", systemd.group(1))
    assert re.search(r"start_time\s*=\s*true", systemd.group(1))
    include = re.search(r'unit_include\s*=\s*"([^"]+)"', systemd.group(1))
    assert include
    pattern = re.compile(include.group(1).replace("\\\\", "\\"))
    kept = ["ros2-web_ui.service", "alloy.service", "mimir.service", "loki.service", "grafana-server.service"]
    for unit in [*kept, "nginx.service"]:
        assert pattern.fullmatch(unit), unit
    for unit in [
        "ssh.service",
        "ros2-web_ui.socket",
        "systemd-journald.service",
        "alloyXservice",
        "prometheus.service",
    ]:
        assert not pattern.fullmatch(unit), unit


# --- alloy: journald -> loki -------------------------------------------------------------------------------------


@pytest.mark.parametrize("host", HOSTS)
def test_alloy_reads_the_journal_with_unit_and_level_labels(host: str) -> None:
    """loki.source.journal (12 h back on first start) labels each line with its unit, level and transport."""
    config = agent("config.alloy.j2", host)
    source = alloy_block(config, 'loki.source.journal "system"')
    assert re.search(r'max_age\s*=\s*"12h"', source)
    assert re.search(r"relabel_rules\s*=\s*loki\.relabel\.journal\.rules", source)
    assert re.search(r'labels\s*=\s*\{\s*job\s*=\s*"systemd-journal"\s*\}', source)
    assert re.search(r"forward_to\s*=\s*\[loki\.process\.journal\.receiver\]", source)
    relabel = alloy_block(config, 'loki.relabel "journal"')
    rules = re.findall(r'source_labels\s*=\s*\["([^"]+)"\]\s*\n\s*target_label\s*=\s*"([^"]+)"', relabel)
    assert ("__journal__systemd_unit", "unit") in rules
    assert ("__journal_priority_keyword", "level") in rules
    assert ("__journal__transport", "transport") in rules


@pytest.mark.parametrize("host", HOSTS)
def test_alloy_drops_debug_lines_before_loki(host: str) -> None:
    """Debug-level lines are dropped in loki.process; the rest goes to the Loki writer."""
    process = alloy_block(agent("config.alloy.j2", host), 'loki.process "journal"')
    assert re.search(r"forward_to\s*=\s*\[loki\.write\.server\.receiver\]", process)
    match = re.search(r"stage\.match \{(.*?)\n  \}", process, re.S)
    assert match
    assert 'selector            = "{level=\\"debug\\"}"' in match.group(1)
    assert re.search(r'action\s*=\s*"drop"', match.group(1))


# --- throttling textfile -----------------------------------------------------------------------------------------


def test_parse_throttled() -> None:
    """vcgencmd get_throttled output is parsed as hex."""
    mod = load_throttle_script()
    assert mod.parse_throttled("throttled=0x50005\n") == 0x50005
    assert mod.parse_throttled("throttled=0x0") == 0
    with pytest.raises(ValueError):
        mod.parse_throttled('error=1 error_msg="Command not registered"')


def test_throttled_metrics_names_every_bit() -> None:
    """Raw flags gauge plus one gauge per named bit (0-3 now, 16-19 occurred)."""
    mod = load_throttle_script()
    text = mod.throttled_metrics(0x50005)
    assert "rpi_throttled_flags 327685" in text
    expected = {
        "under_voltage_now": 1,
        "freq_capped_now": 0,
        "throttled_now": 1,
        "soft_temp_limit_now": 0,
        "under_voltage_occurred": 1,
        "freq_capped_occurred": 0,
        "throttled_occurred": 1,
        "soft_temp_limit_occurred": 0,
    }
    for bit, value in expected.items():
        assert f'rpi_throttled{{bit="{bit}"}} {value}\n' in text
    assert "# TYPE rpi_throttled_flags gauge" in text and "# TYPE rpi_throttled gauge" in text


def run_script(tmp_path: Path, body: str) -> tuple[subprocess.CompletedProcess[str], Path]:
    """Run the throttling script against a fake vcgencmd on PATH.

    Args:
        tmp_path (Path): Scratch directory.
        body (str): Fake vcgencmd shell body.

    Returns:
        tuple[subprocess.CompletedProcess[str], Path]: The run and the output file path.
    """
    bindir = tmp_path / "bin"
    bindir.mkdir(exist_ok=True)
    fake = bindir / "vcgencmd"
    fake.write_text(f"#!/bin/sh\n{body}\n")
    fake.chmod(fake.stat().st_mode | stat.S_IEXEC)
    out = tmp_path / "textfile" / "rpi_throttled.prom"
    out.parent.mkdir(exist_ok=True)
    env = {**os.environ, "PATH": f"{bindir}:{os.environ['PATH']}", "RPI_THROTTLED_TEXTFILE": str(out)}
    proc = subprocess.run([sys.executable, str(THROTTLE_SCRIPT)], env=env, capture_output=True, text=True, check=False)
    return proc, out


def test_script_writes_textfile_atomically(tmp_path: Path) -> None:
    """With a working vcgencmd the textfile appears, with no temp file left behind."""
    proc, out = run_script(tmp_path, 'echo "throttled=0x80000"')
    assert proc.returncode == 0, proc.stderr
    assert 'rpi_throttled{bit="soft_temp_limit_occurred"} 1' in out.read_text()
    assert sorted(p.name for p in out.parent.iterdir()) == ["rpi_throttled.prom"]


def test_script_removes_stale_textfile_on_failure(tmp_path: Path) -> None:
    """A failed read publishes nothing: the old file is removed and the script exits non-zero."""
    proc, out = run_script(tmp_path, 'echo "throttled=0x0"')
    assert proc.returncode == 0 and out.exists()
    proc, out = run_script(tmp_path, "exit 1")
    assert proc.returncode != 0
    assert not out.exists()


@pytest.mark.parametrize("host", HOSTS)
def test_throttle_timer_runs_every_5s(host: str) -> None:
    """The timer fires the oneshot service every 5 s on both Pis; the service runs in monitoring.slice."""
    timer = unit_settings(agent("rpi-throttled.timer.j2", host))
    assert timer["OnUnitActiveSec"] == ["5s"]
    service = unit_settings(agent("rpi-throttled.service.j2", host))
    assert service["Type"] == ["oneshot"]
    assert service["Slice"] == ["monitoring.slice"]
    assert service["Environment"] == ["RPI_THROTTLED_TEXTFILE=/var/lib/alloy/textfile/rpi_throttled.prom"]


# --- start, restart guard, readiness -----------------------------------------------------------------------------


def test_alloy_and_timer_started_then_guarded_then_checked() -> None:
    """Alloy and the timer are enabled, then restarted if a config is newer than their start, then probed."""
    tasks = role_tasks(AGENT_DIR, ("main.yml",))
    names = [t.get("name", "") for t in tasks]
    start = next(t for t in tasks if t.get("name") == "Enable and start Alloy and the throttling timer")
    assert start["loop"] == ["alloy", "rpi-throttled.timer"]
    guard = [t for t in tasks if "newer than" in t.get("name", "")]
    assert len(guard) == 1
    script = guard[0]["ansible.builtin.shell"]
    for path in [
        "/etc/default/alloy",
        "/etc/alloy/config.alloy",
        "$d/alloy$slice",
        "$d/monitoring.slice",
    ]:
        assert path in script, path
    assert "ActiveEnterTimestamp" in script and "systemctl restart" in script
    ready = next(t for t in tasks if t.get("name") == "Wait until Alloy answers")
    assert resolve(ready["ansible.builtin.uri"]["url"], "client") == "http://127.0.0.1:12345/-/ready"
    assert names.index(start["name"]) < names.index(guard[0]["name"]) < names.index(ready["name"])


def test_disabled_agent_stops_alloy_and_timer() -> None:
    """monitoring_agent_enabled false stops and disables Alloy and the throttling timer (probed by service)."""
    tasks = role_tasks(AGENT_DIR, ("main.yml",))
    stop = next(t for t in tasks if t.get("name", "").startswith("Stop and disable Alloy"))
    assert {item["unit"] for item in stop["loop"]} == {"alloy", "rpi-throttled.timer"}
    assert "not monitoring_agent_enabled | bool" in stop["when"]


# --- robot cleanup: the former local Prometheus and Grafana ------------------------------------------------------


def cleanup_tasks() -> list[dict]:
    """Tasks of tasks/remove_local_stack.yml.

    Returns:
        list[dict]: Tasks in order.
    """
    return role_tasks(AGENT_DIR, ("remove_local_stack.yml",))


def test_cleanup_runs_on_the_robot_only() -> None:
    """The robot purges its former Prometheus and Grafana; the server (which now runs Grafana) never does."""
    assert host_vars("client")["monitoring_agent_remove_local_stack"] is True
    assert host_vars("server")["monitoring_agent_remove_local_stack"] is False
    include = [
        t
        for t in role_tasks(AGENT_DIR, ("main.yml",))
        if t.get("ansible.builtin.include_tasks") == "remove_local_stack.yml"
    ]
    assert len(include) == 1
    assert include[0]["when"] == "monitoring_agent_remove_local_stack | bool"


def test_cleanup_stops_and_disables_prometheus_and_grafana_when_present() -> None:
    """Stop + disable runs only for units still installed (service_facts), so a second run skips it."""
    stops = [t for t in tasks_using(cleanup_tasks(), "ansible.builtin.systemd_service")]
    assert len(stops) == 1
    module = stops[0]["ansible.builtin.systemd_service"]
    assert (module["state"], module["enabled"]) == ("stopped", False)
    assert stops[0]["loop"] == ["prometheus", "grafana-server"]
    assert "ansible_facts.services" in str(stops[0]["when"])


def test_cleanup_purges_the_packages_and_their_leftovers() -> None:
    """apt purges prometheus and grafana; drop-ins, config and data dirs are removed (state-based: idempotent)."""
    tasks = cleanup_tasks()
    apt = [t["ansible.builtin.apt"] for t in tasks_using(tasks, "ansible.builtin.apt")]
    assert len(apt) == 1
    assert sorted(apt[0]["name"]) == ["grafana", "prometheus"]
    assert apt[0]["state"] == "absent" and apt[0]["purge"] is True
    files = [t for t in tasks_using(tasks, "ansible.builtin.file")]
    assert len(files) == 1 and files[0]["ansible.builtin.file"]["state"] == "absent"
    removed = set(files[0]["loop"])
    assert {
        "/etc/systemd/system/prometheus.service.d",
        "/etc/systemd/system/grafana-server.service.d",
        "/etc/grafana",
        "/etc/prometheus",
        "/etc/default/prometheus",
        "/var/lib/prometheus",
        "/var/lib/grafana",
    } <= removed
    assert "Reload systemd" in str(files[0].get("notify"))
    # A second run reports no changes: no command/shell task that would always report one.
    assert not [t for t in tasks if "ansible.builtin.shell" in t or "ansible.builtin.command" in t]


def test_cleanup_runs_before_the_slice_is_installed() -> None:
    """The old units leave monitoring.slice before the robot's slice shrinks to 300M."""
    names = [t.get("name", "") for t in role_tasks(AGENT_DIR, ("main.yml",))]
    cleanup = next(n for n in names if "former local" in n)
    assert names.index(cleanup) < names.index(
        "Install monitoring.slice (CPU, IO and memory caps of the monitoring units)"
    )
