"""The rendered monitoring configs and every dashboard query, checked by the real binaries in Docker.

- Mimir and Loki (images of the apt.grafana.com package versions) start from the rendered configs with only a loopback
  interface (as on a host whose LAN interface is not eth0) and answer /ready; Loki also runs -verify-config.
- Alloy (grafana/alloy:latest) runs the robot and the server config: "now listening" and no "failed to evaluate".
- nginx (nginx:stable) passes `nginx -t` on both sites, and the robot's /grafana/ redirect keeps path and query.
- promtool (prom/prometheus) parses every PromQL query of the dashboards (as recording rules), Loki's
  /loki/api/v1/format_query parses every LogQL query.

Skipped when docker or an image is missing (the tests never pull); pull the IMAGES below to run them.
"""

import json
import subprocess
import time
import urllib.parse
import uuid
from collections.abc import Iterator
from pathlib import Path

import pytest
import yaml

from tests.monitoring_helpers import AGENT_DIR, HOSTS, PROXY_DIR, STACK_DIR, docker_image_available, render
from tests.test_monitoring_dashboards import DASHBOARDS, dashboard_exprs, load_dashboard

MIMIR_IMAGE = "grafana/mimir:3.2.2"
LOKI_IMAGE = "grafana/loki:3.7.8"
ALLOY_IMAGE = "grafana/alloy:latest"
NGINX_IMAGE = "nginx:stable"
PROMTOOL_IMAGE = "prom/prometheus:v2.45.3"
PROBE_IMAGE = "busybox:1.36"
IMAGES = [MIMIR_IMAGE, LOKI_IMAGE, ALLOY_IMAGE, NGINX_IMAGE, PROMTOOL_IMAGE, PROBE_IMAGE]
READY_TIMEOUT_S = 120
# Grafana variables -> values promtool and Loki accept.
GRAFANA_VARIABLES = {
    "$__rate_interval": "1m",
    "$__interval": "1m",
    "$__range": "1h",
    "$__auto": "1m",
    "$host": "client",
}

pytestmark = pytest.mark.skipif(
    not all(docker_image_available(image) for image in IMAGES),
    reason=f"docker or one of {IMAGES} is not available locally",
)


def docker(*args: str, timeout: float = 120) -> subprocess.CompletedProcess[str]:
    """Run one docker command.

    Args:
        *args (str): docker arguments.
        timeout (float): Seconds before the command is killed.

    Returns:
        subprocess.CompletedProcess[str]: The finished command (stdout and stderr captured).
    """
    return subprocess.run(["docker", *args], capture_output=True, text=True, timeout=timeout, check=False)


def probe(container: str, url: str) -> subprocess.CompletedProcess[str]:
    """Fetch a URL from inside a container's network namespace (the distroless images have no wget).

    Args:
        container (str): Container whose network namespace is used.
        url (str): URL to GET.

    Returns:
        subprocess.CompletedProcess[str]: wget -S output (headers on stderr).
    """
    return docker("run", "--rm", "--network", f"container:{container}", PROBE_IMAGE, "wget", "-S", "-qO-", url)


def wait_ready(container: str, url: str) -> str:
    """Poll a URL in a container until it answers 200.

    Args:
        container (str): Container to probe.
        url (str): Readiness URL.

    Returns:
        str: The final response body.
    """
    deadline = time.monotonic() + READY_TIMEOUT_S
    while time.monotonic() < deadline:
        result = probe(container, url)
        if result.returncode == 0:
            return result.stdout
        time.sleep(3)
    logs = docker("logs", "--tail", "30", container)
    pytest.fail(f"{container} not ready at {url}:\n{logs.stdout}{logs.stderr}")


def write(tmp: Path, name: str, text: str) -> Path:
    """Write a rendered config into a scratch directory.

    Args:
        tmp (Path): Directory.
        name (str): File name.
        text (str): Content.

    Returns:
        Path: The file.
    """
    path = tmp / name
    path.write_text(text)
    path.chmod(0o644)
    return path


def start(image: str, config: Path, mount: str, *args: str) -> str:
    """Start a detached container with only a loopback interface and one config file mounted.

    Args:
        image (str): Image.
        config (Path): Local config file.
        mount (str): Path inside the container.
        *args (str): Image arguments.

    Returns:
        str: Container name.
    """
    name = f"monitoring-test-{uuid.uuid4().hex[:8]}"
    result = docker("run", "-d", "--rm", "--name", name, "--network", "none", "-u", "0", "-v", f"{config}:{mount}:ro",
                    image, *args)  # fmt: skip
    assert result.returncode == 0, result.stderr
    return name


@pytest.fixture(scope="module")
def stack_containers(tmp_path_factory: pytest.TempPathFactory) -> Iterator[dict[str, str]]:
    """Mimir and Loki started from the rendered server configs (in parallel; both need ~20-40 s to be ready).

    Args:
        tmp_path_factory (pytest.TempPathFactory): Scratch directories.

    Yields:
        dict[str, str]: mimir / loki -> container name.
    """
    tmp = tmp_path_factory.mktemp("stack")
    mimir = write(tmp, "mimir.yml", render(STACK_DIR, "mimir.yml.j2", "server"))
    loki = write(tmp, "loki.yml", render(STACK_DIR, "loki.yml.j2", "server"))
    names = {
        "mimir": start(MIMIR_IMAGE, mimir, "/etc/mimir/config.yml", "-config.file=/etc/mimir/config.yml"),
        "loki": start(LOKI_IMAGE, loki, "/etc/loki/config.yml", "-config.file=/etc/loki/config.yml"),
    }
    yield names
    for name in names.values():
        docker("rm", "-f", name)


def test_mimir_config_starts_and_is_ready(stack_containers: dict[str, str]) -> None:
    """Mimir runs the rendered config (target all, filesystem, in-memory rings on 127.0.0.1) and reports ready."""
    assert wait_ready(stack_containers["mimir"], "http://127.0.0.1:9009/ready").strip() == "ready"
    config = probe(stack_containers["mimir"], "http://127.0.0.1:9009/config").stdout
    effective = yaml.safe_load(config)
    assert effective["limits"]["compactor_blocks_retention_period"] == "1d"
    assert effective["multitenancy_enabled"] is False


def test_loki_config_verifies_starts_and_is_ready(stack_containers: dict[str, str], tmp_path: Path) -> None:
    """loki -verify-config accepts the rendered config, and Loki started from it reports ready."""
    config = write(tmp_path, "loki.yml", render(STACK_DIR, "loki.yml.j2", "server"))
    verify = docker("run", "--rm", "-v", f"{config}:/etc/loki/config.yml:ro", LOKI_IMAGE,
                    "-config.file=/etc/loki/config.yml", "-verify-config")  # fmt: skip
    assert verify.returncode == 0, verify.stderr
    assert wait_ready(stack_containers["loki"], "http://127.0.0.1:3100/ready").strip() == "ready"


def substitute(expr: str) -> str:
    """Replace the Grafana variables of a query with concrete values.

    Args:
        expr (str): Query with Grafana variables.

    Returns:
        str: Query the plain tools accept.
    """
    for variable, value in sorted(GRAFANA_VARIABLES.items(), key=lambda kv: -len(kv[0])):
        expr = expr.replace(variable, value)
    return expr


def test_every_promql_query_parses(tmp_path: Path) -> None:
    """promtool check rules parses every dashboard PromQL expression (each one a recording rule)."""
    exprs = [(name, title, substitute(e)) for name in DASHBOARDS for title, e in dashboard_exprs(load_dashboard(name))]
    assert len(exprs) > 100
    rules = [{"record": f"dashboard:query{i}", "expr": expr} for i, (_, _, expr) in enumerate(exprs)]
    path = write(tmp_path, "rules.yml", yaml.safe_dump({"groups": [{"name": "dashboards", "rules": rules}]}))
    result = docker("run", "--rm", "--entrypoint", "promtool", "-v", f"{path}:/rules.yml:ro", PROMTOOL_IMAGE,
                    "check", "rules", "/rules.yml")  # fmt: skip
    assert result.returncode == 0, result.stdout + result.stderr
    assert f"SUCCESS: {len(rules)} rules found" in result.stdout


def test_every_logql_query_parses(stack_containers: dict[str, str]) -> None:
    """Loki's format_query parses every dashboard LogQL expression (and rejects a broken one)."""
    container = stack_containers["loki"]
    wait_ready(container, "http://127.0.0.1:3100/ready")
    exprs = [substitute(e) for name in DASHBOARDS for _, e in dashboard_exprs(load_dashboard(name), loki=True)]
    assert len(exprs) == 2
    for expr in exprs:
        result = probe(container, "http://127.0.0.1:3100/loki/api/v1/format_query?query=" + urllib.parse.quote(expr))
        assert result.returncode == 0, (expr, result.stderr)
        assert json.loads(result.stdout)["status"] == "success", expr
    broken = probe(container, "http://127.0.0.1:3100/loki/api/v1/format_query?query=" + urllib.parse.quote("{host="))
    assert broken.returncode != 0


@pytest.mark.parametrize("host", HOSTS)
def test_alloy_config_runs(host: str, tmp_path: Path) -> None:
    """alloy run loads the host's rendered config: every component evaluates and the HTTP server listens."""
    config = write(tmp_path, "config.alloy", render(AGENT_DIR, "config.alloy.j2", host))
    name = f"monitoring-test-alloy-{uuid.uuid4().hex[:8]}"
    started = docker("run", "-d", "--name", name, "-v", f"{config}:/etc/alloy/config.alloy:ro", ALLOY_IMAGE, "run",
                     "--server.http.listen-addr=127.0.0.1:12345", "/etc/alloy/config.alloy")  # fmt: skip
    assert started.returncode == 0, started.stderr
    try:
        deadline = time.monotonic() + 60
        logs = ""
        while time.monotonic() < deadline and "now listening" not in logs:
            time.sleep(2)
            result = docker("logs", name)
            logs = result.stdout + result.stderr
        assert "now listening" in logs, logs[-3000:]
        assert "failed to evaluate" not in logs, logs[-3000:]
        assert "Error:" not in logs, logs[-3000:]
    finally:
        docker("rm", "-f", name)


@pytest.mark.parametrize(("host", "site"), [("client", "robot"), ("server", "server")])
def test_nginx_site_passes_nginx_t(host: str, site: str, tmp_path: Path) -> None:
    """nginx -t accepts the host's rendered site (replacing the image's default server)."""
    config = write(tmp_path, f"{site}.conf", render(PROXY_DIR, f"nginx-{site}.conf.j2", host))
    result = docker("run", "--rm", "-v", f"{config}:/etc/nginx/conf.d/default.conf:ro", NGINX_IMAGE, "nginx", "-t")
    assert result.returncode == 0, result.stderr
    assert "test is successful" in result.stderr


def test_robot_nginx_redirects_grafana_keeping_path_and_query(tmp_path: Path) -> None:
    """The robot's /grafana/<path>?<query> answers 302 to http://server.ros2.lan/grafana/<path>?<query>."""
    config = write(tmp_path, "robot.conf", render(PROXY_DIR, "nginx-robot.conf.j2", "client"))
    name = start(NGINX_IMAGE, config, "/etc/nginx/conf.d/default.conf")
    try:
        deadline = time.monotonic() + 30
        headers = ""
        while time.monotonic() < deadline and "302" not in headers:
            time.sleep(1)
            result = docker("run", "--rm", "--network", f"container:{name}", PROBE_IMAGE, "wget", "-S", "--spider",
                            "-Y", "off", "http://127.0.0.1/grafana/d/robot-health?orgId=1&var-host=server")  # fmt: skip
            headers = result.stderr
        assert "HTTP/1.1 302" in headers, headers
        assert "Location: http://server.ros2.lan/grafana/d/robot-health?orgId=1&var-host=server" in headers, headers
    finally:
        docker("rm", "-f", name)
