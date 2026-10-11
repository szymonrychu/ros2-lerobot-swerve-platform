"""nginx on port 80 of both hosts (roles/web_proxy): robot web UI proxy + Grafana redirect, server Grafana."""

import re

import pytest

from tests.monitoring_helpers import HOSTS, PROXY_DIR, host_vars, render, resolve, role_tasks, tasks_using

SITES = {"client": "robot", "server": "server"}


def site(host: str) -> str:
    """Render the nginx site of a host.

    Args:
        host (str): client or server.

    Returns:
        str: Rendered site config.
    """
    return render(PROXY_DIR, f"nginx-{SITES[host]}.conf.j2", host)


def location(text: str, path: str) -> str:
    """Body of one nginx location block.

    Args:
        text (str): Rendered site.
        path (str): Location path, e.g. "/grafana/".

    Returns:
        str: Text between the braces.
    """
    match = re.search(r"location " + re.escape(path) + r" \{([^}]*)\}", text)
    assert match, path
    return match.group(1)


@pytest.mark.parametrize("host", HOSTS)
def test_site_per_host(host: str) -> None:
    """The robot gets the robot site, the server the server site."""
    assert host_vars(host)["web_proxy_site"] == SITES[host]
    assert (PROXY_DIR / "templates" / f"nginx-{SITES[host]}.conf.j2").is_file()


def test_robot_site_proxies_web_ui_with_websockets() -> None:
    """Robot port 80: / -> web_ui (8080) with WebSocket upgrade (live data, agent chat), unbuffered."""
    text = site("client")
    assert re.search(r"listen 80 default_server;", text)
    root = location(text, "/")
    assert "proxy_pass http://127.0.0.1:8080;" in root
    assert "proxy_set_header Upgrade $http_upgrade;" in root
    assert "proxy_set_header Connection $connection_upgrade;" in root
    assert "proxy_buffering off;" in root
    assert "map $http_upgrade $connection_upgrade" in text


def test_robot_ws_location_drops_stalled_clients() -> None:
    """/ws and /ws/agent: a browser that stops reading is cut after 30 s of blocked writes, with the upgrade headers."""
    ws = location(site("client"), "/ws")
    assert "proxy_pass http://127.0.0.1:8080;" in ws
    assert "proxy_send_timeout 30s;" in ws
    assert "send_timeout 30s;" in ws
    assert "proxy_read_timeout 1h;" in ws
    assert "proxy_set_header Upgrade $http_upgrade;" in ws
    assert "proxy_set_header Connection $connection_upgrade;" in ws
    assert "proxy_buffering off;" in ws


def test_robot_site_redirects_grafana_to_the_server() -> None:
    """Robot /grafana/... answers 302 to http://server.ros2.lan/grafana/... with the path and query kept."""
    body = location(site("client"), "/grafana/")
    assert "return 302 http://server.ros2.lan$request_uri;" in body
    assert "proxy_pass" not in body


def test_server_site_serves_grafana_and_redirects_root() -> None:
    """Server port 80: / -> 302 /grafana/, /grafana/ -> Grafana on 127.0.0.1:3000 (sub path kept) with WebSockets."""
    text = site("server")
    assert re.search(r"listen 80 default_server;", text)
    assert "return 302 /grafana/;" in location(text, "/")
    grafana = location(text, "/grafana/")
    assert "proxy_pass http://127.0.0.1:3000;" in grafana
    assert "proxy_set_header Upgrade $http_upgrade;" in grafana
    assert "proxy_set_header Connection $connection_upgrade;" in grafana
    assert "map $http_upgrade $connection_upgrade" in text


@pytest.mark.parametrize("host", HOSTS)
def test_nginx_installed_default_site_removed_and_checked(host: str) -> None:
    """nginx is installed, the distro default site removed, the host's site written, nginx -t run, nginx started."""
    tasks = role_tasks(PROXY_DIR)
    apt = [t for t in tasks_using(tasks, "ansible.builtin.apt") if "nginx" in str(t["ansible.builtin.apt"].get("name"))]
    assert apt, "nginx is not installed"
    sites = [t for t in tasks if "nginx-" in str((t.get("ansible.builtin.template") or {}).get("src", ""))]
    assert len(sites) == 1
    template = sites[0]["ansible.builtin.template"]
    assert resolve(template["src"], host) == f"nginx-{SITES[host]}.conf.j2"
    assert resolve(template["dest"], host) == f"/etc/nginx/conf.d/{SITES[host]}.conf"
    assert "Reload nginx" in str(sites[0]["notify"])
    removed = [
        t
        for t in tasks
        if (t.get("ansible.builtin.file") or {}).get("path") == "/etc/nginx/sites-enabled/default"
        and t["ansible.builtin.file"].get("state") == "absent"
    ]
    assert removed, "the distro default site still claims port 80"
    names = [t.get("name", "") for t in tasks]
    check = [t for t in tasks if "nginx -t" in str(t.get("ansible.builtin.command", ""))]
    assert len(check) == 1, "the nginx config is not checked before the reload"
    flush = next(i for i, t in enumerate(tasks) if t.get("ansible.builtin.meta") == "flush_handlers")
    assert names.index(check[0]["name"]) < flush
    started = [t["ansible.builtin.systemd_service"] for t in tasks_using(tasks, "ansible.builtin.systemd_service")]
    assert any(s["name"] == "nginx" and s["enabled"] is True and s["state"] == "started" for s in started)


def test_readiness_checks_per_host() -> None:
    """The robot's /grafana/ answers the redirect (path + query kept); the server's / redirects to /grafana/."""
    probe = next(t for t in role_tasks(PROXY_DIR) if "ansible.builtin.uri" in t)
    uri = probe["ansible.builtin.uri"]
    assert uri["status_code"] == 302
    assert uri["follow_redirects"] == "none"
    expected = {
        "client": (
            "http://127.0.0.1/grafana/d/robot-health?orgId=1",
            "http://server.ros2.lan/grafana/d/robot-health?orgId=1",
        ),
        # nginx makes the relative `return 302 /grafana/` absolute (absolute_redirect on).
        "server": ("http://127.0.0.1/", "http://127.0.0.1/grafana/"),
    }
    assert (
        probe["failed_when"]
        == "web_proxy_probe_result.location | default('') != web_proxy_probes[web_proxy_site].location"
    )
    for host, (url, header) in expected.items():
        assert resolve(uri["url"], host) == url
        assert host_vars(host)["web_proxy_probes"][SITES[host]]["location"] == header
