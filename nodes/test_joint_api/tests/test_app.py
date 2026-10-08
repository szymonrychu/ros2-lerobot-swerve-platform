"""Tests for GET/POST /joint-updates (publisher mocked so no ROS required)."""

from unittest.mock import patch

import pytest
from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

from test_joint_api.app import create_app, set_ros_publisher
from test_joint_api.config import ApiConfig


def _make_app() -> web.Application:
    """Create app with no ROS publisher so POST path is safe in tests."""
    set_ros_publisher(None, None)
    config = ApiConfig(host="0.0.0.0", port=8080, topic="/filter/input_joint_updates")
    return create_app(config)


@pytest.mark.asyncio
async def test_get_joint_updates_empty() -> None:
    """GET returns empty joints when nothing posted yet."""
    app = _make_app()
    async with TestClient(TestServer(app)) as client:
        resp = await client.get("/joint-updates")
        assert resp.status == 200
        data = await resp.json()
        assert "joints" in data
        assert data["joints"] == {}


@pytest.mark.asyncio
async def test_post_joint_updates_single() -> None:
    """POST single joint update returns 200 and GET returns it."""
    app = _make_app()
    async with TestClient(TestServer(app)) as client:
        with patch("test_joint_api.app._publish_joint_state"):
            resp = await client.post("/joint-updates", json={"joint_6": 0.15})
        assert resp.status == 200
        data = await resp.json()
        assert data.get("ok") is True
        assert data.get("joints") == {"joint_6": 0.15}
        get_resp = await client.get("/joint-updates")
        get_data = await get_resp.json()
        assert get_data["joints"] == {"joint_6": 0.15}


@pytest.mark.asyncio
async def test_post_joint_updates_multiple() -> None:
    """POST multiple joints returns 200 and GET returns them."""
    app = _make_app()
    async with TestClient(TestServer(app)) as client:
        with patch("test_joint_api.app._publish_joint_state"):
            resp = await client.post("/joint-updates", json={"joint_5": -0.2, "joint_6": 0.1})
        assert resp.status == 200
        data = await resp.json()
        assert data.get("joints") == {"joint_5": -0.2, "joint_6": 0.1}
        get_resp = await client.get("/joint-updates")
        get_data = await get_resp.json()
        assert get_data["joints"]["joint_5"] == -0.2
        assert get_data["joints"]["joint_6"] == 0.1


@pytest.mark.asyncio
async def test_post_invalid_json_returns_400() -> None:
    """POST with invalid JSON returns 400."""
    app = _make_app()
    async with TestClient(TestServer(app)) as client:
        resp = await client.post(
            "/joint-updates",
            data="not json",
            headers={"Content-Type": "application/json"},
        )
        assert resp.status == 400
        data = await resp.json()
        assert "error" in data


@pytest.mark.asyncio
async def test_post_non_object_returns_400() -> None:
    """POST with JSON array or number returns 400."""
    app = _make_app()
    async with TestClient(TestServer(app)) as client:
        resp = await client.post("/joint-updates", json=[1, 2])
        assert resp.status == 400
        resp2 = await client.post("/joint-updates", json=42)
        assert resp2.status == 400


@pytest.mark.asyncio
async def test_post_non_numeric_value_returns_400() -> None:
    """POST with non-numeric value returns 400."""
    app = _make_app()
    async with TestClient(TestServer(app)) as client:
        resp = await client.post("/joint-updates", json={"joint_6": "not a number"})
        assert resp.status == 400
        data = await resp.json()
        assert "error" in data


@pytest.mark.asyncio
async def test_post_empty_object_returns_400() -> None:
    """POST with empty object returns 400."""
    app = _make_app()
    async with TestClient(TestServer(app)) as client:
        resp = await client.post("/joint-updates", json={})
        assert resp.status == 400


# Gripper-only: physical tests must use only gripper joints (e.g. joint_5, joint_6).
GRIPPER_JOINT_NAMES = {"joint_5", "joint_6"}


@pytest.mark.asyncio
async def test_post_gripper_only_updates_accepted() -> None:
    """POST with only gripper joints (joint_5, joint_6) is accepted; used by tests that exercise the endpoint."""
    app = _make_app()
    async with TestClient(TestServer(app)) as client:
        with patch("test_joint_api.app._publish_joint_state"):
            resp = await client.post("/joint-updates", json={"joint_5": 0.0, "joint_6": 0.1})
        assert resp.status == 200
        data = await resp.json()
        assert set(data["joints"].keys()) == GRIPPER_JOINT_NAMES
