"""Base node: one status object for /server/gps/status and the caster's GET /status, from every GGA."""

import json
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest
from ros_stubs import install_ros_stubs

install_ros_stubs("sensor_msgs", "sensor_msgs.msg", "std_msgs", "std_msgs.msg")

from gps_rtk import base_node  # noqa: E402
from gps_rtk.config import GpsRtkConfig  # noqa: E402

NO_FIX_GGA = "$GNGGA,000012.000,,,,,0,07,3.5,,,,,,*7A"
FIX_GGA = "$GNGGA,162123.000,5436.320777,N,01818.578046,E,4,27,0.58,11.965,M,33.666,M,,*64"
STATUS_TOPIC = "/server/gps/status"


@pytest.fixture
def node(monkeypatch: pytest.MonkeyPatch) -> Any:
    monkeypatch.setattr(base_node, "String", lambda data: SimpleNamespace(data=data))
    config = GpsRtkConfig.model_validate({"mode": "base", "topic": "/server/gps/fix", "status_topic": STATUS_TOPIC})
    return base_node.GpsRtkBaseNode(config)


def test_no_status_before_any_gga(node: Any) -> None:
    assert node.current_status() is None


def test_no_fix_gga_gives_status_without_position(node: Any) -> None:
    node._on_nmea(NO_FIX_GGA)
    status = node.current_status()
    assert status is not None
    assert status["quality"] == 0
    assert status["num_satellites"] == 7
    assert status["survey_state"] == "not_checked"
    assert status["gga_seq"] == 1
    assert "latitude" not in status


def test_fix_gga_adds_position_and_counts(node: Any) -> None:
    node._on_nmea(NO_FIX_GGA)
    node._on_nmea(FIX_GGA)
    status = node.current_status()
    assert status is not None
    assert status["quality"] == 4
    assert status["latitude"] == pytest.approx(54.605346, abs=1e-5)
    assert status["gga_seq"] == 2


def test_survey_state_comes_from_serial_handler(node: Any) -> None:
    node.serial = SimpleNamespace(survey_state="fixed_position")
    node._on_nmea(FIX_GGA)
    assert node.current_status()["survey_state"] == "fixed_position"


def test_published_status_equals_http_status(node: Any) -> None:
    node._on_nmea(FIX_GGA)
    node._publish_status()
    published = json.loads(node.publishers[STATUS_TOPIC].publish.call_args.args[0].data)
    assert published == json.loads(json.dumps(node.current_status()))


def test_caster_serves_current_status(node: Any, monkeypatch: pytest.MonkeyPatch) -> None:
    caster_cls = MagicMock()
    monkeypatch.setattr(base_node, "NtripCaster", caster_cls)
    monkeypatch.setattr(base_node, "SerialHandler", MagicMock())
    node.config.configure_on_start = False
    node.run()
    assert caster_cls.call_args.kwargs["status_provider"] == node.current_status
