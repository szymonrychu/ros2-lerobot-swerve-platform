"""Rover node: status JSON from every GGA (also without a fix), receiver-silent status, NavSatFix only with a position."""

import json
from types import SimpleNamespace
from typing import Any

import pytest
from ros_stubs import install_ros_stubs

install_ros_stubs("sensor_msgs", "sensor_msgs.msg", "std_msgs", "std_msgs.msg")

from gps_rtk import metrics, rover_node  # noqa: E402
from gps_rtk.config import GpsRtkConfig  # noqa: E402

NO_FIX_GGA = "$GNGGA,000012.000,,,,,0,07,3.5,,,,,,*7A"
FIX_GGA = "$GNGGA,162123.000,5436.320777,N,01818.578046,E,5,27,0.58,11.965,M,33.666,M,1.0,3335*64"
RMC = "$GNRMC,000012.000,V,,,,,,,,,,N,V*20"
STATUS_TOPIC = "/client/gps/status"
FIX_TOPIC = "/client/gps/fix"


class Clock:
    """Settable monotonic clock."""

    def __init__(self, now: float) -> None:
        self.now = now

    def __call__(self) -> float:
        return self.now


@pytest.fixture
def clock() -> Clock:
    return Clock(1000.0)


@pytest.fixture
def node(monkeypatch: pytest.MonkeyPatch, clock: Clock) -> Any:
    monkeypatch.setattr(rover_node, "String", lambda data: SimpleNamespace(data=data))
    config = GpsRtkConfig.model_validate(
        {"mode": "rover", "topic": FIX_TOPIC, "status_topic": STATUS_TOPIC, "nmea_timeout_s": 5.0}
    )
    return rover_node.GpsRtkRoverNode(config, clock=clock)


def published_status(node: Any) -> list[dict[str, Any]]:
    return [json.loads(c.args[0].data) for c in node.publishers[STATUS_TOPIC].publish.call_args_list]


def test_no_fix_gga_publishes_status_but_no_navsatfix(node: Any) -> None:
    node._on_nmea(NO_FIX_GGA)
    node._publish_status()
    node._publish_cb()

    [status] = published_status(node)
    assert status["quality"] == 0
    assert status["fix"] == "No fix"
    assert status["num_satellites"] == 7
    assert status["hdop"] == 3.5
    assert status["nmea_silent"] is False
    node.publishers[FIX_TOPIC].publish.assert_not_called()


def test_no_fix_gga_exports_real_satellite_count(node: Any) -> None:
    node._on_nmea(NO_FIX_GGA)
    assert metrics.SATELLITES._value.get() == 7
    assert metrics.FIX_QUALITY._value.get() == 0


def test_fix_gga_publishes_navsatfix_and_status(node: Any) -> None:
    node._on_nmea(FIX_GGA)
    node._publish_status()
    node._publish_cb()

    [status] = published_status(node)
    assert status["quality"] == 5
    node.publishers[FIX_TOPIC].publish.assert_called_once()


def test_nothing_published_before_timeout_without_nmea(node: Any, clock: Clock) -> None:
    clock.now += 2.0
    node._publish_status()
    assert published_status(node) == []


def test_receiver_silent_since_start(node: Any, clock: Clock) -> None:
    clock.now += 6.0
    node._publish_status()
    [status] = published_status(node)
    assert status["nmea_silent"] is True
    assert status["nmea_received"] is False
    assert status["nmea_age_s"] == 6.0
    assert status["ntrip_connected"] is False
    assert status["ntrip_rx_bytes"] == 0


def test_receiver_silent_after_nmea_stops(node: Any, clock: Clock) -> None:
    node._on_nmea(NO_FIX_GGA)
    clock.now += 10.0
    node._publish_status()
    [status] = published_status(node)
    assert status["nmea_silent"] is True
    assert status["nmea_received"] is True
    assert status["nmea_age_s"] == 10.0
    assert status["quality"] is None


def test_non_gga_sentences_keep_receiver_not_silent(node: Any, clock: Clock) -> None:
    clock.now += 6.0
    node._on_nmea(RMC)
    node._publish_status()
    assert published_status(node) == []
