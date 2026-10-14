"""Save-now and reset calibration services: the pure service logic on a fake BNO055, then wired into run_imu_node."""

import json
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest

import bno055_imu.calibration as calibration_module
import bno055_imu.calibration_service as service_module
import bno055_imu.node as node_module
from bno055_imu.calibration import CalibrationProfile, load_profile, write_profile
from bno055_imu.calibration_service import (
    SYS_TRIGGER_REGISTER,
    SYS_TRIGGER_RST_SYS,
    reset_calibration,
    save_calibration,
)
from bno055_imu.config import ImuNodeConfig

from .ros_stubs import FakeRclpy, install_ros_stubs

install_ros_stubs()

IMUPLUS = 0x08
CONFIG_MODE = 0x00
CHIP_PROFILE = CalibrationProfile(
    accel_offset=(1, -2, 3), gyro_offset=(4, 5, -6), mag_offset=(70, -80, 90), accel_radius=1000, mag_radius=700
)
OLD_PROFILE = CalibrationProfile(
    accel_offset=(0, 0, 0), gyro_offset=(0, 0, 0), mag_offset=(1, 1, 1), accel_radius=1000, mag_radius=500
)


class FakeTime:
    """time module double: monotonic advances 20 ms per call, sleep is a no-op."""

    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        self.now += 0.02
        return self.now

    def sleep(self, _s: float) -> None:
        pass


class FakeBno:
    """BNO055 driver double: records mode writes and register writes, serves offsets in CONFIG mode."""

    def __init__(self, status: tuple = (3, 3, 3, 3), profile: CalibrationProfile = CHIP_PROFILE) -> None:
        self._mode = IMUPLUS
        self.mode_writes: list[int] = []
        self.registers: list[tuple[int, int]] = []
        self.calibration_status = status
        self.profile = profile
        self.read_error: Exception | None = None
        self.reset_error: Exception | None = None

    @property
    def mode(self) -> int:
        return self._mode

    @mode.setter
    def mode(self, value: int) -> None:
        self.mode_writes.append(value)
        self._mode = value

    def offset(self, value: Any) -> Any:
        if self.read_error is not None:
            raise self.read_error
        assert self._mode == CONFIG_MODE, "offsets are only read in CONFIG mode"
        return value

    @property
    def offsets_accelerometer(self) -> tuple[int, int, int]:
        return self.offset(self.profile.accel_offset)

    @property
    def offsets_gyroscope(self) -> tuple[int, int, int]:
        return self.offset(self.profile.gyro_offset)

    @property
    def offsets_magnetometer(self) -> tuple[int, int, int]:
        return self.offset(self.profile.mag_offset)

    @property
    def radius_accelerometer(self) -> int:
        return self.offset(self.profile.accel_radius)

    @property
    def radius_magnetometer(self) -> int:
        return self.offset(self.profile.mag_radius)

    def _write_register(self, register: int, value: int) -> None:
        if self.reset_error is not None:
            raise self.reset_error
        self.registers.append((register, value))

    @property
    def config_reads(self) -> int:
        """Number of CONFIG-mode round trips (each read_profile writes CONFIG mode once on a responsive chip)."""
        return self.mode_writes.count(CONFIG_MODE)


@pytest.fixture(autouse=True)
def no_sleep(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(calibration_module, "time", FakeTime())
    monkeypatch.setattr(service_module, "time", FakeTime())


# ---------------------------------------------------------------------------
# save_calibration
# ---------------------------------------------------------------------------


def test_save_writes_the_chip_profile_when_mag_is_3_and_stationary(tmp_path: Path) -> None:
    path = tmp_path / "state" / "calibration.json"
    bno = FakeBno()
    outcome = save_calibration(bno, path, IMUPLUS, (2, 3, 1, 3), stationary=True)
    assert outcome.success is True
    assert outcome.chip_read is True
    assert outcome.profile == CHIP_PROFILE
    assert load_profile(path) == CHIP_PROFILE
    assert str(path) in outcome.message
    assert "sys 2, gyro 3, accel 1, mag 3" in outcome.message
    assert bno.mode == IMUPLUS  # the operation mode is restored after the CONFIG-mode read


def test_save_writes_even_when_the_profile_is_unchanged(tmp_path: Path) -> None:
    path = tmp_path / "calibration.json"
    write_profile(path, CHIP_PROFILE)
    path.write_text(path.read_text() + "\n")  # marker: a rewrite drops the extra newline
    outcome = save_calibration(FakeBno(), path, IMUPLUS, (3, 3, 3, 3), stationary=True)
    assert outcome.success is True
    assert not path.read_text().endswith("\n\n")
    assert load_profile(path) == CHIP_PROFILE


def test_save_with_unknown_stationarity_is_allowed(tmp_path: Path) -> None:
    path = tmp_path / "calibration.json"
    outcome = save_calibration(FakeBno(), path, IMUPLUS, (0, 3, 0, 3), stationary=None)
    assert outcome.success is True
    assert load_profile(path) == CHIP_PROFILE


@pytest.mark.parametrize("mag", [0, 1, 2])
def test_save_refuses_when_mag_is_below_3(tmp_path: Path, mag: int) -> None:
    path = tmp_path / "calibration.json"
    bno = FakeBno(status=(3, 3, 3, mag))
    outcome = save_calibration(bno, path, IMUPLUS, (3, 3, 3, mag), stationary=True)
    assert outcome.success is False
    assert outcome.message == f"mag is {mag}, needs 3"
    assert outcome.chip_read is False
    assert not path.exists()
    assert bno.mode_writes == []  # the chip was not touched


def test_save_refuses_while_moving(tmp_path: Path) -> None:
    path = tmp_path / "calibration.json"
    bno = FakeBno()
    outcome = save_calibration(bno, path, IMUPLUS, (3, 3, 3, 3), stationary=False)
    assert outcome.success is False
    assert outcome.message == "robot is moving"
    assert not path.exists()
    assert bno.mode_writes == []


def test_save_refuses_when_the_calibration_file_is_disabled() -> None:
    bno = FakeBno()
    outcome = save_calibration(bno, None, IMUPLUS, (3, 3, 3, 3), stationary=True)
    assert outcome.success is False
    assert outcome.message == "calibration_file disabled"
    assert bno.mode_writes == []


def test_save_refuses_when_the_status_is_unavailable(tmp_path: Path) -> None:
    path = tmp_path / "calibration.json"
    outcome = save_calibration(FakeBno(), path, IMUPLUS, None, stationary=True)
    assert outcome.success is False
    assert "unavailable" in outcome.message
    assert not path.exists()


def test_save_surfaces_a_read_error_and_writes_nothing(tmp_path: Path) -> None:
    path = tmp_path / "calibration.json"
    bno = FakeBno()
    bno.read_error = OSError("[Errno 121] Remote I/O error")
    outcome = save_calibration(bno, path, IMUPLUS, (3, 3, 3, 3), stationary=True)
    assert outcome.success is False
    assert "Remote I/O error" in outcome.message
    assert outcome.chip_read is True  # the CONFIG round trip happened: the automatic save must not repeat it
    assert outcome.profile is None
    assert not path.exists()
    assert bno.mode == IMUPLUS


def test_save_refuses_an_implausible_chip_profile(tmp_path: Path) -> None:
    path = tmp_path / "calibration.json"
    bad = replace(CHIP_PROFILE, mag_radius=0)
    outcome = save_calibration(FakeBno(profile=bad), path, IMUPLUS, (3, 3, 3, 3), stationary=True)
    assert outcome.success is False
    assert "mag_radius" in outcome.message
    assert not path.exists()


def test_save_surfaces_a_write_error(tmp_path: Path) -> None:
    blocker = tmp_path / "not_a_dir"
    blocker.write_text("")
    outcome = save_calibration(FakeBno(), blocker / "calibration.json", IMUPLUS, (3, 3, 3, 3), stationary=True)
    assert outcome.success is False
    assert outcome.profile is None
    assert "not_a_dir" in outcome.message


# ---------------------------------------------------------------------------
# reset_calibration
# ---------------------------------------------------------------------------


def test_reset_deletes_the_file_writes_the_reset_bit_and_requests_reinit_without_profile(tmp_path: Path) -> None:
    path = tmp_path / "calibration.json"
    write_profile(path, CHIP_PROFILE)
    bno = FakeBno()
    outcome = reset_calibration(bno, path, stationary=True)
    assert outcome.success is True
    assert outcome.reinit is True
    assert not path.exists()
    assert load_profile(path) is None  # the re-init has no profile to restore
    assert bno.registers == [(SYS_TRIGGER_REGISTER, SYS_TRIGGER_RST_SYS)]
    assert (SYS_TRIGGER_REGISTER, SYS_TRIGGER_RST_SYS) == (0x3F, 0x20)
    assert outcome.message


def test_reset_with_a_missing_file_is_fine(tmp_path: Path) -> None:
    bno = FakeBno()
    outcome = reset_calibration(bno, tmp_path / "calibration.json", stationary=None)
    assert outcome.success is True
    assert outcome.reinit is True
    assert bno.registers == [(SYS_TRIGGER_REGISTER, SYS_TRIGGER_RST_SYS)]


def test_reset_with_persistence_disabled_still_resets_the_chip() -> None:
    bno = FakeBno()
    outcome = reset_calibration(bno, None, stationary=True)
    assert outcome.success is True
    assert outcome.reinit is True
    assert bno.registers == [(SYS_TRIGGER_REGISTER, SYS_TRIGGER_RST_SYS)]


def test_reset_is_refused_while_moving(tmp_path: Path) -> None:
    path = tmp_path / "calibration.json"
    write_profile(path, CHIP_PROFILE)
    bno = FakeBno()
    outcome = reset_calibration(bno, path, stationary=False)
    assert outcome.success is False
    assert outcome.message == "robot is moving"
    assert outcome.reinit is False
    assert path.exists()
    assert bno.registers == []


def test_reset_surfaces_a_delete_error_and_leaves_the_chip_alone(tmp_path: Path) -> None:
    path = tmp_path / "calibration.json"
    path.mkdir()  # unlink on a directory fails
    bno = FakeBno()
    outcome = reset_calibration(bno, path, stationary=True)
    assert outcome.success is False
    assert outcome.reinit is False
    assert str(path) in outcome.message
    assert bno.registers == []


def test_reset_surfaces_a_chip_write_error(tmp_path: Path) -> None:
    bno = FakeBno()
    bno.reset_error = OSError("[Errno 121] Remote I/O error")
    outcome = reset_calibration(bno, tmp_path / "calibration.json", stationary=True)
    assert outcome.success is False
    assert outcome.reinit is False
    assert "Remote I/O error" in outcome.message


# ---------------------------------------------------------------------------
# run_imu_node wiring
# ---------------------------------------------------------------------------

CONFIG = ImuNodeConfig(
    topic="/imu/data",
    frame_id="imu_link",
    publish_hz=50.0,
    i2c_bus=1,
    i2c_address=0x28,
    orientation_covariance=[0.01] * 9,
    angular_velocity_covariance=[0.01] * 9,
    linear_acceleration_covariance=[0.04] * 9,
    compute_covariance=False,
    covariance_window=100,
    covariance_min_samples=20,
    calibration_topic="/imu/calibration",
    calibration_file=None,
    odom_topic=None,
    gyro_bias_file=None,
    gyro_bias_topic=None,
)
SAMPLE = SimpleNamespace(
    quaternion=(1.0, 0.0, 0.0, 0.0), gyro=(0.0, 0.0, 0.0), linear_acceleration=(0.0, 0.0, 9.8), euler=(0, 0, 0)
)
CYCLES = 300


class NodeHarness:
    """Drives run_imu_node on a fake rclpy; services are called from inside a publish, as spin_once would."""

    def __init__(self, monkeypatch: pytest.MonkeyPatch, chips: list[FakeBno]) -> None:
        self.fake = FakeRclpy()
        self.chips = chips
        self.creates: list[bool] = []
        self.imu_pub = MagicMock(name="imu")
        self.calibration_pub = MagicMock(name="calibration")
        self.actions: dict[int, str] = {}
        self.responses: list[SimpleNamespace] = []
        self.published = 0
        self.payloads: list[tuple[int, dict]] = []
        monkeypatch.setattr(node_module, "rclpy", self.fake)
        monkeypatch.setattr(node_module, "Node", MagicMock())
        monkeypatch.setattr(node_module, "SingleThreadedExecutor", MagicMock())
        monkeypatch.setattr(node_module, "start_metrics_server", MagicMock())
        monkeypatch.setattr(node_module, "create_sensor", self.create_sensor)
        monkeypatch.setattr(node_module, "_warmup", lambda *_args: True)
        monkeypatch.setattr(node_module, "read_sample", lambda _bno: SAMPLE)
        monkeypatch.setattr(node_module, "build_imu_message", MagicMock())
        monkeypatch.setattr(node_module, "pace_cycle", lambda _executor, slot_s, _period_s: slot_s)
        monkeypatch.setattr(node_module, "String", lambda data="": SimpleNamespace(data=data))
        monkeypatch.setattr(node_module, "time", FakeTime())
        self.node = node_module.Node.return_value
        self.node.create_publisher.side_effect = lambda _type, topic, _qos: (
            self.calibration_pub if topic == "/imu/calibration" else self.imu_pub
        )
        self.imu_pub.publish.side_effect = self.on_publish
        self.calibration_pub.publish.side_effect = lambda msg: self.payloads.append(
            (self.published, json.loads(msg.data))
        )

    def create_sensor(self, config: ImuNodeConfig, _node: object, restore: bool = True) -> tuple:
        self.creates.append(restore)
        profile = load_profile(Path(config.calibration_file)) if restore and config.calibration_file else None
        return self.chips[len(self.creates) - 1], 0x28, profile

    def service(self, name: str) -> Any:
        for call in self.node.create_service.call_args_list:
            if call.args[1] == name:
                return call.args[2]
        raise AssertionError(f"no service {name}")

    def on_publish(self, _msg: object) -> None:
        self.published += 1
        name = self.actions.get(self.published)
        if name is not None:
            response = SimpleNamespace(success=None, message=None)
            self.responses.append(self.service(name)(SimpleNamespace(), response))
        if self.published >= CYCLES:
            self.fake.alive = False

    def run(self, config: ImuNodeConfig) -> None:
        node_module.run_imu_node(config)


def test_node_serves_the_configured_service_names(monkeypatch: pytest.MonkeyPatch) -> None:
    harness = NodeHarness(monkeypatch, [FakeBno()])
    harness.run(
        replace(CONFIG, save_calibration_service="/imu/calibration/save", reset_calibration_service="/imu/cal/reset")
    )
    names = [call.args[1] for call in harness.node.create_service.call_args_list]
    assert names == ["/imu/calibration/save", "/imu/cal/reset"]


def test_node_serves_nothing_when_the_services_are_disabled(monkeypatch: pytest.MonkeyPatch) -> None:
    harness = NodeHarness(monkeypatch, [FakeBno()])
    harness.run(replace(CONFIG, save_calibration_service=None, reset_calibration_service=None))
    harness.node.create_service.assert_not_called()


def test_node_save_service_saves_and_the_automatic_save_does_not_repeat(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    path = tmp_path / "calibration.json"
    chip = FakeBno(status=(1, 3, 0, 3))
    harness = NodeHarness(monkeypatch, [chip])
    harness.actions = {1: "/imu/calibration/save"}  # before the first automatic check, which is due every cycle
    harness.run(replace(CONFIG, calibration_file=str(path), calibration_save_interval_s=0.0))
    [response] = harness.responses
    assert response.success is True
    assert str(path) in response.message
    assert load_profile(path) == CHIP_PROFILE
    assert chip.config_reads == 1  # the episode is marked read: no second blank cycle from the automatic save
    assert harness.payloads[-1][1]["restored"] is True


def test_node_save_service_refusal_writes_nothing(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    path = tmp_path / "calibration.json"
    chip = FakeBno(status=(0, 3, 0, 2))
    harness = NodeHarness(monkeypatch, [chip])
    harness.actions = {5: "/imu/calibration/save"}
    harness.run(replace(CONFIG, calibration_file=str(path)))
    [response] = harness.responses
    assert response.success is False
    assert response.message == "mag is 2, needs 3"
    assert not path.exists()
    assert chip.config_reads == 0


def test_node_reset_service_resets_and_reinitialises_without_profile(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    path = tmp_path / "calibration.json"
    write_profile(path, OLD_PROFILE)
    before, after = FakeBno(status=(3, 3, 3, 3)), FakeBno(status=(0, 0, 0, 0))
    harness = NodeHarness(monkeypatch, [before, after])
    reset_at = 100
    harness.actions = {reset_at: "/imu/calibration/reset"}
    harness.run(replace(CONFIG, calibration_file=str(path), calibration_save_interval_s=1e9))
    [response] = harness.responses
    assert response.success is True
    assert not path.exists()
    assert before.registers == [(SYS_TRIGGER_REGISTER, SYS_TRIGGER_RST_SYS)]
    assert harness.creates == [True, False]  # the re-init does not restore a profile
    assert after.mode == IMUPLUS
    old = [p for n, p in harness.payloads if n < reset_at]
    new = [p for n, p in harness.payloads if n > reset_at]
    assert old and all(p["restored"] is True for p in old)
    assert new and all(p["restored"] is False for p in new)
    assert new[-1]["mag"] == 0 and new[-1]["sys"] == 0


def test_node_reset_publishes_nothing_between_reset_and_reinit(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    read_on: list[FakeBno] = []
    before, after = FakeBno(), FakeBno(status=(0, 0, 0, 0))
    harness = NodeHarness(monkeypatch, [before, after])

    def read(bno: FakeBno) -> SimpleNamespace:
        read_on.append(bno)
        return SAMPLE

    monkeypatch.setattr(node_module, "read_sample", read)
    reset_at = 50
    harness.actions = {reset_at: "/imu/calibration/reset"}
    harness.run(replace(CONFIG, calibration_file=str(tmp_path / "calibration.json")))
    assert read_on[:reset_at] == [before] * reset_at
    assert all(bno is after for bno in read_on[reset_at:])  # never a read of the reset chip before the re-init


def test_node_save_during_pending_reset_is_refused(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    path = tmp_path / "calibration.json"
    harness = NodeHarness(monkeypatch, [FakeBno(), FakeBno(status=(0, 0, 0, 0))])
    harness.actions = {20: "/imu/calibration/reset"}

    original = harness.on_publish

    def publish_then_save(msg: object) -> None:
        original(msg)
        if harness.published == 20:
            response = SimpleNamespace(success=None, message=None)
            harness.responses.append(harness.service("/imu/calibration/save")(SimpleNamespace(), response))

    harness.imu_pub.publish.side_effect = publish_then_save
    harness.run(replace(CONFIG, calibration_file=str(path)))
    reset, save = harness.responses
    assert reset.success is True
    assert save.success is False
    assert "reset" in save.message
    assert not path.exists()
