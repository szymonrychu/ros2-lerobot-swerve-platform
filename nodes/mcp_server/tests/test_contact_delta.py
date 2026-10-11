"""close_until_effort load-delta contact (arXiv 2608.23983): the rest and free-closing baselines, the persistence count,
the absolute / target_load / stall backstops, and the GripEvidence and GraspCheck a close reports. No ROS."""

import math
from pathlib import Path

import pytest

from mcp_server.arm import ArmController
from mcp_server.config import McpServerConfig
from mcp_server.grasp_check import band_verdict
from mcp_server.grip import resolve_grip_profile
from mcp_server.ik import ArmKinematics, load_joint_limits
from mcp_server.models import ArmMotionResult

from .fakes import FakeArmBackend

CONFIG = McpServerConfig()
KIN = ArmKinematics(CONFIG.arm.urdf_path, margin=CONFIG.limits.arm_limit_margin_rad)
FREE = 40.0  # closing load while the jaw closes freely (gearbox friction)
DELTA = 20.0  # contact_load_delta of the gentle preset in these tests (target_load 80, contact_effort_threshold 120)
START = 0.8
NAN = float("nan")


class Gripper:
    """on_sleep hook of a jaw that follows the command down to an optional wall (an object), with scripted loads.

    Before the close starts (no command below START yet) the load cycles through `rest`; afterwards it is FREE, the
    `wall_load` while the jaw is at the wall, and `spike` for `spike_samples` samples from `spike_at` s after the close
    started.
    """

    def __init__(
        self,
        be: FakeArmBackend,
        start: float = START,
        wall: float | None = None,
        rest: list[float] | None = None,
        free: float = FREE,
        wall_load: float = FREE,
        spike_at: float | None = None,
        spike: float = 0.0,
        spike_samples: int = 0,
    ) -> None:
        self.start = start
        self.wall = wall
        self.rest = rest or [0.0]
        self.free = free
        self.wall_load = wall_load
        self.spike_at = spike_at
        self.spike = spike
        self.spike_samples = spike_samples
        self.spiked = 0
        self.rest_index = 1
        self.t0: float | None = None
        be.positions["gripper"] = start
        be.efforts["gripper"] = self.rest[0]
        be.on_sleep = self

    def __call__(self, b: FakeArmBackend) -> None:
        if self.wall is not None:
            b.positions["gripper"] = max(b.positions["gripper"], self.wall)
        if self.t0 is None and any(c.get("gripper", self.start) < self.start - 1e-9 for c in b.commands):
            self.t0 = b.t
        if self.t0 is None:
            b.efforts["gripper"] = self.rest[self.rest_index % len(self.rest)]
            self.rest_index += 1
            return
        if self.wall is not None and b.positions["gripper"] <= self.wall + 1e-9:
            b.efforts["gripper"] = self.wall_load
            return
        if self.spike_at is not None and b.t - self.t0 >= self.spike_at and self.spiked < self.spike_samples:
            self.spiked += 1
            b.efforts["gripper"] = self.spike
            return
        b.efforts["gripper"] = self.free


def make(tmp_path: Path, be: FakeArmBackend, delta: float | None = DELTA) -> ArmController:
    cfg = CONFIG.model_copy(deep=True)
    cfg.arm.home_file = tmp_path / "home.yaml"
    gentle = cfg.grip_profiles.presets["gentle"]
    cfg.grip_profiles.presets["gentle"] = gentle.model_copy(update={"contact_load_delta": delta})
    return ArmController(be, KIN, load_joint_limits(cfg.arm.urdf_path), cfg)


def close(tmp_path: Path, be: FakeArmBackend, delta: float | None = DELTA) -> ArmMotionResult:
    return make(tmp_path, be, delta).set_gripper(close_until_effort=True, grip_profile="gentle")


def test_free_closing_baseline_is_collected_after_contact_allowed_and_drives_the_contact(tmp_path: Path) -> None:
    be = FakeArmBackend()
    Gripper(be, wall=0.4, rest=[0.0], wall_load=70.0)  # 70 - 40 = 30 >= 20, below target_load 80
    res = close(tmp_path, be)
    assert res.status == "grasped", res.message
    check = res.grasp_check
    assert check is not None
    ev = check.evidence
    assert ev.contact == "load_delta"
    assert ev.baseline_source == "free_closing"
    assert ev.baseline_load == pytest.approx(FREE)  # not the rest load 0
    assert ev.peak_load_delta == pytest.approx(30.0)
    assert ev.hold_load_delta == pytest.approx(30.0)
    assert res.positions["gripper"] == pytest.approx(0.4)
    assert "load" in res.message and "baseline" in res.message


@pytest.mark.parametrize(("samples", "status"), [(2, "closed_no_contact"), (3, "grasped")])
def test_contact_needs_the_persist_count_of_consecutive_samples(tmp_path: Path, samples: int, status: str) -> None:
    be = FakeArmBackend()
    Gripper(be, spike_at=2.5, spike=70.0, spike_samples=samples)
    res = close(tmp_path, be)
    assert res.status == status, res.message
    assert res.grasp_check is not None
    assert res.grasp_check.evidence.contact == ("load_delta" if samples == 3 else None)


def test_delta_contact_is_disarmed_until_the_baseline_is_collected(tmp_path: Path) -> None:
    be = FakeArmBackend()
    # contact_allowed opens 1.44 s into the gentle close (travel); a rise right after it, while the free-closing
    # baseline is still being collected, is no contact.
    g = Gripper(be, spike_at=1.5, spike=70.0, spike_samples=4)
    res = close(tmp_path, be)
    assert g.spiked == 4
    assert res.status == "closed_no_contact", res.message
    assert res.grasp_check is not None and res.grasp_check.evidence.contact is None


def test_a_rise_inside_the_ignore_window_is_not_contact(tmp_path: Path) -> None:
    be = FakeArmBackend()
    g = Gripper(be, spike_at=0.0, spike=70.0, spike_samples=5)  # breakaway strain at the start of the close
    res = close(tmp_path, be)
    assert g.spiked == 5
    assert res.status == "closed_no_contact", res.message
    ev = res.grasp_check.evidence
    assert ev.contact is None and ev.baseline_load == pytest.approx(FREE)


def test_a_profile_without_contact_load_delta_behaves_as_before(tmp_path: Path) -> None:
    spiking = FakeArmBackend()
    Gripper(spiking, spike_at=2.5, spike=70.0, spike_samples=3)
    res = close(tmp_path / "a", spiking, delta=None)
    plain = FakeArmBackend()
    Gripper(plain)
    ref = close(tmp_path / "b", plain, delta=None)
    assert res.status == ref.status == "closed_no_contact"
    assert spiking.commands == plain.commands
    assert res.grasp_check is not None and res.grasp_check.evidence.contact is None


def test_absolute_backstop_still_ends_the_close(tmp_path: Path) -> None:
    be = FakeArmBackend()
    Gripper(be, wall=0.4, wall_load=-130.0)  # opening-direction load: only |load| >= 120 can fire
    res = close(tmp_path, be)
    assert res.status == "grasped", res.message
    assert res.grasp_check.evidence.contact == "absolute"


def test_target_load_backstop_still_ends_the_close(tmp_path: Path) -> None:
    be = FakeArmBackend()
    Gripper(be, wall=0.4, wall_load=90.0)  # >= target_load 80 at once (delta would need 3 samples)
    res = close(tmp_path, be)
    assert res.status == "grasped", res.message
    assert "target_load" in res.message
    assert res.grasp_check.evidence.contact == "target_load"


def test_stall_backstop_still_ends_the_close(tmp_path: Path) -> None:
    be = FakeArmBackend()
    Gripper(be, wall=0.4, wall_load=FREE)  # the object stops the jaw without raising the load
    res = close(tmp_path, be)
    assert res.status == "grasped", res.message
    assert res.grasp_check.evidence.contact == "stall"


def test_a_close_starting_in_contact_uses_the_rest_baseline(tmp_path: Path) -> None:
    be = FakeArmBackend()
    Gripper(be, start=0.3, wall=0.3, rest=[140.0, 160.0], wall_load=150.0)
    res = close(tmp_path, be)
    ev = res.grasp_check.evidence
    assert ev.baseline_source == "rest"
    assert ev.baseline_load == pytest.approx(150.0)  # mean of the alternating rest loads
    assert ev.contact == "target_load"


def test_the_rest_baseline_skips_unmeasured_loads(tmp_path: Path) -> None:
    be = FakeArmBackend()
    Gripper(be, start=0.3, wall=0.3, rest=[NAN, 150.0], wall_load=150.0)
    res = close(tmp_path, be)
    ev = res.grasp_check.evidence
    assert ev.baseline_source == "rest" and ev.baseline_load == pytest.approx(150.0)


def test_nan_loads_give_no_baseline_and_no_delta_contact(tmp_path: Path) -> None:
    be = FakeArmBackend()
    Gripper(be, wall=0.4, rest=[NAN], free=NAN, wall_load=NAN)
    res = close(tmp_path, be)
    ev = res.grasp_check.evidence
    assert ev.baseline_load is None and ev.baseline_source is None
    assert ev.peak_load_delta is None and ev.hold_load_delta is None
    assert ev.contact == "stall"  # the stall backstop still ends the close
    assert res.grasp_check.band == "ambiguous"  # an unmeasured load is never 'object'


def test_grasp_check_of_a_held_close(tmp_path: Path) -> None:
    be = FakeArmBackend()
    Gripper(be, wall=0.4, wall_load=70.0)
    arm = make(tmp_path, be)
    res = arm.set_gripper(close_until_effort=True, grip_profile="gentle")
    check = res.grasp_check
    assert check is not None
    assert len(check.grasp_id) == 12 and all(c in "0123456789abcdef" for c in check.grasp_id)
    assert check.grasp_id == check.evidence.grasp_id
    assert check.image == "not_requested" and check.fused is None and check.crop is None
    ev = check.evidence
    assert ev.jaw_gap_rad == pytest.approx(abs(0.4 - CONFIG.arm.gripper_closed_rad))
    assert ev.in_empty_band is False
    expected = band_verdict(ev, DELTA, None, res.holding_load, CONFIG.grasp.hold_effort_min)
    assert check.band == ev.band == expected == "object"
    again = arm.set_gripper(close_until_effort=True, grip_profile="gentle")
    assert again.grasp_check is not None and again.grasp_check.grasp_id != check.grasp_id


def test_an_empty_close_is_band_empty(tmp_path: Path) -> None:
    be = FakeArmBackend()
    Gripper(be)
    res = close(tmp_path, be)
    assert res.status == "closed_no_contact"
    ev = res.grasp_check.evidence
    assert ev.in_empty_band is True and ev.contact is None
    assert ev.hold_load_delta == pytest.approx(0.0)
    assert res.grasp_check.band == "empty"


def test_current_evidence_from_rest_and_hold_samples(tmp_path: Path) -> None:
    be = FakeArmBackend()
    g = Gripper(be, wall=0.4, wall_load=70.0)

    def with_current(b: FakeArmBackend) -> None:
        g(b)
        b.currents["gripper"] = 0.5 if g.t0 is not None and b.positions["gripper"] <= 0.4 + 1e-9 else 0.1

    be.currents["gripper"] = 0.1
    be.on_sleep = with_current
    res = close(tmp_path, be)
    ev = res.grasp_check.evidence
    assert ev.rest_current_a == pytest.approx(0.1)
    assert ev.hold_current_a == pytest.approx(0.5)
    assert ev.current_delta_a == pytest.approx(0.4)


def test_hold_report_samples_the_gripper_current(tmp_path: Path) -> None:
    be = FakeArmBackend()
    be.currents["gripper"] = 0.3
    arm = make(tmp_path, be)
    grip = resolve_grip_profile(arm.cfg.grip_profiles, arm.cfg.limits, "gentle")
    report = arm.hold_report(grip)
    assert report["hold_current_a"] == pytest.approx(0.3)
    be.currents.clear()
    assert arm.hold_report(grip)["hold_current_a"] is None
    be.currents["gripper"] = NAN
    assert arm.hold_report(grip)["hold_current_a"] is None
    assert not math.isnan(arm.hold_report(grip)["hold_jaw_rad"])
