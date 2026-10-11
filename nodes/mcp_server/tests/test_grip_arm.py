"""ArmController grip profiles: torque_limit write/restore, close speed, target_load stop, squeeze, hold reporting."""

from pathlib import Path

import pytest

from mcp_server.arm import ArmController, ArmError
from mcp_server.config import McpServerConfig
from mcp_server.ik import ArmKinematics, load_joint_limits
from mcp_server.staleness import Stamped

from .fakes import FakeArmBackend

CONFIG = McpServerConfig()
KIN = ArmKinematics(CONFIG.arm.urdf_path, margin=CONFIG.limits.arm_limit_margin_rad)
HOLD_DELAY = 0.37  # distinct hold_check_delay_s so the fake can tell the post-grasp hold check sleeps apart
STALL = -0.09  # jaw stops on a thin object 0.075 rad before closed, within the settle tolerance (0.59 rad travel)
DEFAULT_TORQUE = CONFIG.grip_profiles.presets["normal"].torque_limit
GENTLE = CONFIG.grip_profiles.presets["gentle"]
# STALL settles within the settle tolerance of closed, where the default gripper_empty_stall_rad (0.08) calls it an
# empty jaw: these tests lower it to exercise the settled stall-path hold. Stalls farther from closed (an object
# loading the jaw below target_load) are classified by position: test_*_stall_* below with the default config.
EMPTY_STALL = 0.05


def make(tmp_path: Path, be: FakeArmBackend | None = None) -> tuple[ArmController, FakeArmBackend]:
    be = be or FakeArmBackend()
    cfg = CONFIG.model_copy(deep=True)
    cfg.arm.home_file = tmp_path / "home.yaml"
    cfg.grip_profiles.hold_check_delay_s = HOLD_DELAY
    cfg.limits.gripper_empty_stall_rad = EMPTY_STALL  # STALL is a thin object here, not an empty jaw
    return ArmController(be, KIN, load_joint_limits(cfg.arm.urdf_path), cfg), be


def stalling(stop: float = STALL, start: float = 0.5, hold_load: float = 100.0) -> FakeArmBackend:
    """Follower whose jaw cannot close beyond `stop` (an object between the fingers, no load reported while
    closing) and that reports a healthy `hold_load` during the post-grasp hold check (no re-tighten)."""
    be = FakeArmBackend()
    be.positions["gripper"] = start

    def blocked(b: FakeArmBackend) -> None:
        b.positions["gripper"] = max(b.positions["gripper"], stop)
        b.efforts["gripper"] = hold_load if b.last_sleep == HOLD_DELAY else 0.0

    be.on_sleep = blocked
    return be


def torque_writes(be: FakeArmBackend) -> list[int]:
    return [v for j, r, v in be.register_writes if j == "gripper" and r == "torque_limit"]


def test_gentle_close_writes_the_torque_limit_before_closing(tmp_path: Path) -> None:
    arm, be = make(tmp_path, stalling())
    res = arm.set_gripper(close_until_effort=True, grip_profile="gentle")
    assert res.status == "grasped", res.message
    first_register = next(i for i, e in enumerate(be.timeline) if e[0] == "register")
    first_close = next(i for i, e in enumerate(be.timeline) if e[0] == "command" and e[1]["gripper"] < 0.5)
    assert first_register < first_close
    assert torque_writes(be) == [GENTLE.torque_limit]  # kept while holding the object
    assert res.grip_profile is not None and res.grip_profile["name"] == "gentle"
    assert res.grip_profile["torque_limit"] == GENTLE.torque_limit


def test_close_speed_comes_from_the_profile(tmp_path: Path) -> None:
    arm, be = make(tmp_path, stalling())
    arm.set_gripper(close_until_effort=True, grip_profile="gentle")
    rate = CONFIG.limits.arm_rate_hz
    grip = [c["gripper"] for c in be.commands[:-1]]  # the last command is the squeeze hold
    steps = [abs(b - a) * rate for a, b in zip(grip, grip[1:], strict=False)]
    assert max(steps) <= GENTLE.close_speed_rps * 1.05
    arm2, be2 = make(tmp_path, stalling())
    arm2.set_gripper(close_until_effort=True, grip_profile="normal")
    assert len(be2.commands) < len(be.commands)  # normal closes faster


def test_stall_hold_uses_the_profile_squeeze(tmp_path: Path) -> None:
    arm, be = make(tmp_path, stalling())
    arm.set_gripper(close_until_effort=True, grip_profile="gentle")
    assert be.commands[-1]["gripper"] == pytest.approx(STALL - GENTLE.squeeze_rad)
    arm, be = make(tmp_path, stalling())
    arm.set_gripper(close_until_effort=True, grip_profile="firm")
    assert be.commands[-1]["gripper"] == pytest.approx(STALL - CONFIG.grip_profiles.presets["firm"].squeeze_rad)


def test_plain_close_uses_the_default_profile_squeeze(tmp_path: Path) -> None:
    arm, be = make(tmp_path, stalling())
    res = arm.set_gripper(open_fraction=0.0)
    assert res.status == "grasped"
    assert be.commands[-1]["gripper"] == pytest.approx(STALL - CONFIG.grip_profiles.default.squeeze_rad)


def load_ramp(sign: float) -> FakeArmBackend:
    """Jaw closes freely from 1.0; past 0.6 rad the load grows 1000 units/rad with the given sign."""
    be = FakeArmBackend()
    be.positions["gripper"] = 1.0
    be.on_sleep = lambda b: b.efforts.update(gripper=sign * max(0.0, (0.6 - b.positions["gripper"]) * 1000.0))
    return be


def test_target_load_stops_the_close_once_the_closing_load_reaches_it(tmp_path: Path) -> None:
    arm, be = make(tmp_path, load_ramp(+1.0))
    res = arm.set_gripper(close_until_effort=True, grip_profile="gentle")
    assert res.status == "grasped", res.message
    assert "target_load" in res.message
    jaw = res.positions["gripper"]
    # Stopped near 0.6 - target_load/1000 (0.52 rad for 80), before the contact threshold (0.48 rad for 120).
    stop = 0.6 - GENTLE.target_load / 1000.0
    assert stop - 0.04 < jaw < stop + 0.02
    assert jaw > 0.6 - GENTLE.contact_effort_threshold / 1000.0
    assert be.commands[-1]["gripper"] == pytest.approx(jaw)  # holds where the load reached the target


def test_target_load_ignores_load_in_the_opening_direction(tmp_path: Path) -> None:
    be = FakeArmBackend()
    be.positions["gripper"] = 1.0
    be.on_sleep = lambda b: b.efforts.update(gripper=-110.0 if b.positions["gripper"] < 0.6 else 0.0)
    arm, be = make(tmp_path, be)
    res = arm.set_gripper(close_until_effort=True, grip_profile="gentle")
    assert res.status == "closed_no_contact", res.message


def test_closing_load_sign_is_configurable(tmp_path: Path) -> None:
    arm, _ = make(tmp_path, load_ramp(-1.0))
    arm.cfg.grip_profiles.closing_load_sign = -1
    res = arm.set_gripper(close_until_effort=True, grip_profile="gentle")
    assert res.status == "grasped" and "target_load" in res.message


def test_effort_threshold_argument_overrides_the_profile_contact_threshold(tmp_path: Path) -> None:
    arm, _ = make(tmp_path, load_ramp(+1.0))
    res = arm.set_gripper(close_until_effort=True, effort_threshold=50.0, grip_profile="normal")
    assert res.status == "grasped"
    assert res.positions["gripper"] > 0.53  # contact at 50 units (0.55 rad), not normal's 300


def test_torque_restored_when_the_close_finds_nothing(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.positions["gripper"] = 0.5
    res = arm.set_gripper(close_until_effort=True, grip_profile="gentle")
    assert res.status == "closed_no_contact"
    assert torque_writes(be) == [GENTLE.torque_limit, DEFAULT_TORQUE]


def test_torque_restored_when_the_close_is_aborted(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.positions["gripper"] = 1.0
    ticks = [0]

    def stop_soon(_b: FakeArmBackend) -> None:
        ticks[0] += 1
        if ticks[0] == 5:
            arm.request_stop()

    be.on_sleep = stop_soon
    res = arm.set_gripper(close_until_effort=True, grip_profile="firm")
    assert res.status == "stopped"
    assert torque_writes(be) == [CONFIG.grip_profiles.presets["firm"].torque_limit, DEFAULT_TORQUE]


def test_torque_restored_when_the_close_raises(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.positions["gripper"] = 1.0

    def crash(_b: FakeArmBackend) -> None:
        raise RuntimeError("bus error")

    be.on_sleep = crash
    with pytest.raises(RuntimeError):
        arm.set_gripper(close_until_effort=True, grip_profile="gentle")
    assert torque_writes(be) == [GENTLE.torque_limit, DEFAULT_TORQUE]


def test_torque_restored_after_an_open(tmp_path: Path) -> None:
    arm, be = make(tmp_path, stalling())
    arm.set_gripper(close_until_effort=True, grip_profile="gentle")
    be.on_sleep = None
    arm.set_gripper(open_fraction=0.6)
    assert torque_writes(be) == [GENTLE.torque_limit, DEFAULT_TORQUE]
    # The default is restored only after the open moved (never squeezing harder before the jaw opened).
    restore = max(i for i, e in enumerate(be.timeline) if e[0] == "register")
    assert all(e[0] != "command" for e in be.timeline[restore + 1 :])


def test_open_at_the_default_torque_writes_nothing(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    arm.apply_startup_torque_limit()
    be.register_writes.clear()
    arm.set_gripper(open_fraction=0.6)
    assert torque_writes(be) == []


@pytest.mark.parametrize("end", ["release", "drop_lease"])
def test_torque_restored_when_the_lease_ends(tmp_path: Path, end: str) -> None:
    arm, be = make(tmp_path, stalling())
    arm.set_gripper(close_until_effort=True, grip_profile="gentle")
    if end == "release":
        arm.release()
    else:
        arm.drop_lease()
    assert torque_writes(be) == [GENTLE.torque_limit, DEFAULT_TORQUE]


def test_startup_writes_the_default_torque_limit(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    arm.apply_startup_torque_limit()
    assert torque_writes(be) == [DEFAULT_TORQUE]


def test_inline_profile_is_capped_before_the_write(tmp_path: Path) -> None:
    arm, be = make(tmp_path, stalling())
    arm.cfg.grip_profiles.squeeze_max_rad = 0.05
    res = arm.set_gripper(close_until_effort=True, grip_profile={"torque_limit": 1000, "squeeze_rad": 0.2})
    assert torque_writes(be)[0] == CONFIG.grip_profiles.torque_limit_max
    assert be.commands[-1]["gripper"] == pytest.approx(STALL - 0.05)
    assert res.grip_profile is not None
    assert res.grip_profile["capped"] == ["squeeze_rad", "torque_limit"]
    assert res.grip_profile["name"] == "normal+custom"


def test_grip_profile_needs_close_until_effort(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    with pytest.raises(ArmError, match="close_until_effort"):
        arm.set_gripper(open_fraction=0.5, grip_profile="gentle")
    assert be.commands == []


def test_unknown_profile_is_refused_before_anything_moves(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    with pytest.raises(ArmError, match="unknown grip_profile"):
        arm.set_gripper(close_until_effort=True, grip_profile="crushing")
    assert be.commands == [] and be.register_writes == []


def holding(load: float, slip_per_check: float = 0.0) -> FakeArmBackend:
    """Stalling jaw that reports `load` during the post-grasp hold check and creeps closed by slip_per_check."""
    be = stalling()

    def hold_check(b: FakeArmBackend) -> None:
        if b.last_sleep == HOLD_DELAY:
            b.efforts["gripper"] = load
            b.positions["gripper"] -= slip_per_check
        else:
            b.positions["gripper"] = max(b.positions["gripper"], STALL)

    be.on_sleep = hold_check
    return be


def test_grasp_reports_the_holding_load(tmp_path: Path) -> None:
    arm, _ = make(tmp_path, holding(180.0))
    res = arm.set_gripper(close_until_effort=True, grip_profile="gentle")
    assert res.holding_load == pytest.approx(180.0)
    assert res.slipping is False
    assert res.crush_risk is False


def test_grasp_reports_slipping_when_the_jaw_keeps_closing(tmp_path: Path) -> None:
    arm, _ = make(tmp_path, holding(100.0, slip_per_check=0.02))
    res = arm.set_gripper(close_until_effort=True, grip_profile="normal")
    assert res.status == "grasped"
    assert res.slipping is True


def test_grasp_reports_crush_risk_above_the_profile_limit(tmp_path: Path) -> None:
    arm, _ = make(tmp_path, holding(GENTLE.crush_load + 10.0))
    res = arm.set_gripper(close_until_effort=True, grip_profile="gentle")
    assert res.crush_risk is True
    arm, _ = make(tmp_path, holding(GENTLE.crush_load + 10.0))
    res = arm.set_gripper(close_until_effort=True, grip_profile="firm")  # firm tolerates that load
    assert res.crush_risk is False


def test_torque_limit_read_back(tmp_path: Path) -> None:
    arm, be = make(tmp_path, stalling())
    res = arm.set_gripper(close_until_effort=True, grip_profile="gentle")
    assert res.torque_limit_readback is not None and res.torque_limit_readback.startswith("unverified")
    arm, be = make(tmp_path, stalling())
    be.echo_registers = True
    res = arm.set_gripper(close_until_effort=True, grip_profile="gentle")
    assert res.torque_limit_readback == "verified"
    # A dump older than the write does not count; a newer one with another value is a mismatch.
    be.registers[("gripper", "torque_limit")] = Stamped(value=999, stamp=be.t - 100.0)
    assert arm.torque_limit_readback().startswith("unverified")
    be.registers[("gripper", "torque_limit")] = Stamped(value=999, stamp=be.t + 1.0)
    assert arm.torque_limit_readback().startswith("mismatch")


# --- stall classification by position (default gripper_empty_stall_rad 0.08) -----------------------------------


def make_default(tmp_path: Path, be: FakeArmBackend) -> ArmController:
    cfg = CONFIG.model_copy(deep=True)
    cfg.arm.home_file = tmp_path / "home.yaml"
    cfg.grip_profiles.hold_check_delay_s = HOLD_DELAY
    return ArmController(be, KIN, load_joint_limits(cfg.arm.urdf_path), cfg)


def loaded_stall(stop: float, load: float, start: float = 0.531) -> FakeArmBackend:
    """Follower whose jaw cannot close beyond `stop` and then reports `load` (below the normal target_load)."""
    be = FakeArmBackend()
    be.positions["gripper"] = start

    def blocked(b: FakeArmBackend) -> None:
        if b.positions["gripper"] <= stop + 1e-9:
            b.positions["gripper"] = stop
            b.efforts["gripper"] = load

    be.on_sleep = blocked
    return be


def test_object_stall_far_from_closed_below_target_load_is_grasped(tmp_path: Path) -> None:
    normal = CONFIG.grip_profiles.presets["normal"]
    assert normal.target_load is not None and 150.0 < normal.target_load < normal.contact_effort_threshold
    be = loaded_stall(0.29, 150.0)
    arm = make_default(tmp_path, be)
    res = arm.set_gripper(close_until_effort=True, grip_profile="normal")
    assert res.status == "grasped", res.message
    assert "stalled" in res.message
    assert be.commands[-1]["gripper"] == pytest.approx(0.29 - normal.squeeze_rad)
    assert torque_writes(be) == [normal.torque_limit]  # kept while holding
    assert res.holding_load == 150.0


def test_object_stall_just_outside_the_empty_band_is_grasped(tmp_path: Path) -> None:
    be = loaded_stall(-0.04, 150.0)  # 0.125 rad before closed: beyond the 0.08 settle tolerance
    res = make_default(tmp_path, be).set_gripper(close_until_effort=True, grip_profile="normal")
    assert res.status == "grasped", res.message
    assert be.commands[-1]["gripper"] == pytest.approx(-0.04 - CONFIG.grip_profiles.presets["normal"].squeeze_rad)


def test_stall_inside_the_empty_band_stays_closed_no_contact(tmp_path: Path) -> None:
    be = loaded_stall(-0.115, 140.0)  # 0.05 rad before closed
    res = make_default(tmp_path, be).set_gripper(close_until_effort=True, grip_profile="normal")
    assert res.status == "closed_no_contact", res.message
    assert "gripper_empty_stall_rad" in res.message


def test_stall_after_too_little_travel_is_blocked(tmp_path: Path) -> None:
    be = loaded_stall(0.47, 150.0)  # 0.06 rad travel < gripper_grasp_min_travel_rad: pressing, not holding
    res = make_default(tmp_path, be).set_gripper(close_until_effort=True, grip_profile="normal")
    assert res.status == "blocked", res.message
    assert be.commands[-1]["gripper"] == pytest.approx(0.47)


def test_still_jaw_with_the_command_at_the_close_target_is_stalled(tmp_path: Path) -> None:
    """Less than gripper_stall_lead_rad before closed the lead can never grow: a command at the target counts."""
    arm = make_default(tmp_path, FakeArmBackend())
    closed = CONFIG.arm.gripper_closed_rad
    at_target = [(100.0 + 0.1 * i, -0.04, closed) for i in range(8)]  # lead 0.125 < 0.15
    assert arm.jaw_stalled(at_target) is False
    assert arm.jaw_stalled(at_target, target=closed) is True
    ramping = [(100.0 + 0.1 * i, -0.04, -0.1 - 0.01 * i) for i in range(8)]  # command still on its way
    assert arm.jaw_stalled(ramping, target=closed) is False
    assert arm.stall_contact_message(at_target, started=99.0, target=closed) is not None
    in_band = [(100.0 + 0.1 * i, -0.115, closed) for i in range(8)]
    assert arm.stall_contact_message(in_band, started=99.0, target=closed) is None


# --- hold re-tighten: a soft object relaxes under the hold (2026-10-11 12:18 plush croc head) -------------------


def relaxing(stop: float, close_load: float, hold_load: float, hold_jaw: float | None = None) -> FakeArmBackend:
    """Jaw blocked at `stop` (load `close_load`) whose hold check reads `hold_load`; hold_jaw moves the jaw there."""
    be = FakeArmBackend()
    be.positions["gripper"] = 0.531

    def step(b: FakeArmBackend) -> None:
        if b.last_sleep == HOLD_DELAY:
            b.efforts["gripper"] = hold_load
            b.positions["gripper"] = max(b.positions["gripper"], stop) if hold_jaw is None else hold_jaw
        elif b.positions["gripper"] <= stop + 1e-9:
            b.positions["gripper"] = stop
            b.efforts["gripper"] = close_load

    be.on_sleep = step
    return be


def gripper_holds_after(be: FakeArmBackend, jaw: float) -> list[float]:
    """Gripper targets commanded after the first command that holds the jaw at `jaw` (the grasp hold)."""
    targets = [c["gripper"] for c in be.commands]
    first = next(i for i, t in enumerate(targets) if t == pytest.approx(jaw))
    return [t for t in targets[first + 1 :] if t != pytest.approx(jaw)]


def test_a_relaxed_stall_hold_is_retightened_once_up_to_the_squeeze_cap(tmp_path: Path) -> None:
    normal = CONFIG.grip_profiles.presets["normal"]
    cap = CONFIG.grip_profiles.squeeze_max_rad
    be = relaxing(0.29, 150.0, 0.0)
    res = make_default(tmp_path, be).set_gripper(close_until_effort=True, grip_profile="normal")
    assert res.status == "grasped", res.message
    assert res.retightened is True
    # One step from the profile squeeze to the cap: 0.08 rad from the stall in total, never more.
    assert gripper_holds_after(be, 0.29 - normal.squeeze_rad) == [pytest.approx(0.29 - cap)]
    assert be.commands[-1]["gripper"] == pytest.approx(0.29 - cap)
    assert torque_writes(be) == [normal.torque_limit]  # the torque limit is never raised by the re-tighten
    assert res.holding_load == 0.0  # measured again after the re-tighten


def test_a_relaxed_target_load_hold_is_retightened_like_the_plush_croc(tmp_path: Path) -> None:
    """normal reached target_load (204 at 0.35 rad on the croc head) and held the measured jaw; the hold read 0."""
    be = FakeArmBackend()
    be.positions["gripper"] = 1.0
    contact: list[float] = []

    def step(b: FakeArmBackend) -> None:
        if b.last_sleep == HOLD_DELAY:
            if not contact:
                contact.append(b.positions["gripper"])
            b.efforts["gripper"] = 0.0
            b.positions["gripper"] = max(b.positions["gripper"], contact[0] - 0.005)  # plush gives a little
        else:
            b.efforts["gripper"] = max(0.0, (0.6 - b.positions["gripper"]) * 1000.0)

    be.on_sleep = step
    res = make_default(tmp_path, be).set_gripper(close_until_effort=True, grip_profile="normal")
    assert res.status == "grasped" and "target_load" in res.message, res.message
    jaw = res.positions["gripper"]
    assert res.retightened is True
    assert gripper_holds_after(be, jaw) == [pytest.approx(jaw - CONFIG.grip_profiles.squeeze_max_rad)]


def test_the_retighten_never_goes_past_closed(tmp_path: Path) -> None:
    arm, be = make(tmp_path, relaxing(STALL, 0.0, 0.0))  # STALL 0.075 rad before closed (empty band lowered)
    res = arm.set_gripper(close_until_effort=True, grip_profile="normal")
    assert res.status == "grasped" and res.retightened is True
    assert be.commands[-1]["gripper"] == pytest.approx(CONFIG.arm.gripper_closed_rad)


def test_soft_profile_squeezes_at_the_cap_so_nothing_is_left_to_retighten(tmp_path: Path) -> None:
    soft = CONFIG.grip_profiles.presets["soft"]
    be = relaxing(0.29, 100.0, 0.0)
    res = make_default(tmp_path, be).set_gripper(close_until_effort=True, grip_profile="soft")
    assert res.status == "grasped", res.message
    assert be.commands[-1]["gripper"] == pytest.approx(0.29 - soft.squeeze_rad)
    assert res.retightened is False
    assert torque_writes(be) == [soft.torque_limit]


def test_a_healthy_holding_load_is_not_retightened(tmp_path: Path) -> None:
    normal = CONFIG.grip_profiles.presets["normal"]
    be = relaxing(0.29, 150.0, 150.0)
    res = make_default(tmp_path, be).set_gripper(close_until_effort=True, grip_profile="normal")
    assert res.retightened is False
    assert be.commands[-1]["gripper"] == pytest.approx(0.29 - normal.squeeze_rad)
    assert gripper_holds_after(be, 0.29 - normal.squeeze_rad) == []


def test_a_jaw_that_sank_into_the_empty_band_is_not_retightened(tmp_path: Path) -> None:
    normal = CONFIG.grip_profiles.presets["normal"]
    be = relaxing(-0.04, 150.0, 0.0, hold_jaw=-0.12)  # squashed to 0.045 rad before closed during the hold
    res = make_default(tmp_path, be).set_gripper(close_until_effort=True, grip_profile="normal")
    assert res.status == "grasped", res.message
    assert res.retightened is False
    assert be.commands[-1]["gripper"] == pytest.approx(-0.04 - normal.squeeze_rad)


def test_retighten_can_be_disabled(tmp_path: Path) -> None:
    be = relaxing(0.29, 150.0, 0.0)
    arm = make_default(tmp_path, be)
    arm.cfg.grip_profiles.retighten_load_below = None
    res = arm.set_gripper(close_until_effort=True, grip_profile="normal")
    assert res.retightened is False
    assert be.commands[-1]["gripper"] == pytest.approx(0.29 - CONFIG.grip_profiles.presets["normal"].squeeze_rad)


def test_empty_band_close_reports_no_retighten(tmp_path: Path) -> None:
    be = loaded_stall(-0.115, 140.0)
    res = make_default(tmp_path, be).set_gripper(close_until_effort=True, grip_profile="normal")
    assert res.status == "closed_no_contact", res.message
    assert res.retightened is None and res.holding_load is None
