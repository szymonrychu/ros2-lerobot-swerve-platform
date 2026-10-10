"""Tests for per-object grip profiles: config (grip_profiles section, caps) and profile resolution (mcp_server.grip)."""

import pytest
from pydantic import ValidationError

from mcp_server.config import GripProfile, McpServerConfig
from mcp_server.grip import GripProfileOverride, resolve_grip_profile


def test_default_presets_gentle_normal_firm() -> None:
    grip = McpServerConfig().grip_profiles
    assert grip.default_grip_profile == "normal"
    assert set(grip.presets) == {"gentle", "normal", "firm"}
    gentle, normal, firm = grip.presets["gentle"], grip.presets["normal"], grip.presets["firm"]
    assert gentle.squeeze_rad == pytest.approx(0.02)
    assert gentle.torque_limit == 250
    assert gentle.close_speed_rps < normal.close_speed_rps
    # normal = the behaviour before grip profiles: 0.03 rad squeeze, 0.5 rad/s close, 300 contact load.
    assert normal.squeeze_rad == pytest.approx(0.03)
    assert normal.close_speed_rps == pytest.approx(0.5)
    assert normal.contact_effort_threshold == pytest.approx(300.0)
    assert normal.target_load is None
    assert firm.squeeze_rad > normal.squeeze_rad
    assert firm.torque_limit == 700
    assert gentle.torque_limit < normal.torque_limit < firm.torque_limit
    assert grip.torque_limit_max == 800
    assert all(p.torque_limit <= grip.torque_limit_max for p in grip.presets.values())
    assert all(p.squeeze_rad <= grip.squeeze_max_rad for p in grip.presets.values())


def test_default_profile_must_be_a_preset() -> None:
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"grip_profiles": {"default_grip_profile": "crushing"}})


def test_preset_above_the_torque_cap_is_rejected() -> None:
    presets = McpServerConfig().grip_profiles.model_dump()["presets"]
    presets["firm"]["torque_limit"] = 900
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"grip_profiles": {"presets": presets}})


def test_preset_above_the_squeeze_cap_is_rejected() -> None:
    presets = McpServerConfig().grip_profiles.model_dump()["presets"]
    presets["firm"]["squeeze_rad"] = 0.15
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"grip_profiles": {"presets": presets, "squeeze_max_rad": 0.1}})


def test_torque_cap_cannot_exceed_the_register_range() -> None:
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"grip_profiles": {"torque_limit_max": 1001}})
    with pytest.raises(ValidationError):
        GripProfile(squeeze_rad=0.02, torque_limit=1200, close_speed_rps=0.3, contact_effort_threshold=100)


def test_resolve_none_gives_the_default_profile() -> None:
    cfg = McpServerConfig()
    grip = resolve_grip_profile(cfg.grip_profiles, cfg.limits, None)
    assert grip.name == "normal"
    assert grip.profile == cfg.grip_profiles.presets["normal"]
    assert grip.capped == []


def test_resolve_named_preset_and_unknown_name() -> None:
    cfg = McpServerConfig()
    assert resolve_grip_profile(cfg.grip_profiles, cfg.limits, "gentle").profile.torque_limit == 250
    with pytest.raises(ValueError, match="unknown grip_profile"):
        resolve_grip_profile(cfg.grip_profiles, cfg.limits, "crushing")


def test_resolve_inline_overrides_start_from_a_base_preset() -> None:
    cfg = McpServerConfig()
    grip = resolve_grip_profile(
        cfg.grip_profiles, cfg.limits, {"base": "gentle", "squeeze_rad": 0.01, "target_load": 90}
    )
    assert grip.name == "gentle+custom"
    assert grip.profile.squeeze_rad == pytest.approx(0.01)
    assert grip.profile.target_load == pytest.approx(90.0)
    assert grip.profile.torque_limit == 250  # from the base
    no_base = resolve_grip_profile(cfg.grip_profiles, cfg.limits, GripProfileOverride(torque_limit=400))
    assert no_base.name == "normal+custom"
    assert no_base.profile.torque_limit == 400


def test_inline_overrides_are_capped_server_side() -> None:
    cfg = McpServerConfig()
    grip = resolve_grip_profile(
        cfg.grip_profiles, cfg.limits, {"torque_limit": 1000, "squeeze_rad": 0.2, "close_speed_rps": 1.5}
    )
    assert grip.profile.torque_limit == cfg.grip_profiles.torque_limit_max
    assert grip.profile.squeeze_rad == pytest.approx(cfg.grip_profiles.squeeze_max_rad)
    assert grip.profile.close_speed_rps == pytest.approx(cfg.limits.gripper_velocity_rps)
    assert sorted(grip.capped) == ["close_speed_rps", "squeeze_rad", "torque_limit"]


def test_inline_override_rejects_unknown_keys_and_bad_values() -> None:
    cfg = McpServerConfig()
    with pytest.raises(ValueError, match="invalid grip_profile"):
        resolve_grip_profile(cfg.grip_profiles, cfg.limits, {"grip": 3})
    with pytest.raises(ValueError, match="invalid grip_profile"):
        resolve_grip_profile(cfg.grip_profiles, cfg.limits, {"torque_limit": -5})
    with pytest.raises(ValueError, match="unknown grip_profile"):
        resolve_grip_profile(cfg.grip_profiles, cfg.limits, {"base": "nope"})


def test_report_lists_the_applied_values() -> None:
    cfg = McpServerConfig()
    report = resolve_grip_profile(cfg.grip_profiles, cfg.limits, "firm").report()
    assert report["name"] == "firm"
    assert report["torque_limit"] == 700
    assert report["capped"] == []
    assert set(report) >= {"squeeze_rad", "close_speed_rps", "target_load", "contact_effort_threshold", "crush_load"}
