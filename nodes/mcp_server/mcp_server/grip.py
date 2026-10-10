"""Per-object grip profiles: resolve a preset name or inline overrides to a capped GripProfile."""

from dataclasses import dataclass, field
from typing import Any

from pydantic import ValidationError

from .config import GripProfile, GripProfileOverride, GripProfileSettings, LimitSettings

CUSTOM_SUFFIX = "+custom"

GripChoice = str | GripProfileOverride | dict[str, Any] | None


@dataclass(frozen=True)
class ResolvedGrip:
    """A grip profile ready to apply: its name, the (capped) values and which values were capped."""

    name: str
    profile: GripProfile
    capped: list[str] = field(default_factory=list)

    def report(self) -> dict[str, Any]:
        """Profile as reported in results.

        Returns:
            dict[str, Any]: {name, squeeze_rad, torque_limit, close_speed_rps, target_load, contact_effort_threshold,
                crush_load, capped}.
        """
        return {"name": self.name, **self.profile.model_dump(), "capped": list(self.capped)}


def preset(settings: GripProfileSettings, name: str) -> GripProfile:
    """Look up a preset.

    Args:
        settings (GripProfileSettings): Configured profiles.
        name (str): Preset name.

    Returns:
        GripProfile: The preset.

    Raises:
        ValueError: For an unknown name.
    """
    if name not in settings.presets:
        raise ValueError(f"unknown grip_profile {name!r}; presets: {', '.join(settings.presets)}")
    return settings.presets[name]


def resolve_grip_profile(settings: GripProfileSettings, limits: LimitSettings, choice: GripChoice) -> ResolvedGrip:
    """Resolve a grip profile choice and enforce the hard caps server-side.

    Args:
        settings (GripProfileSettings): Configured presets and caps.
        limits (LimitSettings): limits.gripper_velocity_rps caps close_speed_rps.
        choice (GripChoice): None (default preset), a preset name, or inline overrides (dict or GripProfileOverride;
            'base' picks the preset they start from).

    Returns:
        ResolvedGrip: Name ('<preset>' or '<base>+custom'), capped profile and the capped field names.

    Raises:
        ValueError: For an unknown preset or invalid overrides.
    """
    if choice is None or isinstance(choice, str):
        name = settings.default_grip_profile if choice is None else choice
        return cap(settings, limits, name, preset(settings, name))
    try:
        override = choice if isinstance(choice, GripProfileOverride) else GripProfileOverride.model_validate(choice)
    except ValidationError as exc:
        raise ValueError(f"invalid grip_profile: {exc}") from exc
    base = override.base or settings.default_grip_profile
    values = override.model_dump(exclude={"base"}, exclude_unset=True)
    profile = preset(settings, base).model_copy(update=values)
    return cap(settings, limits, base + CUSTOM_SUFFIX, profile)


def cap(settings: GripProfileSettings, limits: LimitSettings, name: str, profile: GripProfile) -> ResolvedGrip:
    """Clamp torque_limit, squeeze_rad and close_speed_rps to their caps.

    Args:
        settings (GripProfileSettings): torque_limit_max, squeeze_max_rad.
        limits (LimitSettings): gripper_velocity_rps.
        name (str): Profile name.
        profile (GripProfile): Uncapped profile.

    Returns:
        ResolvedGrip: The capped profile.
    """
    caps: dict[str, float] = {
        "torque_limit": settings.torque_limit_max,
        "squeeze_rad": settings.squeeze_max_rad,
        "close_speed_rps": limits.gripper_velocity_rps,
    }
    update: dict[str, Any] = {}
    for key, limit in caps.items():
        if getattr(profile, key) > limit:
            update[key] = int(limit) if key == "torque_limit" else limit
    return ResolvedGrip(name=name, profile=profile.model_copy(update=update), capped=sorted(update))
