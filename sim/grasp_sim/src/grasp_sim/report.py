"""SimReport: the structured result of a plan replay."""

from pydantic import BaseModel, Field


class ClearanceStats(BaseModel):
    """Minimum clearance (m, negative = penetration) of the jaws/wrist to the floor and the support."""

    jaws_floor: float | None = None
    jaws_support: float | None = None
    wrist_floor: float | None = None
    wrist_support: float | None = None


class ContactEvent(BaseModel):
    """One contact between two geoms, with the plan time and label it happened in."""

    t: float
    label: str | None
    kind: str
    geom_a: str
    geom_b: str


class SegmentReport(BaseModel):
    """Per-label statistics (unlabelled samples are reported as label null)."""

    label: str | None
    t_start: float
    t_end: float
    min_clearance: ClearanceStats
    max_actuator_force_nm: float
    max_object_tilt_deg: float | None = None
    max_object_displacement_m: float | None = None


class ObjectReport(BaseModel):
    """What happened to the object (positions in the arm frame, m)."""

    start_pos: tuple[float, float, float]
    final_pos: tuple[float, float, float]
    approach_max_tilt_deg: float
    approach_max_displacement_m: float
    tipped: bool
    pushed: bool
    lift_height_m: float
    lifted_after_lift: bool | None
    lifted_at_end: bool


class SimReport(BaseModel):
    """Replay result. passed is True only when reasons is empty."""

    passed: bool
    reasons: list[str]
    warnings: list[str] = Field(default_factory=list)
    duration_s: float
    segments: list[SegmentReport]
    first_unintended_contact: ContactEvent | None = None
    event_counts: dict[str, int] = Field(default_factory=dict)
    object: ObjectReport | None = None
    grasp_success: bool | None = None
    max_actuator_force_nm: dict[str, float]
    saturated_actuators: list[str] = Field(default_factory=list)
