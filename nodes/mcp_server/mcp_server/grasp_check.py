"""Grasp verification fusion: the sensor band of a close (load delta, jaw gap, current) and its fusion with the visual
verdict the agent records through confirm_grasp.

The image never decides on its own (visual judges are weakest on contact-only outcomes, FailBench arXiv 2609.03611):
only a sensor band 'object' plus image 'yes' is 'grasped', only 'empty' plus 'no' is 'missed', everything else is
'unverified'.
"""

import math
from typing import Literal

from pydantic import BaseModel

Band = Literal["object", "empty", "ambiguous"]
ImageVerdict = Literal["yes", "no", "unsure"]
ImageState = Literal["pending", "not_attached", "not_requested", "yes", "no", "unsure"]
FusedOutcome = Literal["grasped", "unverified", "missed"]
ContactKind = Literal["load_delta", "absolute", "target_load", "stall"]
BaselineSource = Literal["free_closing", "rest"]

FUSE_TABLE: dict[tuple[Band, ImageVerdict], FusedOutcome] = {
    ("object", "yes"): "grasped",
    ("object", "no"): "unverified",
    ("object", "unsure"): "unverified",
    ("ambiguous", "yes"): "unverified",
    ("ambiguous", "no"): "unverified",
    ("ambiguous", "unsure"): "unverified",
    ("empty", "yes"): "unverified",
    ("empty", "no"): "missed",
    ("empty", "unsure"): "unverified",
}


class GripMeasurements(BaseModel):
    """What a close_until_effort measured, before the band verdict (loads in servo units, currents in A).

    None means unmeasured (no finite sample), never 0.
    """

    grasp_id: str  # uuid4 hex[:12], minted per close_until_effort
    baseline_load: float | None  # closing-direction load baseline
    baseline_source: BaselineSource | None
    contact: ContactKind | None  # which contact signal ended the close; None = no contact
    peak_load_delta: float | None  # max closing load - baseline during the close
    hold_load_delta: float | None  # holding_load (hold_report) * closing_load_sign - baseline
    rest_current_a: float | None
    hold_current_a: float | None  # mean gripper current over the hold check samples
    current_delta_a: float | None
    jaw_gap_rad: float | None  # measured jaw - gripper_closed_rad at the hold
    in_empty_band: bool | None  # jaw_gap_rad < limits.gripper_empty_stall_rad


class GripEvidence(GripMeasurements):
    """GripMeasurements plus the band_verdict of them."""

    band: Band


class CropInfo(BaseModel):
    """Jaw crop of the attached head (D435 colour) image, taken with the head aimed at the FK tool point."""

    lookup_id: str | None
    box_px: tuple[int, int, int, int] | None  # u0, v0, u1, v1 in the source frame; None = full frame
    full_frame_reason: str | None


class GraspCheck(BaseModel):
    """Grasp verification state of one close: sensor band, visual verdict and their fusion."""

    grasp_id: str
    band: Band
    evidence: GripEvidence
    image: ImageState
    crop: CropInfo | None
    fused: FusedOutcome | None  # None until confirm_grasp


def finite(value: float | None) -> float | None:
    """A measured value, or None when it is unset or not finite (NaN / inf: unmeasured).

    Args:
        value (float | None): Value.

    Returns:
        float | None: The finite value or None.
    """
    return value if value is not None and math.isfinite(value) else None


def loads_vote(
    m: GripMeasurements, contact_load_delta: float | None, holding_load: float | None, hold_effort_min: float
) -> bool | None:
    """Whether the holding load says an object is between the jaws.

    Args:
        m (GripMeasurements): Measurements (hold_load_delta).
        contact_load_delta (float | None): Profile delta; None = use hold_effort_min on |holding_load|.
        holding_load (float | None): Raw holding load of the hold report (the fallback input).
        hold_effort_min (float): grasp.hold_effort_min (the fallback threshold).

    Returns:
        bool | None: True / False, or None when the load it needs is unmeasured.
    """
    if contact_load_delta is not None:
        delta = finite(m.hold_load_delta)
        return None if delta is None else delta >= contact_load_delta
    load = finite(holding_load)
    return None if load is None else abs(load) >= hold_effort_min


def current_vote(m: GripMeasurements, current_contact_delta_a: float | None) -> bool | None:
    """Whether the gripper current rise says an object is between the jaws.

    Args:
        m (GripMeasurements): Measurements (current_delta_a).
        current_contact_delta_a (float | None): grip_profiles.current_contact_delta_a; None = current does not vote.

    Returns:
        bool | None: None when either value is unset (or the current is not finite), else the comparison.
    """
    delta = finite(m.current_delta_a)
    if delta is None or current_contact_delta_a is None:
        return None
    return delta >= current_contact_delta_a


def band_verdict(
    m: GripMeasurements,
    contact_load_delta: float | None,
    current_contact_delta_a: float | None,
    holding_load: float | None,
    hold_effort_min: float,
) -> Band:
    """Sensor band of a close.

    object: outside the empty band, a contact fired, the loads say object and the current does not vote against.
    empty: inside the empty band, the loads say no object and the current does not vote for one.
    Anything else, including an unmeasured load or an unknown empty band, is ambiguous.

    Args:
        m (GripMeasurements): Measurements (a GripEvidence works too; its stored band is ignored).
        contact_load_delta (float | None): The profile's contact_load_delta; None = hold_effort_min fallback.
        current_contact_delta_a (float | None): grip_profiles.current_contact_delta_a; None = current does not vote.
        holding_load (float | None): Raw holding load of the hold report (used only by the fallback).
        hold_effort_min (float): grasp.hold_effort_min on |holding_load| (used only by the fallback).

    Returns:
        Band: object, empty or ambiguous.
    """
    loads = loads_vote(m, contact_load_delta, holding_load, hold_effort_min)
    current = current_vote(m, current_contact_delta_a)
    if m.in_empty_band is False and m.contact is not None and loads is True and current is not False:
        return "object"
    if m.in_empty_band is True and loads is False and current is not True:
        return "empty"
    return "ambiguous"


def grip_evidence(
    m: GripMeasurements,
    contact_load_delta: float | None,
    current_contact_delta_a: float | None,
    holding_load: float | None,
    hold_effort_min: float,
) -> GripEvidence:
    """GripEvidence of the measurements with their band_verdict.

    Args:
        m (GripMeasurements): Measurements.
        contact_load_delta (float | None): See band_verdict.
        current_contact_delta_a (float | None): See band_verdict.
        holding_load (float | None): See band_verdict.
        hold_effort_min (float): See band_verdict.

    Returns:
        GripEvidence: The measurements plus their band.
    """
    band = band_verdict(m, contact_load_delta, current_contact_delta_a, holding_load, hold_effort_min)
    return GripEvidence(**m.model_dump(), band=band)


def fuse(band: Band, image: ImageVerdict) -> FusedOutcome:
    """Fuse the sensor band with the visual verdict (FUSE_TABLE).

    Args:
        band (Band): Sensor band.
        image (ImageVerdict): Agent's verdict on the jaw crop.

    Returns:
        FusedOutcome: grasped only for object + yes, missed only for empty + no, else unverified.

    Raises:
        ValueError: For a value outside the literals.
    """
    try:
        return FUSE_TABLE[(band, image)]
    except KeyError:
        raise ValueError(f"cannot fuse band {band!r} with image {image!r}") from None
