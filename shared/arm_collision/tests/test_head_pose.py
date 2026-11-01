"""The head as a posed body part: head.urdf forward kinematics at the measured pan/tilt, the full envelope otherwise."""

import math

import numpy as np
import pytest

from arm_collision.head import HEAD_OBSTACLE, HeadModel, HeadPose, head_pose, posed_body
from arm_collision.settings import BodyBox, load_robot_body

HEAD = HeadModel.from_urdf()
TOLERANCE_M = 1e-9
# Envelope samples: pan over the URDF range, tilt over the URDF range (head.urdf head_pan, head_tilt limits).
PAN_SAMPLES = np.linspace(-1.5708, 1.5708, 13)
TILT_SAMPLES = np.linspace(-0.5236, 0.7854, 7)


def box_bounds(box: BodyBox) -> tuple[np.ndarray, np.ndarray]:
    """Lower and upper corner of a body box.

    Args:
        box (BodyBox): Box.

    Returns:
        tuple[np.ndarray, np.ndarray]: (lo, hi) in base_link (m).
    """
    centre, half = np.array(box.center), np.array(box.size) / 2.0
    return centre - half, centre + half


def test_straight_ahead_and_level_head_box_is_the_urdf_bracket_and_d435() -> None:
    """Pan 0, tilt 0: the pan base, pan link, 40 x 90 x 40 mm bracket and 90 x 25 x 25 mm D435 of head.urdf, rounded
    outwards to 1 mm: x 0.125-0.215 (D435 front), y 0.055-0.145 (bracket), z 0.29 (pan base) to 0.37 (bracket top)."""
    lo, hi = box_bounds(HEAD.box(HeadPose(pan=0.0, tilt=0.0)))
    np.testing.assert_allclose(lo, [0.125, 0.055, 0.29], atol=1e-6)
    np.testing.assert_allclose(hi, [0.215, 0.145, 0.37], atol=1e-6)


def test_looking_left_turns_the_d435_to_plus_y() -> None:
    """Pan +90 deg (looking left): the D435 sticks out to +y, the box is 90 mm deep along y and narrow along x."""
    lo, hi = box_bounds(HEAD.box(HeadPose(pan=math.pi / 2, tilt=0.0)))
    assert hi[1] == pytest.approx(0.10 + 0.065, abs=1e-6)
    assert lo[0] == pytest.approx(0.15 - 0.045, abs=1e-6)
    assert hi[0] == pytest.approx(0.15 + 0.045, abs=1e-6)


@pytest.mark.parametrize("pan", PAN_SAMPLES)
@pytest.mark.parametrize("tilt", TILT_SAMPLES)
def test_posed_box_lies_inside_the_envelope_heights(pan: float, tilt: float) -> None:
    """Every posed head box stays within the z range of the body's full head envelope cylinder (rounded to 1 mm)."""
    envelope = next(c for c in load_robot_body().cylinders if c.name == HEAD_OBSTACLE)
    lo, hi = box_bounds(HEAD.box(HeadPose(pan=float(pan), tilt=float(tilt))))
    assert lo[2] >= envelope.z_min - TOLERANCE_M
    assert hi[2] <= envelope.z_max + 0.001 + TOLERANCE_M


def test_posed_body_replaces_the_envelope_and_keeps_every_other_obstacle() -> None:
    """posed_body swaps the head cylinder for the posed box named head; no head pose keeps the full envelope."""
    body = load_robot_body()
    box = HEAD.box(HeadPose(pan=0.2, tilt=0.5))
    posed = posed_body(body, box)
    assert [c.name for c in posed.cylinders] == [c.name for c in body.cylinders if c.name != HEAD_OBSTACLE]
    assert posed.boxes == [*body.boxes, box]
    assert posed_body(body, None) == body


@pytest.mark.parametrize(
    ("angles", "expected"),
    [
        (None, None),
        ({"head_pan": 0.1}, None),
        ({"head_pan": 0.1, "head_tilt": 0.4}, HeadPose(pan=0.1, tilt=0.4)),
        ({"head_pan": 0.1, "head_tilt": math.nan}, None),
    ],
)
def test_head_pose_needs_both_finite_angles(angles: dict[str, float] | None, expected: HeadPose | None) -> None:
    """Missing, partial or non-finite head angles give no pose (the checks then use the full envelope)."""
    assert head_pose(angles) == expected
