"""The head as a posed body part: head.urdf forward kinematics at the measured pan/tilt, the full envelope otherwise."""

import math

import numpy as np
import pytest

from arm_collision.head import HEAD_OBSTACLE, HeadModel, HeadPose, head_pose, posed_body
from arm_collision.settings import load_robot_body

HEAD = HeadModel.from_urdf()
TOLERANCE_M = 1e-9
# Envelope samples: pan over the URDF range, tilt over the URDF range (head.urdf head_pan, head_tilt limits).
PAN_SAMPLES = np.linspace(-1.5708, 1.5708, 13)
TILT_SAMPLES = np.linspace(-0.5236, 0.7854, 7)


def part_bounds(pose: HeadPose) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """Lower and upper corner of every posed head part (cylinders by their bounding box).

    Args:
        pose (HeadPose): Head pose.

    Returns:
        dict[str, tuple[np.ndarray, np.ndarray]]: Part name -> (lo, hi) in base_link (m).
    """
    boxes, cylinders = HEAD.parts(pose)
    bounds = {}
    for box in boxes:
        centre, half = np.array(box.center), np.array(box.size) / 2.0
        bounds[box.name] = (centre - half, centre + half)
    for c in cylinders:
        bounds[c.name] = (
            np.array([c.x - c.radius, c.y - c.radius, c.z_min]),
            np.array([c.x + c.radius, c.y + c.radius, c.z_max]),
        )
    return bounds


def test_straight_ahead_and_level_parts_are_the_urdf_visuals() -> None:
    """Pan 0, tilt 0: the pan base and pan link stay vertical cylinders, the 40 x 90 x 40 mm bracket and the
    90 x 25 x 25 mm D435 of head.urdf are boxes, all rounded outwards to 1 mm."""
    boxes, cylinders = HEAD.parts(HeadPose(pan=0.0, tilt=0.0))
    assert [(c.name, c.x, c.y, c.radius, c.z_min, c.z_max) for c in cylinders] == [
        ("head_pan_base", 0.15, 0.10, 0.025, 0.29, 0.31),
        ("head_pan_link", 0.15, 0.10, 0.02, 0.31, 0.35),
    ]
    bounds = part_bounds(HeadPose(pan=0.0, tilt=0.0))
    np.testing.assert_allclose(bounds["head_link"][0], [0.14, 0.055, 0.33], atol=1e-6)
    np.testing.assert_allclose(bounds["head_link"][1], [0.18, 0.145, 0.37], atol=1e-6)
    np.testing.assert_allclose(bounds["d435_link"][0], [0.125, 0.087, 0.337], atol=1e-6)
    np.testing.assert_allclose(bounds["d435_link"][1], [0.215, 0.113, 0.363], atol=1e-6)
    assert [b.name for b in boxes] == ["head_link", "d435_link"]


def test_looking_left_turns_the_d435_to_plus_y() -> None:
    """Pan +90 deg (looking left): the D435 sticks out to +y, 90 mm along y and 25 mm along x."""
    lo, hi = part_bounds(HeadPose(pan=math.pi / 2, tilt=0.0))["d435_link"]
    assert hi[1] == pytest.approx(0.10 + 0.065, abs=1e-6)
    assert lo[1] == pytest.approx(0.10 - 0.025, abs=1e-6)
    assert hi[0] - lo[0] == pytest.approx(0.026, abs=1e-6)


@pytest.mark.parametrize("pan", PAN_SAMPLES)
@pytest.mark.parametrize("tilt", TILT_SAMPLES)
def test_posed_parts_lie_inside_the_envelope_heights(pan: float, tilt: float) -> None:
    """Every posed head part stays within the z range of the body's full head envelope cylinder (rounded to 1 mm)."""
    envelope = next(c for c in load_robot_body().cylinders if c.name == HEAD_OBSTACLE)
    for lo, hi in part_bounds(HeadPose(pan=float(pan), tilt=float(tilt))).values():
        assert lo[2] >= envelope.z_min - TOLERANCE_M
        assert hi[2] <= envelope.z_max + 0.001 + TOLERANCE_M


def test_posed_body_replaces_the_envelope_and_keeps_every_other_obstacle() -> None:
    """posed_body swaps the head cylinder for the posed parts; no head pose keeps the full envelope."""
    body = load_robot_body()
    boxes, cylinders = HEAD.parts(HeadPose(pan=0.2, tilt=0.5))
    posed = posed_body(body, (boxes, cylinders))
    assert posed.boxes == [*body.boxes, *boxes]
    assert posed.cylinders == [*(c for c in body.cylinders if c.name != HEAD_OBSTACLE), *cylinders]
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
