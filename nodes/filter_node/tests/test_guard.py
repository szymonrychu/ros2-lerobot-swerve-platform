"""ArmGuard (filter_node.guard): every follower command is checked against the robot body before it is published."""

import math
from types import SimpleNamespace
from typing import Any

import pytest
from arm_collision.head import HeadPose
from arm_collision.self_collision import ESCAPE_TOLERANCE_M
from feetech_mapping.command_mapping import RADIANS_PER_STEP, map_position_to_steps, steps_to_radians

from filter_node.config import LeaderGripperSettings
from filter_node.guard import UNKNOWN_POSE, ArmGuard, follower_to_leader, leader_to_follower

from .arm_fixture import (
    CHECKER,
    DRIVE_POSE,
    FK_TOLERANCE_M,
    HARD_MARGIN_M,
    PARKED_HOME,
    WHEEL_FR_INSIDE,
    WHEEL_FR_INSIDE_TOOL_Z_M,
    WHEEL_FR_PRESSED,
    WHEEL_FR_PRESSED_TOOL_Z_M,
    WHEEL_FR_XY,
    tool_base_link,
)

MAX_STEP_RAD = 0.02
ARM_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
SOURCE = "leader"


def ordered(pose: dict[str, float]) -> list[float]:
    """Positions of a pose in ARM_NAMES order.

    Args:
        pose (dict[str, float]): Joint name -> rad.

    Returns:
        list[float]: Positions.
    """
    return [pose[n] for n in ARM_NAMES]


def real_guard(start: dict[str, float]) -> ArmGuard:
    """Guard on the real checker, seeded with a follower pose.

    Args:
        start (dict[str, float]): Follower feedback pose.

    Returns:
        ArmGuard: The guard.
    """
    guard = ArmGuard(CHECKER, MAX_STEP_RAD, enabled=True)
    guard.seed(ARM_NAMES, ordered(start))
    return guard


class FakeChecker:
    """Records the samples and head poses it is asked about; reports no violation unless told to raise."""

    def __init__(self, error: Exception | None = None) -> None:
        self.paths: list[tuple[list[dict[str, float]], bool]] = []
        self.heads: list[HeadPose | None] = []
        self.error = error

    def posed(self, head: HeadPose | None) -> "FakeChecker":
        self.heads.append(head)
        return self

    def path(self, samples: list[dict[str, float]], allow_escape: bool = True) -> Any:
        if self.error is not None:
            raise self.error
        self.paths.append((samples, allow_escape))
        return SimpleNamespace(violation=None)


@pytest.mark.parametrize(
    ("pose", "tool_z"),
    [(WHEEL_FR_PRESSED, WHEEL_FR_PRESSED_TOOL_Z_M), (WHEEL_FR_INSIDE, WHEEL_FR_INSIDE_TOOL_Z_M)],
    ids=["pressed", "inside"],
)
def test_the_wheel_poses_put_the_tool_over_wheel_fr(pose: dict[str, float], tool_z: float) -> None:
    """FK check of the fixture poses: the tool point is on the wheel_fr steer axis at the stated height."""
    assert tool_base_link(pose) == pytest.approx([*WHEEL_FR_XY, tool_z], abs=FK_TOLERANCE_M)
    assert CHECKER.pose(pose).obstacle == "wheel_fr"


def test_the_wheel_incident_stops_at_the_last_sample_before_contact() -> None:
    """From the parked home, a request pressing the gripper 2 cm into wheel_fr publishes the last safe sample (the
    side frame over the wheel is the first part of the body the arm reaches on the way)."""
    request = WHEEL_FR_PRESSED
    guard = real_guard(PARKED_HOME)
    result = guard.admit(SOURCE, ARM_NAMES, ordered(request))
    assert result.blocked and result.obstacle == "frame_right"
    assert result.clearance_m < HARD_MARGIN_M
    assert result.positions is not None and result.positions != ordered(request)
    clamp = dict(zip(ARM_NAMES, result.positions, strict=True))
    assert CHECKER.pose(clamp).min_clearance >= HARD_MARGIN_M
    # The clamp lies on the joint-space segment and the next sample along it breaks the margin.
    span = max(abs(request[n] - PARKED_HOME[n]) for n in ARM_NAMES)
    steps = math.ceil(span / MAX_STEP_RAD)
    k = round(
        (clamp["shoulder_lift"] - PARKED_HOME["shoulder_lift"])
        / (request["shoulder_lift"] - PARKED_HOME["shoulder_lift"])
        * steps
    )
    assert 0 < k < steps
    on_segment = {n: PARKED_HOME[n] + (request[n] - PARKED_HOME[n]) * k / steps for n in ARM_NAMES}
    assert result.positions == pytest.approx(ordered(on_segment), abs=1e-9)
    following = {n: PARKED_HOME[n] + (request[n] - PARKED_HOME[n]) * (k + 1) / steps for n in ARM_NAMES}
    assert CHECKER.pose(following).min_clearance < HARD_MARGIN_M
    assert result.check_seconds > 0.0


def test_a_blocked_request_from_the_clamp_publishes_nothing() -> None:
    """The next request into the wheel starts at the published clamp: its clamp is the start: nothing goes out."""
    request = WHEEL_FR_PRESSED
    guard = real_guard(PARKED_HOME)
    first = guard.admit(SOURCE, ARM_NAMES, ordered(request))
    assert first.positions is not None
    second = guard.admit(SOURCE, ARM_NAMES, ordered(request))
    assert second.blocked and second.positions is None


def test_home_to_drive_pose_publishes_the_request() -> None:
    """The parked home to the drive pose stays clear of the body: the request goes out unchanged."""
    result = real_guard(PARKED_HOME).admit(SOURCE, ARM_NAMES, ordered(DRIVE_POSE))
    assert not result.blocked
    assert result.positions == ordered(DRIVE_POSE)


def test_a_start_inside_the_margin_may_move_away_but_not_deeper() -> None:
    """Gripper 1 cm above the wheel top (inside the 3 cm wheel envelope): back home is published, deeper is not."""
    start = WHEEL_FR_INSIDE
    start_clearance = CHECKER.pose(start).per_obstacle["wheel_fr"][0]
    assert start_clearance < HARD_MARGIN_M
    escape = real_guard(start).admit(SOURCE, ARM_NAMES, ordered(PARKED_HOME))
    assert not escape.blocked and escape.positions == ordered(PARKED_HOME)
    deeper = WHEEL_FR_PRESSED
    result = real_guard(start).admit(SOURCE, ARM_NAMES, ordered(deeper))
    assert result.blocked and result.obstacle == "wheel_fr"
    if result.positions is not None:
        clamp = dict(zip(ARM_NAMES, result.positions, strict=True))
        assert CHECKER.pose(clamp).per_obstacle["wheel_fr"][0] >= start_clearance - ESCAPE_TOLERANCE_M
        assert result.positions != ordered(deeper)


def test_a_request_missing_joints_takes_them_from_the_last_published_pose() -> None:
    """A gripper-only request is checked with the arm joints of the last published command."""
    checker = FakeChecker()
    guard = ArmGuard(checker, MAX_STEP_RAD, enabled=True)
    guard.seed(ARM_NAMES, ordered(PARKED_HOME))
    guard.admit(SOURCE, ARM_NAMES, ordered(DRIVE_POSE))
    result = guard.admit(SOURCE, ["gripper"], [0.3])
    assert result.positions == [0.3]
    samples, allow_escape = checker.paths[-1]
    assert allow_escape
    assert samples[0] == DRIVE_POSE
    assert samples[-1] == {**DRIVE_POSE, "gripper": 0.3}


def test_follower_feedback_replaces_the_start_pose() -> None:
    """The newest follower feedback is where the arm is: the next segment starts there."""
    checker = FakeChecker()
    guard = ArmGuard(checker, MAX_STEP_RAD, enabled=True)
    guard.seed(ARM_NAMES, ordered(PARKED_HOME))
    guard.admit(SOURCE, ARM_NAMES, ordered(DRIVE_POSE))
    guard.seed(ARM_NAMES, ordered(PARKED_HOME))
    guard.admit(SOURCE, ARM_NAMES, ordered(DRIVE_POSE))
    assert checker.paths[-1][0][0] == PARKED_HOME


def test_without_a_prior_pose_only_the_end_pose_is_checked() -> None:
    """No published command and no feedback yet: the request alone is checked, held to the hard margin."""
    checker = FakeChecker()
    result = ArmGuard(checker, MAX_STEP_RAD, enabled=True).admit(SOURCE, ARM_NAMES, ordered(DRIVE_POSE))
    assert result.positions == ordered(DRIVE_POSE)
    assert checker.paths == [([DRIVE_POSE], False)]


def test_without_a_prior_pose_a_request_inside_the_body_publishes_nothing() -> None:
    """No prior pose: a request pressing into the wheel is refused outright (there is no segment to clamp on)."""
    request = WHEEL_FR_PRESSED
    result = ArmGuard(CHECKER, MAX_STEP_RAD, enabled=True).admit(SOURCE, ARM_NAMES, ordered(request))
    assert result.blocked and result.positions is None and result.obstacle == "wheel_fr"


def test_a_partial_request_without_a_prior_pose_publishes_nothing() -> None:
    """Missing arm joints and nothing to take them from: the pose is unknown, so nothing is published."""
    checker = FakeChecker()
    result = ArmGuard(checker, MAX_STEP_RAD, enabled=True).admit(SOURCE, ["shoulder_pan"], [0.1])
    assert result.blocked and result.positions is None and result.obstacle == UNKNOWN_POSE
    assert checker.paths == []


def test_a_raising_checker_publishes_nothing_and_counts_an_error() -> None:
    """A checker exception refuses the command (fail closed) and is counted."""
    guard = ArmGuard(FakeChecker(RuntimeError("kernel gone")), MAX_STEP_RAD, enabled=True)
    guard.seed(ARM_NAMES, ordered(PARKED_HOME))
    result = guard.admit(SOURCE, ARM_NAMES, ordered(DRIVE_POSE))
    assert result.positions is None and not result.blocked
    assert "kernel gone" in result.error
    assert guard.errors == 1


def test_a_disabled_guard_returns_the_request() -> None:
    """enabled: false publishes every request unchecked."""
    checker = FakeChecker()
    request = WHEEL_FR_PRESSED
    result = ArmGuard(checker, MAX_STEP_RAD, enabled=False).admit(SOURCE, ARM_NAMES, ordered(request))
    assert result.positions == ordered(request) and not result.blocked
    assert checker.paths == []


def test_the_head_pose_poses_the_body() -> None:
    """The fresh head pose of head_source is passed to the checker (the D435 head where it is)."""
    checker = FakeChecker()
    head = HeadPose(pan=0.2, tilt=0.5)
    guard = ArmGuard(checker, MAX_STEP_RAD, enabled=True, head_source=lambda: head)
    guard.admit(SOURCE, ARM_NAMES, ordered(DRIVE_POSE))
    assert checker.heads == [head]


# The deployed follower gripper mapping (client.yml lerobot_follower joint gripper): leader source range and the
# follower command range (the servo's min/max_angle_limit).
LEADER_GRIPPER = LeaderGripperSettings(
    source_min_steps=2045, source_max_steps=3289, command_min_steps=1900, command_max_steps=3377
)
# FakeChecker below: a gripper below this (follower rad) breaks the margin.
GRIPPER_FLOOR_RAD = -0.1


class GripperFloorChecker(FakeChecker):
    """Reports the first sample whose gripper is below GRIPPER_FLOOR_RAD as the violation."""

    def path(self, samples: list[dict[str, float]], allow_escape: bool = True) -> Any:
        self.paths.append((samples, allow_escape))
        bad = next((i for i, s in enumerate(samples) if s["gripper"] < GRIPPER_FLOOR_RAD), None)
        return SimpleNamespace(
            violation=bad, violation_part="moving_jaw_tip", violation_obstacle="crossbar", violation_clearance=-0.01
        )


def bridge_follower_rad(leader_rad: float) -> float:
    """Where the bridge puts a leader gripper command, in follower radians (its own mapping).

    Args:
        leader_rad (float): Leader gripper position (rad).

    Returns:
        float: Follower gripper position (rad), as /follower/joint_states reports it.
    """
    return steps_to_radians(map_position_to_steps(leader_rad, 2045, 3289, 1900, 3377))


def test_a_leader_gripper_command_is_judged_in_follower_units() -> None:
    """Leader gripper 0.0 rad is safe read raw but the bridge sends it to about -0.22 follower rad: refused."""
    assert bridge_follower_rad(0.0) < GRIPPER_FLOOR_RAD
    checker = GripperFloorChecker()
    guard = ArmGuard(checker, MAX_STEP_RAD, enabled=True, leader_gripper=LEADER_GRIPPER)
    request = {**DRIVE_POSE, "gripper": 0.0}
    result = guard.admit("leader", ARM_NAMES, ordered(request))
    assert result.blocked
    assert checker.paths[-1][0][-1]["gripper"] == pytest.approx(bridge_follower_rad(0.0))


def test_a_direct_gripper_command_is_judged_as_sent() -> None:
    """Web UI and autonomy positions already are follower radians: the same 0.0 rad is checked as 0.0 and goes out."""
    checker = GripperFloorChecker()
    guard = ArmGuard(checker, MAX_STEP_RAD, enabled=True, leader_gripper=LEADER_GRIPPER)
    request = {**DRIVE_POSE, "gripper": 0.0}
    for source in ("web_ui", "autonomy"):
        result = guard.admit(source, ARM_NAMES, ordered(request))
        assert not result.blocked and result.positions == ordered(request)
        assert checker.paths[-1][0][-1]["gripper"] == 0.0


def test_a_clean_leader_command_goes_out_in_leader_units() -> None:
    """A clear leader command is published unchanged (leader units: the bridge maps it)."""
    checker = GripperFloorChecker()
    guard = ArmGuard(checker, MAX_STEP_RAD, enabled=True, leader_gripper=LEADER_GRIPPER)
    request = {**DRIVE_POSE, "gripper": 1.0}
    result = guard.admit("leader", ARM_NAMES, ordered(request))
    assert result.positions == ordered(request)
    assert checker.paths[-1][0][-1]["gripper"] == pytest.approx(bridge_follower_rad(1.0))


def test_a_clamped_leader_gripper_goes_out_in_leader_units() -> None:
    """The clamp of a leader command is converted back: the bridge maps the published gripper onto the safe sample."""
    checker = GripperFloorChecker()
    guard = ArmGuard(checker, MAX_STEP_RAD, enabled=True, leader_gripper=LEADER_GRIPPER)
    guard.seed(ARM_NAMES, ordered({**DRIVE_POSE, "gripper": 0.5}))
    result = guard.admit("leader", ARM_NAMES, ordered({**DRIVE_POSE, "gripper": 0.0}))
    assert result.blocked and result.positions is not None
    samples = checker.paths[-1][0]
    first_bad = next(i for i, s in enumerate(samples) if s["gripper"] < GRIPPER_FLOOR_RAD)
    safe = samples[first_bad - 1]["gripper"]
    published = result.positions[ARM_NAMES.index("gripper")]
    # One servo step of rounding: the bridge maps the published leader value back onto the safe follower sample.
    assert bridge_follower_rad(published) == pytest.approx(safe, abs=2 * RADIANS_PER_STEP)
    assert bridge_follower_rad(published) >= GRIPPER_FLOOR_RAD


@pytest.mark.parametrize("leader_rad", [-0.5, 0.0, 0.3, 1.0, 1.9, 2.5])
def test_follower_to_leader_inverts_the_bridge_mapping(leader_rad: float) -> None:
    """follower_to_leader lands on a leader value the bridge maps back to the same follower position (to one step)."""
    follower = bridge_follower_rad(leader_rad)
    assert bridge_follower_rad(follower_to_leader(follower, LEADER_GRIPPER)) == pytest.approx(
        follower, abs=RADIANS_PER_STEP
    )
    assert leader_to_follower(leader_rad, LEADER_GRIPPER) == pytest.approx(follower)


@pytest.mark.parametrize("source", ["leader", "web_ui", "autonomy"])
@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
@pytest.mark.parametrize("joint", ARM_NAMES)
def test_a_non_finite_position_is_refused_without_raising(source: str, bad: float, joint: str) -> None:
    """NaN or inf in any joint (the gripper included) refuses the command (fail closed), counted as an error."""
    checker = FakeChecker()
    guard = ArmGuard(
        checker,
        MAX_STEP_RAD,
        enabled=True,
        leader_gripper=LeaderGripperSettings(
            source_min_steps=2045, source_max_steps=3289, command_min_steps=1900, command_max_steps=3377
        ),
    )
    guard.seed(ARM_NAMES, ordered(PARKED_HOME))
    request = {**DRIVE_POSE, joint: bad}
    result = guard.admit(source, ARM_NAMES, ordered(request))
    assert result.positions is None and not result.blocked
    assert result.error
    assert guard.errors == 1
    assert checker.paths == []
    assert guard.pose == {**PARKED_HOME}
