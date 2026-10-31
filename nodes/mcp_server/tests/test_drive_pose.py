"""Tests for mcp_server.drive_pose: the drive pose state machine with a fake arm, gates, clock and worker."""

import threading
from collections.abc import Callable
from pathlib import Path

import pytest
import yaml

from mcp_server.arm import ArmError
from mcp_server.config import DrivePoseSettings
from mcp_server.drive_pose import DrivePose, DrivePoseState
from mcp_server.drive_pose_check import DrivePoseViolation
from mcp_server.models import ArmMotionResult

HOME = {
    "shoulder_pan": -0.003,
    "shoulder_lift": -1.35,
    "elbow_flex": 1.578,
    "wrist_flex": -0.1,
    "wrist_roll": -1.637,
    "gripper": 0.1,
}
DRIVE = HOME | {"shoulder_lift": -1.7, "elbow_flex": -0.9, "wrist_flex": 1.4}


class FakeSample:
    def __init__(self, positions: dict[str, float]) -> None:
        self.positions = dict(positions)


class FakeArm:
    """The ArmController surface DrivePose uses."""

    def __init__(self) -> None:
        self.positions = dict(HOME)
        self.fresh = True
        self.released = False
        self.home_stored = True
        self.control_held = False
        self.lease_generation = 0
        self.calls: list[tuple] = []
        self.status = "converged"
        self.lift_error: ArmError | None = None

    def fresh_sample(self) -> FakeSample | None:
        return FakeSample(self.positions) if self.fresh else None

    def measured(self, sample: FakeSample) -> dict[str, float]:
        return dict(sample.positions)

    def torque_off(self) -> bool:
        return self.released

    def stored_home(self) -> dict[str, float] | None:
        return dict(HOME) if self.home_stored else None

    def at_home(self) -> bool | None:
        if not self.fresh or not self.home_stored:
            return None
        return all(abs(self.positions[j] - HOME[j]) <= 0.05 for j in HOME if j != "gripper")

    def acquire(self) -> None:
        self.calls.append(("acquire",))
        self.control_held = True
        self.lease_generation += 1

    def release(self) -> None:
        self.calls.append(("release",))
        if self.control_held:
            self.lease_generation += 1
        self.control_held = False

    def home_motion(self, pose: dict[str, float], floor: object, speed_scale: float | None) -> ArmMotionResult:
        self.calls.append(("home_motion", dict(pose), floor, speed_scale))
        if self.lift_error is not None:
            raise self.lift_error
        if self.status == "converged":
            self.positions = dict(pose)
        return ArmMotionResult(status=self.status)

    def home(self, keep_prior_control: bool = True) -> ArmMotionResult:
        self.calls.append(("home", keep_prior_control))
        if self.status == "converged":
            self.positions = dict(HOME)
        return ArmMotionResult(status=self.status)

    def request_stop(self) -> None:
        self.calls.append(("request_stop",))
        self.status = "stopped"

    def motions(self) -> list[str]:
        return [c[0] for c in self.calls if c[0] in ("home_motion", "home")]


class FakeGates:
    def __init__(self) -> None:
        self.agent = False
        self.web_ui = False
        self.carry = False
        self.leader = False
        self.cutoff = False

    def agent_lease_held(self) -> bool:
        return self.agent

    def web_ui_lease_held(self) -> bool:
        return self.web_ui

    def carrying(self) -> bool:
        return self.carry

    def leader_moving(self) -> bool:
        return self.leader

    def battery_cutoff(self) -> bool:
        return self.cutoff


class FakeWorker:
    """Collects submitted jobs; run() executes them on the test thread."""

    def __init__(self) -> None:
        self.jobs: list[Callable[[], None]] = []

    def submit(self, job: Callable[[], None]) -> None:
        self.jobs.append(job)

    def run(self) -> None:
        while self.jobs:
            self.jobs.pop(0)()


class Rig:
    def __init__(
        self, enabled: bool = True, joints: dict[str, float] | None = DRIVE, stored_file: Path | None = None
    ) -> None:
        self.arm = FakeArm()
        self.gates = FakeGates()
        self.worker = FakeWorker()
        self.now = 100.0
        self.statics: list[str] = []
        self.settings = DrivePoseSettings(
            enabled=enabled, joints=joints, lower_delay_s=3.0, speed_scale=0.3, stored_file=stored_file
        )
        self.drive = DrivePose(
            self.settings,
            self.arm,
            self.gates,
            publish_static=lambda: self.statics.append("static"),
            submit=self.worker.submit,
            clock=lambda: self.now,
            clock_ns=lambda: int(self.now * 1e9),
        )

    def tick(self, goal: bool, dt: float = 0.2) -> DrivePoseState:
        self.now += dt
        self.drive.tick(self.now, goal)
        return self.drive.state()

    def lift(self) -> None:
        assert self.tick(True).state == "lifting"
        self.worker.run()
        assert self.tick(True).state == "lifted"


def test_goal_edge_lifts_along_plan_home_at_speed_scale_then_releases_keeping_position() -> None:
    rig = Rig()
    assert rig.drive.state().state == "idle"
    assert rig.tick(True).state == "lifting"
    assert rig.arm.calls == []  # the motion runs on the worker, never in tick
    rig.worker.run()
    assert rig.arm.calls == [("acquire",), ("home_motion", DRIVE, None, 0.3), ("release",)]
    state = rig.tick(True)
    assert (state.state, state.enabled) == ("lifted", True)


def test_disabled_reports_disabled_and_never_moves() -> None:
    rig = Rig(enabled=False)
    assert rig.drive.state().state == "disabled"
    assert rig.tick(True).state == "disabled"
    rig.worker.run()
    assert rig.arm.calls == []
    assert rig.drive.state().enabled is False


def setup_not_home(rig: Rig) -> None:
    rig.arm.positions = rig.arm.positions | {"shoulder_lift": -1.0}


def setup_stale(rig: Rig) -> None:
    rig.arm.fresh = False


@pytest.mark.parametrize(
    "setup, reason",
    [
        (setup_not_home, "arm not at home"),
        (lambda rig: setattr(rig.gates, "agent", True), "agent holds the arm"),
        (lambda rig: setattr(rig.gates, "web_ui", True), "web UI holds the arm"),
        (lambda rig: setattr(rig.arm, "released", True), "torque off"),
        (setup_stale, "no joint states"),
        (lambda rig: setattr(rig.gates, "carry", True), "carrying an object"),
        (lambda rig: setattr(rig.gates, "leader", True), "leader arm moving"),
        (lambda rig: setattr(rig.gates, "cutoff", True), "battery cut-off"),
        (lambda rig: setattr(rig.arm, "home_stored", False), "no home pose stored"),
    ],
    ids=["not-home", "agent", "web-ui", "torque-off", "stale", "carrying", "leader", "battery", "no-home"],
)
def test_a_failed_precondition_skips_without_motion(setup: Callable[[Rig], None], reason: str) -> None:
    rig = Rig()
    setup(rig)
    state = rig.tick(True)
    assert (state.state, state.reason) == (f"skipped: {reason}", reason)
    rig.worker.run()
    assert rig.arm.calls == []
    assert rig.tick(True).state == f"skipped: {reason}"  # once per goal: no retry while the goal runs


def test_no_configured_pose_skips() -> None:
    rig = Rig(joints=None)
    assert rig.tick(True).state == "skipped: no drive pose"


@pytest.mark.parametrize("outcome", ["succeeded", "aborted", "canceled"])
def test_goal_end_lowers_after_the_delay_through_the_home_path(outcome: str) -> None:
    """Success, abort and cancel all end the goal the same way: no goal active."""
    rig = Rig()
    rig.lift()
    assert rig.tick(False, dt=2.9).state == "lifted"
    assert rig.arm.motions() == ["home_motion"]
    state = rig.tick(False, dt=0.2)
    assert state.state == "lowering"
    assert rig.statics == ["static"]  # the static footprint goes out before the arm moves
    rig.worker.run()
    assert rig.arm.calls[-1] == ("home", False)
    assert rig.tick(False).state == "idle"


def test_a_new_goal_within_the_delay_keeps_the_arm_lifted() -> None:
    rig = Rig()
    rig.lift()
    assert rig.tick(False, dt=2.0).state == "lifted"
    assert rig.tick(True).state == "lifted"
    assert rig.tick(False, dt=2.0).state == "lifted"
    assert rig.tick(False, dt=2.0).state == "lowering"
    rig.worker.run()
    assert rig.arm.motions() == ["home_motion", "home"]


@pytest.mark.parametrize("who", ["agent", "web_ui"])
def test_a_lease_taken_mid_drive_means_no_lower(who: str, caplog: pytest.LogCaptureFixture) -> None:
    rig = Rig()
    rig.lift()
    setattr(rig.gates, who, True)
    state = rig.tick(False, dt=3.5)
    assert state.state == "idle"
    assert "lease" in state.reason
    rig.worker.run()
    rig.tick(False, dt=5.0)
    assert rig.arm.motions() == ["home_motion"]
    assert sum("lease" in r.getMessage() for r in caplog.records) == 1


def test_an_arm_moved_off_the_drive_pose_is_left_alone() -> None:
    rig = Rig()
    rig.lift()
    rig.arm.positions = rig.arm.positions | {"elbow_flex": DRIVE["elbow_flex"] + 0.1}
    state = rig.tick(False, dt=3.5)
    assert state.state == "idle"
    assert "moved" in state.reason
    rig.worker.run()
    assert rig.arm.motions() == ["home_motion"]


def test_within_tolerance_still_lowers() -> None:
    rig = Rig()
    rig.lift()
    rig.arm.positions = rig.arm.positions | {"elbow_flex": DRIVE["elbow_flex"] + 0.04}
    assert rig.tick(False, dt=3.5).state == "lowering"


def test_torque_off_mid_drive_means_no_lower() -> None:
    rig = Rig()
    rig.lift()
    rig.arm.released = True
    state = rig.tick(False, dt=3.5)
    assert (state.state, "torque off" in state.reason) == ("idle", True)
    assert rig.worker.jobs == []


def test_a_goal_during_lowering_lifts_again_after_the_lowering_motion() -> None:
    rig = Rig()
    rig.lift()
    assert rig.tick(False, dt=3.5).state == "lowering"
    assert rig.tick(True).state == "lowering"  # nothing new while the lowering motion runs
    assert len(rig.worker.jobs) == 1
    rig.worker.run()
    assert rig.tick(True).state == "lifting"
    rig.worker.run()
    assert rig.tick(True).state == "lifted"
    assert rig.arm.motions() == ["home_motion", "home", "home_motion"]


def test_skipped_is_cleared_at_goal_end_and_never_carries_over() -> None:
    rig = Rig()
    rig.gates.carry = True
    assert rig.tick(True).state == "skipped: carrying an object"
    assert rig.tick(False).state == "idle"
    rig.gates.carry = False
    assert rig.tick(True).state == "lifting"


@pytest.mark.parametrize("status", ["timeout", "aborted_tracking", "interrupted"])
def test_a_lift_that_does_not_converge_skips_and_leaves_the_arm(status: str) -> None:
    rig = Rig()
    rig.arm.status = status
    rig.tick(True)
    rig.worker.run()
    state = rig.tick(True)
    assert state.state == f"skipped: motion {status}"
    assert rig.arm.calls[-1] == ("release",)  # the lease is handed back, the arm stays where it stopped
    assert rig.tick(False, dt=5.0).state == "idle"
    rig.worker.run()
    assert rig.arm.motions() == ["home_motion"]


def test_a_lower_that_does_not_converge_skips() -> None:
    rig = Rig()
    rig.lift()
    rig.arm.status = "timeout"
    rig.tick(False, dt=3.5)
    rig.worker.run()
    assert rig.tick(False).state == "skipped: motion timeout"


def test_an_arm_error_during_the_lift_skips_with_its_message() -> None:
    rig = Rig()
    rig.arm.lift_error = ArmError("no collision-free route")
    rig.tick(True)
    rig.worker.run()
    state = rig.tick(True)
    assert state.state == "skipped: no collision-free route"
    assert rig.arm.calls[-1] == ("release",)


def test_the_lift_does_not_release_a_lease_someone_else_took_over() -> None:
    rig = Rig()
    rig.tick(True)

    def taken(pose: dict[str, float], floor: object, speed_scale: float | None) -> ArmMotionResult:
        rig.arm.lease_generation += 1  # a take-over during the lift
        return ArmMotionResult(status="stopped")

    rig.arm.home_motion = taken  # type: ignore[method-assign]
    rig.worker.run()
    assert ("release",) not in rig.arm.calls


def test_abort_stops_a_running_motion() -> None:
    rig = Rig()
    rig.tick(True)
    rig.drive.abort()
    assert ("request_stop",) in rig.arm.calls
    rig.worker.run()
    assert rig.tick(True).state == "skipped: stopped"


def test_abort_without_a_motion_changes_nothing() -> None:
    rig = Rig()
    rig.lift()
    rig.drive.abort()
    assert ("request_stop",) not in rig.arm.calls
    assert rig.drive.state().state == "lifted"


def test_request_lift_returns_once_lifted() -> None:
    rig = Rig()
    done: list[DrivePoseState] = []
    thread = threading.Thread(target=lambda: done.append(rig.drive.request_lift(5.0)))
    thread.start()
    while not rig.worker.jobs:
        threading.Event().wait(0.01)
    rig.worker.run()
    thread.join(2.0)
    assert done and done[0].state == "lifted"
    assert rig.tick(True).state == "lifted"  # the goal edge that follows does not lift again
    assert rig.arm.motions() == ["home_motion"]


def test_request_lift_returns_skipped_and_disabled_at_once() -> None:
    rig = Rig()
    rig.gates.leader = True
    assert rig.drive.request_lift(5.0).state == "skipped: leader arm moving"
    assert Rig(enabled=False).drive.request_lift(5.0).state == "disabled"


def test_request_lift_times_out() -> None:
    rig = Rig()
    state = rig.drive.request_lift(0.05)
    assert state.state == "lifting"
    assert len(rig.worker.jobs) == 1


def test_set_enabled_toggles_new_lifts() -> None:
    rig = Rig()
    rig.drive.set_enabled(False)
    assert rig.tick(True).state == "disabled"
    rig.tick(False)
    rig.drive.set_enabled(True)
    assert rig.drive.state().state == "idle"
    assert rig.tick(True).state == "lifting"


def test_disabling_while_lifted_still_lowers_at_goal_end() -> None:
    rig = Rig()
    rig.lift()
    rig.drive.set_enabled(False)
    assert rig.drive.state().state == "lifted"
    assert rig.tick(False, dt=3.5).state == "lowering"
    rig.worker.run()
    assert rig.tick(False).state == "disabled"


def test_state_stamp_moves_on_every_change() -> None:
    rig = Rig()
    first = rig.drive.state().stamp_ns
    rig.tick(True)
    assert rig.drive.state().stamp_ns > first
    stamp = rig.drive.state().stamp_ns
    rig.tick(True)
    assert rig.drive.state().stamp_ns == stamp


# --- stored drive pose (/arm/set_drive_pose) -------------------------------------------------------------------------

STORED = DRIVE | {"elbow_flex": -1.2}


def no_violation(_home: dict[str, float], _pose: dict[str, float]) -> list[DrivePoseViolation]:
    return []


def test_store_pose_writes_the_measured_pose_after_the_check_and_uses_it(tmp_path: Path) -> None:
    path = tmp_path / "arm" / "drive_pose.yaml"
    rig = Rig(stored_file=path)
    rig.arm.positions = dict(STORED)
    checked: list[tuple[dict[str, float], dict[str, float]]] = []

    def check(home: dict[str, float], pose: dict[str, float]) -> list[DrivePoseViolation]:
        checked.append((dict(home), dict(pose)))
        return []

    ok, message = rig.drive.store_pose(check)
    assert ok is True and str(path) in message
    assert checked == [(HOME, STORED)], "checked from the stored home"
    assert yaml.safe_load(path.read_text()) == {"joints": STORED}
    assert [p.name for p in path.parent.iterdir()] == ["drive_pose.yaml"], "atomic: no temp file left"
    rig.arm.positions = dict(HOME)
    rig.tick(True)
    rig.worker.run()
    assert rig.arm.calls[1] == ("home_motion", STORED, None, 0.3), "the stored pose wins over the config joints"


def test_a_failed_check_refuses_naming_the_checks_and_stores_nothing(tmp_path: Path) -> None:
    path = tmp_path / "drive_pose.yaml"
    rig = Rig(stored_file=path)

    def check(_home: dict[str, float], _pose: dict[str, float]) -> list[DrivePoseViolation]:
        return [DrivePoseViolation("body", "lift leg 2 sample 3"), DrivePoseViolation("view", "pan -60 deg")]

    ok, message = rig.drive.store_pose(check)
    assert ok is False and "body" in message and "view" in message and "lift leg 2 sample 3" in message
    assert not path.exists()
    assert rig.drive.joints == DRIVE


def test_store_pose_is_refused_without_fresh_joint_states_or_home(tmp_path: Path) -> None:
    rig = Rig(stored_file=tmp_path / "drive_pose.yaml")
    rig.arm.fresh = False
    ok, message = rig.drive.store_pose(no_violation)
    assert ok is False and "joint states" in message
    rig.arm.fresh, rig.arm.home_stored = True, False
    ok, message = rig.drive.store_pose(no_violation)
    assert ok is False and "home" in message
    assert not (tmp_path / "drive_pose.yaml").exists()


def test_the_stored_pose_survives_a_controller_restart(tmp_path: Path) -> None:
    path = tmp_path / "drive_pose.yaml"
    rig = Rig(stored_file=path)
    rig.arm.positions = dict(STORED)
    assert rig.drive.store_pose(no_violation)[0] is True
    assert Rig(stored_file=path).drive.joints == STORED
    assert Rig(stored_file=path, joints=None).drive.joints == STORED, "a stored pose needs no config joints"


@pytest.mark.parametrize(
    "content",
    ["joints: [not, a, mapping", "joints:\n  shoulder_pan: 0.0\n", "joints:\n  shoulder_pan: .nan\n"],
    ids=["unparsable", "missing joints", "not finite"],
)
def test_an_unreadable_stored_file_falls_back_to_the_config_with_a_warning(
    tmp_path: Path, content: str, caplog: pytest.LogCaptureFixture
) -> None:
    path = tmp_path / "drive_pose.yaml"
    path.write_text(content)
    rig = Rig(stored_file=path)
    assert rig.drive.joints == DRIVE
    assert any(r.levelname == "WARNING" and str(path) in r.getMessage() for r in caplog.records)


def test_no_stored_file_uses_the_config_joints(tmp_path: Path) -> None:
    assert Rig(stored_file=tmp_path / "missing.yaml").drive.joints == DRIVE


# --- at_drive_pose (the drive footprint, footprint.NavFootprint.drive_ready) -----------------------------------------


def test_at_drive_pose_within_the_tolerance_and_not_while_moving() -> None:
    rig = Rig()
    assert rig.drive.at_drive_pose() is False, "at home"
    rig.arm.positions = DRIVE | {"elbow_flex": DRIVE["elbow_flex"] + 0.04}
    assert rig.drive.at_drive_pose() is True
    rig.arm.positions = DRIVE | {"elbow_flex": DRIVE["elbow_flex"] + 0.06}
    assert rig.drive.at_drive_pose() is False
    rig.arm.positions = dict(DRIVE)
    rig.arm.fresh = False
    assert rig.drive.at_drive_pose() is False, "stale joint states"


def test_at_drive_pose_is_false_while_lifting_and_lowering() -> None:
    rig = Rig()
    rig.tick(True)
    rig.arm.positions = dict(DRIVE)
    assert rig.drive.state().state == "lifting" and rig.drive.at_drive_pose() is False
    rig.worker.run()
    assert rig.drive.state().state == "lifted" and rig.drive.at_drive_pose() is True
    rig.tick(False, dt=3.5)
    assert rig.drive.state().state == "lowering" and rig.drive.at_drive_pose() is False
    assert rig.statics == ["static"], "static published before the lowering motion"


def test_at_drive_pose_without_a_pose_is_false() -> None:
    rig = Rig(joints=None)
    assert rig.drive.at_drive_pose() is False
