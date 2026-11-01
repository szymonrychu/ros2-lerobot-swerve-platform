"""The guard state topic payload and when it is published: on change, at 1 Hz while blocked, nothing while clear."""

import json
import math

from filter_node.guard import GuardResult, GuardStateReporter, guard_state_payload

BLOCKED = GuardResult([0.1], blocked=True, link="frame_right", obstacle="wheel_fr", clearance_m=0.004)
CLEAR = GuardResult([0.1])
REFUSED_UNKNOWN = GuardResult(None, blocked=True, obstacle="unknown_pose")
FAILED = GuardResult(None, error="RuntimeError: kernel gone")


def test_payload_of_a_blocked_result() -> None:
    assert json.loads(guard_state_payload("web_ui", BLOCKED)) == {
        "blocked": True,
        "source": "web_ui",
        "link": "frame_right",
        "obstacle": "wheel_fr",
        "clearance_m": 0.004,
    }


def test_payload_of_a_clear_result_is_valid_json_without_infinity() -> None:
    payload = json.loads(guard_state_payload("leader", CLEAR))
    assert payload == {"blocked": False, "source": "", "link": "", "obstacle": "", "clearance_m": None}
    assert math.isinf(CLEAR.clearance_m)


def test_nothing_is_published_while_clear_and_never_seen_blocked() -> None:
    reporter = GuardStateReporter()
    assert reporter.poll(0.0) is None
    reporter.update("leader", CLEAR, 0.1)
    assert reporter.poll(0.2) is None
    assert reporter.poll(5.0) is None


def test_publishes_on_change_and_repeats_at_1_hz_while_blocked() -> None:
    reporter = GuardStateReporter()
    reporter.update("web_ui", BLOCKED, 10.0)
    first = reporter.poll(10.0)
    assert first is not None and json.loads(first)["blocked"] is True
    assert reporter.poll(10.5) is None
    reporter.update("web_ui", BLOCKED, 10.9)
    again = reporter.poll(11.0)
    assert again == first
    assert reporter.poll(11.5) is None


def test_a_changed_obstacle_publishes_at_once() -> None:
    reporter = GuardStateReporter()
    reporter.update("web_ui", BLOCKED, 1.0)
    reporter.poll(1.0)
    other = GuardResult([0.1], blocked=True, link="gripper", obstacle="crossbar", clearance_m=0.001)
    reporter.update("web_ui", other, 1.1)
    payload = reporter.poll(1.1)
    assert payload is not None and json.loads(payload)["obstacle"] == "crossbar"


def test_a_clear_result_publishes_unblocked_once() -> None:
    reporter = GuardStateReporter()
    reporter.update("web_ui", BLOCKED, 1.0)
    reporter.poll(1.0)
    reporter.update("web_ui", CLEAR, 1.2)
    cleared = reporter.poll(1.2)
    assert cleared is not None and json.loads(cleared)["blocked"] is False
    assert reporter.poll(1.3) is None
    assert reporter.poll(9.0) is None


def test_a_blocked_state_without_new_blocked_commands_clears_itself() -> None:
    reporter = GuardStateReporter()
    reporter.update("leader", BLOCKED, 1.0)
    reporter.poll(1.0)
    cleared = reporter.poll(4.0)
    assert cleared is not None and json.loads(cleared)["blocked"] is False
    assert reporter.poll(5.0) is None


def test_a_failed_check_leaves_the_state_alone() -> None:
    reporter = GuardStateReporter()
    reporter.update("web_ui", BLOCKED, 1.0)
    reporter.poll(1.0)
    reporter.update("web_ui", FAILED, 1.2)
    assert reporter.poll(1.3) is None


def test_an_unknown_pose_refusal_reports_blocked_by_unknown_pose() -> None:
    reporter = GuardStateReporter()
    reporter.update("web_ui", REFUSED_UNKNOWN, 1.0)
    payload = reporter.poll(1.0)
    assert payload is not None and json.loads(payload)["obstacle"] == "unknown_pose"
