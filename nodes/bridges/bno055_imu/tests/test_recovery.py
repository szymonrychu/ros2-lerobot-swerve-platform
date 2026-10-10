"""Unit tests for the BNO055 recovery policy (soft restore vs full re-initialisation) with an injected clock."""

from bno055_imu.recovery import Action, RecoveryPolicy


class FakeClock:
    """Manually advanced monotonic clock."""

    def __init__(self) -> None:
        self.now = 1000.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


def make_policy(max_soft: int = 3, reinit_after_s: float = 10.0) -> tuple[RecoveryPolicy, FakeClock]:
    clock = FakeClock()
    return RecoveryPolicy(max_soft, reinit_after_s, clock=clock), clock


def test_continue_when_healthy() -> None:
    policy, clock = make_policy()
    clock.advance(1.0)
    assert policy.decide(recovery_needed=False, soft_possible=True).action is Action.CONTINUE


def test_soft_restore_when_needed_and_possible() -> None:
    policy, _ = make_policy()
    assert policy.decide(True, True).action is Action.SOFT_RESTORE


def test_full_reinit_when_soft_not_possible() -> None:
    policy, _ = make_policy()
    decision = policy.decide(True, False)
    assert decision.action is Action.FULL_REINIT


def test_soft_restores_do_not_clear_escalation_state() -> None:
    """Rule (a): soft restores without a publish accumulate; rule (b): the 4th escalates."""
    policy, _ = make_policy(max_soft=3)
    for _ in range(3):
        assert policy.decide(True, True).action is Action.SOFT_RESTORE
    decision = policy.decide(True, True)
    assert decision.action is Action.FULL_REINIT
    assert "soft restores exhausted" in decision.reason


def test_publish_resets_soft_restore_count() -> None:
    policy, _ = make_policy(max_soft=3)
    for _ in range(3):
        policy.decide(True, True)
    policy.on_publish()
    assert policy.decide(True, True).action is Action.SOFT_RESTORE
    assert policy.soft_restores == 1


def test_watchdog_forces_reinit_even_when_no_recovery_needed() -> None:
    """Rule (c): no publish for reinit_after_s escalates regardless of failure path."""
    policy, clock = make_policy(reinit_after_s=10.0)
    clock.advance(9.9)
    assert policy.decide(False, True).action is Action.CONTINUE
    clock.advance(0.2)
    decision = policy.decide(False, True)
    assert decision.action is Action.FULL_REINIT
    assert "watchdog" in decision.reason
    assert "10" in decision.reason


def test_watchdog_beats_soft_restore() -> None:
    policy, clock = make_policy()
    clock.advance(11.0)
    assert policy.decide(True, True).action is Action.FULL_REINIT


def test_publish_feeds_watchdog() -> None:
    policy, clock = make_policy(reinit_after_s=10.0)
    clock.advance(9.0)
    policy.on_publish()
    clock.advance(9.0)
    assert policy.decide(False, True).action is Action.CONTINUE


def test_reinit_restarts_watchdog_and_soft_count() -> None:
    policy, clock = make_policy()
    for _ in range(3):
        policy.decide(True, True)
    clock.advance(11.0)
    policy.on_reinit()
    assert policy.soft_restores == 0
    assert policy.decide(False, True).action is Action.CONTINUE
    clock.advance(10.1)
    assert policy.decide(False, True).action is Action.FULL_REINIT


def test_reinit_backoff_is_exponential_capped_and_resets_on_publish() -> None:
    """Rule (d): min(30, 2**n) with n capped at 5, reset after a successful publish."""
    policy, _ = make_policy()
    seen = []
    for _ in range(7):
        seen.append(policy.backoff_s())
        policy.on_reinit()
    assert seen == [1.0, 2.0, 4.0, 8.0, 16.0, 30.0, 30.0]
    assert policy.reinit_count == 7
    policy.on_publish()
    assert policy.backoff_s() == 1.0
    assert policy.reinit_count == 0


def test_failed_reinit_forces_another_full_reinit_not_soft() -> None:
    policy, _ = make_policy()
    policy.on_reinit(success=False)
    assert policy.decide(True, True).action is Action.FULL_REINIT
