"""Unit tests for smooth joint interpolation used by follower command writes."""

from feetech_servos.trajectory import make_joint_interpolator, sample_joint, set_joint_target, smoothstep01


def test_smoothstep01_clamps_bounds() -> None:
    assert smoothstep01(-1.0) == 0.0
    assert smoothstep01(0.0) == 0.0
    assert smoothstep01(1.0) == 1.0
    assert smoothstep01(2.0) == 1.0


def test_joint_interpolator_reaches_target_over_duration() -> None:
    interp = make_joint_interpolator(1000.0, now_s=0.0)
    set_joint_target(interp, 2000.0, now_s=0.0, duration_s=1.0)

    start = sample_joint(interp, now_s=0.0)
    middle = sample_joint(interp, now_s=0.5)
    end = sample_joint(interp, now_s=1.0)

    assert start == 1000.0
    assert 1000.0 < middle < 2000.0
    assert end == 2000.0


def test_joint_interpolator_midflight_retarget_is_continuous() -> None:
    interp = make_joint_interpolator(0.0, now_s=0.0)
    set_joint_target(interp, 1000.0, now_s=0.0, duration_s=1.0)
    before_retarget = sample_joint(interp, now_s=0.5)

    set_joint_target(interp, 2000.0, now_s=0.5, duration_s=1.0)
    at_retarget = sample_joint(interp, now_s=0.5)
    later = sample_joint(interp, now_s=1.5)

    assert before_retarget == at_retarget
    assert before_retarget < later <= 2000.0
