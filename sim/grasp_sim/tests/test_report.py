"""SimReport schema: stable JSON, defaults and round trip."""

from grasp_sim.report import ClearanceStats, ContactEvent, ObjectReport, SegmentReport, SimReport


def make_report() -> SimReport:
    return SimReport(
        passed=False,
        reasons=["tipped"],
        duration_s=2.0,
        segments=[
            SegmentReport(
                label="approach",
                t_start=0.0,
                t_end=1.0,
                min_clearance=ClearanceStats(jaws_floor=0.01, jaws_support=None, wrist_floor=0.05, wrist_support=None),
                max_actuator_force_nm=1.2,
                max_object_tilt_deg=3.0,
                max_object_displacement_m=0.001,
            )
        ],
        first_unintended_contact=ContactEvent(
            t=0.5, label="approach", kind="jaw_object", geom_a="fixed_jaw_box1", geom_b="object_box"
        ),
        object=ObjectReport(
            start_pos=(0.2, 0.0, -0.13),
            final_pos=(0.2, 0.0, -0.13),
            approach_max_tilt_deg=3.0,
            approach_max_displacement_m=0.001,
            tipped=False,
            pushed=False,
            lift_height_m=0.0,
            lifted_after_lift=None,
            lifted_at_end=False,
        ),
        grasp_success=False,
        max_actuator_force_nm={"gripper": 1.0},
        saturated_actuators=[],
    )


def test_report_json_round_trip_and_top_level_keys() -> None:
    report = make_report()
    again = SimReport.model_validate_json(report.model_dump_json())
    assert again == report
    keys = set(report.model_dump())
    assert {
        "passed",
        "reasons",
        "warnings",
        "duration_s",
        "segments",
        "first_unintended_contact",
        "object",
        "grasp_success",
        "max_actuator_force_nm",
        "saturated_actuators",
        "event_counts",
    } <= keys


def test_report_defaults_allow_a_sceneless_report() -> None:
    report = SimReport(passed=True, reasons=[], duration_s=1.0, segments=[], max_actuator_force_nm={})
    assert report.object is None
    assert report.grasp_success is None
    assert report.first_unintended_contact is None
