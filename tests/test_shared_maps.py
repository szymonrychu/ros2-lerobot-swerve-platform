"""Named maps topic contract (ros2_common.maps): constants, name rule and pydantic models."""

import math

import pytest
from pydantic import ValidationError

from shared.ros2_common import maps
from shared.ros2_common.maps import (
    Localisation,
    MapCommand,
    MapResult,
    MapStatus,
    MapSummary,
    Pose2D,
    validate_map_name,
)

POSE = {"x": 1.0, "y": -2.0, "yaw": 0.5}


def test_topic_constants() -> None:
    assert maps.COMMAND_TOPIC == "/map_manager/command"
    assert maps.RESULT_TOPIC == "/map_manager/result"
    assert maps.STATUS_TOPIC == "/map_manager/status"
    assert maps.LOCALISED_TOPIC == "/map_manager/localised"


@pytest.mark.parametrize("name", ["a", "Lab-1", "kitchen_2", "A" * 40])
def test_validate_map_name_accepts(name: str) -> None:
    assert validate_map_name(name) == name


@pytest.mark.parametrize("name", ["", "-a", "_a", "a b", "a/b", "..", "A" * 41, "ä"])
def test_validate_map_name_rejects(name: str) -> None:
    with pytest.raises(ValueError, match="map name"):
        validate_map_name(name)


def test_pose2d_finite_only() -> None:
    assert Pose2D(**POSE).yaw == 0.5
    for bad in (math.nan, math.inf, -math.inf):
        with pytest.raises(ValidationError):
            Pose2D(x=bad, y=0, yaw=0)


def test_command_save_load_delete_roundtrip() -> None:
    cmd = MapCommand(op="save", name="lab", request_id="r1", overwrite=True, note="hi")
    assert MapCommand.model_validate(cmd.model_dump(mode="json")) == cmd
    load = MapCommand(op="load", name="lab", pose=POSE)
    assert load.pose == Pose2D(**POSE)
    assert MapCommand.model_validate(load.model_dump(mode="json")) == load
    assert MapCommand(op="delete", name="lab").request_id == ""


@pytest.mark.parametrize("op", ["save", "load", "delete"])
def test_command_name_required_and_validated(op: str) -> None:
    with pytest.raises(ValidationError):
        MapCommand(op=op)
    with pytest.raises(ValidationError):
        MapCommand(op=op, name="bad name")


def test_command_rejects_unknown_op_and_extra_fields() -> None:
    with pytest.raises(ValidationError):
        MapCommand(op="rename", name="a")
    with pytest.raises(ValidationError):
        MapCommand(op="save", name="a", bogus=1)


@pytest.mark.parametrize("op", ["save", "delete", "relocalise"])
def test_command_pose_only_on_load(op: str) -> None:
    kwargs = {} if op == "relocalise" else {"name": "a"}
    with pytest.raises(ValidationError):
        MapCommand(op=op, pose=POSE, **kwargs)


def test_command_note_save_only_and_limited() -> None:
    assert MapCommand(op="save", name="a", note="x" * 500).note == "x" * 500
    with pytest.raises(ValidationError):
        MapCommand(op="save", name="a", note="x" * 501)
    for op in ("load", "delete"):
        with pytest.raises(ValidationError):
            MapCommand(op=op, name="a", note="x")


def test_command_relocalise_with_and_without_options() -> None:
    bare = MapCommand(op="relocalise")
    assert bare.since is None and bare.discard_on_fail is False and bare.name == ""
    full = MapCommand(op="relocalise", request_id="map_guard-1", since=1760000000.5, discard_on_fail=True)
    assert MapCommand.model_validate(full.model_dump(mode="json")) == full
    with_pose = MapCommand(op="relocalise", since=1.0)
    assert with_pose.since == 1.0


def test_command_relocalise_rejects_name() -> None:
    with pytest.raises(ValidationError):
        MapCommand(op="relocalise", name="lab")


@pytest.mark.parametrize("op", ["save", "load", "delete"])
def test_command_since_and_discard_rejected_off_relocalise(op: str) -> None:
    with pytest.raises(ValidationError):
        MapCommand(op=op, name="a", since=1.0)
    with pytest.raises(ValidationError):
        MapCommand(op=op, name="a", discard_on_fail=True)


def test_command_overwrite_only_save() -> None:
    with pytest.raises(ValidationError):
        MapCommand(op="relocalise", overwrite=True)


def test_result_roundtrip() -> None:
    res = MapResult(request_id="r", ok=False, message="busy: save lab", data={"a": [1]})
    assert MapResult.model_validate(res.model_dump(mode="json")) == res
    assert MapResult(request_id="r", ok=True, message="").data is None


def test_localisation_states_and_ratio_bounds() -> None:
    for state in ("unknown", "loading", "localised", "not_localised", "ambiguous"):
        loc = Localisation(state=state)
        assert loc.ratio is None and loc.pose is None and loc.message == "" and loc.updated_at is None
    with pytest.raises(ValidationError):
        Localisation(state="lost")
    for bad in (-0.1, 1.1):
        with pytest.raises(ValidationError):
            Localisation(state="localised", ratio=bad)
    loc = Localisation(state="localised", ratio=0.9, pose=POSE, message="ok", updated_at=5.0)
    assert Localisation.model_validate(loc.model_dump(mode="json")) == loc


def test_summary_and_status_roundtrip() -> None:
    summary = MapSummary(name="lab", saved_at=1.0, note="n", poi_count=3, last_pose=POSE, valid=True)
    assert summary.error == ""
    bad = MapSummary(name="x", saved_at=0.0, note="", poi_count=0, last_pose=POSE, valid=False, error="corrupt")
    status = MapStatus(maps=[summary, bad], active_map="lab", load_seq=2, localisation=Localisation(state="loading"))
    assert MapStatus.model_validate(status.model_dump(mode="json")) == status
    with pytest.raises(ValidationError):
        MapStatus(maps=[], active_map=None, load_seq=-1, localisation=Localisation(state="unknown"))
    assert MapStatus(maps=[], active_map=None, load_seq=0, localisation=Localisation(state="unknown")).maps == []


def test_command_clear_live_takes_no_fields() -> None:
    cmd = MapCommand(op="clear_live", request_id="web-1")
    assert cmd.name == "" and MapCommand.model_validate(cmd.model_dump(mode="json")) == cmd
    with pytest.raises(ValidationError, match="clear_live takes no map name"):
        MapCommand(op="clear_live", name="lab")
    for extra in (
        {"pose": Pose2D(x=0.0, y=0.0, yaw=0.0)},
        {"note": "n"},
        {"overwrite": True},
        {"since": 1.0},
        {"discard_on_fail": True},
    ):
        with pytest.raises(ValidationError):
            MapCommand(op="clear_live", **extra)


def test_summary_photo_count_defaults_and_bounds() -> None:
    old = {"name": "lab", "saved_at": 1.0, "note": "", "poi_count": 2, "last_pose": POSE, "valid": True}
    assert MapSummary.model_validate(old).photo_count == 0
    assert MapSummary.model_validate({**old, "photo_count": 4}).photo_count == 4
    with pytest.raises(ValidationError):
        MapSummary.model_validate({**old, "photo_count": -1})
