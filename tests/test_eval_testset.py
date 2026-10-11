"""Unit tests for scripts/eval/testset.py: the fixed 10x3 test set, case expansion and prompt rendering."""

import copy
from pathlib import Path
from typing import Any

import pydantic
import pytest
import yaml

from scripts.eval.testset import (
    MODE_ARM_ONLY,
    MODE_FULL,
    TESTSET_V1,
    TOTAL_CASES,
    Case,
    TestSet,
    load_testset,
)

TODO_WORDS = ("todo", "tbd", "fixme", "xxx", "...")


def raw_testset() -> dict[str, Any]:
    """Build a valid raw test set dict (10 objects, 3 placements).

    Returns:
        dict[str, Any]: Test set data.
    """
    return {
        "version": 1,
        "place_target": {"id": "bin", "description": "the grey box"},
        "objects": [{"id": f"obj{i}", "name": f"object {i}", "description": f"thing {i}"} for i in range(10)],
        "placements": [
            {"id": "near", "description": "in front", "arm_only": True},
            {"id": "left", "description": "front left", "arm_only": True},
            {"id": "far", "description": "far away", "arm_only": False},
        ],
        "prompt_template": (
            "Pick up the {object_name} lying {placement_description} and put it into {place_description}."
        ),
        "arm_only_suffix": " Do not drive.",
    }


def test_valid_testset_loads() -> None:
    ts = TestSet.model_validate(raw_testset())
    assert len(ts.objects) == 10
    assert len(ts.placements) == 3
    assert TOTAL_CASES == 30


@pytest.mark.parametrize("count", [9, 11])
def test_wrong_object_count_rejected(count: int) -> None:
    data = raw_testset()
    data["objects"] = [{"id": f"o{i}", "name": "n", "description": "d"} for i in range(count)]
    with pytest.raises(pydantic.ValidationError, match="exactly 10 objects"):
        TestSet.model_validate(data)


@pytest.mark.parametrize("count", [2, 4])
def test_wrong_placement_count_rejected(count: int) -> None:
    data = raw_testset()
    data["placements"] = [{"id": f"p{i}", "description": "d", "arm_only": True} for i in range(count)]
    with pytest.raises(pydantic.ValidationError, match="exactly 3 placements"):
        TestSet.model_validate(data)


def test_duplicate_object_ids_rejected() -> None:
    data = raw_testset()
    data["objects"][3]["id"] = "obj0"
    with pytest.raises(pydantic.ValidationError, match="duplicate object id"):
        TestSet.model_validate(data)


def test_duplicate_placement_ids_rejected() -> None:
    data = raw_testset()
    data["placements"][2]["id"] = "near"
    with pytest.raises(pydantic.ValidationError, match="duplicate placement id"):
        TestSet.model_validate(data)


def test_no_arm_only_placement_rejected() -> None:
    data = raw_testset()
    for placement in data["placements"]:
        placement["arm_only"] = False
    with pytest.raises(pydantic.ValidationError, match="arm_only"):
        TestSet.model_validate(data)


def test_unknown_key_rejected() -> None:
    data = raw_testset()
    data["extra"] = 1
    with pytest.raises(pydantic.ValidationError):
        TestSet.model_validate(data)


def test_template_with_unknown_placeholder_rejected() -> None:
    data = raw_testset()
    data["prompt_template"] = "Pick up the {object_name} at {where}."
    with pytest.raises(pydantic.ValidationError, match="prompt_template"):
        TestSet.model_validate(data)


def test_full_mode_expands_to_30_object_major() -> None:
    ts = TestSet.model_validate(raw_testset())
    cases = ts.cases(MODE_FULL)
    assert len(cases) == 30
    assert cases[0] == Case(object_id="obj0", placement_id="near")
    assert cases[1] == Case(object_id="obj0", placement_id="left")
    assert cases[2] == Case(object_id="obj0", placement_id="far")
    assert cases[3].object_id == "obj1"
    assert len({c.key for c in cases}) == 30


def test_arm_only_expansion_gives_20_arm_only_cases() -> None:
    ts = TestSet.model_validate(raw_testset())
    cases = ts.cases(MODE_ARM_ONLY)
    assert len(cases) == 20
    assert {c.placement_id for c in cases} == {"near", "left"}
    assert cases[0].key == "obj0/near"


def test_unknown_mode_rejected() -> None:
    ts = TestSet.model_validate(raw_testset())
    with pytest.raises(ValueError, match="mode"):
        ts.cases("drive_everywhere")


def test_render_prompt_full_and_arm_only() -> None:
    ts = TestSet.model_validate(raw_testset())
    case = Case(object_id="obj2", placement_id="near")
    full = ts.render_prompt(case, MODE_FULL)
    assert full == "Pick up the object 2 lying in front and put it into the grey box."
    assert ts.render_prompt(case, MODE_ARM_ONLY) == full + " Do not drive."


def test_render_prompt_unknown_case_rejected() -> None:
    ts = TestSet.model_validate(raw_testset())
    with pytest.raises(KeyError):
        ts.render_prompt(Case(object_id="nope", placement_id="near"), MODE_FULL)


def test_shipped_v1_testset() -> None:
    ts = load_testset(TESTSET_V1)
    assert ts.version == 1
    assert len(ts.objects) == 10
    assert sum(p.arm_only for p in ts.placements) == 2
    assert sum(not p.arm_only for p in ts.placements) == 1
    assert len(ts.cases(MODE_ARM_ONLY)) == 20
    text = Path(TESTSET_V1).read_text().lower()
    assert not any(word in text for word in TODO_WORDS)
    for item in [*ts.objects, *ts.placements, ts.place_target]:
        assert item.description.strip()
    prompt = ts.render_prompt(ts.cases(MODE_ARM_ONLY)[0], MODE_ARM_ONLY)
    assert ts.objects[0].name in prompt
    assert ts.place_target.description in prompt
    assert prompt.endswith(ts.arm_only_suffix)


def test_load_testset_from_file(tmp_path: Path) -> None:
    path = tmp_path / "ts.yaml"
    path.write_text(yaml.safe_dump(copy.deepcopy(raw_testset())))
    assert load_testset(path).place_target.id == "bin"
