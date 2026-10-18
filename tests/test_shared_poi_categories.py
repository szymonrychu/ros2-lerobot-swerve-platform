"""POI category tag grammar and mapping rules (ros2_common.poi_categories)."""

import pytest

from shared.ros2_common.poi_categories import (
    CATEGORIES,
    MAX_NOTE_LEN,
    PROBLEM_TYPES,
    SURVEY_SPOT_TAG,
    SURVEY_TAG,
    PoiTags,
    apply_tags,
    category_tags,
    check_category,
    has_tag,
    parse_tags,
    strip_tags,
)

PLAIN_CATEGORIES = [c for c in CATEGORIES if c != "problem"]


def test_constants() -> None:
    """The category and problem type lists and tag names are fixed."""
    assert CATEGORIES == ("target", "slow", "keepout", "problem", "detail")
    assert "overhang" in PROBLEM_TYPES and "low_obstacle" in PROBLEM_TYPES and len(PROBLEM_TYPES) == 10
    assert (SURVEY_TAG, SURVEY_SPOT_TAG, MAX_NOTE_LEN) == ("#survey", "#survey-spot", 2000)


@pytest.mark.parametrize("category", PLAIN_CATEGORIES)
def test_round_trip_plain_category(category: str) -> None:
    """Every non-problem category survives apply_tags then parse_tags."""
    note = apply_tags("free text", category_tags(category))
    assert parse_tags(note) == PoiTags(category=category, problem_type=None, survey=False, survey_spot=False)
    assert strip_tags(note) == "free text"


@pytest.mark.parametrize("problem_type", PROBLEM_TYPES)
def test_round_trip_every_problem_type(problem_type: str) -> None:
    """Every problem type survives the round trip, with survey flags."""
    tags = category_tags("problem", problem_type, survey=True, survey_spot=True)
    assert tags[0] == f"#problem/{problem_type}"
    assert parse_tags(apply_tags("", tags)) == PoiTags(
        category="problem", problem_type=problem_type, survey=True, survey_spot=True
    )


def test_category_tags_validation() -> None:
    """Problem needs a type; a type on another category and unknown names are errors listing valid values."""
    with pytest.raises(ValueError, match="problem_type"):
        category_tags("problem")
    with pytest.raises(ValueError, match="problem_type"):
        category_tags("target", "window")
    with pytest.raises(ValueError, match="target"):
        category_tags("bogus")
    with pytest.raises(ValueError, match="mirror"):
        category_tags("problem", "bogus")
    assert category_tags(None, survey=True) == ["#survey"]
    assert category_tags(None) == []


def test_parse_tags_edge_cases() -> None:
    """Only the first line is read, unknown words are ignored, no known tag gives an empty PoiTags."""
    assert parse_tags("") == PoiTags(category=None, problem_type=None, survey=False, survey_spot=False)
    assert parse_tags("hello #target").category == "target"
    assert parse_tags("#nonsense #slow").category == "slow"
    assert parse_tags("#problem/bogus").category is None
    assert parse_tags("text\n#target").category is None
    assert parse_tags("#survey-spot").survey_spot is True
    assert parse_tags("#survey-spot").survey is False


def test_apply_tags_replaces_category_keeps_survey_and_text() -> None:
    """A new category replaces the old one, existing survey tags and the free text stay."""
    note = "#target #survey\nsecond line\nthird"
    out = apply_tags(note, category_tags("problem", "window"))
    assert parse_tags(out) == PoiTags(category="problem", problem_type="window", survey=True, survey_spot=False)
    assert strip_tags(out) == "second line\nthird"
    assert "#target" not in out


def test_apply_tags_no_duplicates_when_repeated() -> None:
    """Applying the same tags again does not duplicate any tag."""
    tags = category_tags("slow", survey=True, survey_spot=True)
    once = apply_tags("note", tags)
    assert apply_tags(once, tags) == once
    assert apply_tags(apply_tags(once, tags), tags).splitlines()[0].split().count("#survey") == 1


def test_apply_tags_on_plain_note_prepends_line() -> None:
    """A note without tags gets a tag line in front; an empty note becomes just the tag line."""
    assert apply_tags("just text", ["#detail"]) == "#detail\njust text"
    assert apply_tags("", ["#detail"]) == "#detail"


def test_apply_tags_length_error() -> None:
    """A result longer than MAX_NOTE_LEN raises ValueError naming the length."""
    with pytest.raises(ValueError, match=str(MAX_NOTE_LEN)):
        apply_tags("x" * MAX_NOTE_LEN, ["#target"])


def test_has_tag_and_strip_tags() -> None:
    """has_tag checks the first line only; strip_tags returns the free text only."""
    note = "#problem/mirror #survey\nfree"
    assert has_tag(note, "#survey") and has_tag(note, "#problem/mirror")
    assert not has_tag(note, "#target")
    assert not has_tag("free\n#target", "#target")
    assert strip_tags(note) == "free"
    assert strip_tags("#target") == ""
    assert strip_tags("no tags here") == "no tags here"


VALID = [
    ("target", "point", "none", None, None),
    ("target", "object", "none", None, None),
    ("target", "area", "none", None, None),
    ("slow", "area", "slow", None, 50),
    ("keepout", "area", "keepout", None, None),
    ("problem", "point", "none", "window", None),
    ("problem", "area", "slow", "tight", 30),
    ("problem", "area", "none", "glass", None),
    ("detail", "point", "none", None, None),
    ("detail", "area", "none", None, None),
]

INVALID = [
    ("target", "point", "slow", None, None, "zone"),
    ("target", "area", "keepout", None, None, "zone"),
    ("slow", "point", "none", None, None, "area"),
    ("slow", "area", "none", None, None, "zone"),
    ("slow", "area", "slow", None, None, "speed_limit_pct"),
    ("keepout", "point", "none", None, None, "area"),
    ("keepout", "area", "slow", None, 50, "zone"),
    ("problem", "point", "keepout", "window", None, "zone"),
    ("problem", "point", "none", None, None, "problem_type"),
    ("problem", "point", "none", "bogus", None, "window"),
    ("problem", "object", "none", "window", None, "kind"),
    ("detail", "object", "none", None, None, "kind"),
    ("detail", "point", "slow", None, None, "zone"),
    ("target", "point", "none", "window", None, "problem_type"),
    ("bogus", "point", "none", None, None, "keepout"),
]


@pytest.mark.parametrize(("category", "kind", "zone", "problem_type", "speed"), VALID)
def test_check_category_accepts_valid(
    category: str, kind: str, zone: str, problem_type: str | None, speed: int | None
) -> None:
    """Each valid category / kind / zone combination passes."""
    check_category(category, kind, zone, problem_type, speed)


@pytest.mark.parametrize(("category", "kind", "zone", "problem_type", "speed", "word"), INVALID)
def test_check_category_rejects_invalid(
    category: str, kind: str, zone: str, problem_type: str | None, speed: int | None, word: str
) -> None:
    """Each invalid combination raises ValueError whose message names the broken rule."""
    with pytest.raises(ValueError, match=word):
        check_category(category, kind, zone, problem_type, speed)
