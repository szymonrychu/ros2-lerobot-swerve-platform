"""Survey, sightseeing and poi-categories skills: they load and carry the asserted procedure elements."""

from claude_agent.skills import OPTIONAL_SECTION, PLUGIN_DIR, check_skill, read_skill

CATEGORIES = ("target", "slow", "keepout", "problem", "detail")
PROBLEM_TYPES = (
    "window",
    "mirror",
    "glass",
    "tight",
    "overhang",
    "low_obstacle",
    "drop",
    "cable",
    "soft_floor",
    "other",
)
MAX_SKILL_BYTES = 8 * 1024


def load(name: str) -> str:
    skill = read_skill(PLUGIN_DIR, name)
    assert check_skill(skill, name) == []
    assert len((PLUGIN_DIR / "skills" / name / "SKILL.md").read_bytes()) < MAX_SKILL_BYTES
    return skill.body


def test_skills_exist_and_pass_check() -> None:
    for name in ("survey", "sightseeing", "poi-categories"):
        load(name)


def test_survey_names_procedure_and_categories() -> None:
    body = load("survey")
    for word in ("get_topdown_view", "look_around", "mark_survey_spot", "add_poi", "category", "survey=true"):
        assert word in body
    for word in (*CATEGORIES, *PROBLEM_TYPES[:4]):
        assert word in body


def test_survey_optional_section() -> None:
    body = load("survey")
    assert OPTIONAL_SECTION in body
    section = body.split(OPTIONAL_SECTION)[1].split("\n## ")[0]
    assert "head_look_at" in section and "look_around" in section


def test_sightseeing_loop_stops_and_summary() -> None:
    body = load("sightseeing")
    for word in ("get_exploration_status", "look_toward_frontier", "go_to_frontier", "survey", "complete"):
        assert word in body
    assert "limit_reached" in body and "3 minutes" in body
    for word in ("coverage", "reasons", "battery", "keep-out", "problem"):
        assert word in body


def test_poi_categories_mapping_and_consent() -> None:
    body = load("poi-categories")
    for word in (*CATEGORIES, *PROBLEM_TYPES):
        assert word in body
    for word in ("remember_object", "annotate_lookup", "consent", "slow zone", "keep-out"):
        assert word in body
