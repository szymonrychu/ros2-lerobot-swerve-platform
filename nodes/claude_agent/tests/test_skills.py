"""Skill plugin loader: discovery, qualified names, structure checks and the init-message skill check."""

import json
import re
from pathlib import Path

from claude_agent.config import WORKFLOW_TEMPLATE_DIR
from claude_agent.skills import (
    OPTIONAL_SECTION,
    PLUGIN_DIR,
    PLUGIN_NAME,
    check_skill,
    discover_skills,
    missing_skills,
    plugin_dir,
    qualified_names,
    read_skill,
)

RETIRED_CAMERA_RE = re.compile(
    r"gripper (camera|image)|gripper for close work|camera_0|\bToF\b|Camera Module 3|\bzoom", re.IGNORECASE
)
GOOD = "---\nname: demo\ndescription: Use when testing.\n---\n\n# Demo\n\nDo the thing.\n"


def write_skill(root: Path, directory: str, text: str) -> None:
    (root / "skills" / directory).mkdir(parents=True, exist_ok=True)
    (root / "skills" / directory / "SKILL.md").write_text(text)


def test_plugin_dir_default_and_configured(tmp_path: Path) -> None:
    assert plugin_dir("") == PLUGIN_DIR
    assert plugin_dir(str(tmp_path)) == tmp_path.resolve()


def test_discover_skills_sorted_and_needs_skill_md(tmp_path: Path) -> None:
    write_skill(tmp_path, "b-skill", GOOD)
    write_skill(tmp_path, "a-skill", GOOD)
    (tmp_path / "skills" / "empty-dir").mkdir()
    assert discover_skills(tmp_path) == ["a-skill", "b-skill"]
    assert discover_skills(tmp_path / "missing") == []


def test_qualified_names() -> None:
    assert qualified_names(["a", "b"]) == [f"{PLUGIN_NAME}:a", f"{PLUGIN_NAME}:b"]


def test_plugin_json_valid() -> None:
    data = json.loads((PLUGIN_DIR / ".claude-plugin" / "plugin.json").read_text())
    assert data["name"] == "robot-skills"
    assert data["version"] and data["description"]


def test_shipped_skills_discovered_and_pass_check() -> None:
    names = discover_skills(PLUGIN_DIR)
    assert "recover-from-failure" in names
    for name in names:
        assert check_skill(read_skill(PLUGIN_DIR, name), name) == []


def test_read_skill_parses_frontmatter(tmp_path: Path) -> None:
    write_skill(tmp_path, "demo", GOOD)
    skill = read_skill(tmp_path, "demo")
    assert (skill.name, skill.description) == ("demo", "Use when testing.")
    assert skill.body.startswith("# Demo")


def test_check_skill_good(tmp_path: Path) -> None:
    write_skill(tmp_path, "demo", GOOD)
    assert check_skill(read_skill(tmp_path, "demo"), "demo") == []


def test_check_skill_reports_name_mismatch(tmp_path: Path) -> None:
    write_skill(tmp_path, "other", GOOD)
    problems = check_skill(read_skill(tmp_path, "other"), "other")
    assert any("name" in p and "other" in p for p in problems)


def test_check_skill_reports_empty_and_long_description(tmp_path: Path) -> None:
    write_skill(tmp_path, "demo", "---\nname: demo\ndescription: ''\n---\nbody\n")
    assert any("description" in p for p in check_skill(read_skill(tmp_path, "demo"), "demo"))
    write_skill(tmp_path, "demo", f"---\nname: demo\ndescription: {'x' * 1025}\n---\nbody\n")
    assert any("1024" in p for p in check_skill(read_skill(tmp_path, "demo"), "demo"))


def test_check_skill_reports_empty_body(tmp_path: Path) -> None:
    write_skill(tmp_path, "demo", "---\nname: demo\ndescription: d\n---\n\n")
    assert any("body" in p for p in check_skill(read_skill(tmp_path, "demo"), "demo"))


def test_check_skill_reports_sibling_file_reference(tmp_path: Path) -> None:
    for text in ("See [more](reference.md).\n", "Read `notes/extra.md` first.\n", "Run `scripts/go.py`.\n"):
        write_skill(tmp_path, "demo", f"---\nname: demo\ndescription: d\n---\n{text}")
        assert any("self-contained" in p for p in check_skill(read_skill(tmp_path, "demo"), "demo")), text


def test_check_skill_allows_urls_and_plain_prose(tmp_path: Path) -> None:
    write_skill(tmp_path, "demo", "---\nname: demo\ndescription: d\n---\nSee https://example.com/a.md and NOTES.md.\n")
    assert check_skill(read_skill(tmp_path, "demo"), "demo") == []


def test_optional_section_constant() -> None:
    assert OPTIONAL_SECTION == "## When extra perception tools exist"


def test_missing_skills_strings_objects_and_absent_key() -> None:
    expected = ["robot-skills:a", "robot-skills:b"]
    assert missing_skills({"skills": ["robot-skills:a"]}, expected) == ["robot-skills:b"]
    assert missing_skills({"skills": [{"name": "robot-skills:a"}, {"name": "robot-skills:b"}]}, expected) == []
    assert missing_skills({"skills": [{"name": "robot-skills:b"}, "x"]}, expected) == ["robot-skills:a"]
    assert missing_skills({}, expected) is None


def test_no_skill_or_workflow_names_a_retired_camera() -> None:
    """No shipped skill or workflow template names the gripper camera, the head ToF, Camera Module 3 or zoom."""
    files = sorted((PLUGIN_DIR / "skills").glob("*/SKILL.md")) + sorted(WORKFLOW_TEMPLATE_DIR.glob("*.yaml"))
    assert files
    hits = [
        f"{path.parent.name}/{path.name}:{lineno}: {line.strip()}"
        for path in files
        for lineno, line in enumerate(path.read_text().splitlines(), 1)
        if RETIRED_CAMERA_RE.search(line)
    ]
    assert hits == []
