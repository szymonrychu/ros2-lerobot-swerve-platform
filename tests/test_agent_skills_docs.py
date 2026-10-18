"""Docs guard for the driving-agent skills, workflows and retrospectives."""

from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
AGENT = REPO / "nodes" / "claude_agent"
README = AGENT / "README.md"
SKILLS_DIR = AGENT / "claude_agent" / "agent_plugin" / "skills"
TEMPLATES_DIR = AGENT / "claude_agent" / "workflow_templates"


def section(text: str, heading: str) -> str:
    """Return the body of a level-2 section.

    Args:
        text: Markdown text.
        heading: Heading without the leading "## ".

    Returns:
        The section body, or "" when the heading is missing.
    """
    marker = f"\n## {heading}\n"
    start = text.find(marker)
    if start < 0:
        return ""
    body = text[start + len(marker) :]
    end = body.find("\n## ")
    return body if end < 0 else body[:end]


def test_readme_has_skills_workflows_retrospectives_sections() -> None:
    text = README.read_text()
    for heading in ("Skills", "Workflows", "Retrospectives"):
        assert section(text, heading).strip(), f"claude_agent README lacks '## {heading}'"


def test_every_skill_directory_is_listed_in_readme() -> None:
    body = section(README.read_text(), "Skills")
    names = sorted(p.name for p in SKILLS_DIR.iterdir() if p.is_dir())
    assert names, "no skills found"
    missing = [n for n in names if n not in body]
    assert not missing, f"skills missing from README Skills section: {missing}"


def test_every_workflow_template_is_listed_in_readme() -> None:
    body = section(README.read_text(), "Workflows")
    names = sorted(p.stem for p in TEMPLATES_DIR.glob("*.yaml"))
    assert names, "no workflow templates found"
    missing = [n for n in names if n not in body]
    assert not missing, f"templates missing from README Workflows section: {missing}"


def test_nodes_readme_mentions_the_new_features() -> None:
    lines = (REPO / "nodes" / "README.md").read_text().splitlines()
    agent = next(line for line in lines if line.startswith("- **claude_agent/**")).lower()
    for word in ("skills", "workflows", "retrospectives"):
        assert word in agent, f"nodes/README.md claude_agent line lacks '{word}'"
    zones = next(line for line in lines if line.startswith("- **costmap_zones/**")).lower()
    assert "slow-zone cost" in zones


def test_memory_records_the_decision() -> None:
    text = (REPO / "MEMORY.md").read_text().lower()
    for phrase in (
        "read-only plugin",
        "start_workflow",
        "retrospectives store",
        "note tags",
        "keepout mask",
    ):
        assert phrase in text, f"MEMORY.md lacks '{phrase}'"


def test_tests_readme_lists_this_file() -> None:
    assert "test_agent_skills_docs.py" in (REPO / "tests" / "README.md").read_text()
