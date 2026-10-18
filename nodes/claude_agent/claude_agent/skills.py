"""Loader and checker for the read-only skill plugin shipped inside the node.

Skills are Agent SDK skill files under ``agent_plugin/skills/<name>/SKILL.md``. The plugin ships with the deployed
checkout (cloned as root, service runs as ``claude_agent``), so the agent cannot edit it; the agent's file tools stay
inside its workdir, which is why a skill must be self-contained (no reference to a file next to SKILL.md).
"""

import re
from pathlib import Path
from typing import Any

import yaml
from pydantic import BaseModel

PLUGIN_NAME = "robot-skills"
PLUGIN_DIR = Path(__file__).parent / "agent_plugin"
OPTIONAL_SECTION = "## When extra perception tools exist"
SKILL_FILE = "SKILL.md"
MAX_DESCRIPTION_LEN = 1024
FRONTMATTER = re.compile(r"\A---\s*\n(.*?)\n---[ \t]*\n?(.*)\Z", re.DOTALL)
MARKDOWN_LINK = re.compile(r"\]\((?!https?://|mailto:|#)([^)\s]+)\)")
BACKTICKED_PATH = re.compile(r"`((?:\./)?[\w.-]+(?:/[\w.-]+)+\.\w+|\./[\w.-]+\.\w+)`")


class SkillFile(BaseModel):
    """A parsed SKILL.md.

    Attributes:
        name: Frontmatter ``name`` ("" when absent).
        description: Frontmatter ``description`` ("" when absent).
        body: Markdown after the frontmatter.
    """

    name: str
    description: str
    body: str


def plugin_dir(configured: str) -> Path:
    """Resolve the plugin directory.

    Args:
        configured: Configured path; "" selects the plugin shipped with the node.

    Returns:
        Path: ``PLUGIN_DIR`` for "", else the resolved configured path.
    """
    return Path(configured).expanduser().resolve() if configured else PLUGIN_DIR


def discover_skills(root: Path) -> list[str]:
    """List the skills of a plugin.

    Args:
        root: Plugin directory.

    Returns:
        list[str]: Sorted names of the directories under ``skills/`` that hold a SKILL.md.
    """
    skills = root / "skills"
    if not skills.is_dir():
        return []
    return sorted(d.name for d in skills.iterdir() if (d / SKILL_FILE).is_file())


def qualified_names(names: list[str]) -> list[str]:
    """Plugin-qualified skill names as the SDK lists them.

    Args:
        names: Bare skill names.

    Returns:
        list[str]: ``robot-skills:<name>`` for each.
    """
    return [f"{PLUGIN_NAME}:{name}" for name in names]


def read_skill(root: Path, name: str) -> SkillFile:
    """Parse ``skills/<name>/SKILL.md``.

    Args:
        root: Plugin directory.
        name: Skill directory name.

    Returns:
        SkillFile: Frontmatter name and description plus the body (empty fields when the frontmatter is missing).
    """
    text = (root / "skills" / name / SKILL_FILE).read_text()
    match = FRONTMATTER.match(text)
    meta: dict[str, Any] = {}
    body = text
    if match:
        loaded = yaml.safe_load(match.group(1))
        meta = loaded if isinstance(loaded, dict) else {}
        body = match.group(2)
    return SkillFile(
        name=str(meta.get("name") or ""), description=str(meta.get("description") or "").strip(), body=body.strip()
    )


def check_skill(skill: SkillFile, directory: str) -> list[str]:
    """Check a skill's structure.

    Args:
        skill: Parsed skill.
        directory: Name of the directory holding it.

    Returns:
        list[str]: Problems; empty when the skill is fine.
    """
    problems = []
    if skill.name != directory:
        problems.append(f"frontmatter name {skill.name!r} != directory {directory!r}")
    if not skill.description:
        problems.append("description is empty")
    elif len(skill.description) > MAX_DESCRIPTION_LEN:
        problems.append(f"description is longer than {MAX_DESCRIPTION_LEN} characters")
    if not skill.body:
        problems.append("body is empty")
    for target in (*MARKDOWN_LINK.findall(skill.body), *BACKTICKED_PATH.findall(skill.body)):
        problems.append(f"references a file next to SKILL.md ({target}); skills must be self-contained")
    return problems


def missing_skills(init_data: dict[str, Any], expected: list[str]) -> list[str] | None:
    """Compare the init message's skills with the expected qualified names.

    Args:
        init_data: ``data`` of the SDK init system message.
        expected: Qualified skill names that must be loaded.

    Returns:
        list[str] | None: Expected names the init message does not list; None when it has no ``skills`` key.
    """
    if "skills" not in init_data:
        return None
    listed = {entry.get("name") if isinstance(entry, dict) else entry for entry in init_data["skills"] or []}
    return [name for name in expected if name not in listed]
