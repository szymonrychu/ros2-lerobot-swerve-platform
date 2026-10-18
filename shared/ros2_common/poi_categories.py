"""POI category tags carried in the existing POI ``note`` field (no ROS imports).

poi_store drops unknown fields, so a category is a tag on the first line of the note: ``#target``, ``#slow``,
``#keepout``, ``#detail`` or ``#problem/<type>``, plus optional ``#survey`` and ``#survey-spot``. The rest of the
note is free text. mcp_server, web_ui and the agent skills all read and write categories through this module.
"""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict

CATEGORIES = ("target", "slow", "keepout", "problem", "detail")
# overhang and low_obstacle are obstacles above or below the lidar plane
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
SURVEY_TAG = "#survey"
SURVEY_SPOT_TAG = "#survey-spot"
MAX_NOTE_LEN = 2000
PROBLEM_PREFIX = "#problem/"

KINDS_BY_CATEGORY = {
    "target": ("point", "object", "area"),
    "slow": ("area",),
    "keepout": ("area",),
    "problem": ("point", "area"),
    "detail": ("point", "area"),
}
ZONES_BY_CATEGORY = {
    "target": ("none",),
    "slow": ("slow",),
    "keepout": ("keepout",),
    "problem": ("none", "slow"),
    "detail": ("none",),
}
SURVEY_TAGS = (SURVEY_TAG, SURVEY_SPOT_TAG)


class PoiTags(BaseModel):
    """Tags parsed from the first line of a POI note.

    Attributes:
        category: One of CATEGORIES or None when the line has no category tag.
        problem_type: One of PROBLEM_TYPES for category "problem", else None.
        survey: True when ``#survey`` is present.
        survey_spot: True when ``#survey-spot`` is present.
    """

    model_config = ConfigDict(frozen=True)

    category: str | None = None
    problem_type: str | None = None
    survey: bool = False
    survey_spot: bool = False


def category_tag(token: str) -> bool:
    """Tell whether a token is a category or problem tag.

    Args:
        token: One whitespace-separated word.

    Returns:
        bool: True for ``#target`` style and ``#problem/<known type>`` tokens.
    """
    if token.startswith(PROBLEM_PREFIX):
        return token[len(PROBLEM_PREFIX) :] in PROBLEM_TYPES
    return token.startswith("#") and token[1:] in CATEGORIES and token != "#problem"


def known_tag(token: str) -> bool:
    """Tell whether a token is any tag this module owns.

    Args:
        token: One whitespace-separated word.

    Returns:
        bool: True for category, problem and survey tags.
    """
    return category_tag(token) or token in SURVEY_TAGS


def split_first_line(note: str) -> tuple[list[str], str]:
    """Split a note into the first line's tokens and the remaining lines.

    Args:
        note: The POI note.

    Returns:
        tuple[list[str], str]: Tokens of the first line, and the text after the first newline ("" if none).
    """
    first, _, rest = note.partition("\n")
    return first.split(), rest


def category_tags(
    category: str | None,
    problem_type: str | None = None,
    survey: bool = False,
    survey_spot: bool = False,
) -> list[str]:
    """Build the tag list for a category.

    Args:
        category: One of CATEGORIES, or None for survey tags only.
        problem_type: Required for "problem", forbidden for every other category.
        survey: Add ``#survey``.
        survey_spot: Add ``#survey-spot``.

    Returns:
        list[str]: Tags, category first.

    Raises:
        ValueError: Unknown category or problem type, or problem_type used wrongly.
    """
    tags: list[str] = []
    if category is None:
        if problem_type is not None:
            raise ValueError("problem_type needs category 'problem'")
    elif category not in CATEGORIES:
        raise ValueError(f"unknown category {category!r}; valid: {', '.join(CATEGORIES)}")
    elif category == "problem":
        if problem_type is None:
            raise ValueError(f"category 'problem' needs a problem_type; valid: {', '.join(PROBLEM_TYPES)}")
        if problem_type not in PROBLEM_TYPES:
            raise ValueError(f"unknown problem_type {problem_type!r}; valid: {', '.join(PROBLEM_TYPES)}")
        tags.append(PROBLEM_PREFIX + problem_type)
    else:
        if problem_type is not None:
            raise ValueError(f"problem_type is only allowed for category 'problem', not {category!r}")
        tags.append(f"#{category}")
    if survey:
        tags.append(SURVEY_TAG)
    if survey_spot:
        tags.append(SURVEY_SPOT_TAG)
    return tags


def parse_tags(note: str) -> PoiTags:
    """Read the tags on the first line of a note.

    Args:
        note: The POI note.

    Returns:
        PoiTags: Parsed tags; unknown words are ignored, no known tag gives an empty PoiTags.
    """
    category: str | None = None
    problem_type: str | None = None
    tokens, _ = split_first_line(note)
    for token in tokens:
        if category is None and category_tag(token):
            if token.startswith(PROBLEM_PREFIX):
                category, problem_type = "problem", token[len(PROBLEM_PREFIX) :]
            else:
                category = token[1:]
    return PoiTags(
        category=category,
        problem_type=problem_type,
        survey=SURVEY_TAG in tokens,
        survey_spot=SURVEY_SPOT_TAG in tokens,
    )


def apply_tags(note: str, tags: list[str]) -> str:
    """Put tags on the first line of a note.

    Category and problem tags already on the line are replaced; survey tags already present stay; no tag is
    duplicated; other first-line words and the later lines are kept. A note without any known tag gets a new
    tag line in front.

    Args:
        note: The POI note.
        tags: Tags to set, usually from category_tags.

    Returns:
        str: The new note.

    Raises:
        ValueError: The result is longer than MAX_NOTE_LEN (message names the length).
    """
    tokens, rest = split_first_line(note)
    if any(known_tag(t) for t in tokens):
        has_rest = "\n" in note
    else:
        tokens, rest, has_rest = [], note, bool(note)
    merged = list(dict.fromkeys(tags))
    merged += [t for t in tokens if not category_tag(t) and t not in merged]
    line = " ".join(merged)
    result = f"{line}\n{rest}" if has_rest else line
    if len(result) > MAX_NOTE_LEN:
        raise ValueError(f"note would be {len(result)} characters, over the limit of {MAX_NOTE_LEN}")
    return result


def strip_tags(note: str) -> str:
    """Return the free text of a note.

    Args:
        note: The POI note.

    Returns:
        str: The note without known tags; unknown first-line words and later lines stay.
    """
    tokens, rest = split_first_line(note)
    if not any(known_tag(t) for t in tokens):
        return note
    line = " ".join(t for t in tokens if not known_tag(t))
    return f"{line}\n{rest}" if line and "\n" in note else (line or rest)


def has_tag(note: str, tag: str) -> bool:
    """Tell whether the first line of a note holds a tag.

    Args:
        note: The POI note.
        tag: The tag, e.g. ``#survey`` or ``#problem/window``.

    Returns:
        bool: True when the tag is on the first line.
    """
    return tag in split_first_line(note)[0]


def check_category(
    category: str,
    kind: str,
    zone: str,
    problem_type: str | None,
    speed_limit_pct: int | None,
) -> None:
    """Check a category against the POI kind, zone and fields it maps onto.

    Args:
        category: One of CATEGORIES.
        kind: POI kind: "point", "area" or "object".
        zone: POI zone: "none", "keepout" or "slow".
        problem_type: One of PROBLEM_TYPES for "problem", None otherwise.
        speed_limit_pct: Speed cap in percent, required for category "slow".

    Raises:
        ValueError: The combination breaks a rule; the message names the rule.
    """
    if category not in CATEGORIES:
        raise ValueError(f"unknown category {category!r}; valid: {', '.join(CATEGORIES)}")
    if category == "problem":
        if problem_type is None or problem_type not in PROBLEM_TYPES:
            raise ValueError(f"category 'problem' needs a problem_type from: {', '.join(PROBLEM_TYPES)}")
    elif problem_type is not None:
        raise ValueError(f"problem_type is only allowed for category 'problem', not {category!r}")
    kinds = KINDS_BY_CATEGORY[category]
    if kind not in kinds:
        raise ValueError(f"category {category!r} needs kind {' or '.join(kinds)}, not {kind!r}")
    zones = ZONES_BY_CATEGORY[category]
    if zone not in zones:
        raise ValueError(f"category {category!r} needs zone {' or '.join(zones)}, not {zone!r}")
    if category == "slow" and speed_limit_pct is None:
        raise ValueError("category 'slow' needs speed_limit_pct")
