"""Short state digest carried into a fresh session (context policy ``fresh_with_digest``)."""

from typing import Any

LAST_TEXT_CHARS = 400


def build_digest(instruction: str, status: str, plan: dict[str, Any] | None, last_text: str, max_chars: int) -> str:
    """Summarise the previous instruction so a fresh session can continue from it.

    Args:
        instruction (str): The previous user instruction.
        status (str): How it ended (done, interrupted, ...).
        plan (dict[str, Any] | None): The plan payload (phases with name, status, summary) or None.
        last_text (str): The agent's last chat text of that instruction.
        max_chars (int): Maximum length of the result.

    Returns:
        str: The digest, at most max_chars long.
    """
    lines = [f"Previous instruction ({status}): {instruction.strip()}"]
    for phase in (plan or {}).get("phases", []):
        summary = f": {phase['summary']}" if phase.get("summary") else ""
        lines.append(f"- phase {phase['name']} [{phase['status']}]{summary}")
    if last_text.strip():
        lines.append(f"Your last message: {last_text.strip()[:LAST_TEXT_CHARS]}")
    return "\n".join(lines)[:max_chars]
