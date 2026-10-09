"""State digest carried into a fresh session."""

from claude_agent.digest import build_digest


def test_digest_lists_instruction_outcome_phases_and_last_text() -> None:
    plan = {
        "phases": [
            {"name": "Locate", "status": "done", "summary": "found"},
            {"name": "Drive", "status": "failed", "summary": "blocked"},
        ]
    }
    text = build_digest("find tomato", "done", plan, "It is left.", 1000)
    assert "find tomato" in text and "done" in text
    assert "Locate" in text and "found" in text and "Drive" in text and "failed" in text
    assert "It is left." in text


def test_digest_without_plan_and_truncated() -> None:
    text = build_digest("x" * 50, "interrupted", None, "y" * 500, 120)
    assert len(text) <= 120
    assert text.startswith("Previous instruction")
