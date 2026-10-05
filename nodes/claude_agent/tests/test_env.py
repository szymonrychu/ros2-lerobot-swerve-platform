"""Child process environment sanitization."""

from claude_agent.runner import build_child_env


def test_api_key_removed_and_oauth_kept() -> None:
    env = build_child_env(
        {"ANTHROPIC_API_KEY": "sk-x", "ANTHROPIC_AUTH_TOKEN": "t", "CLAUDE_CODE_OAUTH_TOKEN": "oauth", "PATH": "/bin"}
    )
    assert "ANTHROPIC_API_KEY" not in env
    assert "ANTHROPIC_AUTH_TOKEN" not in env
    assert env["CLAUDE_CODE_OAUTH_TOKEN"] == "oauth"
    assert env["DISABLE_AUTOUPDATER"] == "1"
    assert env["PATH"] == "/bin"


def test_input_not_mutated() -> None:
    base = {"ANTHROPIC_API_KEY": "sk-x"}
    build_child_env(base)
    assert base == {"ANTHROPIC_API_KEY": "sk-x"}
