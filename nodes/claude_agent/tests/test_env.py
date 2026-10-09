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


def test_cache_ttl_and_autocompact_env_from_config() -> None:
    from claude_agent.config import ClaudeAgentConfig

    env = build_child_env({"PATH": "/bin"}, ClaudeAgentConfig(prompt_cache_ttl="1h", autocompact_pct=55))
    assert env["CLAUDE_CODE_PROMPT_CACHE_TTL"] == "1h"
    assert env["CLAUDE_AUTOCOMPACT_PCT_OVERRIDE"] == "55"


def test_empty_cache_ttl_and_other_policies_set_no_env() -> None:
    from claude_agent.config import ClaudeAgentConfig

    env = build_child_env({}, ClaudeAgentConfig(prompt_cache_ttl="", context_policy="fresh_with_digest"))
    assert "CLAUDE_CODE_PROMPT_CACHE_TTL" not in env
    assert "CLAUDE_AUTOCOMPACT_PCT_OVERRIDE" not in env
