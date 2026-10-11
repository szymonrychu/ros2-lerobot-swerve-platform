"""Config/token errors in the mcp_server and claude_agent mains exit os.EX_CONFIG (78), not restarted."""

import ast
import os
from pathlib import Path

import pytest

NODES = Path(__file__).resolve().parent.parent / "nodes"
MAINS = [NODES / "mcp_server" / "mcp_server" / "__main__.py", NODES / "claude_agent" / "claude_agent" / "__main__.py"]


def main_returns(path: Path) -> list[ast.expr]:
    """Collect the return values of the module-level main().

    Args:
        path (Path): Source file of the entry point.

    Returns:
        list[ast.expr]: The expression of every `return` directly in main().
    """
    tree = ast.parse(path.read_text())
    main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "main")
    return [n.value for n in ast.walk(main) if isinstance(n, ast.Return) and n.value is not None]


@pytest.mark.parametrize("path", MAINS, ids=lambda p: p.parent.name)
def test_main_returns_ex_config_not_1_for_config_errors(path: Path) -> None:
    values = main_returns(path)
    assert os.EX_CONFIG == 78
    assert not [v for v in values if isinstance(v, ast.Constant) and v.value == 1]
    assert [v for v in values if ast.unparse(v) == "os.EX_CONFIG"]
