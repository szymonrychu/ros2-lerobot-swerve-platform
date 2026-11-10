"""Speed budgets of the mcp_server tests, set on the dev Mac (MacBook M4).

Budget tests measure the CPU time of the thread doing the work (time.thread_time), not the wall time: a loaded or
CPU-throttled machine (parallel suites, a container's CPU quota) stretches the wall time of the same work several fold
while the CPU time stays. The budgets still assume a dev Mac or GitHub runner class core: on a slower host (the RPi 5,
a shared x86 build pod) set MCP_SPEED_SKIP=1 to skip them. CI does not set it, so they run there.
"""

import os

import pytest

SKIP_SPEED = os.environ.get("MCP_SPEED_SKIP") == "1"
speed_budget = pytest.mark.skipif(SKIP_SPEED, reason="MCP_SPEED_SKIP=1: the budgets assume a dev Mac or CI runner core")
