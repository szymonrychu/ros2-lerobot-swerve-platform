---
title: Skills suite and runbook
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "Everything is green together and the skills, workflows and retrospectives go live without killing an agent session."
files: [tests/test_agent_skills_docs.py, tests/README.md, nodes/README.md, MEMORY.md]
verify: "uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && (cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/poi_store && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/costmap_zones && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build)"
depends_on: [eval-workflow-testset, prompt-skills-and-workflows, web-ui-poi-category-chips, costmap-verify-slow-cost]
owns: []
uses: []
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Skills suite and runbook

## Requirement

- New `tests/test_agent_skills_docs.py`, written first and seen failing until the docs are complete: claude_agent README has `## Skills`, `## Workflows`, `## Retrospectives`; every skill directory is listed in the README Skills section; every workflow template name appears in the Workflows section; `nodes/README.md` mentions skills, workflows and retrospectives in the claude_agent line and slow-zone cost in the costmap_zones line; `MEMORY.md` records the decision (skills as a read-only plugin, workflows via start_workflow, retrospectives store + feed, POI categories as note tags, slow zones costed through the keepout mask). `tests/README.md` lists the file.
- The full verify command passes; the report quotes every summary line (root pytest, lint, lint-nodes, test-ansible, claude_agent, mcp_server, poi_store, costmap_zones, web_ui, frontend).
- The runbook below is complete; the orchestrator executes it after merge (not the implementer).

Orchestrator runbook (after merge, robot reachable; every deploy through the ansible-deploy skill and `scripts/deploy-nodes.sh`, logged to `.logs/`):
1. Deploy `./scripts/deploy-nodes.sh client costmap_zones poi_store nav2_bringup web_ui 2>&1 | tee .logs/deploy-client-skills-1.log`; run `scripts/costmap_zones_verify.sh` and expect every step PASS including the slow step (costmap value 59 at mask value 60).
2. Check the agent is idle: `GET /api/agent/state` busy false and no instruction running (MEMORY: deploying claude_agent/mcp_server kills a running session, exit 143). If busy, ask the user before continuing.
3. Deploy `./scripts/deploy-nodes.sh client mcp_server claude_agent 2>&1 | tee .logs/deploy-client-skills-2.log`; check the claude_agent journal (via the deploy log or an Ansible ad-hoc command, no manual SSH) for the init check: no "skills not loaded" error; an Ansible ad-hoc `stat` of `/opt/ros2-lerobot-swerve-platform/nodes/claude_agent/claude_agent/agent_plugin` shows root ownership and no group/other write bit.
4. Discovery job from the web UI Agent tab: "Look around this room and map it" (base only; no arm motion): the plan shows workflow discovery with mandatory steps; survey POIs appear with category chips; get_exploration_status is called; the job ends with a summary; a retrospective appears in the Retros tab.
5. Drive-to-target job: create a POI "test spot" 1 m ahead in free space, then "Drive to the test spot": workflow drive_to_target, verify_arrival done, retrospective in the Retros tab.
6. `uv run python scripts/agent_retrospectives.py` writes both retrospectives to `.logs/retrospectives/`.
7. "New session" in the web UI: record the POI count and map before and after; every POI (survey, user, keep-out) and the map stay.
7b. POI list in the web UI: filter by one category, "Mark all visible", check the count in "Delete marked", confirm, only those POIs disappear; marking a keep-out demands the extra confirmation.
8. Optional when the user agrees: `uv run python -m scripts.eval.run scripts/eval/run.yaml` with `testset: scripts/eval/testset_workflows_v1.yaml`, mode full.
9. If the robot is offline, report steps 1-8 as pending on the robot with everything else done.

## Acceptance conditions

- a1: docs cover skills, workflows, retrospectives and MEMORY records the decision [evidence: command: uv run pytest tests/test_agent_skills_docs.py -q]
- a2: every suite and lint is green [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && (cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/poi_store && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/costmap_zones && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build)]
- a3: the runbook stages the deploy, the idle check, the init check, the read-only check, one discovery and one drive-to-target job, the Retros tab, the script, New session (no POI or map deleted) and POI list delete marked [evidence: judgement]

## Files

- `tests/test_agent_skills_docs.py`
- `tests/README.md`
- `nodes/README.md`
- `MEMORY.md`

## Interfaces

- `tests/test_agent_skills_docs.py`.

## Expected outcome

All suites green; the orchestrator brings the feature live with the runbook in the requirement.

## Steps

- Write `tests/test_agent_skills_docs.py`; run it; expect failures until docs are complete.
- Complete the docs and MEMORY.md; run the full verify command and quote every summary line; commit `test: skills, workflows and retrospectives docs check`.

## Notes

- Deploy only via scripts/deploy-nodes.sh; never deploy claude_agent/mcp_server while an instruction runs.
- Survey motions: base rotation only; no arm motion in the runbook jobs (CLAUDE.md least-dramatic rule).
