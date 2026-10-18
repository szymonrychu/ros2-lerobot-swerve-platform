---
title: Web UI retros tab
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The user reads every job's retrospective in a Retros tab."
files: [nodes/web_ui/frontend/src/tabs/RetrosTab.tsx, nodes/web_ui/frontend/src/tabs/RetrosTab.test.tsx, nodes/web_ui/frontend/src/agent/agentApi.ts, nodes/web_ui/frontend/src/App.tsx, nodes/web_ui/frontend/src/tabSelection.ts, nodes/web_ui/frontend/src/tabSelection.test.ts, nodes/web_ui/README.md]
verify: "cd nodes/web_ui/frontend && npx vitest run src/tabs/RetrosTab.test.tsx src/tabSelection.test.ts && npx tsc --noEmit && npm run lint"
depends_on: [web-ui-retros-proxy, retro-api-endpoint]
owns: []
uses: [retro-api]
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

# Web UI retros tab

## Requirement

- `agentApi.ts` (D10): `fetchRetrospectives(limit?: number, before?: number): Promise<RetrospectivesPage>` against `/api/agent/retrospectives`, typed to the claude_agent JSON (Retrospective fields as in the API).
- New `tabs/RetrosTab.tsx`: lists records newest first: local date/time, workflow (or "free"), outcome chip (colour per outcome), author marker for runner records, summary; a row expands to went well, went wrong, improvements, notes for fixers, skipped mandatory steps (highlighted), phases with outcomes, cost/turns/duration; Refresh button; auto-refresh every 30 s while the tab is visible; "Load older" using `before`; empty state "No retrospectives yet"; error state with the message. Existing MUI components and theme tokens.
- `App.tsx`: `agent_retros` case renders RetrosTab, with an icon; `tabSelection.ts`: `agent_retros` in SUPPORTED_TAB_TYPES.
- `nodes/web_ui/README.md`: the Retros tab described.
- Tests first in `tabs/RetrosTab.test.tsx` and `tabSelection.test.ts`, seen failing: renders a list from a mocked fetch; expands a row to show sections and skipped steps; empty and error states; Load older passes before; agent_retros is a supported tab type.

## Acceptance conditions

- a1: RetrosTab lists retrospectives and expands sections including skipped mandatory steps [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/tabs/RetrosTab.test.tsx]
- a2: agent_retros is a supported tab type [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/tabSelection.test.ts]
- a3: types and lint pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint]
- a4: README describes the Retros tab [evidence: command: grep -qi 'retros tab' nodes/web_ui/README.md]

## Files

- `nodes/web_ui/frontend/src/tabs/RetrosTab.tsx`
- `nodes/web_ui/frontend/src/tabs/RetrosTab.test.tsx`
- `nodes/web_ui/frontend/src/agent/agentApi.ts`
- `nodes/web_ui/frontend/src/App.tsx`
- `nodes/web_ui/frontend/src/tabSelection.ts`
- `nodes/web_ui/frontend/src/tabSelection.test.ts`
- `nodes/web_ui/README.md`

## Interfaces

- `agentApi.ts:fetchRetrospectives`; `tabs/RetrosTab.tsx`.

## Expected outcome

A Retros tab shows the robot's retrospectives.

## Steps

- Write the tests; run `cd nodes/web_ui/frontend && npx vitest run src/tabs/RetrosTab.test.tsx`; expect failures.
- Implement; run the verify command; commit `feat(web_ui): Retros tab`.

## Notes

- Look at AgentChatTab and PhasePanel for the house style and fetch patterns.
