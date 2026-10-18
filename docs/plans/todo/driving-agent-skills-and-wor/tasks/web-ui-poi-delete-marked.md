---
title: Web UI POI delete marked
status: todo
repo: ros2-lerobot-sverve-platform
value: "The user deletes all marked POIs in one action after a confirmation that names the count and categories, with an extra confirmation when keep-outs are in the batch."
files: [nodes/web_ui/frontend/src/poi/DeleteMarkedDialog.tsx, nodes/web_ui/frontend/src/poi/DeleteMarkedDialog.test.tsx, nodes/web_ui/frontend/src/poi/PoiPanels.tsx, nodes/web_ui/README.md]
verify: "cd nodes/web_ui/frontend && npx vitest run src/poi/DeleteMarkedDialog.test.tsx src/poi/PoiPanels.mark.test.tsx src/poi/PoiPanels.test.tsx && npx tsc --noEmit && npm run lint"
depends_on: [web-ui-poi-list-filter-mark]
owns: []
uses: [poi-selection, poi-clear-filters]
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

# Web UI POI delete marked

## Requirement

- New `DeleteMarkedDialog.tsx` (U10 b, c): props `pois`, `marked`, `onDone`, `onClose`; the body comes from `summariseMarked`: "Delete N POIs?" with the per-category list (for example "3 target, 2 slow, 1 keepout"). Confirm is disabled at 0 marked. When `keepouts.length > 0` the dialog shows a warning naming the keep-outs and requires an extra, unchecked-by-default checkbox "I understand this removes N keep-out zone(s)" before Confirm enables (consistent with the existing rule that deleting a keep-out needs explicit consent). Confirm calls the EXISTING `clearPois(tab, { ids: [...marked] })` from `poi/api.ts` (owned by the named-maps epic task web-ui-clear-pois-dialog, route `POST /api/poi` op clear with `ids`, poi_store `op_clear` ids filter: poi-store-clear-filters). Do NOT add another API function, route or poi_store operation. The result "Removed n POIs" or the error is shown; on success the marks are cleared via `onDone`.
- `PoiPanels.tsx`: a "Delete marked (N)" button (disabled at 0) opens the dialog; after success the list refreshes through the existing poi list subscription.
- `nodes/web_ui/README.md`: documents delete marked and the extra keep-out confirmation.
- Tests first in new `DeleteMarkedDialog.test.tsx` with `clearPois` mocked, seen failing: the confirmation text shows the count and each category with its count; without keep-outs Confirm posts exactly the marked ids; with a keep-out Confirm stays disabled until the extra checkbox is ticked and the warning names the count; a keep-out without any tag is still caught; an error from clearPois is displayed and marks are kept; nothing marked disables the button.

## Acceptance conditions

- a1: the confirmation names the count and lists categories, and Confirm posts exactly the marked ids [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/DeleteMarkedDialog.test.tsx -t 'count|ids']
- a2: deleting keep-outs requires the extra confirmation, including untagged keep-out zones [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/DeleteMarkedDialog.test.tsx -t keepout]
- a3: an error is shown and marks are kept; zero marked disables delete [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/DeleteMarkedDialog.test.tsx -t 'error|disabled']
- a4: existing POI panel tests, types and lint pass [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/PoiPanels.test.tsx src/poi/PoiPanels.mark.test.tsx && npx tsc --noEmit && npm run lint]
- a5: README documents delete marked and the keep-out confirmation [evidence: command: grep -qi 'delete marked' nodes/web_ui/README.md]

## Files

- `nodes/web_ui/frontend/src/poi/DeleteMarkedDialog.tsx`
- `nodes/web_ui/frontend/src/poi/DeleteMarkedDialog.test.tsx`
- `nodes/web_ui/frontend/src/poi/PoiPanels.tsx`
- `nodes/web_ui/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
