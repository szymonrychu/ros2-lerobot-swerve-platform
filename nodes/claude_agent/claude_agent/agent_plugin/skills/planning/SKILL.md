---
name: planning
description: Use at the start of every job, before any other tool. Picks the workflow that fits the request (discovery, drive_to_target, pick_place) and seeds the task plan with agent.start_workflow, or builds a generous plan with agent.set_task_plan when no workflow fits.
---

# Plan the job

Plan before acting. A plan is accepted once; make it generous, verifiable and honest.

## 1. Choose the workflow from the request

- discovery: the request says explore, map, survey or look around (learn the surroundings, fill map gaps).
- drive_to_target: the request says go to, drive to, or find and go (reach a place, POI or object and confirm arrival).
- pick_place: the request says pick up, put down, bring, or move an object.
- Several fit: take the one for the main goal; add the others as phases (see 3).
- None fits: build the plan yourself with `agent.set_task_plan`.

## 2. Start the workflow

- Call `agent.start_workflow` with the workflow name and its params. It seeds the phases, caps, complexity and time budget.
- Use `dry_run: true` first when the request needs extra or reordered steps (a detour, an extra photo, a different order). Read the expanded tree and its mandatory step ids, edit the tree, then call `agent.start_workflow` again with `plan` set to the edited tree.
- Set the plan once. The tool refuses a second plan.

## 3. Breakdown rules (for edited and hand-built plans)

- Orient first: the first phase looks at the map, your pose and the surroundings (topdown view, camera) before anything moves.
- One phase per measurable goal. State the goal so that "done" can be checked from a sensor or an image, not from a feeling.
- Verification phase after every effector phase (drive, grasp, release, arm or head move): look with the camera and compare with the goal before moving on.
- Explicit branches: an `if` where the outcome is uncertain (target visible? grasp held?) and a `while` for searching (rotate and look again, next frontier), each with a max_iterations.
- Recovery branch: for each risky effector phase, a branch that runs when verification fails (re-approach, re-look, other side, report unreachable). See the recover-from-failure skill.
- Mandatory steps are never silently dropped. If you drop or replace one, say why in the rationale. Dropped mandatory steps are flagged at the end of the job.
- POI recording phases: wherever something is learned (a target, a hazard, a window, a place name), add a phase that records it with a category (poi-categories skill).
- Caps and time budget with margin: give each phase rw_cap and turn_cap above your honest estimate, and a time_budget_min about twice the estimate, within the ceiling. Raise them early with `agent.raise_phase_budget` instead of giving up.

## 4. When reality differs

- Call `agent.revise_plan` with a rationale when a phase failed for plan reasons or the situation changed. It can be used once, plus the one more allowed after the workflow check grant. Keep finished phases as they are and keep mandatory steps in the new remainder.
- Close each phase honestly with `agent.complete_phase`: done, failed or skipped.
