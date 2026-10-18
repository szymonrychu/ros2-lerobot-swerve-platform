---
name: retrospective
description: Use when the RETROSPECTIVE message asks you to look back at the job that just ended. Writes an honest, specific record of what went well, what went wrong, what could be better and notes for the agents who fix the robot, then stores it.
---

# Write the retrospective

The record is stored on the robot and shown to the developers and to future jobs. Honesty beats self-praise: a failure stated plainly helps; a flattering summary hides the problem.

## Sections

1. What went well: only what really worked and is worth repeating (a tool sequence, a recovery that held).
2. What went wrong: every failure, refusal, timeout, retry, wrong guess or skipped mandatory step, in plain words. Include the error text.
3. What could be better: concrete changes to procedure, plan, budgets, skills or tools that would have avoided the problem.
4. Notes for future fixing agents: what a developer needs to reproduce and fix it. Name the tools involved, quote the error texts, give the coordinates of problem spots (map x, y), and state the suspected cause and how sure you are.

## Short form for trivial jobs

When the job was trivial (a single look, a quick answer, nothing failed), write one line of summary and add only what went wrong, if anything. Do not pad empty sections.

## Store it

Call `agent.record_retrospective` once with outcome (done, partial or failed), the summary (at most 500 characters) and the lists went_well, went_wrong, improvements and notes_for_fixers (up to 10 items of 300 characters each; omit empty ones). A second call in the same instruction replaces the first narrative. Then stop; do not start new work.
