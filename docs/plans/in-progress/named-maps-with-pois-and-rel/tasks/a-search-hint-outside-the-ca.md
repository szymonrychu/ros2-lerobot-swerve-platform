---
title: A search hint outside the candidate window can return a wrong pose with ratio 1.00 over only 126 of about 330 points and no ambiguity flag.
status: done
repo: ros2-lerobot-sverve-platform
value: "Measured on the live scan with hint (3.0, 2.0, 1.0). The post-load agreement check is the only guard against it. Consider requiring a minimum evaluated fraction of the scan for ok."
files: []
verify: ""
depends_on: []
owns: []
uses: []
model: ""
estimate: ""
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# A search hint outside the candidate window can return a wrong pose with ratio 1.00 over only 126 of about 330 points and no ambiguity flag.

## Requirement

A search hint outside the candidate window can return a wrong pose with ratio 1.00 over only 126 of about 330 points and no ambiguity flag.

Where: nodes/map_manager/map_manager/relocalise.py search_candidates

Why: Measured on the live scan with hint (3.0, 2.0, 1.0). The post-load agreement check is the only guard against it. Consider requiring a minimum evaluated fraction of the scan for ok.

## Acceptance conditions

- AC1: A search hint outside the candidate window can return a wrong pose with ratio 1.00 over only 126 of about 330 points and no ambiguity flag. at nodes/map_manager/map_manager/relocalise.py search_candidates is resolved [evidence: judgement]

## Files

- (none)

## Interfaces


Keep the interfaces around nodes/map_manager/map_manager/relocalise.py search_candidates unchanged unless resolving it requires otherwise: A search hint outside the candidate window can return a wrong pose with ratio 1.00 over only 126 of about 330 points and no ambiguity flag..

## Expected outcome


A search hint outside the candidate window can return a wrong pose with ratio 1.00 over only 126 of about 330 points and no ambiguity flag. at nodes/map_manager/map_manager/relocalise.py search_candidates is resolved, and Measured on the live scan with hint (3.0, 2.0, 1.0). The post-load agreement check is the only guard against it. Consider requiring a minimum evaluated fraction of the scan for ok. no longer applies.

## Steps


1. Write a failing test that shows: A search hint outside the candidate window can return a wrong pose with ratio 1.00 over only 126 of about 330 points and no ambiguity flag..
2. Fix it at nodes/map_manager/map_manager/relocalise.py search_candidates.
3. Run the task verify command and confirm it passes.

## Notes


Discovered in task `final-suite-and-runbook` during run `named-maps-with-pois-and-rel`.


## Resolution

Superseded by task relocalise-requires-a-minimu (written by the orchestrator with ACs).
