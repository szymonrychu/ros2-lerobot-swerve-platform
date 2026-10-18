---
title: Driving agent skills and workflows
blockers: [Remove the Jetson coprocessor]
stage: build
owner: Szymon Richert
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-18
---

# Driving agent skills and workflows

## Project brief

What: Survey, retrospective, sightseeing and planning skills plus discovery, drive-to-target and pick/place workflows for the driving agent
How: Read-only Agent SDK skills in claude_agent, start_workflow plan seeding, POI category mapping, Nav2 slow-zone path cost, exploration status, retrospective store + web tab
Why: The agent needs repeatable, thorough procedures and memory of what went wrong

## Description

The driving agent (nodes/claude_agent, Claude Agent SDK, acting through nodes/mcp_server tools) gets dedicated skills and workflows. Skills: survey (rotate the robot - later the head - and record every POI: task targets, slow/risky zones the planner takes into account, keep-out zones it must avoid at all costs, problematic spots such as windows, mirrors, tight places and obstacles above the lidar plane, and details useful for future requests), retrospective (what went right/wrong, what could be better, notes for future fixing agents), sightseeing (guides the agent through filling map gaps until the surroundings are completely mapped and surveyed), planning (generous with required steps), and helper skills the workflows need. Workflows - discovery, driving to a target, picking up / putting down - scaffold the task plan of a job the agent performs.

## Goals

- Skills are Agent SDK skill files shipped read-only inside the claude_agent node (loaded on demand; the agent cannot edit them); runner options enable skills for the agent while keeping the agent's file tools away from the skill directory.
- POI categories are mapped onto the existing POI kinds/zones and fields (no new schema field): targets, slow zones, keep-out zones, problem spots, details - documented mapping, tools/validation so the agent records each category consistently; creating a keep-out is free, removing one still needs user consent as today.
- Slow (risky) zones add path cost in Nav2 (the planner routes around them when possible) and cap speed inside; keep-out zones stay hard (verify the Nav2 Jazzy filter behaviour and choose the mechanism).
- Survey skill: base rotation now, automatically using head rotation, zoom and the 3D map when those tools exist; records POIs of all categories with photos/notes where available.
- Sightseeing skill + an exploration-status tool (reachable frontiers, surveyed areas): done when no reachable frontier is left and every frontier area has been surveyed; produces a summary for the user; stops at configurable time and battery limits.
- Planning skill (generous step breakdown) and a start_workflow tool that seeds the job's task plan from a template; templates for discovery, drive-to-target and pick-up/put-down; the agent may add/reorder steps; the job-end check flags skipped mandatory steps. Jobs are started by free text only (the agent selects the workflow).
- Retrospective skill: written after every job (short for trivial jobs) to a robot-side store; shown in a web UI tab and available through an API/script; recent retrospectives are fed to future agents.
- "New session" deletes no POIs or maps (neither survey POIs nor keep-outs); it only resets the agent conversation and job state. POIs are removed only through explicit buttons and tools (U11).
- Web UI POI list: filter by category, mark POIs individually or "mark all visible", and "delete marked" in one confirmed action (extra confirmation for keep-outs), through the shared POI delete operation owned by the named-maps epic (U10); this is the way survey POIs are removed.
- Tests (claude_agent, mcp_server, poi_store, web_ui), evals extended beyond pick-and-place where cheap, READMEs; deployed (respecting the rule that deploying claude_agent/mcp_server kills a running agent session).

## Non-goals

- Web UI workflow buttons (jobs start from free text only).
- New perception hardware features (head, zoom, 3D map are used when their epics land, not built here).

## Time











Estimate: 1535 min | Actual: 0 min | Remaining estimate: 1535 min

## Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.
