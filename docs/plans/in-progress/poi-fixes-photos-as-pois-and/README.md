---
title: POI fixes photos as POIs and cleanups
blockers: [Driving agent skills and workflows, Pan tilt head control with agent and web UI]
stage: review
owner: Szymon Richert
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-18
---

# POI fixes photos as POIs and cleanups

## Project brief

What: Make photos first-class map POIs and make map and POI cleanups remove POIs and photos
How: poi_store photo/POI unification with map ownership, map_manager delete/reset fixes, web UI and MCP clear paths, test-first
Why: Map and POI cleanups leave stale POIs and photos behind, which misleads navigation and the agent

## Description

User-reported POI defects after the named-maps delivery (2026-10-18):

- "photos are also POIs" - robot photos (kind "photo", today a separate global store in index.jsonl outside poi.json, /poi/list and the map bundles; design D8) must be first-class POIs of the map they were taken on.
- "map cleanup didn't clean POIs (was supposed to)" - map_manager delete only clears the live POIs when active_map == name; active_map is None after a save or a restart, so save-then-delete leaves the POIs while reporting success. The web UI "Reset map" clears SLAM only (design D3).
- "POIs cleanup didn't delete photos" - POI delete, clear and replace only strip photo links; photo files and records stay, and clear filters cannot select photos.

## Goals

- Photos are POIs: listed, filtered, marked and deleted with the other POIs, owned by the map they were taken on, saved in and loaded from that map's bundle.
- Every map cleanup clears that map's POIs including photos: deleting a saved map (whatever the active-map state, including after save and after restart), resetting the live map, and the kidnap discard.
- Every POI cleanup deletes the photos it covers: single delete of a photo POI, clear by filter (photo kind selectable), delete-marked, replace; photo files on disk go with their records.
- No silent success: delete results report what was actually removed.

## Non-goals

- "New session" still deletes nothing (user rule U9).
- Saved map bundles are never deleted by kidnap or reset, only by an explicit map delete.
- Web UI zoom captures stay transient (not stored as photos).

## Time

Estimate: 855 min | Actual: 0 min | Remaining estimate: 0 min

## Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.

Verdict: pass (2026-10-18)

All 15 tasks verified and merged into main. Two stale tests fixed on main (named-maps prompt phrases, concurrent reset fake answering clear_live). Full suite green on main: root 1417, map_manager 172, poi_store 248, mcp_server 2678, claude_agent 931, web_ui 879, frontend 1017, lint, test-ansible. Client deployed (--all, failed=0). Live hand checks (save/load/delete a map with photos, Reset map clearing POIs and photos) are manual checks for the user.
