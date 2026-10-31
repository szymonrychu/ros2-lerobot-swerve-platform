---
title: Front and back Camera Module 3 on CSI
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-28
owner: ""
rung: ""
---

# Front and back Camera Module 3 on CSI

## What and why

What: Keep the CSI cameras: two Camera Module 3 as body-fixed front (cam0) and back (cam1) cameras
How: restore head_camera_cpp from history for two instances, imx708 overlays on both ports, static TF, MCP/web_ui/marker_detector on head/front/back names
Why: User decision 2026-10-28: keep CSI cameras alongside the D435 head, labelled front and back

## Scope ladder

### Rung 1: agreed scope (migrated)

- 1.1: Two Camera Module 3 (IMX708) run on the client Pi 5: `front` on CSI cam0, `back` on CSI cam1 (cam1 overlay arducam-pivariety -> imx708,cam1), fixed on the body (static TF from base_link), published as /front_camera/* and /back_camera/*.
- 1.2: head_camera_cpp (retired by the D435 epic) is restored from git history and generalised to one instance per camera (or one node serving both), deployed by Ansible with the ros2_nodes schema.
- 1.3: Camera names are head (D435) / front / back everywhere: MCP camera enum, web_ui, claude_agent prompt/skills/evals, topics; the D435 epic's "front" (= D435) becomes "head".
- 1.4: MCP agent images: get_camera_image and related image tools accept head, front, back.
- 1.5: web_ui shows live front and back streams.
- 1.6: marker_detector scans front and back as well as the D435.
- 1.7: READMEs, tests/README.md, MEMORY.md updated; all lint and tests green; deployed to the client.

## Agreed scope

not agreed yet

## Non-goals

- Zoom crops on the CSI cameras (user decision 2026-10-28: roles are MCP images, web_ui streams, marker detection).
- Mounting front/back on the pan/tilt head (they are body-fixed).
- Any change to the D435 head pipeline beyond the rename to "head".

## Design

none recorded in the epic format

## Decisions

none recorded in the epic format

## Open questions

none

## Research

none recorded in the epic format

## Irreversible operations

none

## Migrated notes

### README.md: Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.
