---
title: Front and back Camera Module 3 on CSI
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-28
owner: ""
rung: "4"
---

# Front and back Camera Module 3 on CSI

## What and why

What: Keep the CSI cameras: two Camera Module 3 as body-fixed front (cam0) and back (cam1) cameras
How: restore head_camera_cpp from history for two instances, imx708 overlays on both ports, static TF, MCP/web_ui/marker_detector on head/front/back names
Why: User decision 2026-10-28: keep CSI cameras alongside the D435 head, labelled front and back

## Status 2026-10-31

Still relevant: a user decision of 2026-10-28 (docs/research/2026-10-27-d435-user-decisions.md, "2026-10-28 decisions
(CSI epic)"), queued after the D435 code steps. Not superseded by the Jetson removal (the cameras are on the client
Pi). It conflicts with D435 S75 (removes the imx708,cam0 overlay) and S78 (deletes head_camera_cpp): D435 Q8.

## Scope ladder

Rungs are cumulative: each rung includes everything above it.

### Rung 1: MVP - front camera and the rename

- 1.1: Camera Module 3 (IMX708) on CSI cam0 runs as the body-fixed `front` camera: head_camera_cpp kept or restored as
  one instance, static TF from base_link, /front_camera/* topics, deployed by Ansible (ros2_nodes schema).
- 1.2: The D435 is renamed `head` everywhere (MCP camera enum, claude_agent prompt and skills, web_ui labels);
  get_camera_image accepts head and front.
- 1.3: Tests, READMEs, tests/README.md and MEMORY.md; lint and tests green; deployed.

### Rung 2: + back camera

- 2.1: A second Camera Module 3 on cam1 (overlay arducam-pivariety -> imx708,cam1) as body-fixed `back`, second
  instance, /back_camera/*, MCP `back`.

### Rung 3: + web_ui streams

- 3.1: web_ui shows live front and back streams.

### Rung 4: complete - markers, agent and live proof (recommended, agreed)

- 4.1: marker_detector scans front and back as well as the D435.
- 4.2: claude_agent skills and evals use head/front/back; measured CPU and memory of the two cameras on the client
  (cgroup values from Mimir); live check on the robot.

## Agreed scope

Rung 4 (complete), all items 1.1-4.2. The user's gate answer, verbatim: "Go" (accepting recommendations: rung 4
complete; Q16 yes run after the D435 code steps, reusing head_camera_cpp and the cam0 overlay; Q17 yes second CM3
fitted on cam1 in place of the ToF before rung 2 deploys, until then rung 2 stops at deploy; Q18 yes front/back static
TF from the user's tape measurement, no CAD guess; Q19 yes both cameras use the old 1152x648 compressed profile, no
zoom). The gate's Q16-Q19 are this file's Q1-Q4. Orchestrator binding: the TF values come from the user's tape
measurement, so the code takes them from config with no default guess and the plan names where the user enters them;
deploys, the cam1 overlay and reboot, and live checks run after merge with the user.

## Non-goals

- Zoom crops on the CSI cameras (user decision 2026-10-28: roles are MCP images, web_ui streams, marker detection).
- Mounting front/back on the pan/tilt head (they are body-fixed).
- Any change to the D435 head pipeline beyond the rename to "head".

## Design

none recorded in the epic format

## Decisions

none recorded in the epic format

## Open questions

- Q1: Run this plan after realsense-d435-head-switchov's code steps, keeping head_camera_cpp and the cam0 overlay rather than deleting and restoring them (D435 Q8)? [answer: yes; the D435 code steps are merged, head_camera_cpp and the imx708,cam0 overlay are kept]
- Q2: The second Camera Module 3 is (or will be) fitted on cam1 in place of the ToF before rung 2 is deployed; until then rung 2 ships code and stops at deploy? [answer: yes]
- Q3: The front and back mount poses for the static TF come from the user's tape measurement of the mounted cameras (no CAD guess published)? [answer: yes; config only, no default value]
- Q4: Both cameras reuse the retired head_camera_cpp profile (1152x648 compressed, no zoom)? [answer: yes]

## Research

none recorded in the epic format

## Irreversible operations

- Client boot config change (cam1 overlay arducam-pivariety -> imx708,cam1) and the reboot it triggers. Safeguard: the
  reboot-only-on-change rule of ansible/playbooks/tasks/pi_camera_boot_config.yml; the old line is in git.

## Migrated notes

### README.md: Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.
