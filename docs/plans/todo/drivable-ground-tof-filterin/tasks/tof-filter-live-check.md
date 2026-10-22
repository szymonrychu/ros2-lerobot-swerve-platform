---
title: ToF filter live check
status: todo
repo: ros2-lerobot-sverve-platform
value: "Proof on the robot that the retuned filter keeps the ground fit fed (removed share <= 10 %, plane valid >= 95 %)"
files: [nodes/bridges/arducam_tof/README.md]
verify: "cd nodes/bridges/arducam_tof && uv run poe lint"
depends_on: [capture-gated-recordings, tof-flying-pixel-retune]
owns: []
uses: [flying-pixel-retune, gated-fixtures]
model: ""
estimate: 30
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# ToF filter live check

## Requirement

Live task: prove on the robot that the retuned flying-pixel filter no longer starves the ground fit (design D27;
approval G2). It uses parts flying-pixel-retune and gated-fixtures.

Precondition, checked first and stopped on when not met (report `blocked`, `whoCanClear: user` or orchestrator):
- tof-flying-pixel-retune and tof-amplitude-gate are merged and the orchestrator has deployed arducam_tof (live step
  2) and recorded the deploy time (`date +%s`);
- capture-gated-recordings has run on this deploy (its `.logs/ground_recordings/*.ndjson` exist);
- the user: the scene in place, the light on, the robot static and charging; only the head moves (no arm, no drive).

- Wait at least 10 min after the deploy with the head at the home pose of `scripts/ground_record.yaml`.
- Query Mimir (`http://server.ros2.lan:9009/prometheus/api/v1/query`, the same Prometheus API
  `scripts/node_cpu_compare.py` uses) for the mean over the last 10 min of `rate(tof_flying_pixels_removed_total[1m])`
  and `rate(tof_amplitude_gated_pixels_total[1m])` on the client, with `curl` (no rclpy, no `ros2` CLI, no SSH).
- Removed share = rate / (ToF frame rate from the deployed arducam_tof config x 43,200 pixels). Acceptance: the
  flying-pixel removed share is <= 0.10 (my call, design D27; it was about 0.70) and the combined removed share is
  reported.
- The ground plane is valid in >= 95 % of the `/ground/state` lines of every gated recording's `.ndjson` (after the
  2 s warm-up), read with `scripts/ground_noise_report.py`'s reader.
- README `nodes/bridges/arducam_tof/README.md` `## Live check`: date, deploy time, the two rates, the shares, the
  plane-valid share per scene, PASS/FAIL per row. A FAIL row names the metric; the task then escalates with the
  numbers (the filter is retuned, never switched off silently).

## Acceptance conditions

- a1: the README records the live filter check with the measured rates and shares [evidence: doc-section:nodes/bridges/arducam_tof/README.md#Live check]
- a2: live, the flying-pixel filter removes <= 10 % of the pixels and the ground plane is valid in >= 95 % of the states of every gated recording, measured after the deploy through Mimir and the web_ui captures only [evidence: judgement]
- a3: node lint passes [evidence: command:cd nodes/bridges/arducam_tof && uv run poe lint]

## Files

- `nodes/bridges/arducam_tof/README.md`

## Interfaces

Uses part flying-pixel-retune (the deployed filter and its counter `tof_flying_pixels_removed_total`), the amplitude
gate counter `tof_amplitude_gated_pixels_total` and part gated-fixtures (the `.ndjson` captures of
capture-gated-recordings). Writes only the README `## Live check` section.

## Expected outcome

The README `## Live check` table shows the live removed shares (flying-pixel <= 0.10) and the plane-valid share
(>= 0.95) per gated scene, each row PASS, measured after the live step 2 deploy.

## Steps

- Check the preconditions (deploy time recorded, gated `.ndjson` files present, the user's scene and light); stop and
  report when one is not met.
- After 10 min settle, `curl -s 'http://server.ros2.lan:9009/prometheus/api/v1/query' --data-urlencode
  'query=avg_over_time(rate(tof_flying_pixels_removed_total[1m])[10m:1m])'` and the same for
  `tof_amplitude_gated_pixels_total`; expect one series each.
- Compute the shares and the plane-valid shares; write the README table; `cd nodes/bridges/arducam_tof && uv run poe
  lint` PASS; commit `docs(arducam_tof): live check of the retuned flying-pixel filter`.

## Notes

- Live task at live step 2 (design D29): needs the user (scene in place, light on, robot static and charging, head
  moves allowed). No rclpy, no `ros2` CLI, no SSH on the client (MEMORY feedback_no_rclpy_probes_on_client).
- The metric labels on Mimir follow the node's metrics port scrape (`tests/test_arducam_tof_config.py::
  test_metrics_port_is_scraped_exactly_when_the_node_is_enabled`); select the client series by its job/host label.
