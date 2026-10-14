# Jetson depth offload: hardware deploy and e2e results

Date: 2026-10-14. Scope: minimum (disparity + depth on ROS; no monocular fill, no depthd-bench).

## Deploys

Jetson (`./scripts/deploy-nodes.sh jetson base build_deps depthd`, log .logs/deploy-jetson-hil.log):

    jetson-nano : ok=63 changed=5 unreachable=0 failed=0 skipped=25 rescued=0 ignored=0

Client (`./scripts/deploy-nodes.sh client jetson_link overview_camera stereo_depth stereo_depth_cpp jetson_bridge_cpp monitoring`, log .logs/deploy-client-hil.log):

- First run failed: network role asserted on the string "[]" (length 2) instead of an empty list. Fixed in commit "fix(network): parse the foreign-address probe fact as JSON before asserting" (test first).
- Second run: `client-rpi5 : ok=178 changed=17 unreachable=0 failed=0 skipped=146 rescued=0 ignored=6` (6 ignored = uninstall of units that were already gone).

## e2e (scripts/jetson_e2e_check.py, hosts overridden to szymonri@client.ros2.lan / jetson@192.168.1.32)

Final run:

    ok   jetson_health: state ok
    FAIL chrony: reference 10.77.0.1, offset 3.554 ms (need 10.77.0.1, < 1 ms)
    ok   rgb_rate: 10.01 Hz (need >= 10)
    ok   rgb_latency: p95 9.4 ms (need <= 150 ms)
    FAIL depth_rate: 0.13 Hz (need >= 10)
    FAIL depth_latency: p95 195.0 ms (need <= 150 ms)
    ok   bridge_cpu: 0.009 core (need <= 0.3)
    ok   retired_units: all absent

Earlier run (right after the client deploy): rgb 10.01 Hz p95 42.6 ms, depth 8.08 Hz p95 235 ms, bridge CPU 0.023 core, chrony offset 2.158 ms but referenced to a public pool server (Pi chrony had just restarted).

## State

- Calibration: present, /var/lib/depthd/stereo.yaml loaded at 19:29:56 (rms 0.353 px, baseline 0.0595 m). Board 27 mm squares, 19.8 mm markers.
- Bridge CPU on the Pi: 0.009-0.023 core.
- Chrony on the Jetson: reference 10.77.0.1 (Pi) after about 15 minutes of convergence; offset still 3.1-3.5 ms and falling; the < 1 ms check needs a re-run later.
- Depth: about 8 Hz in the first run (match stage averages 99 ms per frame, so SGM tops out near 10 Hz), then depthd's health went to "no depth frame" with 0.2 fps and many "busy" drops. depthd was not restarted after this (restart denied); needs a look.
- Not done: depthd-bench (T16 is future work).

## Open

1. Depth stalls to about 0.2 fps after running at about 8 Hz: investigate (restart depthd, check "busy" drops).
2. Depth rate < 10 Hz at 640x480 / 128 disparities: matcher choice is T16 (future).
3. Re-run `uv run python scripts/jetson_e2e_check.py` once chrony settles.
