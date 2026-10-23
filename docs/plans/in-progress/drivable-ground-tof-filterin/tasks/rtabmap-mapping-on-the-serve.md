---
title: "rtabmap_mapping on the server logged 'Did not receive data since 5 seconds' for /uplink/scan_filtered and the /uplink/head_rgbd streams the whole time around 16:30 (Wi-Fi up), while map3d_uplink was sending about 100 KB/s to the server"
status: done
repo: ros2-lerobot-sverve-platform
value: The map3d uplink seems not to be feeding RTAB-Map even while the link is up.
files: []
verify: ""
depends_on: []
owns: []
uses: []
model: ""
estimate: ""
actual: ""
review: ""
branch: main
verified_head: 8c90b001886329e16d37c81b7554295d3af5ca7f
run: drivable-ground-tof-filterin
provides: []
acs_for_audit: [AC1]
process_gaps: []
---

# rtabmap_mapping on the server logged 'Did not receive data since 5 seconds' for /uplink/scan_filtered and the /uplink/head_rgbd streams the whole time around 16:30 (Wi-Fi up), while map3d_uplink was sending about 100 KB/s to the server

## Requirement

rtabmap_mapping on the server logged 'Did not receive data since 5 seconds' for /uplink/scan_filtered and the /uplink/head_rgbd streams the whole time around 16:30 (Wi-Fi up), while map3d_uplink was sending about 100 KB/s to the server

Where: server ros2-rtabmap_mapping / client map3d_uplink

Why: The map3d uplink seems not to be feeding RTAB-Map even while the link is up.

## Acceptance conditions

- AC1: rtabmap_mapping on the server logged 'Did not receive data since 5 seconds' for /uplink/scan_filtered and the /uplink/head_rgbd streams the whole time around 16:30 (Wi-Fi up), while map3d_uplink was sending about 100 KB/s to the server at server ros2-rtabmap_mapping / client map3d_uplink is resolved [evidence: judgement]

## Files

- `nodes/map3d_uplink/README.md`
- `ansible/group_vars/server.yml`

## Interfaces

Keep the interfaces around server ros2-rtabmap_mapping / client map3d_uplink unchanged unless resolving it requires otherwise: rtabmap_mapping on the server logged 'Did not receive data since 5 seconds' for /uplink/scan_filtered and the /uplink/head_rgbd streams the whole time around 16:30 (Wi-Fi up), while map3d_uplink was sending about 100 KB/s to the server.

## Expected outcome

rtabmap_mapping on the server logged 'Did not receive data since 5 seconds' for /uplink/scan_filtered and the /uplink/head_rgbd streams the whole time around 16:30 (Wi-Fi up), while map3d_uplink was sending about 100 KB/s to the server at server ros2-rtabmap_mapping / client map3d_uplink is resolved, and The map3d uplink seems not to be feeding RTAB-Map even while the link is up. no longer applies.

## Steps

1. Write a failing test that shows: rtabmap_mapping on the server logged 'Did not receive data since 5 seconds' for /uplink/scan_filtered and the /uplink/head_rgbd streams the whole time around 16:30 (Wi-Fi up), while map3d_uplink was sending about 100 KB/s to the server.
2. Fix it at server ros2-rtabmap_mapping / client map3d_uplink.
3. Run the task verify command and confirm it passes.

## Notes

Discovered in task `collision-source-stall-fix` during run `drivable-ground-tof-filterin`.

