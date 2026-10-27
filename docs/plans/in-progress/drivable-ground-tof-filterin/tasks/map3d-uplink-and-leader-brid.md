---
title: "map3d_uplink and leader_bridge (the static-peer Wi-Fi bridges) make blocking Fast DDS UDP sends. Fast DDS 2.14 UDPTransportInterface::send sets SO_SNDTIMEO to the remaining max_blocking_time, and once that time has passed it sets 0, which means no timeout. So during a Wi-Fi stall these processes block until the carrier is lost. 720 UDP SndbufErrors were counted in the 16:30 outage."
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "This is a likely part of how Wi-Fi outages spread into the robot graph. To check at the live reproduction: does a blocked bridge stall same-host topics?"
files: []
verify: ""
depends_on: []
model: ""
review: ""
verified_head: ""
points: ""
expected_minutes: ""
expected_tokens: ""
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# map3d_uplink and leader_bridge (the static-peer Wi-Fi bridges) make blocking Fast DDS UDP sends. Fast DDS 2.14 UDPTransportInterface::send sets SO_SNDTIMEO to the remaining max_blocking_time, and once that time has passed it sets 0, which means no timeout. So during a Wi-Fi stall these processes block until the carrier is lost. 720 UDP SndbufErrors were counted in the 16:30 outage.

## Requirement

map3d_uplink and leader_bridge (the static-peer Wi-Fi bridges) make blocking Fast DDS UDP sends. Fast DDS 2.14 UDPTransportInterface::send sets SO_SNDTIMEO to the remaining max_blocking_time, and once that time has passed it sets 0, which means no timeout. So during a Wi-Fi stall these processes block until the carrier is lost. 720 UDP SndbufErrors were counted in the 16:30 outage.

Where: ansible/group_vars/client.yml (map3d_uplink, leader_bridge env), nodes/map3d_uplink

Why: This is a likely part of how Wi-Fi outages spread into the robot graph. To check at the live reproduction: does a blocked bridge stall same-host topics?

## Acceptance conditions

- AC1: map3d_uplink and leader_bridge (the static-peer Wi-Fi bridges) make blocking Fast DDS UDP sends. Fast DDS 2.14 UDPTransportInterface::send sets SO_SNDTIMEO to the remaining max_blocking_time, and once that time has passed it sets 0, which means no timeout. So during a Wi-Fi stall these processes block until the carrier is lost. 720 UDP SndbufErrors were counted in the 16:30 outage. at ansible/group_vars/client.yml (map3d_uplink, leader_bridge env), nodes/map3d_uplink is resolved [evidence: judgement]

## Files

- `ansible/group_vars/client.yml`
- `nodes/map3d_uplink/README.md`

## Interfaces

Keep the interfaces around ansible/group_vars/client.yml (map3d_uplink, leader_bridge env), nodes/map3d_uplink unchanged unless resolving it requires otherwise: map3d_uplink and leader_bridge (the static-peer Wi-Fi bridges) make blocking Fast DDS UDP sends. Fast DDS 2.14 UDPTransportInterface::send sets SO_SNDTIMEO to the remaining max_blocking_time, and once that time has passed it sets 0, which means no timeout. So during a Wi-Fi stall these processes block until the carrier is lost. 720 UDP SndbufErrors were counted in the 16:30 outage..

## Expected outcome

map3d_uplink and leader_bridge (the static-peer Wi-Fi bridges) make blocking Fast DDS UDP sends. Fast DDS 2.14 UDPTransportInterface::send sets SO_SNDTIMEO to the remaining max_blocking_time, and once that time has passed it sets 0, which means no timeout. So during a Wi-Fi stall these processes block until the carrier is lost. 720 UDP SndbufErrors were counted in the 16:30 outage. at ansible/group_vars/client.yml (map3d_uplink, leader_bridge env), nodes/map3d_uplink is resolved, and This is a likely part of how Wi-Fi outages spread into the robot graph. To check at the live reproduction: does a blocked bridge stall same-host topics? no longer applies.

## Steps

1. Write a failing test that shows: map3d_uplink and leader_bridge (the static-peer Wi-Fi bridges) make blocking Fast DDS UDP sends. Fast DDS 2.14 UDPTransportInterface::send sets SO_SNDTIMEO to the remaining max_blocking_time, and once that time has passed it sets 0, which means no timeout. So during a Wi-Fi stall these processes block until the carrier is lost. 720 UDP SndbufErrors were counted in the 16:30 outage..
2. Fix it at ansible/group_vars/client.yml (map3d_uplink, leader_bridge env), nodes/map3d_uplink.
3. Run the task verify command and confirm it passes.

## Notes

Discovered in task `collision-source-stall-fix` during run `drivable-ground-tof-filterin`.

