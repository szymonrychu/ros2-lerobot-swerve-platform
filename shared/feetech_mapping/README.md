# feetech-mapping

Feetech STS3215 step/radian conversions (import name `feetech_mapping.command_mapping`), with no dependencies:
centred radians <-> raw steps, the sign-magnitude speed registers, the speed/acceleration cap registers and
`map_position_to_steps`, the leader -> follower source range mapping of a joint (the arm gripper).

Used by the servo bridge (`nodes/bridges/feetech_servos`, which writes the servos with it) and by filter_node's
self-collision guard (which converts a leader gripper command to follower radians exactly as the bridge will, so the
check sees the gripper where the follower puts it). Tests live with the bridge
(`nodes/bridges/feetech_servos/tests/test_command_mapping.py`).

## Using it from a node

```toml
[project]
dependencies = ["feetech-mapping"]

[tool.uv.sources]
feetech-mapping = { path = "../../shared/feetech_mapping", editable = true }   # nodes/bridges/<node>: ../../../shared/feetech_mapping
```

Then `uv lock`, and set `src_shared: true` on the node type so a change under `shared/` redeploys the node.
