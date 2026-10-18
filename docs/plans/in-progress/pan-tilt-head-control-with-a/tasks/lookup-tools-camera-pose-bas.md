---
title: "lookup_tools.camera_pose_base still uses np.eye(4) for the front parent pose instead of parent_transform(..., head=robot.head)"
status: done
repo: ros2-lerobot-sverve-platform
value: "Lookups record a wrong front camera pose once the head moves. This belongs to task mcp-lookups-follow-head-pose, which should call camera_scene.parent_transform with head=robot.head."
files: ["nodes/mcp_server/mcp_server/lookup_tools.py:~133"]
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
verified_head: "829b9a2825d5cbea6171dee1e4fdc7b21862b6fc"
run: ""
provides: []
---

# lookup_tools.camera_pose_base still uses np.eye(4) for the front parent pose instead of parent_transform(..., head=robot.head)

## Requirement

lookup_tools.camera_pose_base still uses np.eye(4) for the front parent pose instead of parent_transform(..., head=robot.head)

Where: nodes/mcp_server/mcp_server/lookup_tools.py:~133

Why: Lookups record a wrong front camera pose once the head moves. This belongs to task mcp-lookups-follow-head-pose, which should call camera_scene.parent_transform with head=robot.head.

## Acceptance conditions

- AC1: lookup_tools.camera_pose_base still uses np.eye(4) for the front parent pose instead of parent_transform(..., head=robot.head) at nodes/mcp_server/mcp_server/lookup_tools.py:~133 is resolved [evidence: judgement]

## Files

- `nodes/mcp_server/mcp_server/lookup_tools.py:~133`

## Interfaces

Keep the interfaces around nodes/mcp_server/mcp_server/lookup_tools.py:~133 unchanged unless resolving it requires otherwise: lookup_tools.camera_pose_base still uses np.eye(4) for the front parent pose instead of parent_transform(..., head=robot.head).

## Expected outcome

lookup_tools.camera_pose_base still uses np.eye(4) for the front parent pose instead of parent_transform(..., head=robot.head) at nodes/mcp_server/mcp_server/lookup_tools.py:~133 is resolved, and Lookups record a wrong front camera pose once the head moves. This belongs to task mcp-lookups-follow-head-pose, which should call camera_scene.parent_transform with head=robot.head. no longer applies.

## Steps

1. Write a failing test that shows: lookup_tools.camera_pose_base still uses np.eye(4) for the front parent pose instead of parent_transform(..., head=robot.head).
2. Fix it at nodes/mcp_server/mcp_server/lookup_tools.py:~133.
3. Run the task verify command and confirm it passes.

## Notes

Discovered in task `mcp-front-camera-on-head` during run `pan-tilt-head-control-with-a`.


## Ruling

Already delivered by mcp-lookups-follow-head-pose (lookup_tools.py:132 uses head_parent_transform with robot.head), merged into main. Closed by the orchestrator.
