# Retro: webui-arm-tab-and-3d-goal

## What went well

- All of rung 1 (1.1, 1.2, 1.3, 1.5, 1.6 offline half) landed in 11 step commits, each test-first, with the verify
  suites green: web_ui pytest 1105 passed, frontend vitest 1347 passed, tsc/lint/build clean.
- Controls were moved, not duplicated: ArmTab reuses the existing home/grasp APIs; MAP code paths were deleted with
  retirement tests.
- One pure disabled-reason model (armControls.ts) feeds drag, home, set home, grasp and jog, so the reasons cannot drift.
- Orbit-view goal bindings were extracted into a tested module (cameraBindings.ts) and picks are clamped (40 m).

## What went wrong / could improve

- Item 1.4 (live diagnosis of the always-greyed arm, possible bridge fix) and the deploy/live check could not run: the
  robot was unreachable. They remain post-merge work.
- InteractiveArm keeps `position` and `visible` props that no caller passes any more.
- The map_nav tab config still lists arm_home_service / arm_set_home_service (inert; grasp topics there still serve as
  the bridge's grasp topic source).

## Recommendations

- Discovered, not built: drop the unused InteractiveArm `position`/`visible` props; remove arm_home_service and
  arm_set_home_service from the map_nav tab config once nothing reads them.
- Beyond rung 1: rung 2 (drag under the arm lease via mcp_server), rung 3 (goal from the ARM tab 3D view, Add point /
  Add area in orbit view, picks on the 3D cloud surface).
- Impossible at planning time: whether the grey-out is a stuck released joint in the bridge needs the live robot
  (GET /api/calibration/servos); run Post-merge P1-P5 when client.ros2.lan answers.
