# Runbook: head floor fit

After the head mount reprint the URDF head chain is the pre-reprint CAD, so `/ground/state` sees the flat floor about
6 cm high, 3-4 deg tilted, with 170+ false obstacle points. The Head floor fit step on the Calibrate page sweeps the
head over a pan/tilt grid on an empty flat floor, fits mount z, mount roll, mount pitch and tilt zero from the ToF floor
plane in one robust least-squares fit, and applies it as the `head_chain.json` override that `head_state_publisher`
publishes. Run from the dev machine against the client (Pi 5, `client.ros2.lan`); the agent drives deploy and
telemetry, the user clears the floor and runs the UI steps. Deploys only through `scripts/deploy-nodes.sh` (never
`ansible-playbook`). ROS commands on the client need
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4`. Related:
`docs/runbooks/head-remount-floor-check.md`, `docs/runbooks/head-rgbd-fusion.md`, `docs/runbooks/ground-estimation.md`.

Servo safety: only the head moves (pan and tilt, within the sweep grid); no arm or wheel joint moves.

## Preconditions

- The robot is on a flat floor and not on blocks.
- About 2 m in front and 1.5 m to each side are clear; nothing else is in view of the head.
- The RGB-D calibration is applied (the ToF depth and the ToF-to-camera extrinsic are the base of this fit).
- No agent session is running if mcp_server is deployed (deploying claude_agent/mcp_server kills a running session,
  exit 143): check the claude_agent session state, ask when one runs. This epic does not deploy mcp_server.
- `topic_scraper_api` is enabled for the client in `ansible/group_vars/client.yml` (`enabled: true`) and deployed, else
  port 18100 refuses connections (CLAUDE.md).

## Deploy

Invoke the `ansible-deploy` skill for the exact command first, then (all output to `.logs/`):

```bash
mkdir -p .logs
./scripts/deploy-nodes.sh client head_rgbd_calibration head_state_publisher web_ui 2>&1 | tee .logs/deploy-client-head-floor-fit.log
```

Expect `failed=0` and every unit `active`.

## Before: floor numbers

Collect `/ground/state` at home (pan 0, tilt 29.2 deg down) for the "before" row of the results table:

```bash
python scripts/topic_scraper_collect.py --source client=http://client.ros2.lan:18100 --select /ground/state:.plane --interval 0.1
```

## UI steps

1. Open the web UI, hamburger menu, Calibrate. The Head floor fit step is below the RGB-D section.
2. Confirm the precondition text with the floor as it is, then **Start sweep**. Watch "Pose i of N, observations,
   skipped"; the head moves through the grid. **Stop** ends it early (the poses done so far are kept).
3. When the sweep has ended, **Compute**. Read the table: mount z (mm), mount roll, mount pitch, tilt zero (deg) with
   current, fitted, delta and sigma, the height and normal RMS and the per-pose residuals. A failed result lists the
   failed limits and Apply stays disabled: fix the cause (floor, clearance, lighting) and sweep again.
4. **Apply** a passed result. The step shows the unit restart state (`head_state_publisher` and the head consumers
   restart); wait until every unit shows restarted, and the applied head chain (session, result, note) is listed.

## Acceptance checks

Collect `/ground/state` with `scripts/topic_scraper_collect.py` (not raw `ros2 topic echo`) at home and at two other
pan/tilt poses (for example pan +40 / tilt 20 down and pan -40 / tilt 25 down; head only, minimal amplitude, return to
home afterwards), 60 s each:

```bash
python scripts/topic_scraper_collect.py --source client=http://client.ros2.lan:18100 --select /ground/state:.plane --interval 0.1
python scripts/topic_scraper_collect.py --source client=http://client.ros2.lan:18100 --select /ground/obstacle_points --interval 0.1
```

At each of the three poses:

- Plane height 0 +- 0.01 m.
- plane_vs_base < 0.6 deg (`ground_plane_vs_base_deg` on the metrics port 19119).
- Fewer than 10 obstacle points on the flat floor.

If a check fails, repeat the sweep (Start sweep again); do not hand-edit `head_chain.json`.

## Results

Filled when the orchestrator runs this with the user after merge.

Date:

| Check | Number | Result |
|---|---|---|
| Deploy (failed=0, units active) | | |
| Sweep (observations, skipped) | | |
| Fit passed (height RMS mm, normal RMS deg) | | |
| Apply (units restarted) | | |
| Home: plane height (0 +- 0.01 m), before / after | | |
| Home: plane_vs_base (< 0.6 deg), before / after | | |
| Home: obstacle points (< 10), before / after | | |
| Pose 2: plane height / plane_vs_base / obstacle points | | |
| Pose 3: plane height / plane_vs_base / obstacle points | | |
