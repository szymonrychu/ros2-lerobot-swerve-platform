# Ground robustness runbook

Proves on the real scene that the ground estimator classes (obstacle, drop, step, ramp, colour) are stable enough to drive on. Static: the orchestrator runs it after deploy and fills the Results table.

## Safety

- The robot stays parked and charging for the whole run.
- No drive commands: nothing is sent to the wheels or the drivetrain.
- Only the head pan/tilt moves, to point the cameras at each scene.
- No arm motion.

## Scene

Ground truth (base_link metres, x forward, y left):

| Object | Place | Expected class |
|--------|-------|----------------|
| Box | exactly 5 cm high, x 0.60-0.78, y +0.05..+0.33 | obstacle |
| Black rubber ramp | 25 cm long, 63 mm high, ~14 deg, x 0.75-1.10, y -0.45..-0.15 | never lethal (a ramp area is welcome but not required, see Sensor limits) |
| ChArUco board | flat, x 0.37-0.61, y -0.12..+0.14 | warning only, never lethal |
| Empty floor | nothing placed | nothing lethal |

The robot's own parked gripper at x 0.30-0.36 reads +3..+11 cm until the head floor fit of `head-chain-floor-plane-fit-a` is applied; it lies outside every scene ROI above and is not a scene object.

## Prerequisites

1. `head-chain-floor-plane-fit-a` and the ground-detection-robustness epic are deployed:

   ```bash
   mkdir -p .logs
   ./scripts/deploy-nodes.sh client ground_estimator_cpp costmap_zones 2>&1 | tee .logs/deploy-ground-robustness.log
   ```

2. `topic_scraper_api` is disabled by default (port 18100 is refused). Set `enabled: true` for it in `ansible/group_vars/client.yml`, then deploy:

   ```bash
   ./scripts/deploy-nodes.sh client topic_scraper_api 2>&1 | tee .logs/deploy-topic-scraper.log
   ```

3. Teardown, run as the last step of the Procedure: set `enabled: false` for `topic_scraper_api` in `ansible/group_vars/client.yml` and deploy it:

   ```bash
   ./scripts/deploy-nodes.sh client topic_scraper_api 2>&1 | tee .logs/deploy-topic-scraper-off.log
   ```

## Procedure

For each scene (`box`, `ramp`, `board`, `empty`): point the head at it, wait 10 s, then capture 2 minutes and run the report:

```bash
timeout 120 python scripts/topic_scraper_collect.py --source client=http://client.ros2.lan:18100 \
  --select /ground/state:.data --interval 0.5 > .logs/ground-<scene>.ndjson
python scripts/ground_noise_report.py --config scripts/ground_noise_report.yaml \
  --capture .logs/ground-<scene>.ndjson --scene <scene>
```

The report prints one JSON summary and exits 0 when every metric of the scene passes (thresholds in `scripts/ground_noise_report.yaml`).

## Acceptance

- Box: obstacle in >= 95% of states.
- Ramp never lethal (0 lethal states): no obstacle region with its edge point in the ramp ROI in any state. Warning share and slope error are reported for information only.
- Board: no lethal state.
- Empty floor: 0 lethal points at p95.
- Warning-area churn < 10% of states in every scene.

## Sensor limits

- The ramp is black rubber: the ToF gets no returns on it above the confidence threshold, so the ramp is invisible to the ToF. A ramp area, its warning share and its slope error therefore cannot be required; only "never lethal" is checkable, and that is the ramp gate.
- The ChArUco board reads -4..+3 cm from albedo bias (the black and white squares return different depths), which is why the board is held to warning only and never lethal rather than to a height.

## Results

| Date | Scene | Metric | Value | Threshold | Pass |
|------|-------|--------|-------|-----------|------|
|  | box | box_obstacle_share |  | >= 0.95 |  |
|  | ramp | ramp_lethal_states |  | 0 |  |
|  | ramp | ramp_warning_share |  | info only |  |
|  | board | board_lethal_states |  | 0 |  |
|  | empty | empty_lethal_points_p95 |  | 0 |  |
|  | box | warning_churn_share |  | < 0.10 |  |
|  | ramp | warning_churn_share |  | < 0.10 |  |
|  | board | warning_churn_share |  | < 0.10 |  |
|  | empty | warning_churn_share |  | < 0.10 |  |

On a failure: keep the capture in `.logs/`, record the numbers in the table, and name the failing metric in the epic record.
