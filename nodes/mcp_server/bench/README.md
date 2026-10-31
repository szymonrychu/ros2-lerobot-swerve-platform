# mcp_server kernel bench

Profiles the compute kernels of the mcp_server tools (geometry, planning, map and image work) so that only the
measurably slow ones are ported to the `mcp_core_cpp` C++ extension. No ROS is needed.

```bash
cd nodes/mcp_server
uv run python -m bench.bench_kernels                      # dev Mac, threshold 5 ms
uv run python -m bench.bench_kernels --threshold-ms 15    # run on the Pi 5, threshold about 15 ms
uv run python -m bench.bench_kernels --out /tmp/bench     # write the reports elsewhere
```

## Files

| File | Purpose |
|---|---|
| `bench_kernels.py` | Kernel registry (`KERNELS`), timing, Mimir reader, ranking, report writers, CLI. |
| `bench_kernels.yaml` | `repetitions`, `threshold_ms`, `families`, `prometheus_url`, `lookback_hours` (pydantic-validated, unknown keys rejected). |
| `inputs.py` | Inputs of every kernel: the deployed `ansible/group_vars/client.yml` config, the recorded grasp cases of `tests/data/grasp_golden.json`, `tests/camera_helpers.py` for the head camera (not calibrated on the robot), and seeded synthetic scans, maps and images. Reused by the parity harness of `mcp_core_cpp`; golden case functions (`list[dict]` of keyword args): `point_reach_cases`, `reach_map_cases` (ik_reach). |
| `REPORT.md` | The ranked table of the last committed run. |
| `report.json` | The same rows, machine readable (also CPU times, tools, Mimir window). |

## What is timed

One call of a kernel is timed after one warm-up call: median and p95 of wall time (`perf_counter`) and thread CPU time
(`thread_time`), `repetitions` calls (a few very slow kernels cap it lower). The input is built outside the timing.
Kernels are grouped into families:

- camera: `camera_scene`, `camera_candidates`, `geometry` pixel-to-ground, `camera_overlay`, `perception`
- grasp: `grasp` (cold planning of the first recorded case, IK cache cleared), `grasp_check`, `jaw_crop`
- ik_reach: `ik`, `arm_reach`, `reach_map` (coarse grid), `arm_geometry`, `sag`
- collision: `self_collision` (40 sample path), `footprint_geometry`, `body_model` (4000 points)
- frontiers: `frontiers` on a synthetic 800 x 800 map at 5 cm

Adding a kernel: write an `inputs.<name>_call()` that builds the inputs and returns a zero-argument callable, then add a
`KernelSpec(name, family, tools, make)` to `KERNELS`. `tools` are the MCP tool names whose calls count as calls of the
kernel. `tests/test_bench_kernels.py` runs every registered kernel once.

## Verdict and calls/day

`port` when the median wall time is at least `threshold_ms` (5 ms on the dev Mac; the Pi 5 is about 3 times slower, so
use about 15 ms there), else `below threshold`. The table is ranked by median wall time, slowest first.

calls/day comes from Mimir: `mcp_tool_calls_total` per tool over `lookback_hours` (Mimir keeps 1 day, so the window
actually available is printed in the report), scaled to 24 h and summed over the tools of the kernel; counter resets are
handled. `mcp_tool_duration_seconds` gives the mean real call duration per tool (in `report.json`, `mimir`). When Mimir
is unreachable the report says so and shows `n/a`; nothing is estimated.
