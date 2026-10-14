# Monocular depth on the Jetson Nano: measured results

Default mono model: MiDaS v2.1 small at 128x128 (`midas_small_128`, `mono.rate_hz: 2.0`).

Measured 2026-10-14/15 on the robot's Jetson Nano 4 GB (L4T R32.6.1, TensorRT 8.0.1, MAXN + jetson_clocks), with the
deployed depthd running (cuda_sgm, 640x480, 128 disparities, post-processing on), so every number includes contention
with the live stereo pipeline.

## Engine builds (trtexec --fp16 --workspace=1024)

| Model | Input | Build time | trtexec throughput | GPU compute (mean) |
|---|---|---|---|---|
| Depth Anything V2 Small | 266x266 | 467 s (7:57 wall, 1.9 GB RSS) | 3.97 qps | 250 ms |
| Depth Anything V2 Small | 126x126 | 387 s wall | - | 78 ms |
| MiDaS v2.1 small | 256x256 | 785 s (role build, 18:59-19:12) | 17.0 qps | 59 ms |
| MiDaS v2.1 small | 128x128 | 1136 s wall (1.5 GB RSS) | 26.6 qps | 36 ms |

The DAv2 266 engine builds and runs at 4 Hz standalone (6.0 qps when depthd was idle), so it passes the D5 ">= 2 Hz"
gate on its own.

## In the pipeline (dev depthd binary, live cameras, 40 s window after 35 s warm-up, mono.rate_hz 2.0)

| Config | Stereo depth | Fused | Mono fit inliers | Fill ratio |
|---|---|---|---|---|
| mono off (baseline, two runs) | 12.17 / 12.26 fps | - | - | - |
| DAv2 Small 266 | 8.19 fps | 1.87 Hz | 10569 | 1.0 |
| MiDaS small 256 | 9.97 fps | 1.67 Hz | 7724 | 1.0 |
| MiDaS small 128 | 11.51 fps | 1.92 Hz | 6997 | 1.0 |

(These runs used a strict per-frame rate gate; the deployed stage keeps the average at `rate_hz`.)

Deployed (role build, 2026-10-15, midas_small_128 engine built by the role in 1146 s): stereo 11.6 fps on the Jetson
(11.55 Hz received on the Pi), fused 2.00 Hz on the Jetson and 2.01 Hz on the Pi (`/stereo/depth/fused`), mono fit
~7600 inliers, fill ratio 1.0, `mono_state` ok. Depth latency p95 is 195 ms with mono on and off alike (not caused by
the mono stage).

## Why MiDaS small 128 and not Depth Anything V2

The stereo pipeline already keeps the Nano's single-SM GPU busy (GPU load ~100 %: rectify 2.5 ms + SGM 49 ms + post
33 ms per frame at 12.2 fps), so every millisecond of mono inference is taken from stereo, whatever the CUDA stream
priorities (mono runs on the lowest, stereo on the highest). DAv2 Small at 2 Hz costs ~0.5 s of GPU per second and
drops stereo to 8.2 fps; shrinking it to 126x126 still costs 78 ms per run (more than MiDaS 128) at a far lower token
count. MiDaS v2.1 small at 128x128 costs 36 ms per run, keeps stereo at ~11.5 fps (from 12.2) and fuses at ~2 Hz. The
fill only touches stereo holes and is scale-aligned per frame, so the lower mono resolution only coarsens the filled
regions. Design D5's fallback clause covers the choice: DAv2 builds and passes 2 Hz alone, but not without breaking the
stereo rate the pipeline has to keep.
