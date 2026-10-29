# camera kernel family

Kernels of the camera tools (`camera_scene.py`, `camera_candidates.py`, `geometry.py`, `camera_overlay.py`,
`perception.py`) as ranked by `../../bench/report.json` (dev Mac, threshold 5 ms). Only verdict `port` kernels moved
to `mcp_core_cpp.camera`; `tests/test_core_parity_camera.py` checks this table against the report.

| Kernel | Median ms | Status |
|---|---|---|
| `camera_overlay.draw_lidar` | 27.105 | ported |
| `perception.render_map_crop` | 9.400 | ported |
| `perception.map_stats` | 9.389 | ported |
| `camera_candidates.candidate_store` | 1.651 | below threshold, not ported |
| `geometry.pixel_to_ground` | 1.386 | below threshold, not ported |
| `perception.recompress_jpeg` | 1.302 | below threshold, not ported |
| `perception.encode_jpeg` | 1.025 | below threshold, not ported |
| `camera_overlay.draw_grid` | 0.766 | below threshold, not ported |
| `camera_overlay.grid_polylines` | 0.325 | below threshold, not ported |
| `perception.summarize_scan` | 0.091 | below threshold, not ported |
| `camera_overlay.project_points` | 0.065 | below threshold, not ported |
| `camera_overlay.circle_runs` | 0.065 | below threshold, not ported |
| `perception.scan_points` | 0.038 | below threshold, not ported |
| `camera_scene.build_scene` | 0.029 | below threshold, not ported |
| `perception.image_to_bgr` | 0.012 | below threshold, not ported |

## Ported kernels

The Python entry points keep their signatures and call the C++ kernel for the per-cell / per-point work; OpenCV
(cv2) still draws, resizes and encodes, so the outputs are bit-identical to the Python originals. The originals live in
`mcp_server/reference/camera.py`, used only by `tests/regenerate_core_golden.py`.

| Entry point | C++ kernel (`src/camera/`) | What moved to C++ | What stays in Python |
|---|---|---|---|
| `camera_overlay.draw_lidar(img, scene, points)` | `lidar_dots(points, t_base_ref, t_ref_optical, camera_matrix, distortion, width, height, max_range_m)` | per point: base_link -> reference -> optical transform, `cv2.projectPoints` model (0, 4, 5, 8 or 12 coefficients), in-image test, rounded centre, JET index of the range | one `cv2.circle` per visible dot, colour from the JET table |
| `perception.map_stats(data, width, height, resolution)` | `map_counts(data)` | known / occupied / free counts over every cell | `MapStats` with the rounded sizes |
| `perception.render_map_crop(...)` | `map_crop(data, width, height, x0, y0, size)` | grey crop around the robot (only the crop cells are read), flipped north-up | crop origin and size, BGR, resize, robot circle and arrow, PNG |

`data` (an occupancy grid) may be a list of ints (the bench), an `array.array('b')` (rclpy's `OccupancyGrid.data`) or
a numpy array: buffers are read in place, lists item by item, both as numpy's `int16` conversion did (a list item
outside int16 raises `OverflowError`, a non-integer raises `TypeError`). The Python version converted the whole grid
to numpy on every call; that conversion was most of its time.

Goldens: `tests/data/core_golden/camera/{draw_lidar,map_stats,render_map_crop}.json`, all outputs exact (default
tolerance 0). Inputs: `bench/inputs.py` `draw_lidar_cases`, `map_stats_cases`, `render_map_crop_cases` (the bench scan
and room map, deployed front and gripper cameras, plumb-bob and rational distortion, edge cases).
