# map_manager

Named maps: each map is a self-contained bundle directory under `/var/lib/ros2/maps/named/<name>/`:

- `map.posegraph`, `map.data` - slam_toolbox serialized pose graph
- `occupancy.npz` - int8 grid (-1 unknown kept), resolution, origin x/y/yaw
- `bundle.json` - `BundleMeta`: name, saved_at, note, last pose, POIs, POI revision, map info, laser frame

`map_manager/bundle.py` (`BundleStore`, no ROS imports) writes bundles atomically: `begin` makes a
`.tmp-<name>-<ms>` directory, `commit` validates and renames it into place (overwrite swaps via a rename-aside),
failure removes the temp dir and leaves any existing bundle untouched. `list`, `load`, `delete`, `cleanup_temp`
complete the store. Names are checked with `ros2_common.maps.validate_map_name`.

`map_manager/relocalise.py` (numpy only, no ROS) refines the load pose hint before slam_toolbox sees it:
`search_candidates` scores the base-frame scan against the bundle grid at every pose in a window around the hint
(`SearchSettings`: +-1.0 m at 0.1 m, full turn at 5 deg) with the shared `ros2_common.scan_agreement` rules, returns
the best pose with `ok` (ratio >= `candidate_min_ratio`, evaluated >= `min_points`) and `ambiguous` (a candidate
farther than `ambiguity_distance_m` / `ambiguity_yaw_deg` scores within `ambiguity_margin` of the best); with too
few evaluated points it keeps the hint unchanged. `LocalisationJudge` turns live agreement samples into
`localised` / `not_localised` / `ambiguous` (median >= `localised_ratio`, at least `min_samples`).

Tests: `uv run --frozen pytest tests/ -v`; lint: `uv run --frozen poe lint`.
