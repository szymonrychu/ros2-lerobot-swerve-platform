# map_manager

Named maps: each map is a self-contained bundle directory under `/var/lib/ros2/maps/named/<name>/`:

- `map.posegraph`, `map.data` - slam_toolbox serialized pose graph
- `occupancy.npz` - int8 grid (-1 unknown kept), resolution, origin x/y/yaw
- `bundle.json` - `BundleMeta`: name, saved_at, note, last pose, POIs, POI revision, map info, laser frame

`map_manager/bundle.py` (`BundleStore`, no ROS imports) writes bundles atomically: `begin` makes a
`.tmp-<name>-<ms>` directory, `commit` validates and renames it into place (overwrite swaps via a rename-aside),
failure removes the temp dir and leaves any existing bundle untouched. `list`, `load`, `delete`, `cleanup_temp`
complete the store. Names are checked with `ros2_common.maps.validate_map_name`.

Tests: `uv run --frozen pytest tests/ -v`; lint: `uv run --frozen poe lint`.
