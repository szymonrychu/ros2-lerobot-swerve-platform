"""Tests for mcp_server.frontiers (pure frontier core on synthetic occupancy grids, no ROS)."""

import math
import threading
import time

import numpy as np
import pytest
from ros2_common.lookups import CameraPose, LookupRecord, PlanarPose

from mcp_server import frontiers
from mcp_server.config import FrontierSettings
from mcp_server.frontiers import (
    EPOCH_RESET_NOTE,
    FREE,
    OCCUPIED,
    UNKNOWN,
    Frontier,
    FrontierTracker,
    MapGrid,
    RetiredFrontier,
    cell_to_map,
    extract_frontiers,
    facing_lookup,
    geodesic_from,
    line_of_sight,
    map_to_cell,
    min_cells_for,
    nearest_poi,
    observation_pose,
    snap_to_traversable,
    traversability,
)

RES = 0.025
ROOM_LO = 10  # room interior rows/cols ROOM_LO..ROOM_HI-1, walls at ROOM_LO-1 and ROOM_HI
ROOM_HI = 90


def make_grid(
    data: np.ndarray, version: tuple = (1,), origin: tuple[float, float, float] = (0.0, 0.0, 0.0), res: float = RES
) -> MapGrid:
    """MapGrid around an int16 array with known_cells counted."""
    return MapGrid.from_array(data, res, origin[0], origin[1], origin[2], version)


def corridor_map() -> np.ndarray:
    """A 0.5 m wide corridor (rows 20..39) open to unknown space at column 120."""
    data = np.full((60, 200), UNKNOWN, np.int16)
    data[19:41, 0:120] = OCCUPIED
    data[20:40, 1:120] = FREE
    return data


def room_map(gaps: list[tuple[int, int]], extra_cols: int = 0, h: int = 100, w: int = 140) -> np.ndarray:
    """A square room with its east wall (column ROOM_HI + extra_cols) open to unknown space at the given row gaps."""
    data = np.full((h, w), UNKNOWN, np.int16)
    east = ROOM_HI + extra_cols
    data[ROOM_LO - 1 : ROOM_HI + 1, ROOM_LO - 1 : east + 1] = OCCUPIED
    data[ROOM_LO:ROOM_HI, ROOM_LO:east] = FREE
    for lo, hi in gaps:
        data[lo:hi, east] = UNKNOWN
    return data


def walled_room(h: int = 100, w: int = 100) -> np.ndarray:
    """Free room with a one-cell occupied border."""
    data = np.full((h, w), FREE, np.int16)
    data[0, :] = data[-1, :] = data[:, 0] = data[:, -1] = OCCUPIED
    return data


def settings(**overrides: object) -> FrontierSettings:
    """FrontierSettings with test overrides."""
    return FrontierSettings.model_validate(overrides)


def is_frontier_cell(data: np.ndarray, rc: tuple[int, int]) -> bool:
    """True when the cell is unknown and has a free 8-neighbour."""
    r, c = rc
    if data[r, c] != UNKNOWN:
        return False
    window = data[max(r - 1, 0) : r + 2, max(c - 1, 0) : c + 2]
    return bool(((window >= 0) & (window <= frontiers.FREE_THRESHOLD)).any())


# --- geometry -------------------------------------------------------------------------------------------------------


def test_cell_map_conversion_includes_origin_yaw() -> None:
    grid = make_grid(np.zeros((10, 10), np.int16), origin=(1.0, 2.0, math.pi / 2))
    x, y = cell_to_map(grid, 0, 0)
    assert (x, y) == pytest.approx((1.0 - 0.5 * RES, 2.0 + 0.5 * RES))
    x, y = cell_to_map(grid, 3, 7)  # col 7 runs along map +y, row 3 along map -x
    assert (x, y) == pytest.approx((1.0 - 3.5 * RES, 2.0 + 7.5 * RES))
    assert map_to_cell(grid, x, y) == (3, 7)
    assert map_to_cell(grid, 5.0, 5.0) is None  # outside the grid


def test_known_cells_counted_and_min_cells_from_length() -> None:
    grid = make_grid(corridor_map())
    assert grid.known_cells == int((corridor_map() >= 0).sum())
    assert min_cells_for(0.30, 0.025) == 12
    assert min_cells_for(0.30, 0.05) == 6


# --- extraction -----------------------------------------------------------------------------------------------------


def test_corridor_with_unknown_end_has_one_frontier_with_target_on_it() -> None:
    data = corridor_map()
    found = extract_frontiers(make_grid(data), min_cells=12)
    assert len(found) == 1
    f = found[0]
    assert len(f.cells) == 22  # column 120, rows 19..40 (the corners touch free space diagonally)
    assert set(f.cells[:, 1].tolist()) == {120}
    assert f.length_m == pytest.approx(22 * RES)
    assert f.centroid_xy == pytest.approx((120.5 * RES, 30.0 * RES))
    assert is_frontier_cell(data, f.target_rc)
    assert math.dist(f.target_xy, f.centroid_xy) <= RES
    assert f.target_xy == pytest.approx(cell_to_map(make_grid(data), *f.target_rc))
    assert f.bbox == (19, 120, 40, 120)


def test_min_length_filter_drops_short_clusters() -> None:
    grid = make_grid(corridor_map())
    assert len(extract_frontiers(grid, min_cells=22)) == 1
    assert extract_frontiers(grid, min_cells=23) == []


def test_occupied_boundary_is_not_a_frontier() -> None:
    assert extract_frontiers(make_grid(room_map(gaps=[])), min_cells=1) == []


def test_free_space_reaching_the_map_border_yields_no_frontier_outside_the_map() -> None:
    data = np.full((40, 40), UNKNOWN, np.int16)
    data[:, :20] = FREE  # free space touching the north, south and west map borders
    found = extract_frontiers(make_grid(data), min_cells=1)
    assert len(found) == 1
    assert set(found[0].cells[:, 1].tolist()) == {20}  # only the in-map unknown column next to free space


def test_unknown_padding_crop_matches_the_full_grid() -> None:
    small = corridor_map()
    pad = 300
    big = np.pad(small, pad, constant_values=UNKNOWN)
    a = extract_frontiers(make_grid(small), min_cells=12)
    b = extract_frontiers(make_grid(big, origin=(-pad * RES, -pad * RES, 0.0)), min_cells=12)
    assert len(a) == len(b) == 1
    assert b[0].centroid_xy == pytest.approx(a[0].centroid_xy)
    assert b[0].target_xy == pytest.approx(a[0].target_xy)
    np.testing.assert_array_equal(b[0].cells - pad, a[0].cells)


def test_extraction_with_rotated_origin_reports_map_coordinates() -> None:
    grid = make_grid(corridor_map(), origin=(1.0, 2.0, math.pi / 2))
    f = extract_frontiers(grid, min_cells=12)[0]
    assert f.centroid_xy == pytest.approx((1.0 - 30.0 * RES, 2.0 + 120.5 * RES))


# --- traversability and geodesic --------------------------------------------------------------------------------------


def test_traversability_treats_unknown_and_occupied_as_obstacles() -> None:
    data = walled_room()
    data[40:60, 40:60] = UNKNOWN
    trav, clearance = traversability(make_grid(data), pass_clearance_m=0.1)
    assert not trav[50, 50] and clearance[50, 50] == 0.0
    assert not trav[50, 38]  # two cells from the unknown block
    assert trav[20, 20]
    assert clearance[20, 20] == pytest.approx((20 - 0.5) * RES)  # cell centre to the border wall's edge
    assert not trav[2, 50]  # next to the wall


def wall_with_gap(gap: bool) -> np.ndarray:
    """Room split by a wall at column 50, open at the bottom when gap."""
    data = walled_room()
    data[: 80 if gap else 100, 50] = OCCUPIED
    return data


@pytest.mark.parametrize("downsample", [1, 2, 3])
def test_geodesic_around_a_wall_exceeds_the_straight_line(downsample: int) -> None:
    trav, _ = traversability(make_grid(wall_with_gap(gap=True)), pass_clearance_m=0.05)
    dist = geodesic_from(trav, (20, 20), downsample, RES)
    straight = 60 * RES
    assert math.isfinite(dist[20, 80])
    assert dist[20, 80] > 2 * straight
    assert dist[20, 20] <= downsample * RES * math.sqrt(2)
    assert math.isfinite(dist[20, 30]) and dist[20, 30] == pytest.approx(10 * RES, abs=downsample * RES * 1.5)
    assert np.isinf(dist[~trav]).all()


def test_geodesic_into_a_closed_room_is_infinite() -> None:
    trav, _ = traversability(make_grid(wall_with_gap(gap=False)), pass_clearance_m=0.05)
    dist = geodesic_from(trav, (20, 20), 2, RES)
    assert np.isinf(dist[20, 80])
    assert math.isfinite(dist[80, 20])


def test_wall_hugging_start_snaps_to_traversable_space() -> None:
    trav, _ = traversability(make_grid(wall_with_gap(gap=True)), pass_clearance_m=0.05)
    assert not trav[20, 49]
    snapped = snap_to_traversable(trav, (20, 49), RES)
    assert snapped is not None
    (r, c), snap_m = snapped
    assert trav[r, c] and snap_m <= 0.3
    dist = geodesic_from(trav, (20, 49), 2, RES)
    assert math.isfinite(dist[20, 20])


def test_start_deep_inside_an_obstacle_reaches_nothing() -> None:
    data = walled_room()
    data[:, 30:70] = OCCUPIED  # a 1 m thick block
    trav, _ = traversability(make_grid(data), pass_clearance_m=0.05)
    assert snap_to_traversable(trav, (50, 50), RES) is None  # 0.5 m from the nearest traversable cell
    assert np.isinf(geodesic_from(trav, (50, 50), 2, RES)).all()


# --- observation pose -----------------------------------------------------------------------------------------------


def opening_setup(
    view_clearance_m: float = 0.3,
) -> tuple[MapGrid, Frontier, np.ndarray, frontiers.FrontierField, np.ndarray]:
    data = room_map(gaps=[(40, 60)])
    grid = make_grid(data)
    tracker = FrontierTracker(settings(), footprint_width_m=0.2)
    snap = tracker.update(grid)
    assert len(snap.frontiers) == 1
    geo = tracker.geodesic(snap, cell_to_map(grid, 50, 30)).distance_m
    return grid, snap.frontiers[0], snap.clearance_m, snap.field, geo


def test_observation_pose_is_free_within_standoff_with_line_of_sight() -> None:
    grid, frontier, clearance, field, geo = opening_setup()
    pose = observation_pose(frontier, grid, clearance, field, 0.3, (0.4, 1.2), 5, geodesic=geo)
    assert pose is not None
    r, c = pose.rc
    assert grid.data[r, c] == FREE
    assert clearance[r, c] >= 0.3
    nearest = min(math.dist((pose.x, pose.y), cell_to_map(grid, int(a), int(b))) for a, b in frontier.cells)
    assert 0.4 - RES <= nearest <= 1.2 + RES
    assert pose.distance_m == pytest.approx(nearest, abs=RES)
    assert line_of_sight(grid, pose.rc, pose.frontier_rc)
    assert pose.yaw == pytest.approx(math.atan2(frontier.target_xy[1] - pose.y, frontier.target_xy[0] - pose.x))
    assert pose.geodesic_m == pytest.approx(geo[r, c])
    assert pose.score == pytest.approx(geo[r, c] + abs(pose.distance_m - 0.8))


def test_observation_pose_without_geodesic_scores_the_standoff_only() -> None:
    grid, frontier, clearance, field, _ = opening_setup()
    pose = observation_pose(frontier, grid, clearance, field, 0.3, (0.4, 1.2), 5)
    assert pose is not None
    assert pose.geodesic_m is None
    assert pose.distance_m == pytest.approx(0.8, abs=RES)


def test_observation_pose_never_in_unknown_space() -> None:
    data = room_map(gaps=[(40, 60)])
    data[35:65, 55:89] = UNKNOWN  # the whole ideal standoff band is unexplored
    data[34, 55:89] = data[65, 55:89] = OCCUPIED
    grid = make_grid(data)
    snap = FrontierTracker(settings(), footprint_width_m=0.2).update(grid)
    for f in snap.frontiers:
        pose = observation_pose(f, grid, snap.clearance_m, snap.field, 0.15, (0.4, 1.2), 5)
        if pose is not None:
            assert grid.data[pose.rc] == FREE


def test_observation_pose_none_when_no_cell_has_the_view_clearance() -> None:
    grid, frontier, clearance, field, geo = opening_setup()
    assert observation_pose(frontier, grid, clearance, field, 5.0, (0.4, 1.2), 5, geodesic=geo) is None


def test_observation_pose_none_when_every_candidate_is_unreachable() -> None:
    grid, frontier, clearance, field, geo = opening_setup()
    assert (
        observation_pose(frontier, grid, clearance, field, 0.3, (0.4, 1.2), 5, geodesic=np.full_like(geo, np.inf))
        is None
    )


def test_observation_pose_takes_the_first_candidate_with_line_of_sight(monkeypatch: pytest.MonkeyPatch) -> None:
    grid, frontier, clearance, field, geo = opening_setup()
    best = observation_pose(frontier, grid, clearance, field, 0.3, (0.4, 1.2), 5, geodesic=geo)
    calls: list[tuple[int, int]] = []

    def blocked_first(g: MapGrid, a: tuple[int, int], b: tuple[int, int]) -> bool:
        calls.append(a)
        return len(calls) > 1

    monkeypatch.setattr(frontiers, "line_of_sight", blocked_first)
    second = observation_pose(frontier, grid, clearance, field, 0.3, (0.4, 1.2), 5, geodesic=geo)
    assert best is not None and second is not None
    assert calls[0] == best.rc and second.rc == calls[1] != best.rc
    assert second.score >= best.score
    monkeypatch.setattr(frontiers, "line_of_sight", lambda g, a, b: False)
    assert observation_pose(frontier, grid, clearance, field, 0.3, (0.4, 1.2), 5, geodesic=geo) is None


def test_line_of_sight_blocked_by_occupied_or_unknown_cells() -> None:
    data = walled_room()
    grid = make_grid(data)
    assert line_of_sight(grid, (10, 10), (10, 60))
    data[10, 30] = OCCUPIED
    assert not line_of_sight(make_grid(data), (10, 10), (10, 60))
    data[10, 30] = UNKNOWN
    assert not line_of_sight(make_grid(data), (10, 10), (10, 60))
    data[10, 30] = FREE
    data[10, 60] = UNKNOWN  # the frontier cell itself is unknown: it is the end, not an obstruction
    assert line_of_sight(make_grid(data), (10, 10), (10, 60))
    data[30, 30] = OCCUPIED
    assert not line_of_sight(make_grid(data), (10, 10), (50, 50))  # diagonal


# --- tracker --------------------------------------------------------------------------------------------------------


def ids(snap: frontiers.FrontierSnapshot) -> list[int]:
    return sorted(f.id for f in snap.frontiers)


def test_tracker_ids_stable_under_origin_shift_and_growth() -> None:
    tracker = FrontierTracker(settings(), footprint_width_m=0.2)
    data = room_map(gaps=[(40, 60)])
    first = tracker.update(make_grid(data, version=(1,)))
    assert ids(first) == [1]
    shifted = np.pad(data, ((0, 0), (1, 0)), constant_values=UNKNOWN)  # slam grew the map by one column west
    second = tracker.update(make_grid(shifted, version=(2,), origin=(-RES, 0.0, 0.0)))
    assert ids(second) == [1]
    assert second.frontiers[0].centroid_xy == pytest.approx(first.frontiers[0].centroid_xy)
    grown = room_map(gaps=[(40, 60)], extra_cols=4)  # explored 0.1 m further east
    third = tracker.update(make_grid(grown, version=(3,)))
    assert ids(third) == [1]
    assert tracker.pop_retired() == []


def test_tracker_split_keeps_one_id_and_numbers_the_other() -> None:
    tracker = FrontierTracker(settings(), footprint_width_m=0.2)
    tracker.update(make_grid(room_map(gaps=[(20, 80)]), version=(1,)))
    split = room_map(gaps=[(20, 80)])
    split[45:55, ROOM_HI] = OCCUPIED
    snap = tracker.update(make_grid(split, version=(2,)))
    assert ids(snap) == [1, 2]
    assert tracker.pop_retired() == []


def test_tracker_merge_retires_one_id_with_successor() -> None:
    tracker = FrontierTracker(settings(), footprint_width_m=0.2)
    assert ids(tracker.update(make_grid(room_map(gaps=[(20, 45), (55, 80)]), version=(1,)))) == [1, 2]
    snap = tracker.update(make_grid(room_map(gaps=[(20, 80)]), version=(2,)))
    assert len(snap.frontiers) == 1
    survivor = snap.frontiers[0].id
    gone = 3 - survivor
    retired = tracker.get(gone)
    assert isinstance(retired, RetiredFrontier)
    assert retired.successor == survivor
    assert retired.reason == f"merged into {survivor}"
    assert isinstance(tracker.get(survivor), Frontier)
    assert tracker.get(99) is None
    assert tracker.pop_retired() == [retired]
    assert tracker.pop_retired() == []


def test_tracker_far_split_retires_with_a_new_successor_and_explored_without() -> None:
    tracker = FrontierTracker(settings(), footprint_width_m=0.2)
    tracker.update(make_grid(room_map(gaps=[(20, 80)]), version=(1,)))
    snap = tracker.update(make_grid(room_map(gaps=[(12, 26), (74, 88)]), version=(2,)))
    assert ids(snap) == [2, 3]
    retired = tracker.get(1)
    assert isinstance(retired, RetiredFrontier)
    assert retired.successor in (2, 3)
    assert retired.reason == f"split into {retired.successor}"
    closed = tracker.update(make_grid(room_map(gaps=[]), version=(3,)))
    assert closed.frontiers == ()
    for gone in (2, 3):
        r = tracker.get(gone)
        assert isinstance(r, RetiredFrontier)
        assert (r.reason, r.successor) == ("explored", None)


def test_tracker_epoch_reset_on_known_collapse_and_resolution_change() -> None:
    tracker = FrontierTracker(settings(), footprint_width_m=0.2)
    tracker.update(make_grid(room_map(gaps=[(20, 45), (55, 80)]), version=(1,)))
    first_epoch = tracker.epoch
    small = np.full((100, 140), UNKNOWN, np.int16)
    small[40:61, 40:61] = OCCUPIED
    small[41:60, 41:60] = FREE
    small[44:58, 60] = UNKNOWN  # 14 cells: 0.35 m
    snap = tracker.update(make_grid(small, version=(2,)))
    assert snap.epoch == first_epoch + 1
    assert EPOCH_RESET_NOTE in snap.notes
    assert ids(snap) == [1]
    assert tracker.get(2) is None  # retired ids forgotten
    assert tracker.pop_retired() == []
    again = tracker.update(make_grid(small, version=(3,)))
    assert again.epoch == snap.epoch and EPOCH_RESET_NOTE not in again.notes
    coarse = tracker.update(make_grid(small, version=(4,), res=0.05))
    assert coarse.epoch == snap.epoch + 1 and ids(coarse) == [1]


def test_tracker_recomputes_only_on_version_change(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[int] = []
    real = frontiers.extract_frontiers

    def counting(grid: MapGrid, min_cells: int) -> list[frontiers.RawFrontier]:
        calls.append(1)
        return real(grid, min_cells)

    monkeypatch.setattr(frontiers, "extract_frontiers", counting)
    tracker = FrontierTracker(settings(), footprint_width_m=0.2)
    data = room_map(gaps=[(40, 60)])
    a = tracker.update(make_grid(data, version=(1,)))
    b = tracker.update(make_grid(data.copy(), version=(1,)))
    assert a is b and len(calls) == 1
    c = tracker.update(make_grid(data, version=(2,)))
    assert c is not a and len(calls) == 2
    assert set(a.timings_ms) >= {"extract", "traversability", "field", "match"}


def test_tracker_is_thread_safe_and_computes_once_per_version(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[int] = []
    real = frontiers.extract_frontiers

    def slow(grid: MapGrid, min_cells: int) -> list[frontiers.RawFrontier]:
        calls.append(1)
        time.sleep(0.05)
        return real(grid, min_cells)

    monkeypatch.setattr(frontiers, "extract_frontiers", slow)
    tracker = FrontierTracker(settings(), footprint_width_m=0.2)
    grid = make_grid(room_map(gaps=[(40, 60)]), version=(7,))
    results: list[frontiers.FrontierSnapshot] = []
    threads = [threading.Thread(target=lambda: results.append(tracker.update(grid))) for _ in range(8)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert len(calls) == 1
    assert len({id(r) for r in results}) == 1


def test_tracker_caps_retired_ids() -> None:
    tracker = FrontierTracker(settings(retired_keep=2), footprint_width_m=0.2)
    layouts = [[(20, 40)], [], [(60, 80)], [], [(20, 40)], []]
    for version, gaps in enumerate(layouts):
        tracker.update(make_grid(room_map(gaps=gaps), version=(version,)))
    assert len(tracker.retired) == 2
    assert tracker.get(1) is None  # oldest retirement dropped
    assert isinstance(tracker.get(3), RetiredFrontier)


def test_tracker_geodesic_cached_per_version_and_start_cell() -> None:
    tracker = FrontierTracker(settings(), footprint_width_m=0.2)
    grid = make_grid(room_map(gaps=[(40, 60)]))
    snap = tracker.update(grid)
    first = tracker.geodesic(snap, cell_to_map(grid, 50, 30))
    second = tracker.geodesic(snap, cell_to_map(grid, 50, 30))
    assert not first.cached and second.cached
    assert second.distance_m is first.distance_m
    assert first.compute_ms >= 0.0 and first.note is None
    other = tracker.geodesic(snap, cell_to_map(grid, 50, 40))
    assert not other.cached


def test_tracker_geodesic_notes_an_unsnappable_or_outside_start() -> None:
    tracker = FrontierTracker(settings(), footprint_width_m=0.2)
    grid = make_grid(room_map(gaps=[(40, 60)]))
    snap = tracker.update(grid)
    outside = tracker.geodesic(snap, (-5.0, -5.0))
    assert np.isinf(outside.distance_m).all() and outside.note
    walled = tracker.geodesic(snap, cell_to_map(grid, 5, 5))  # in unknown space beyond the wall
    assert np.isinf(walled.distance_m).all() and "unreachable" in (walled.note or "")


def test_tracker_view_pose_uses_settings() -> None:
    tracker = FrontierTracker(settings(standoff_min_m=0.4, standoff_max_m=1.2), footprint_width_m=0.2)
    grid = make_grid(room_map(gaps=[(40, 60)]))
    snap = tracker.update(grid)
    geo = tracker.geodesic(snap, cell_to_map(grid, 50, 30))
    pose = tracker.view_pose(snap, snap.frontiers[0], 0.3, geo.distance_m)
    assert pose is not None and 0.4 - RES <= pose.distance_m <= 1.2 + RES


# --- facing lookup and POIs -----------------------------------------------------------------------------------------


def record(
    rid: str,
    camera: str,
    ts: float,
    pose: tuple[float, float, float] | None = (0.0, 0.0, 0.0),
    cam: tuple[list[float], list[float]] | None = None,
) -> LookupRecord:
    return LookupRecord(
        id=rid,
        ts=ts,
        tool="get_camera_image",
        camera=camera,
        width=320,
        height=240,
        file=f"{rid}.jpg",
        bytes=1,
        base_pose_map=None if pose is None else PlanarPose(x=pose[0], y=pose[1], yaw=pose[2]),
        camera_pose_base=None if cam is None else CameraPose(xyz=cam[0], rpy=cam[1]),
    )


def test_facing_lookup_front_camera_without_pose_uses_the_base_heading() -> None:
    hit = facing_lookup((2.0, 0.5), [record("a", "head", 100.0)], 30.0, 6.0, now=160.0)
    assert hit is not None
    assert hit["lookup_id"] == "a" and hit["camera"] == "head"
    assert hit["age_s"] == pytest.approx(60.0)
    assert hit["distance_m"] == pytest.approx(math.hypot(2.0, 0.5), abs=1e-3)
    assert hit["off_axis_deg"] == pytest.approx(math.degrees(math.atan2(0.5, 2.0)), abs=0.1)


def test_facing_lookup_angle_and_distance_gates() -> None:
    recs = [record("a", "head", 100.0, pose=(0.0, 0.0, 0.0))]
    assert facing_lookup((0.0, 2.0), recs, 30.0, 6.0, now=160.0) is None  # 90 deg off axis
    assert facing_lookup((7.0, 0.0), recs, 30.0, 6.0, now=160.0) is None  # too far
    turned = [record("b", "head", 100.0, pose=(0.0, 0.0, math.pi / 2))]
    assert facing_lookup((0.0, 2.0), turned, 30.0, 6.0, now=160.0) is not None


def test_facing_lookup_gripper_without_pose_skipped_and_with_pose_used() -> None:
    assert facing_lookup((2.0, 0.0), [record("g", "gripper", 100.0)], 30.0, 6.0, now=160.0) is None
    side = record("g", "gripper", 100.0, cam=([0.1, 0.0, 0.3], [0.0, 0.5, math.pi / 2]))  # looks left, pitched down
    hit = facing_lookup((0.1, 2.0), [side], 30.0, 6.0, now=160.0)
    assert hit is not None and hit["off_axis_deg"] == pytest.approx(0.0, abs=0.1)
    assert hit["distance_m"] == pytest.approx(2.0, abs=1e-3)  # from the camera position, not the base
    down = record("d", "gripper", 100.0, cam=([0.0, 0.0, 0.3], [0.0, math.pi / 2, 0.0]))
    assert facing_lookup((2.0, 0.0), [down], 30.0, 6.0, now=160.0) is None  # straight down: no heading


def test_facing_lookup_newest_wins_and_unposed_records_skipped() -> None:
    recs = [
        record("old", "head", 100.0),
        record("new", "head", 200.0),
        record("nopose", "head", 300.0, pose=None),
        record("away", "head", 400.0, pose=(0.0, 0.0, math.pi)),
    ]
    hit = facing_lookup((2.0, 0.0), recs, 30.0, 6.0, now=250.0)
    assert hit is not None and hit["lookup_id"] == "new" and hit["age_s"] == pytest.approx(50.0)


def test_nearest_poi_all_kinds_within_radius() -> None:
    pois = [
        {"id": "p1", "name": "sofa", "kind": "object", "x": 3.0, "y": 0.0},
        {"id": "p2", "name": "kitchen", "kind": "area", "x": 1.0, "y": 1.0, "polygon": [[0, 0], [2, 0], [2, 2]]},
        {"id": "p3", "name": "broken", "kind": "point"},
    ]
    hit = nearest_poi((0.0, 0.0), pois, 2.0)
    assert hit == {"id": "p2", "name": "kitchen", "kind": "area", "distance_m": pytest.approx(math.sqrt(2), abs=1e-3)}
    assert nearest_poi((0.0, 0.0), pois, 1.0) is None
    near_sofa = nearest_poi((3.2, 0.0), pois, 2.0)
    assert near_sofa is not None and near_sofa["id"] == "p1"
    assert nearest_poi((0.0, 0.0), [], 2.0) is None


# --- performance ----------------------------------------------------------------------------------------------------


def big_map(size: int = 1200) -> np.ndarray:
    """A 30 m map: a 25 m house of 2.5 m square rooms with doorways, unexplored rooms and openings to unknown space."""
    data = np.full((size, size), UNKNOWN, np.int16)
    lo, hi = 100, size - 100
    data[lo:hi, lo:hi] = FREE
    data[lo, lo:hi] = data[hi - 1, lo:hi] = OCCUPIED
    data[lo:hi, lo] = data[lo:hi, hi - 1] = OCCUPIED
    for k in range(lo + 100, hi - 1, 100):
        data[k, lo:hi] = OCCUPIED
        data[lo:hi, k] = OCCUPIED
        for d in range(lo + 40, hi - 1, 100):
            data[k, d : d + 30] = FREE  # doorways
            data[d : d + 30, k] = FREE
    for k in range(lo + 150, hi - 100, 200):
        data[k : k + 40, k : k + 40] = UNKNOWN  # unexplored pockets
        data[lo, k : k + 40] = UNKNOWN  # openings in the outer wall
    return data


def test_performance_1200_map_under_two_seconds() -> None:
    grid = make_grid(big_map())
    tracker = FrontierTracker(settings(), footprint_width_m=0.386)
    start = time.perf_counter()
    snap = tracker.update(grid)
    geo = tracker.geodesic(snap, cell_to_map(grid, 620, 620))
    poses = [tracker.view_pose(snap, f, 0.35, geo.distance_m) for f in snap.frontiers[:15]]
    elapsed = time.perf_counter() - start
    print(
        f"1200x1200 frontiers: {len(snap.frontiers)} in {elapsed * 1000:.0f} ms "
        f"(stages {snap.timings_ms}, geodesic {geo.compute_ms:.0f} ms)"
    )
    assert len(snap.frontiers) >= 5
    assert any(p is not None for p in poses)
    assert elapsed < 2.0
