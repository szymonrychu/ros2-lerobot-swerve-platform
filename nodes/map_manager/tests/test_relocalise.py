"""Tests for the pure relocalisation logic (candidate search around a pose hint and the localisation verdict)."""

import math
from pathlib import Path

import numpy as np
import pytest
from pydantic import ValidationError
from ros2_common.maps import MapCommand, Pose2D
from ros2_common.scan_agreement import GridMap, agreement, prepare_map, transform_points, yaw_quaternion

from map_manager.config import MapManagerConfig
from map_manager.manager import ACTIVE_MAP_FILE
from map_manager.relocalise import LocalisationJudge, SearchSettings, search_candidates, yaw_difference
from tests.fakes import room_grid as fake_room_grid
from tests.fakes import wall_points_base
from tests.test_manager import SYMMETRIC_SEARCH, TRUE_POSE, Rig, make_rig

RESOLUTION = 0.05
GRID_SIZE_M = 8.0
BEAMS = 360
RAY_STEP_M = 0.01


def room_grid(interior: np.ndarray) -> GridMap:
    """Grid with free interior cells, a one-cell occupied wall around them and unknown outside.

    Args:
        interior (np.ndarray): (H, W) True for free cells inside the room.

    Returns:
        GridMap: The room grid with its origin at (0, 0).
    """
    padded = np.pad(interior, 1)
    grown = np.zeros_like(interior)
    for dy in range(3):
        for dx in range(3):
            grown |= padded[dy : dy + interior.shape[0], dx : dx + interior.shape[1]]
    cells = np.full(interior.shape, -1, dtype=np.int8)
    cells[grown] = 100
    cells[interior] = 0
    return GridMap(resolution=RESOLUTION, origin_x=0.0, origin_y=0.0, origin_yaw=0.0, cells=cells)


def rect_mask(x0: float, y0: float, x1: float, y1: float) -> np.ndarray:
    """Interior mask of an axis-aligned rectangle in metres."""
    cells = int(GRID_SIZE_M / RESOLUTION)
    centres = (np.arange(cells) + 0.5) * RESOLUTION
    xs, ys = np.meshgrid(centres, centres)
    return (xs > x0) & (xs < x1) & (ys > y0) & (ys < y1)


def l_room() -> GridMap:
    """L-shaped room: a 6 x 3 m arm along x joined to a 3 x 6 m arm along y."""
    return room_grid(rect_mask(1.0, 1.0, 7.0, 4.0) | rect_mask(1.0, 1.0, 4.0, 7.0))


def square_room() -> GridMap:
    """A 4 x 4 m square room."""
    return room_grid(rect_mask(1.0, 1.0, 5.0, 5.0))


def cast_scan(grid: GridMap, pose: Pose2D) -> np.ndarray:
    """Base-frame endpoints of a 360-beam scan ray-cast from pose to the first occupied cell.

    Args:
        grid (GridMap): Room grid.
        pose (Pose2D): Sensor pose in the map frame.

    Returns:
        np.ndarray: (N, 2) endpoints in the base frame.
    """
    angles = np.arange(BEAMS) * (2 * math.pi / BEAMS)
    steps = np.arange(1, int(GRID_SIZE_M / RAY_STEP_M)) * RAY_STEP_M
    world = pose.yaw + angles
    xs = pose.x + np.outer(np.cos(world), steps)
    ys = pose.y + np.outer(np.sin(world), steps)
    cols = np.clip(np.floor(xs / grid.resolution).astype(int), 0, grid.cells.shape[1] - 1)
    rows = np.clip(np.floor(ys / grid.resolution).astype(int), 0, grid.cells.shape[0] - 1)
    hit = grid.cells[rows, cols] >= 65
    assert hit.any(axis=1).all()
    ranges = steps[hit.argmax(axis=1)]
    return np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))


def test_search_recovers_offset_hint() -> None:
    grid = l_room()
    truth = Pose2D(x=2.2, y=2.6, yaw=0.3)
    points = cast_scan(grid, truth)
    hint = Pose2D(x=truth.x + 0.36, y=truth.y - 0.48, yaw=truth.yaw + math.radians(40))
    settings = SearchSettings()
    prepared = prepare_map(grid, settings.occupied_threshold, settings.hit_radius_cells)

    result = search_candidates(prepared, points, hint, settings)

    assert result.ok, result.reason
    assert not result.ambiguous, result.reason
    assert math.hypot(result.pose.x - truth.x, result.pose.y - truth.y) <= 0.1
    assert abs(math.degrees(yaw_difference(result.pose.yaw, truth.yaw))) <= 5.0
    assert result.evaluated >= settings.min_points
    assert result.ratio is not None and result.ratio >= 0.9
    # The reported score is the shared agreement metric at the returned pose.
    shared = agreement(
        transform_points(points, (result.pose.x, result.pose.y, 0.0), yaw_quaternion(result.pose.yaw)),
        prepared,
        settings.min_points,
    )
    assert shared is not None
    assert (shared.ratio, shared.evaluated, shared.agreeing) == (result.ratio, result.evaluated, result.agreeing)


def test_search_flags_symmetric_room_as_ambiguous() -> None:
    grid = square_room()
    truth = Pose2D(x=3.0, y=3.0, yaw=0.2)
    points = cast_scan(grid, truth)
    settings = SearchSettings()
    prepared = prepare_map(grid, settings.occupied_threshold, settings.hit_radius_cells)

    result = search_candidates(prepared, points, truth, settings)

    assert result.ok, result.reason
    assert result.ambiguous
    assert "ambiguous" in result.reason
    assert result.rival is not None
    far = math.hypot(result.rival.x - result.pose.x, result.rival.y - result.pose.y) > settings.ambiguity_distance_m
    turned = abs(math.degrees(yaw_difference(result.rival.yaw, result.pose.yaw))) > settings.ambiguity_yaw_deg
    assert far or turned


def test_search_ignores_rival_with_little_evidence() -> None:
    # Measured on the robot: a rival that only has a handful of scan points inside the known map scores a ratio
    # of 1.0 and made an unmoved robot "ambiguous". A rival must be backed by a comparable number of points.
    grid = l_room()
    truth = Pose2D(x=6.5, y=2.5, yaw=0.0)
    points = cast_scan(grid, truth)
    settings = SearchSettings()
    prepared = prepare_map(grid, settings.occupied_threshold, settings.hit_radius_cells)

    result = search_candidates(prepared, points, truth, settings)

    assert result.ok, result.reason
    assert not result.ambiguous, result.reason
    assert result.rival is None
    assert math.hypot(result.pose.x - truth.x, result.pose.y - truth.y) <= 0.1

    unsupported = search_candidates(
        prepared,
        points,
        truth,
        settings.model_copy(update={"ambiguity_min_support": 0.0, "min_evaluated_fraction": 0.01}),
    )
    assert unsupported.ambiguous, "the rival must exist so that the support rule is what removes it"


def test_search_with_too_few_points_returns_hint_not_ok() -> None:
    grid = l_room()
    truth = Pose2D(x=2.2, y=2.6, yaw=0.3)
    points = cast_scan(grid, truth)[::20]
    hint = Pose2D(x=2.5, y=2.4, yaw=0.9)
    settings = SearchSettings()
    prepared = prepare_map(grid, settings.occupied_threshold, settings.hit_radius_cells)

    result = search_candidates(prepared, points, hint, settings)

    assert not result.ok
    assert not result.ambiguous
    assert result.pose == hint
    assert result.ratio is None
    assert "too few" in result.reason


def test_search_drops_points_beyond_max_range() -> None:
    grid = l_room()
    truth = Pose2D(x=2.2, y=2.6, yaw=0.3)
    points = cast_scan(grid, truth)
    settings = SearchSettings(max_range_m=0.5)
    prepared = prepare_map(grid, settings.occupied_threshold, settings.hit_radius_cells)

    result = search_candidates(prepared, points, truth, settings)

    assert not result.ok
    assert result.pose == truth
    assert "too few" in result.reason


def test_search_low_agreement_is_not_ok() -> None:
    grid = l_room()
    rng = np.random.default_rng(3)
    points = rng.uniform(-0.8, 0.8, size=(200, 2))
    hint = Pose2D(x=2.5, y=2.5, yaw=0.0)
    settings = SearchSettings(radius_m=0.2, yaw_range_deg=10.0)
    prepared = prepare_map(grid, settings.occupied_threshold, settings.hit_radius_cells)

    result = search_candidates(prepared, points, hint, settings)

    assert not result.ok
    assert result.ratio is not None and result.ratio < settings.candidate_min_ratio
    assert "below" in result.reason


def test_search_rejects_partial_overlap_pose() -> None:
    # Live robot: a wrong pose scored 1.00 on a minority of the scan (the rest fell outside the map grid) and was
    # accepted. The map here is only a strip of the room, so every candidate sees under half of the scan.
    full = square_room()
    truth = Pose2D(x=3.0, y=3.0, yaw=0.0)
    points = cast_scan(full, truth)
    strip = GridMap(
        resolution=RESOLUTION, origin_x=0.0, origin_y=0.0, origin_yaw=0.0, cells=full.cells[: int(1.2 / RESOLUTION), :]
    )
    settings = SearchSettings()
    prepared = prepare_map(strip, settings.occupied_threshold, settings.hit_radius_cells)

    lenient = search_candidates(prepared, points, truth, settings.model_copy(update={"min_evaluated_fraction": 0.01}))
    assert lenient.ok and lenient.ratio == 1.0 and lenient.evaluated < 0.6 * len(points), "premise: accepted before"

    result = search_candidates(prepared, points, truth, settings)

    assert not result.ok
    assert result.pose == truth
    assert result.ratio is None
    assert (
        f"evaluated {lenient.evaluated}/{len(points)} = {lenient.evaluated / len(points):.2f} < 0.60" in result.reason
    )


def test_search_full_overlap_passes_fraction() -> None:
    grid = l_room()
    truth = Pose2D(x=2.2, y=2.6, yaw=0.3)
    points = cast_scan(grid, truth)
    settings = SearchSettings()
    prepared = prepare_map(grid, settings.occupied_threshold, settings.hit_radius_cells)

    result = search_candidates(prepared, points, truth, settings)

    assert result.ok, result.reason
    assert result.evaluated >= 0.6 * len(points)


def test_judge_localised() -> None:
    judge = LocalisationJudge()
    for ratio in (0.7, 0.8, 0.65, 0.9, 0.75):
        judge.add(ratio)

    state, median, reason = judge.verdict(ambiguous=False)

    assert state == "localised"
    assert median == pytest.approx(0.75)
    assert reason


def test_judge_ambiguous() -> None:
    judge = LocalisationJudge()
    for ratio in (0.9, 0.9, 0.9, 0.9, 0.9):
        judge.add(ratio)

    state, median, _ = judge.verdict(ambiguous=True)

    assert state == "ambiguous"
    assert median == pytest.approx(0.9)


def test_judge_not_localised_low_median() -> None:
    judge = LocalisationJudge(localised_ratio=0.6, min_samples=5)
    for ratio in (0.9, 0.3, 0.4, 0.5, 0.95):
        judge.add(ratio)

    state, median, reason = judge.verdict(ambiguous=False)

    assert state == "not_localised"
    assert median == pytest.approx(0.5)
    assert "below" in reason


def test_judge_too_few_samples() -> None:
    judge = LocalisationJudge(min_samples=5)
    state, median, reason = judge.verdict(ambiguous=False)
    assert (state, median, reason) == ("not_localised", None, "too few agreement samples")

    for ratio in (0.9, 0.9, 0.9, 0.9):
        judge.add(ratio)
    state, median, reason = judge.verdict(ambiguous=False)

    assert state == "not_localised"
    assert median == pytest.approx(0.9)
    assert reason == "too few agreement samples"


def test_judge_rejects_invalid() -> None:
    with pytest.raises(ValueError):
        LocalisationJudge(localised_ratio=1.5)
    with pytest.raises(ValueError):
        LocalisationJudge(min_samples=0)
    with pytest.raises(ValueError):
        LocalisationJudge().add(-0.1)
    with pytest.raises(ValueError):
        LocalisationJudge().add(float("nan"))


@pytest.mark.parametrize(
    "field,value",
    [
        ("radius_m", 0.0),
        ("step_m", -0.1),
        ("yaw_range_deg", 0.0),
        ("yaw_range_deg", 181.0),
        ("yaw_step_deg", 0.0),
        ("occupied_threshold", 0),
        ("occupied_threshold", 101),
        ("hit_radius_cells", 0),
        ("min_points", 0),
        ("max_range_m", 0.0),
        ("candidate_min_ratio", 1.5),
        ("candidate_min_ratio", -0.1),
        ("ambiguity_distance_m", 0.0),
        ("ambiguity_yaw_deg", -5.0),
        ("ambiguity_margin", 1.1),
        ("ambiguity_margin", -0.01),
        ("ambiguity_min_support", 1.1),
        ("ambiguity_min_support", -0.1),
        ("min_evaluated_fraction", 0.0),
        ("min_evaluated_fraction", 1.01),
    ],
)
def test_settings_reject_invalid(field: str, value: float) -> None:
    with pytest.raises(ValidationError):
        SearchSettings(**{field: value})


def test_settings_defaults() -> None:
    settings = SearchSettings()
    assert (settings.radius_m, settings.step_m, settings.yaw_range_deg, settings.yaw_step_deg) == (1.0, 0.1, 180, 5)
    assert (settings.occupied_threshold, settings.hit_radius_cells, settings.min_points) == (65, 1, 40)
    assert (settings.max_range_m, settings.candidate_min_ratio) == (8.0, 0.5)
    assert (settings.ambiguity_distance_m, settings.ambiguity_yaw_deg, settings.ambiguity_margin) == (0.5, 30, 0.05)
    assert settings.min_evaluated_fraction == 0.6


def test_yaw_difference_wraps() -> None:
    assert yaw_difference(math.radians(179), math.radians(-179)) == pytest.approx(math.radians(-2))
    assert yaw_difference(0.5, 0.2) == pytest.approx(0.3)


# ---------------------------------------------------------------- relocalise op (D13, U8)

RING_S = 120.0
LIVE_POIS = [{"id": "live-1", "name": "sofa"}, {"id": "live-2", "name": "tv"}]


class TickClock:
    """Settable clock for ring tests."""

    def __init__(self, now: float = 1000.0) -> None:
        self.now = now

    def __call__(self) -> float:
        return self.now


def relocalise_rig(tmp_path: Path, **config: object) -> tuple[Rig, TickClock]:
    """Rig with a settable clock and two live POIs."""
    rig = make_rig(tmp_path, **config)
    clock = TickClock()
    rig.manager.clock = clock
    rig.poi.pois = [dict(p) for p in LIVE_POIS]
    return rig, clock


def relocalise(rig: Rig, **fields: object):
    return rig.manager.handle(MapCommand(op="relocalise", request_id="r9", **fields))


def named_tree(rig: Rig) -> dict[str, bytes]:
    root = Path(rig.config.maps_root)
    return {
        str(p.relative_to(root)): p.read_bytes()
        for p in sorted(root.rglob("*"))
        if p.is_file() and p.name != ACTIVE_MAP_FILE
    }


def test_pose_ring_keeps_120_seconds(tmp_path: Path) -> None:
    rig, clock = relocalise_rig(tmp_path)
    for second in range(0, 300, 10):
        clock.now = 1000.0 + second
        rig.sensors.pose = Pose2D(x=float(second), y=0.0, yaw=0.0)
        rig.manager.record_pose()
    times = [t for t, _ in rig.manager.pose_ring]
    assert min(times) >= clock.now - RING_S
    assert max(times) == clock.now
    assert len(times) == 13


def test_pose_ring_ignores_missing_pose(tmp_path: Path) -> None:
    rig, _ = relocalise_rig(tmp_path)
    rig.sensors.pose = None
    rig.manager.record_pose()
    assert len(rig.manager.pose_ring) == 0


def test_hint_chosen_by_since(tmp_path: Path) -> None:
    rig, clock = relocalise_rig(tmp_path)
    poses = {1000.0: Pose2D(x=1.0, y=0.8, yaw=0.0), 1010.0: Pose2D(x=5.0, y=5.0, yaw=0.0)}
    for stamp, pose in poses.items():
        clock.now = stamp
        rig.sensors.pose = pose
        rig.manager.record_pose()
    clock.now = 1020.0
    result = relocalise(rig, since=1005.0)
    assert result.data["hint"] == poses[1000.0].model_dump()
    result = relocalise(rig)
    assert result.data["hint"] == poses[1010.0].model_dump()


def test_no_ring_pose_is_refused_and_changes_nothing(tmp_path: Path) -> None:
    rig, clock = relocalise_rig(tmp_path)
    rig.sensors.pose = TRUE_POSE
    rig.manager.record_pose()
    rig.log.calls.clear()
    result = relocalise(rig, since=clock.now - 50.0, discard_on_fail=True)
    assert not result.ok
    assert "no last known pose" in result.message
    assert "slam.serialize" not in rig.log.names()
    assert "poi.replace" not in rig.log.names()
    assert rig.poi.pois == LIVE_POIS
    empty, _ = relocalise_rig(tmp_path / "empty")
    assert "no last known pose" in relocalise(empty).message


def test_recovered_move_keeps_pois_and_state(tmp_path: Path) -> None:
    rig, clock = relocalise_rig(tmp_path)
    rig.manager.record_pose()
    before = rig.manager.status()
    clock.now += 5
    result = relocalise(rig, discard_on_fail=True)
    assert result.ok, result.message
    assert result.data["state"] == "localised"
    assert rig.manager.localised() is True
    assert rig.manager.status().load_seq == before.load_seq
    assert rig.manager.status().active_map == before.active_map
    assert "poi.replace" not in rig.log.names()
    assert rig.poi.pois == LIVE_POIS
    assert rig.slam.paused is False
    names = rig.log.names()
    assert names.index("slam.serialize") < names.index("slam.deserialize") < names.index("slam.wait_pose_after")


def unrecoverable(tmp_path: Path, discard: bool) -> tuple[Rig, object]:
    rig, clock = relocalise_rig(tmp_path)
    rig.save_map("home")
    rig.manager.record_pose()
    rig.slam.confirmed_pose = None
    rig.log.calls.clear()
    clock.now += 5
    return rig, relocalise(rig, discard_on_fail=discard)


def test_unrecoverable_with_discard_empties_pois_once(tmp_path: Path) -> None:
    rig, result = unrecoverable(tmp_path, True)
    assert not result.ok
    assert result.data["discarded_pois"] == len(LIVE_POIS)
    assert result.data["discarded_photos"] == 0
    assert rig.manager.status().active_map is None
    assert rig.log.args_of("poi.replace") == [([],)]
    assert rig.poi.pois == []
    assert rig.slam.paused is True
    assert rig.manager.status().localisation.state == "not_localised"
    assert rig.manager.localised() is False


def test_unrecoverable_without_discard_keeps_pois(tmp_path: Path) -> None:
    rig, result = unrecoverable(tmp_path, False)
    assert not result.ok
    assert result.data["discarded_pois"] == 0
    assert "poi.replace" not in rig.log.names()
    assert rig.poi.pois == LIVE_POIS
    assert rig.slam.paused is True


def test_ambiguous_with_discard_empties_pois(tmp_path: Path) -> None:
    rig, clock = relocalise_rig(tmp_path, search=SYMMETRIC_SEARCH)
    square = fake_room_grid(3.0, 3.0)
    centre = Pose2D(x=1.5, y=1.5, yaw=0.0)
    rig.sensors.grid = square
    rig.sensors.points = wall_points_base(square, centre)
    rig.sensors.pose = centre
    rig.slam.confirmed_pose = centre
    rig.manager.record_pose()
    result = relocalise(rig, discard_on_fail=True)
    assert not result.ok
    assert rig.manager.status().localisation.state == "ambiguous"
    assert rig.poi.pois == []
    assert result.data["discarded_pois"] == 2


def test_discard_replace_failure_reported(tmp_path: Path) -> None:
    rig, clock = relocalise_rig(tmp_path)
    rig.manager.record_pose()
    rig.slam.confirmed_pose = None
    rig.poi.replace_ok = False
    result = relocalise(rig, discard_on_fail=True)
    assert not result.ok
    assert "replace rejected" in result.message
    assert result.data["discarded_pois"] == 0


@pytest.mark.parametrize("discard", [False, True])
def test_named_bundles_untouched_and_scratch_removed(tmp_path: Path, discard: bool) -> None:
    rig, clock = relocalise_rig(tmp_path)
    rig.save_map("home")
    rig.save_map("office")
    rig.manager.record_pose()
    tree = named_tree(rig)
    rig.slam.confirmed_pose = None
    relocalise(rig, discard_on_fail=discard)
    assert named_tree(rig) == tree
    assert not (Path(rig.config.maps_root) / ".relocalise").exists() or not any(
        (Path(rig.config.maps_root) / ".relocalise").iterdir()
    )
    assert [m.name for m in rig.manager.status().maps] == ["home", "office"]
    scratch = [args[0] for args in rig.log.args_of("slam.serialize")]
    assert all("/.relocalise/" in base for base in scratch)
    assert not any(Path(base).parent.exists() for base in scratch)


def test_no_autosave_while_relocalising(tmp_path: Path) -> None:
    rig, _ = relocalise_rig(tmp_path)
    rig.manager.record_pose()
    relocalise(rig)
    assert len(rig.log.args_of("slam.serialize")) == 1
    assert not (Path(rig.config.maps_root) / "autosave").exists()


def test_relocalise_busy_rejection(tmp_path: Path) -> None:
    rig, _ = relocalise_rig(tmp_path)
    rig.manager.record_pose()
    nested: list = []
    rig.slam.on_deserialize = lambda: nested.append(relocalise(rig))
    result = relocalise(rig)
    assert result.ok, result.message
    assert not nested[0].ok
    assert nested[0].message.startswith("busy")


def test_serialize_failure_changes_nothing(tmp_path: Path) -> None:
    rig, _ = relocalise_rig(tmp_path)
    rig.manager.record_pose()
    rig.slam.serialize_ok = False
    rig.log.calls.clear()
    result = relocalise(rig, discard_on_fail=True)
    assert not result.ok
    assert "slam.deserialize" not in rig.log.names()
    assert "poi.replace" not in rig.log.names()
    assert rig.poi.pois == LIVE_POIS


def test_pose_ring_config_default() -> None:
    assert MapManagerConfig().pose_ring_s == 120.0
