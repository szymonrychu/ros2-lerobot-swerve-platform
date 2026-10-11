"""Tests of the forward-sampled inverse reachability map (mcp_server.reach_map) and ReachMapSettings."""

import hashlib
import json
import math
import tracemalloc
from pathlib import Path

import numpy as np
import pytest
import yaml
from pydantic import ValidationError

from mcp_server import arm_reach, reach_map
from mcp_server.arm_reach import ENVELOPE_BATCH_SAMPLES, POINT_PITCHES_DEG, ReachEnvelope, joint_range, point_reach
from mcp_server.body_model import BodyModel
from mcp_server.config import ArmBaseOffset, BodyCheckSettings, McpServerConfig, ReachMapSettings
from mcp_server.floor_guard import JawModel, arm_to_base_link
from mcp_server.ik import ArmKinematics, CartesianPose
from mcp_server.reach_map import (
    REACH_MAP_VERSION,
    ReachMap,
    ReachMapMissing,
    ReachMapStale,
    build,
    checker_from_config,
    pan_geometry,
    reach_inputs,
    reach_map_key,
)
from mcp_server.self_collision import SampleGeometry, SelfCollisionChecker, link_model_description

CONFIG = McpServerConfig()
CHECKER = checker_from_config(CONFIG)
# Coarse grid fixture: the deployed r / z cells at every 30 deg of heading and a lower planar density (about 12 s per
# build on the M4). Measured on it: doubling the density adds 2.2 % reachable cell-pitches; 19 of the 20 sampled
# reachable cells are confirmed by point_reach (the miss is a cell 4 cm from the pan axis the numeric IK does not find).
COARSE = ReachMapSettings(
    heading_step_deg=30.0,
    r_step_m=0.02,
    r_max_m=0.48,
    z_step_m=0.025,
    z_max_m=0.45,
    planar_samples=(48, 48, 26),
    samples_per_cell=2,
)
DOUBLED = COARSE.model_copy(update={"planar_samples": (96, 96, 52)})
# Tiny grid for the CLI round trip.
TINY = {
    "heading_step_deg": 90.0,
    "r_step_m": 0.08,
    "r_max_m": 0.48,
    "z_step_m": 0.1,
    "z_max_m": 0.4,
    "planar_samples": [8, 8, 5],
    "samples_per_cell": 1,
}
AGREEMENT_CELLS = 20
AGREEMENT_SEED = 20261011
MEMORY_LIMIT_BYTES = 64 * 2**20


@pytest.fixture(scope="module")
def coarse_traced() -> tuple[ReachMap, int]:
    """The coarse map built under tracemalloc, with its peak traced allocation (bytes)."""
    tracemalloc.start()
    try:
        result = build(CHECKER, COARSE)
        _, peak = tracemalloc.get_traced_memory()
    finally:
        tracemalloc.stop()
    return result, peak


@pytest.fixture(scope="module")
def coarse() -> ReachMap:
    """The coarse map, built without tracing."""
    return build(CHECKER, COARSE)


def tool_point(checker: SelfCollisionChecker, joints: dict[str, float]) -> np.ndarray:
    pose = checker.kin.forward(joints)
    return arm_to_base_link((pose.x, pose.y, pose.z), checker.mount)


# --- settings -------------------------------------------------------------------------------------------------------


def test_reach_map_settings_defaults() -> None:
    settings = ReachMapSettings()
    assert settings.heading_step_deg == 5.0
    assert settings.r_step_m == 0.02
    assert settings.r_max_m == 0.48
    assert settings.z_step_m == 0.025
    assert settings.z_max_m == 0.45
    assert tuple(settings.planar_samples) == (72, 72, 37)
    assert settings.samples_per_cell == 2
    assert settings.file == Path(reach_map.__file__).resolve().parent / "data" / "reach_map.npz"
    assert McpServerConfig().reach_map == settings


def test_reach_map_settings_reject_bad_values() -> None:
    with pytest.raises(ValidationError):
        ReachMapSettings(r_step_m=0.0)
    with pytest.raises(ValidationError):
        ReachMapSettings(planar_samples=(72, 72))
    with pytest.raises(ValidationError):
        ReachMapSettings(planar_samples=(72, 1, 37))
    with pytest.raises(ValidationError):
        ReachMapSettings(samples_per_cell=0)
    with pytest.raises(ValidationError):
        ReachMapSettings(r_step_m=0.1, r_max_m=0.05)
    with pytest.raises(ValidationError):
        ReachMapSettings(unknown=1)


# --- key ------------------------------------------------------------------------------------------------------------


def old_envelope_key(checker: SelfCollisionChecker, settings: BodyCheckSettings) -> str:
    """ReachEnvelope.cache_key exactly as written before reach_inputs was extracted from it."""
    kin = checker.kin
    inputs = {
        "version": arm_reach.CACHE_VERSION,
        "body": checker.body.describe(),
        "links": link_model_description(),
        "mount": checker.mount.model_dump(),
        "limits": {k: list(v) for k, v in sorted(kin.limits.items())},
        "offsets": dict(sorted(kin.offsets.items())),
        "margin": kin.margin,
        "settings": settings.model_dump(
            include={"hard_margin_m", "reach_heading_step_deg", "reach_height_max_m", "reach_height_step_m"}
        ),
        "samples": [arm_reach.ENVELOPE_SAMPLES_PER_JOINT, arm_reach.ENVELOPE_WRIST_SAMPLES],
    }
    return hashlib.sha256(json.dumps(inputs, sort_keys=True, default=str).encode()).hexdigest()


def test_envelope_key_is_unchanged_by_the_extraction() -> None:
    """ReachEnvelope keeps its key (a deployed disk cache stays valid) for the default and the deployed config."""
    assert ReachEnvelope(CHECKER, CONFIG.body_check).cache_key() == old_envelope_key(CHECKER, CONFIG.body_check)
    deployed = reach_map.config_from_client_yml(reach_map.REPO_ROOT / "ansible/group_vars/client.yml")
    checker = checker_from_config(deployed)
    assert ReachEnvelope(checker, deployed.body_check).cache_key() == old_envelope_key(checker, deployed.body_check)


def test_reach_inputs_cover_body_links_mount_limits_offsets_margin() -> None:
    inputs = reach_inputs(CHECKER)
    assert set(inputs) == {"body", "links", "mount", "limits", "offsets", "margin"}
    assert inputs["mount"] == CHECKER.mount.model_dump()


def with_limits(overrides: dict[str, tuple[float, float]]) -> SelfCollisionChecker:
    kin = ArmKinematics(CONFIG.arm.urdf_path, margin=CONFIG.limits.arm_limit_margin_rad, limit_overrides=overrides)
    return SelfCollisionChecker.from_config(
        kin, CONFIG, JawModel(kin, CONFIG.arm.jaw_open_axis, CONFIG.arm.gripper_closed_rad)
    )


def variants() -> dict[str, tuple[SelfCollisionChecker, ReachMapSettings]]:
    kin, settings = CHECKER.kin, COARSE
    jaw = CHECKER.jaw
    body = BodyModel(CHECKER.body.obstacles[1:])
    mount = CHECKER.mount.model_copy(update={"x": CHECKER.mount.x + 0.01})
    tool_kin = ArmKinematics(
        CONFIG.arm.urdf_path, margin=CONFIG.limits.arm_limit_margin_rad, tool_offset=(0.0, 0.0, 0.01)
    )
    tool_jaw = JawModel(tool_kin, CONFIG.arm.jaw_open_axis, CONFIG.arm.gripper_closed_rad)
    return {
        "body": (SelfCollisionChecker(kin, body, CONFIG.body_check, CHECKER.mount, jaw), settings),
        "mount": (
            SelfCollisionChecker(kin, CHECKER.body, CONFIG.body_check, ArmBaseOffset(**mount.model_dump()), jaw),
            settings,
        ),
        "limits": (with_limits({"elbow_flex": (-1.0, 1.0)}), settings),
        "tool": (SelfCollisionChecker(tool_kin, CHECKER.body, CONFIG.body_check, CHECKER.mount, tool_jaw), settings),
        "hard_margin": (
            SelfCollisionChecker(
                kin, CHECKER.body, CONFIG.body_check.model_copy(update={"hard_margin_m": 0.005}), CHECKER.mount, jaw
            ),
            settings,
        ),
        "grid_heading": (CHECKER, settings.model_copy(update={"heading_step_deg": 15.0})),
        "grid_r": (CHECKER, settings.model_copy(update={"r_step_m": 0.03})),
        "grid_z": (CHECKER, settings.model_copy(update={"z_max_m": 0.40})),
        "planar_samples": (CHECKER, settings.model_copy(update={"planar_samples": (48, 48, 27)})),
        "samples_per_cell": (CHECKER, settings.model_copy(update={"samples_per_cell": 3})),
    }


@pytest.mark.parametrize("change", sorted(variants()))
def test_key_changes_with_every_input(change: str) -> None:
    checker, settings = variants()[change]
    assert reach_map_key(checker, settings) != reach_map_key(CHECKER, COARSE)


def test_key_ignores_the_file_location_and_includes_the_version(monkeypatch: pytest.MonkeyPatch) -> None:
    base = reach_map_key(CHECKER, COARSE)
    assert reach_map_key(CHECKER, COARSE.model_copy(update={"file": Path("/elsewhere.npz")})) == base
    monkeypatch.setattr(reach_map, "REACH_MAP_VERSION", REACH_MAP_VERSION + 1)
    assert reach_map_key(CHECKER, COARSE) != base


# --- pan axis -------------------------------------------------------------------------------------------------------


def test_pan_axis_is_vertical() -> None:
    """FK at pan theta equals FK at pan 0 rotated about the pan axis by theta (1e-6 m), and that axis is vertical
    (the URDF's rpy 3.14159 tilts it 2.65e-6 rad, so the map's vertical rotation is off by under 2 um)."""
    geometry = pan_geometry(CHECKER)
    assert abs(abs(geometry.direction[2]) - 1.0) < 1e-9
    assert geometry.sign == math.copysign(1.0, geometry.direction[2])
    rng = np.random.default_rng(7)
    lo, hi = joint_range(CHECKER, "shoulder_pan")
    for _ in range(25):
        joints = {
            j: float(rng.uniform(*joint_range(CHECKER, j))) for j in ("shoulder_lift", "elbow_flex", "wrist_flex")
        }
        joints["wrist_roll"] = 0.0
        theta = float(rng.uniform(lo, hi))
        at_zero = tool_point(CHECKER, joints | {"shoulder_pan": 0.0})
        at_theta = tool_point(CHECKER, joints | {"shoulder_pan": theta})
        rotated = reach_map.rotate_about(at_zero[None, :], geometry.axis, geometry.direction, theta)[0]
        assert np.linalg.norm(at_theta - rotated) < 1e-6
        # The same about the exactly vertical line through the axis (what the map does): micrometres.
        angle = geometry.sign * theta
        vertical = reach_map.rotate_about(at_zero[None, :], geometry.axis, np.array([0.0, 0.0, 1.0]), angle)[0]
        assert np.linalg.norm(at_theta - vertical) < 5e-6


def test_build_refuses_a_tilted_pan_axis(monkeypatch: pytest.MonkeyPatch) -> None:
    kin = ArmKinematics(CONFIG.arm.urdf_path, margin=CONFIG.limits.arm_limit_margin_rad)
    jaw = JawModel(kin, CONFIG.arm.jaw_open_axis, CONFIG.arm.gripper_closed_rad)
    checker = SelfCollisionChecker.from_config(kin, CONFIG, jaw)
    original = kin.forward

    def tilted(joints: dict[str, float], extra_offset: tuple[float, float, float] | None = None) -> CartesianPose:
        pose = original(joints, extra_offset)
        return CartesianPose(x=pose.x, y=pose.y, z=pose.z + 0.01 * joints.get("shoulder_pan", 0.0), pitch=pose.pitch)

    monkeypatch.setattr(kin, "forward", tilted)
    with pytest.raises(ValueError, match="vertical"):
        pan_geometry(checker)
    with pytest.raises(ValueError, match="vertical"):
        build(checker, ReachMapSettings(**TINY))


def test_build_refuses_a_tilted_mount_axis(monkeypatch: pytest.MonkeyPatch) -> None:
    kin = ArmKinematics(CONFIG.arm.urdf_path, margin=CONFIG.limits.arm_limit_margin_rad)
    checker = SelfCollisionChecker.from_config(
        kin, CONFIG, JawModel(kin, CONFIG.arm.jaw_open_axis, CONFIG.arm.gripper_closed_rad)
    )
    tilt = 0.01
    monkeypatch.setattr(
        checker,
        "rotation",
        np.array([[1.0, 0.0, 0.0], [0.0, math.cos(tilt), -math.sin(tilt)], [0.0, math.sin(tilt), math.cos(tilt)]]),
    )
    with pytest.raises(ValueError, match="not vertical"):
        pan_geometry(checker)


# --- build ----------------------------------------------------------------------------------------------------------


def test_build_shapes_grid_and_meta(coarse: ReachMap) -> None:
    grid = coarse.grid
    assert coarse.pitch_mask.dtype == np.uint16
    assert coarse.clearance.dtype == np.float16
    shape = (len(grid["headings_deg"]), len(grid["r_m"]), len(grid["z_m"]))
    assert coarse.pitch_mask.shape == coarse.clearance.shape == shape
    assert grid["r_m"][0] == pytest.approx(COARSE.r_step_m) and grid["r_m"][-1] == pytest.approx(COARSE.r_max_m)
    assert grid["z_m"][0] == pytest.approx(COARSE.z_step_m) and grid["z_m"][-1] == pytest.approx(COARSE.z_max_m)
    assert grid["pitches_deg"] == list(POINT_PITCHES_DEG)
    lo, hi = (math.degrees(v) for v in joint_range(CHECKER, "shoulder_pan"))
    assert all(lo <= h <= hi for h in grid["headings_deg"])
    assert all(
        round(h / COARSE.heading_step_deg) * COARSE.heading_step_deg == pytest.approx(h) for h in grid["headings_deg"]
    )
    meta = coarse.meta
    assert meta["key"] == reach_map_key(CHECKER, COARSE)
    assert meta["version"] == REACH_MAP_VERSION
    assert meta["planar_samples"] == [48, 48, 26]
    assert meta["compute_s"] > 0 and meta["created_at"]
    counts = meta["counts"]
    assert counts["reachable_cell_pitches"] == int(sum(bin(int(v)).count("1") for v in coarse.pitch_mask.ravel()))
    assert counts["reachable_cells"] == int((coarse.pitch_mask > 0).sum()) > 0
    assert (
        0
        < counts["checked_samples"]
        <= len(grid["headings_deg"]) * coarse.pitch_mask[0].size * 9 * COARSE.samples_per_cell
    )
    # Clearance where some pitch works is at least the hard margin; no pitch at all means no clearance stored.
    reachable = coarse.pitch_mask > 0
    assert np.all(coarse.clearance[reachable].astype(float) >= CONFIG.body_check.hard_margin_m - 1e-3)
    assert np.all(np.isnan(coarse.clearance[~reachable].astype(float)))


def test_build_is_deterministic(coarse: ReachMap, coarse_traced: tuple[ReachMap, int]) -> None:
    again, _ = coarse_traced
    assert np.array_equal(coarse.pitch_mask, again.pitch_mask)
    assert np.array_equal(coarse.clearance, again.clearance, equal_nan=True)
    assert coarse.meta["counts"] == again.meta["counts"]
    assert coarse.key == again.key


def test_build_peak_memory_is_bounded(coarse_traced: tuple[ReachMap, int]) -> None:
    _, peak = coarse_traced
    assert peak < MEMORY_LIMIT_BYTES


def test_build_checks_bounded_batches(monkeypatch: pytest.MonkeyPatch) -> None:
    sizes: list[int] = []
    original = CHECKER.geometry

    def recording(samples: list[dict[str, float]]) -> SampleGeometry:
        sizes.append(len(samples))
        return original(samples)

    monkeypatch.setattr(CHECKER, "geometry", recording)
    build(CHECKER, ReachMapSettings(**TINY))
    assert sizes and max(sizes) <= ENVELOPE_BATCH_SAMPLES


def test_body_verdict_equals_clear_of() -> None:
    """The map's clearance pass gives clear_of's verdict at hard_margin_m for every sample."""
    rng = np.random.default_rng(3)
    samples = [
        {
            j: float(rng.uniform(*joint_range(CHECKER, j)))
            for j in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")
        }
        | {"wrist_roll": 0.0}
        for _ in range(4 * ENVELOPE_BATCH_SAMPLES)
    ]
    for start in range(0, len(samples), ENVELOPE_BATCH_SAMPLES):
        geometry = CHECKER.geometry(samples[start : start + ENVELOPE_BATCH_SAMPLES])
        clearance = reach_map.body_clearance(CHECKER, geometry)
        hard = CONFIG.body_check.hard_margin_m
        assert np.array_equal(clearance >= hard, CHECKER.clear_of(geometry, hard))
        assert np.all(clearance <= CONFIG.body_check.soft_margin_m + 1e-12)


def test_convergence_doubling_planar_samples(coarse: ReachMap) -> None:
    """Doubling the planar density changes the reachable cell-pitch count by less than 3 %."""
    dense = build(CHECKER, DOUBLED)
    base = coarse.meta["counts"]["reachable_cell_pitches"]
    change = abs(dense.meta["counts"]["reachable_cell_pitches"] - base) / base
    assert change < 0.03, f"{base} -> {dense.meta['counts']['reachable_cell_pitches']} ({change:.1%})"


def cell_point(rmap: ReachMap, h: int, r: int, z: int) -> dict[str, float | None]:
    grid = rmap.grid
    axis = grid["pan_axis_base_link"]
    heading = math.radians(grid["headings_deg"][h])
    radius = grid["r_m"][r]
    return {
        "x": axis[0] + radius * math.cos(heading),
        "y": axis[1] + radius * math.sin(heading),
        "z": grid["z_m"][z],
        "pitch": None,
    }


def test_agreement_with_point_reach(coarse: ReachMap) -> None:
    """20 reachable cells are collision-free per point_reach (>= 90 %), 20 cells unreachable at every pitch are not."""
    rng = np.random.default_rng(AGREEMENT_SEED)
    envelope = ReachEnvelope(CHECKER, CONFIG.body_check)
    reachable = np.argwhere(coarse.pitch_mask > 0)
    unreachable = np.argwhere(coarse.pitch_mask == 0)
    hits = 0
    for idx in rng.choice(len(reachable), AGREEMENT_CELLS, replace=False):
        result = point_reach(CHECKER, cell_point(coarse, *reachable[idx]), arm_reach.DEFAULT_SEED, envelope)
        hits += bool(result["collision_free"])
    misses = 0
    for idx in rng.choice(len(unreachable), AGREEMENT_CELLS, replace=False):
        result = point_reach(CHECKER, cell_point(coarse, *unreachable[idx]), arm_reach.DEFAULT_SEED, envelope)
        misses += not result["collision_free"]
    assert hits >= 0.9 * AGREEMENT_CELLS, f"reachable cells confirmed: {hits}/{AGREEMENT_CELLS}"
    assert misses >= 0.9 * AGREEMENT_CELLS, f"unreachable cells confirmed: {misses}/{AGREEMENT_CELLS}"


# --- save / load ----------------------------------------------------------------------------------------------------


def test_save_and_load_round_trip(coarse: ReachMap, tmp_path: Path) -> None:
    path = tmp_path / "sub" / "reach_map.npz"
    coarse.save(path)
    assert path.exists() and not list(path.parent.glob("*.tmp"))
    loaded = ReachMap.load(path, CHECKER, COARSE)
    assert np.array_equal(loaded.pitch_mask, coarse.pitch_mask)
    assert np.array_equal(loaded.clearance, coarse.clearance, equal_nan=True)
    assert loaded.meta == coarse.meta


def test_load_missing_file_raises(tmp_path: Path) -> None:
    with pytest.raises(ReachMapMissing):
        ReachMap.load(tmp_path / "absent.npz", CHECKER, COARSE)


def test_load_unreadable_file_raises_missing(tmp_path: Path) -> None:
    path = tmp_path / "broken.npz"
    path.write_bytes(b"not a zip")
    with pytest.raises(ReachMapMissing):
        ReachMap.load(path, CHECKER, COARSE)


def test_load_stale_file_raises_with_both_keys(coarse: ReachMap, tmp_path: Path) -> None:
    path = tmp_path / "reach_map.npz"
    coarse.save(path)
    other = COARSE.model_copy(update={"heading_step_deg": 15.0})
    with pytest.raises(ReachMapStale) as info:
        ReachMap.load(path, CHECKER, other)
    assert info.value.expected == reach_map_key(CHECKER, other)
    assert info.value.found == coarse.key
    assert info.value.expected in str(info.value) and info.value.found in str(info.value)


# --- query ----------------------------------------------------------------------------------------------------------


def synthetic(mask: np.ndarray, headings: list[float], radii: list[float], heights: list[float]) -> ReachMap:
    clearance = np.where(mask > 0, 0.01, np.nan).astype(np.float16)
    grid = {
        "heading_step_deg": headings[1] - headings[0] if len(headings) > 1 else 360.0,
        "headings_deg": headings,
        "r_step_m": radii[0],
        "r_m": radii,
        "z_step_m": heights[0],
        "z_m": heights,
        "pitches_deg": list(POINT_PITCHES_DEG),
        "pan_axis_base_link": [0.0, 0.0, 0.0],
    }
    return ReachMap(mask.astype(np.uint16), clearance, {"key": "synthetic", "grid": grid})


def test_margins_on_a_synthetic_mask() -> None:
    """margin_m = horizontal distance to the nearest unreachable cell centre (grid cells plus the ring outside it)."""
    headings, radii, heights = [-90.0, 0.0, 90.0], [0.1, 0.2, 0.3, 0.4], [0.1, 0.2]
    mask = np.zeros((3, 4, 2), dtype=np.uint16)
    mask[1, 1, 0] = 0b1  # heading 0, r 0.2, pitch -30 only: neighbours r 0.1 / 0.3 are 0.1 away
    mask[:, :, 1] = 0b11  # the whole upper slice: only the ring outside the grid is unreachable
    rmap = synthetic(mask, headings, radii, heights)

    low = rmap.cells(0.1, None)
    assert low.heading_deg.tolist() == [0.0] and low.r_m.tolist() == [0.2]
    assert low.margin_m.tolist() == pytest.approx([0.1])
    assert low.pitches == [[-30.0]]
    assert low.clearance.tolist() == pytest.approx([0.01], abs=1e-4)

    high = rmap.cells(0.2, None)
    by_cell = {(h, r): m for h, r, m in zip(high.heading_deg, high.r_m, high.margin_m, strict=True)}
    # Outer ring r 0.5: a cell at r 0.4 is 0.1 from it.
    assert by_cell[(0.0, 0.4)] == pytest.approx(0.1)
    # Heading 0, r 0.1: the out-of-range rows at +-180 deg (r 0.1) are 0.2 away; the ring is 0.4 away.
    assert by_cell[(0.0, 0.1)] == pytest.approx(0.2)
    # Heading 90, r 0.1: the row at 180 deg, r 0.1 is sqrt(0.02) away.
    assert by_cell[(90.0, 0.1)] == pytest.approx(math.sqrt(0.02))
    assert high.pitches[0] == [-30.0, -15.0]


def test_cells_filter_by_pitch_and_height() -> None:
    headings, radii, heights = [0.0, 30.0], [0.1, 0.2], [0.1]
    mask = np.zeros((2, 2, 1), dtype=np.uint16)
    mask[0, 0, 0] = 1 << POINT_PITCHES_DEG.index(45.0)
    mask[1, 1, 0] = 1 << POINT_PITCHES_DEG.index(0.0)
    rmap = synthetic(mask, headings, radii, heights)
    at45 = rmap.cells(0.11, 45.0)
    assert at45.heading_deg.tolist() == [0.0] and at45.r_m.tolist() == [0.1]
    assert rmap.cells(0.1, 50.0).r_m.tolist() == [0.1]  # nearest pitch bin within half the spacing
    assert len(rmap.cells(0.1, None).r_m) == 2
    assert len(rmap.cells(0.5, None).r_m) == 0  # above the grid: nothing reachable
    assert len(rmap.cells(0.0, None).r_m) == 0
    with pytest.raises(ValueError):
        rmap.cells(0.1, -60.0)


def test_describe_reports_key_grid_and_counts(coarse: ReachMap) -> None:
    info = coarse.describe()
    assert info["key"] == coarse.key
    assert info["version"] == REACH_MAP_VERSION
    assert info["grid"]["heading_step_deg"] == COARSE.heading_step_deg
    assert info["counts"] == coarse.meta["counts"]
    assert info["created_at"] == coarse.meta["created_at"]
    json.dumps(info)


# --- CLI ------------------------------------------------------------------------------------------------------------


def write_client_yml(path: Path, grid: dict[str, object]) -> Path:
    node_config = yaml.safe_dump({"reach_map": grid})
    path.write_text(
        yaml.safe_dump(
            {"ros2_nodes": [{"name": "other", "config": "{}"}, {"name": "mcp_server", "config": node_config}]}
        )
    )
    return path


def test_cli_build_then_check(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    client = write_client_yml(tmp_path / "client.yml", TINY)
    out = tmp_path / "map.npz"
    assert reach_map.main(["build", "--client-yml", str(client), "--out", str(out)]) == 0
    assert out.exists()
    assert reach_map.main(["check", "--client-yml", str(client), "--file", str(out)]) == 0
    stale = write_client_yml(tmp_path / "stale.yml", TINY | {"z_max_m": 0.3})
    assert reach_map.main(["check", "--client-yml", str(stale), "--file", str(out)]) == 1
    assert "stale" in capsys.readouterr().out
    assert reach_map.main(["check", "--client-yml", str(client), "--file", str(tmp_path / "none.npz")]) == 1


def test_config_from_client_yml_reads_the_mcp_server_node(tmp_path: Path) -> None:
    client = write_client_yml(tmp_path / "client.yml", TINY)
    config = reach_map.config_from_client_yml(client)
    assert config.reach_map.heading_step_deg == 90.0
    assert tuple(config.reach_map.planar_samples) == (8, 8, 5)
