"""Tests for the map_manager operations (save, load, delete) behind fake ports, and its config."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import pytest
from pydantic import ValidationError
from ros2_common.maps import COMMAND_TOPIC, LOCALISED_TOPIC, RESULT_TOPIC, STATUS_TOPIC, MapCommand, MapStatus, Pose2D

from map_manager.bundle import META_FILE, OCCUPANCY_FILE, POSEGRAPH_DATA_FILE, POSEGRAPH_FILE, BundleStore
from map_manager.config import (
    DEFAULT_CONFIG_PATH,
    ENV_CONFIG_PATH_KEY,
    MapManagerConfig,
    load_config,
    load_config_from_env,
)
from map_manager.manager import START_AT_GIVEN_POSE, UNCONFIRMED_REASON, MapManager
from map_manager.relocalise import SearchSettings
from tests.fakes import LASER_FRAME, CallLog, FakeClock, FakePoi, FakeSensors, FakeSlam, room_grid, wall_points_base

NARROW_SEARCH = SearchSettings(radius_m=0.2, step_m=0.1, yaw_range_deg=10.0, yaw_step_deg=5.0)
SYMMETRIC_SEARCH = SearchSettings(radius_m=0.1, step_m=0.1, yaw_range_deg=180.0, yaw_step_deg=90.0)
TRUE_POSE = Pose2D(x=1.0, y=0.8, yaw=0.0)
QUIET_CALLS = {
    "sensors.robot_pose",
    "sensors.latest_grid",
    "sensors.laser_frame",
    "poi.latest_list",
    "slam.pose_covariance",
}
BUNDLE_POIS = [{"id": "saved-1", "name": "kitchen"}, {"id": "saved-2", "name": "door"}]


@dataclass
class Rig:
    """A manager wired to fakes sharing one call log."""

    log: CallLog
    slam: FakeSlam
    poi: FakePoi
    sensors: FakeSensors
    config: MapManagerConfig
    store: BundleStore
    manager: MapManager

    def key_calls(self) -> list[str]:
        """Call names without the plain reads."""
        return [name for name in self.log.names() if name not in QUIET_CALLS]

    def save_map(self, name: str, pose: Pose2D = TRUE_POSE, pois: list[dict] | None = None) -> None:
        """Save a bundle of the room with the robot at pose and then reset the log and the live POIs."""
        live = self.poi.pois
        self.poi.pois = [dict(p) for p in (BUNDLE_POIS if pois is None else pois)]
        self.sensors.pose = pose
        result = self.manager.handle(MapCommand(op="save", request_id="setup", name=name))
        assert result.ok, result.message
        self.poi.pois = live
        self.log.calls.clear()


def make_rig(tmp_path: Path, **config: object) -> Rig:
    """Build a manager with a room as the live map, the robot at TRUE_POSE and its scan."""
    log = CallLog()
    grid = room_grid(4.0, 3.0)
    slam = FakeSlam(log=log, confirmed_pose=TRUE_POSE)
    poi = FakePoi(log=log)
    sensors = FakeSensors(log=log, pose=TRUE_POSE, grid=grid, points=wall_points_base(grid, TRUE_POSE))
    settings: dict[str, object] = {"maps_root": str(tmp_path / "named"), "search": NARROW_SEARCH}
    settings.update(config)
    cfg = MapManagerConfig(**settings)
    store = BundleStore(Path(cfg.maps_root))
    holder: list[MapManager] = []

    def on_change() -> None:
        status = holder[0].status()
        log.add(f"status:{status.localisation.state}", holder[0].localised())

    manager = MapManager(cfg, store, slam, poi, sensors, clock=FakeClock(), on_change=on_change)
    holder.append(manager)
    return Rig(log=log, slam=slam, poi=poi, sensors=sensors, config=cfg, store=store, manager=manager)


@pytest.fixture
def rig(tmp_path: Path) -> Rig:
    return make_rig(tmp_path)


def load(rig: Rig, name: str = "home", pose: Pose2D | None = None):
    return rig.manager.handle(MapCommand(op="load", request_id="r1", name=name, pose=pose))


# ---------------------------------------------------------------- config


def test_config_defaults() -> None:
    cfg = MapManagerConfig()
    assert cfg.maps_root == "/var/lib/ros2/maps/named"
    assert (cfg.command_topic, cfg.result_topic, cfg.status_topic, cfg.localised_topic) == (
        COMMAND_TOPIC,
        RESULT_TOPIC,
        STATUS_TOPIC,
        LOCALISED_TOPIC,
    )
    assert cfg.poi_list_topic == "/poi/list"
    assert cfg.poi_command_topic == "/poi/command"
    assert cfg.poi_result_topic == "/poi/result"
    assert cfg.map_topic == "/map"
    assert cfg.scan_topic == "/scan_filtered"
    assert cfg.pose_topic == "/pose"
    assert cfg.map_frame == "map"
    assert cfg.base_frame == "base_link"
    assert cfg.serialize_service == "/slam_toolbox/serialize_map"
    assert cfg.deserialize_service == "/slam_toolbox/deserialize_map"
    assert cfg.pause_service == "/slam_toolbox/pause_new_measurements"
    assert cfg.parameters_service == "/slam_toolbox/get_parameters"
    assert cfg.service_timeout_s == 15.0
    assert cfg.load_timeout_s == 60.0
    assert cfg.poi_timeout_s == 5.0
    assert cfg.max_pose_age_s == 1.0
    assert cfg.autosave_before_load is True
    assert cfg.autosave_name == "autosave"
    assert cfg.verify_window_s == 5.0
    assert cfg.agreement_hz == 2.0
    assert cfg.localised_ratio == 0.6
    assert cfg.min_samples == 5
    assert cfg.search == SearchSettings()
    assert cfg.metrics_port == 19113
    assert DEFAULT_CONFIG_PATH == Path("/etc/ros2/map_manager/config.yaml")
    assert ENV_CONFIG_PATH_KEY == "MAP_MANAGER_CONFIG"


def test_config_rejects_unknown_key() -> None:
    with pytest.raises(ValidationError):
        MapManagerConfig(map_root="/tmp/typo")
    with pytest.raises(ValidationError):
        MapManagerConfig(search={"radius": 1.0})


def test_config_rejects_invalid_autosave_name_and_ratio() -> None:
    with pytest.raises(ValidationError):
        MapManagerConfig(autosave_name="../escape")
    with pytest.raises(ValidationError):
        MapManagerConfig(localised_ratio=1.5)


def test_config_loads_yaml_from_env(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    path = tmp_path / "config.yaml"
    path.write_text("maps_root: /data/maps\nverify_window_s: 3\nsearch:\n  radius_m: 0.5\n")
    monkeypatch.setenv(ENV_CONFIG_PATH_KEY, str(path))
    cfg = load_config_from_env()
    assert cfg.maps_root == "/data/maps"
    assert cfg.verify_window_s == 3.0
    assert cfg.search.radius_m == 0.5


def test_config_empty_file_gives_defaults_and_bad_files_fail(tmp_path: Path) -> None:
    empty = tmp_path / "empty.yaml"
    empty.write_text("")
    assert load_config(empty) == MapManagerConfig()
    listing = tmp_path / "list.yaml"
    listing.write_text("- a\n")
    with pytest.raises(ValueError, match="mapping"):
        load_config(listing)
    with pytest.raises(FileNotFoundError):
        load_config(tmp_path / "missing.yaml")
    unknown = tmp_path / "unknown.yaml"
    unknown.write_text("bogus: 1\n")
    with pytest.raises(ValidationError):
        load_config(unknown)


# ---------------------------------------------------------------- status


def test_status_before_any_load_is_unknown_and_lists_existing_maps(tmp_path: Path) -> None:
    first = make_rig(tmp_path)
    first.save_map("home")
    rig = make_rig(tmp_path)
    status = rig.manager.status()
    assert isinstance(status, MapStatus)
    assert [m.name for m in status.maps] == ["home"]
    assert status.active_map is None
    assert status.load_seq == 0
    assert status.localisation.state == "unknown"
    assert status.localisation.ratio is None
    assert status.localisation.pose is None
    assert rig.manager.localised() is None


# ---------------------------------------------------------------- save


def test_save_writes_bundle_with_pose_pois_and_laser_frame(rig: Rig) -> None:
    result = rig.manager.handle(MapCommand(op="save", request_id="s1", name="home", note="ground floor"))
    assert result.ok, result.message
    assert result.request_id == "s1"
    directory = rig.store.path_of("home")
    for filename in (POSEGRAPH_FILE, POSEGRAPH_DATA_FILE, OCCUPANCY_FILE, META_FILE):
        assert (directory / filename).is_file()
    meta = json.loads((directory / META_FILE).read_text())
    assert meta["last_pose"] == {"x": 1.0, "y": 0.8, "yaw": 0.0}
    assert meta["pois"] == [{"id": "live-1", "name": "sofa"}]
    assert meta["poi_revision"] == 3
    assert meta["laser_frame"] == LASER_FRAME
    assert meta["note"] == "ground floor"
    assert meta["map_info"]["width"] == rig.sensors.grid.cells.shape[1]
    assert meta["map_info"]["height"] == rig.sensors.grid.cells.shape[0]
    assert result.data is not None and result.data["name"] == "home" and result.data["poi_count"] == 1
    assert rig.key_calls() == ["sensors.start_operation", "slam.serialize", "status:unknown", "sensors.end_operation"]
    assert ".tmp-home-" in rig.log.args_of("slam.serialize")[0][0]
    assert [m.name for m in rig.manager.status().maps] == ["home"]
    assert not rig.sensors.active


@pytest.mark.parametrize(
    ("attribute", "words"),
    [("pose", "robot pose"), ("grid", "map grid"), ("pois", "POI list"), ("frame", "laser frame")],
)
def test_save_refuses_when_input_missing(rig: Rig, attribute: str, words: str) -> None:
    target = rig.poi if attribute == "pois" else rig.sensors
    setattr(target, attribute, None)
    result = rig.manager.handle(MapCommand(op="save", name="home"))
    assert not result.ok
    assert words in result.message
    assert "slam.serialize" not in rig.log.names()
    assert not Path(rig.config.maps_root).exists() or list(Path(rig.config.maps_root).iterdir()) == []
    assert rig.manager.status().maps == []
    assert not rig.sensors.active


def test_save_names_every_missing_input(rig: Rig) -> None:
    rig.sensors.pose = None
    rig.sensors.frame = None
    result = rig.manager.handle(MapCommand(op="save", name="home"))
    assert not result.ok
    assert "robot pose" in result.message and "laser frame" in result.message


def test_save_serialize_failure_removes_temp_dir(rig: Rig) -> None:
    rig.slam.serialize_ok = False
    result = rig.manager.handle(MapCommand(op="save", name="home"))
    assert not result.ok
    assert "serialize" in result.message
    assert list(Path(rig.config.maps_root).iterdir()) == []
    assert rig.manager.status().maps == []


def test_save_existing_name_needs_overwrite(rig: Rig) -> None:
    rig.save_map("home")
    refused = rig.manager.handle(MapCommand(op="save", name="home"))
    assert not refused.ok
    assert "already exists" in refused.message
    assert "slam.serialize" not in rig.log.names()
    rig.poi.pois = [{"id": "new", "name": "new"}]
    replaced = rig.manager.handle(MapCommand(op="save", name="home", overwrite=True))
    assert replaced.ok, replaced.message
    meta = json.loads((rig.store.path_of("home") / META_FILE).read_text())
    assert meta["pois"] == [{"id": "new", "name": "new"}]


# ---------------------------------------------------------------- load


def test_load_happy_path_call_order(rig: Rig) -> None:
    rig.save_map("home", pois=BUNDLE_POIS)
    result = load(rig)
    assert result.ok, result.message
    assert rig.key_calls() == [
        "sensors.start_operation",
        "slam.serialize",
        "status:unknown",
        "status:loading",
        "sensors.scan_points_base",
        "slam.query_paused",
        "slam.toggle_pause",
        "slam.query_paused",
        "slam.deserialize",
        "slam.wait_pose_after",
        "poi.replace",
        "sensors.agreement_samples",
        "slam.query_paused",
        "slam.toggle_pause",
        "slam.query_paused",
        "status:localised",
        "sensors.end_operation",
    ]
    assert ".tmp-autosave-" in rig.log.args_of("slam.serialize")[0][0]
    assert rig.log.args_of("status:loading") == [(False,)]
    assert rig.log.args_of("status:localised") == [(True,)]
    base, pose = rig.log.args_of("slam.deserialize")[0]
    assert base == str(rig.store.path_of("home") / "map")
    assert pose == TRUE_POSE
    assert rig.log.args_of("poi.replace") == [(BUNDLE_POIS,)]
    assert rig.log.args_of("sensors.agreement_samples") == [(5.0,)]
    assert START_AT_GIVEN_POSE == 2


def test_load_happy_unpauses_and_reports_localised(rig: Rig) -> None:
    rig.save_map("home")
    result = load(rig)
    assert result.ok
    assert rig.slam.paused is False
    assert rig.poi.pois == BUNDLE_POIS
    assert [m.name for m in rig.manager.status().maps] == ["autosave", "home"]
    status = rig.manager.status()
    assert status.active_map == "home"
    assert status.load_seq == 1
    assert status.localisation.state == "localised"
    assert status.localisation.ratio == pytest.approx(0.9)
    assert status.localisation.pose == TRUE_POSE
    assert rig.manager.localised() is True
    data = result.data
    assert data is not None
    assert data["state"] == "localised"
    assert data["ratio"] == pytest.approx(0.9)
    assert data["pose"] == TRUE_POSE.model_dump()
    assert data["hint"] == TRUE_POSE.model_dump()
    assert data["search_ratio"] == pytest.approx(1.0)
    assert data["ambiguous"] is False
    assert data["covariance"] == [0.01, 0.02, 0.003]
    assert data["pois_replaced"] is True
    assert not rig.sensors.active


def test_load_explicit_pose_replaces_saved_hint(rig: Rig) -> None:
    rig.save_map("home", pose=TRUE_POSE)
    moved = Pose2D(x=2.5, y=1.5, yaw=0.0)
    rig.sensors.pose = moved
    rig.sensors.points = wall_points_base(rig.sensors.grid, moved)
    rig.slam.confirmed_pose = moved
    hint = Pose2D(x=2.4, y=1.6, yaw=math.radians(5.0))
    result = load(rig, pose=hint)
    assert result.ok, result.message
    _, pose = rig.log.args_of("slam.deserialize")[0]
    assert pose.x == pytest.approx(2.5) and pose.y == pytest.approx(1.5) and pose.yaw == pytest.approx(0.0)
    assert result.data["hint"] == hint.model_dump()


def test_load_search_not_ok_deserializes_at_the_hint(rig: Rig) -> None:
    rig.save_map("home")
    rig.sensors.points = None
    result = load(rig)
    _, pose = rig.log.args_of("slam.deserialize")[0]
    assert pose == TRUE_POSE
    assert result.data["search_ratio"] is None
    assert "no scan" in result.data["search_reason"]


def test_load_missing_files_makes_no_slam_call(rig: Rig) -> None:
    rig.save_map("home")
    (rig.store.path_of("home") / POSEGRAPH_DATA_FILE).unlink()
    result = load(rig)
    assert not result.ok
    assert POSEGRAPH_DATA_FILE in result.message
    assert not [name for name in rig.log.names() if name.startswith("slam.")]
    assert rig.manager.status().localisation.state == "unknown"
    assert rig.manager.localised() is None


def test_load_unknown_map_is_refused(rig: Rig) -> None:
    result = load(rig, name="nowhere")
    assert not result.ok
    assert "not found" in result.message
    assert not [name for name in rig.log.names() if name.startswith("slam.")]


def test_load_laser_frame_mismatch_is_refused(rig: Rig) -> None:
    rig.save_map("home")
    rig.sensors.frame = "other_laser"
    result = load(rig)
    assert not result.ok
    assert "other_laser" in result.message and LASER_FRAME in result.message
    assert not [name for name in rig.log.names() if name.startswith("slam.")]
    rig.sensors.frame = None
    unknown = load(rig)
    assert not unknown.ok
    assert "laser frame" in unknown.message
    assert not rig.sensors.active


def test_load_autosave_failure_aborts(rig: Rig) -> None:
    rig.save_map("home")
    rig.sensors.grid = None
    result = load(rig)
    assert not result.ok
    assert result.message.startswith("autosave failed")
    assert "map grid" in result.message
    assert "slam.deserialize" not in rig.log.names()
    assert "slam.toggle_pause" not in rig.log.names()
    assert rig.manager.status().localisation.state == "unknown"
    assert rig.manager.status().load_seq == 0
    assert rig.poi.pois == [{"id": "live-1", "name": "sofa"}]


def test_load_autosave_disabled_skips_autosave(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, autosave_before_load=False)
    rig.save_map("home")
    result = load(rig)
    assert result.ok
    assert "slam.serialize" not in rig.log.names()
    assert result.data["autosaved"] is False


def test_load_of_the_autosave_map_does_not_overwrite_it(rig: Rig) -> None:
    rig.save_map("autosave")
    result = load(rig, name="autosave")
    assert result.ok, result.message
    assert "slam.serialize" not in rig.log.names()
    assert rig.log.args_of("slam.deserialize")[0][0] == str(rig.store.path_of("autosave") / "map")


def test_load_unconfirmed_stays_paused(rig: Rig) -> None:
    rig.save_map("home")
    rig.slam.confirmed_pose = None
    result = load(rig)
    assert not result.ok
    assert UNCONFIRMED_REASON == "load not confirmed by slam_toolbox"
    assert UNCONFIRMED_REASON in result.message
    assert rig.slam.paused is True
    assert "sensors.agreement_samples" not in rig.log.names()
    assert rig.log.names().count("slam.toggle_pause") == 1
    status = rig.manager.status()
    assert status.localisation.state == "not_localised"
    assert status.localisation.message == UNCONFIRMED_REASON
    assert status.localisation.ratio is None
    assert status.localisation.pose is None
    assert status.active_map == "home"
    assert status.load_seq == 1
    assert rig.manager.localised() is False
    assert result.data["state"] == "not_localised"
    assert result.data["hint"] == TRUE_POSE.model_dump()


def test_load_deserialize_refused_stays_paused(rig: Rig) -> None:
    rig.save_map("home")
    rig.slam.deserialize_ok = False
    result = load(rig)
    assert not result.ok
    assert "deserialize" in result.message
    assert rig.slam.paused is True
    assert "slam.wait_pose_after" not in rig.log.names()
    assert rig.manager.status().localisation.state == "not_localised"
    assert rig.manager.status().load_seq == 1
    assert rig.manager.localised() is False


def test_load_low_agreement_stays_paused(rig: Rig) -> None:
    rig.save_map("home")
    rig.sensors.samples = [0.3] * 6
    result = load(rig)
    assert not result.ok
    assert rig.slam.paused is True
    assert rig.log.names().count("slam.toggle_pause") == 1
    status = rig.manager.status()
    assert status.localisation.state == "not_localised"
    assert "below" in status.localisation.message
    assert status.localisation.ratio == pytest.approx(0.3)
    assert status.active_map == "home"
    assert status.load_seq == 1
    assert rig.manager.localised() is False
    assert result.data["state"] == "not_localised"


def test_load_too_few_samples_stays_paused(rig: Rig) -> None:
    rig.save_map("home")
    rig.sensors.samples = [0.9, 0.9]
    result = load(rig)
    assert not result.ok
    assert rig.slam.paused is True
    assert rig.manager.status().localisation.state == "not_localised"
    assert "too few" in rig.manager.status().localisation.message


def test_load_ambiguous_stays_paused(tmp_path: Path) -> None:
    rig = make_rig(tmp_path, search=SYMMETRIC_SEARCH)
    square = room_grid(3.0, 3.0)
    centre = Pose2D(x=1.5, y=1.5, yaw=0.0)
    rig.sensors.grid = square
    rig.sensors.points = wall_points_base(square, centre)
    rig.slam.confirmed_pose = centre
    rig.save_map("square", pose=centre)
    result = load(rig, name="square")
    assert not result.ok
    assert rig.slam.paused is True
    status = rig.manager.status()
    assert status.localisation.state == "ambiguous"
    assert "another pose" in status.localisation.message
    assert rig.manager.localised() is False
    assert result.data["ambiguous"] is True
    assert result.data["state"] == "ambiguous"
    assert status.active_map == "square"
    assert status.load_seq == 1


def test_load_already_paused_is_not_toggled(rig: Rig) -> None:
    rig.save_map("home")
    rig.slam.paused = True
    result = load(rig)
    assert result.ok
    names = rig.log.names()
    before_deserialize = names[: names.index("slam.deserialize")]
    assert "slam.toggle_pause" not in before_deserialize
    assert names.count("slam.toggle_pause") == 1
    assert rig.slam.paused is False


def test_load_unreadable_pause_state_falls_back_to_tracked_state(rig: Rig) -> None:
    rig.save_map("home")
    rig.slam.confirmed_pose = None
    load(rig)
    assert rig.slam.paused is True
    rig.slam.pause_readable = False
    rig.slam.confirmed_pose = TRUE_POSE
    rig.log.calls.clear()
    result = load(rig)
    assert result.ok
    names = rig.log.names()
    assert "slam.toggle_pause" not in names[: names.index("slam.deserialize")]
    assert names.count("slam.toggle_pause") == 1
    assert rig.slam.paused is False
    assert rig.manager.status().load_seq == 2


def test_load_pause_failure_aborts_before_deserialize(rig: Rig) -> None:
    rig.save_map("home")
    rig.slam.toggle_ok = False
    result = load(rig)
    assert not result.ok
    assert "pause" in result.message
    assert "slam.deserialize" not in rig.log.names()
    status = rig.manager.status()
    assert status.localisation.state == "unknown"
    assert status.load_seq == 0
    assert status.active_map is None
    assert rig.manager.localised() is None


def test_load_poi_replace_failure_is_reported(rig: Rig) -> None:
    rig.save_map("home")
    rig.poi.replace_ok = False
    result = load(rig)
    assert not result.ok
    assert "replace rejected" in result.message
    assert result.data["pois_replaced"] is False
    assert result.data["state"] == "localised"
    assert rig.manager.status().localisation.state == "localised"
    assert rig.poi.pois == [{"id": "live-1", "name": "sofa"}]


def test_busy_rejects_second_command(rig: Rig) -> None:
    rig.save_map("home")
    rig.save_map("other")
    answers = []
    rig.slam.on_deserialize = lambda: answers.append(rig.manager.handle(MapCommand(op="delete", name="other")))
    result = load(rig)
    assert result.ok
    assert len(answers) == 1
    assert not answers[0].ok
    assert answers[0].message == "busy: load home"
    assert rig.store.path_of("other").is_dir()
    rig.slam.on_deserialize = None
    after = rig.manager.handle(MapCommand(op="delete", name="other"))
    assert after.ok


def test_relocalise_op_is_not_handled_here(rig: Rig) -> None:
    result = rig.manager.handle(MapCommand(op="relocalise", request_id="m1"))
    assert not result.ok
    assert "relocalise" in result.message
    assert rig.log.names() == []


def test_unexpected_error_in_load_leaves_a_truthful_state(rig: Rig) -> None:
    rig.save_map("home")

    def explode() -> None:
        raise RuntimeError("service client gone")

    rig.slam.on_deserialize = explode
    result = load(rig)
    assert not result.ok
    assert "service client gone" in result.message
    assert rig.manager.status().localisation.state == "not_localised"
    assert rig.manager.localised() is False
    assert not rig.sensors.active
    rig.slam.on_deserialize = None
    assert load(rig).ok


# ---------------------------------------------------------------- delete


def test_delete_removes_bundle_and_its_pois_but_not_live_pois(rig: Rig) -> None:
    rig.save_map("home")
    rig.save_map("other", pois=[{"id": "o1", "name": "garage"}])
    assert load(rig).ok
    rig.log.calls.clear()
    result = rig.manager.handle(MapCommand(op="delete", request_id="d1", name="other"))
    assert result.ok, result.message
    assert not rig.store.path_of("other").exists()
    assert "poi.replace" not in rig.log.names()
    assert rig.poi.pois == BUNDLE_POIS
    status = rig.manager.status()
    assert [m.name for m in status.maps] == ["autosave", "home"]
    assert status.active_map == "home"
    assert "slam.serialize" not in rig.log.names() and "sensors.start_operation" not in rig.log.names()


def test_delete_active_map_empties_live_pois(rig: Rig) -> None:
    rig.save_map("home")
    assert load(rig).ok
    rig.log.calls.clear()
    result = rig.manager.handle(MapCommand(op="delete", name="home"))
    assert result.ok, result.message
    assert not rig.store.path_of("home").exists()
    assert rig.log.args_of("poi.replace") == [([],)]
    assert rig.poi.pois == []
    assert rig.manager.status().active_map is None
    assert result.data["pois_cleared"] is True


def test_delete_active_map_reports_replace_failure(rig: Rig) -> None:
    rig.save_map("home")
    assert load(rig).ok
    rig.poi.replace_ok = False
    result = rig.manager.handle(MapCommand(op="delete", name="home"))
    assert not result.ok
    assert "replace rejected" in result.message
    assert not rig.store.path_of("home").exists()
    assert rig.manager.status().active_map is None
    assert result.data["pois_cleared"] is False


def test_delete_unknown_name_changes_nothing(rig: Rig) -> None:
    rig.save_map("home")
    result = rig.manager.handle(MapCommand(op="delete", name="ghost"))
    assert not result.ok
    assert "not found" in result.message
    assert rig.store.path_of("home").is_dir()
    assert rig.log.names() == []
