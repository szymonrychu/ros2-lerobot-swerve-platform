"""run_node republishes /poi/list when a retention prune removed photos."""

import json
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest

from . import test_node_photos
from .test_node_photos import DAY_S, FakeString, make_config, photo_command, run

node_module = test_node_photos.node_module


def test_prune_that_removed_photos_republishes_list_with_higher_revision(
    node_module: ModuleType, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    photo_dir = tmp_path / "photos"
    photo_dir.mkdir()
    now = [1_760_000_000.0]
    monkeypatch.setattr(node_module, "time", SimpleNamespace(time=lambda: now[0]))
    node = run(node_module, make_config(tmp_path, max_age_days=1.0))
    node.subscriptions["/poi/command"](FakeString(photo_command(photo_dir, 1, ts=now[0])))
    node.subscriptions["/poi/command"](FakeString(photo_command(photo_dir, 2, ts=now[0] + DAY_S)))
    sent = node.publishers["/poi/list"].sent
    count_before = len(sent)
    revision_before = json.loads(sent[-1])["revision"]
    assert len(json.loads(sent[-1])["pois"]) == 2

    now[0] += 1.5 * DAY_S
    node.timers[0][1]()

    assert len(sent) == count_before + 1
    data = json.loads(sent[-1])
    assert data["revision"] > revision_before
    assert [p["id"] for p in data["pois"]] == [f"{2:032x}"]


def test_prune_that_removed_nothing_does_not_republish_list(node_module: ModuleType, tmp_path: Path) -> None:
    (tmp_path / "photos").mkdir()
    node = run(node_module, make_config(tmp_path))
    count = len(node.publishers["/poi/list"].sent)
    node.timers[0][1]()
    assert len(node.publishers["/poi/list"].sent) == count


def test_startup_prune_that_removed_photos_publishes_pruned_list(
    node_module: ModuleType, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    photo_dir = tmp_path / "photos"
    photo_dir.mkdir()
    now = [1_760_000_000.0]
    monkeypatch.setattr(node_module, "time", SimpleNamespace(time=lambda: now[0]))
    config = make_config(tmp_path, max_age_days=1.0)
    first = run(node_module, config)
    first.subscriptions["/poi/command"](FakeString(photo_command(photo_dir, 1, ts=now[0])))
    revision_before = json.loads(first.publishers["/poi/list"].sent[-1])["revision"]

    now[0] += 2 * DAY_S
    second = run(node_module, config)

    data = json.loads(second.publishers["/poi/list"].sent[-1])
    assert data["pois"] == []
    assert data["revision"] > revision_before
