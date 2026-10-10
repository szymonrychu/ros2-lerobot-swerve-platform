"""Tests for map backup naming and pruning."""

from datetime import UTC, datetime, timedelta
from pathlib import Path

from map_guard.backup import backup_name, prune_backups

PREFIX = "map_guard_backup"


def test_backup_name_is_a_sortable_utc_timestamp() -> None:
    when = datetime(2026, 10, 10, 7, 5, 3, tzinfo=UTC)
    assert backup_name(PREFIX, when) == "map_guard_backup_20261010T070503Z"


def test_backup_name_converts_to_utc() -> None:
    from datetime import timezone

    when = datetime(2026, 10, 10, 9, 0, 0, tzinfo=timezone(timedelta(hours=2)))
    assert backup_name(PREFIX, when) == "map_guard_backup_20261010T070000Z"


def make_backups(directory: Path, count: int) -> list[str]:
    """Create count serialized maps (.posegraph + .data), oldest first; returns their stems."""
    stems = []
    base = datetime(2026, 1, 1, tzinfo=UTC)
    for i in range(count):
        stem = backup_name(PREFIX, base + timedelta(minutes=i))
        (directory / f"{stem}.posegraph").write_text("g")
        (directory / f"{stem}.data").write_text("d")
        stems.append(stem)
    return stems


def test_prune_keeps_the_newest(tmp_path: Path) -> None:
    stems = make_backups(tmp_path, 8)
    (tmp_path / "slam_map.posegraph").write_text("keep")  # the main map is never touched
    (tmp_path / "slam_map.data").write_text("keep")
    deleted = prune_backups(tmp_path, PREFIX, 5)
    assert sorted(p.name for p in deleted) == sorted(f"{s}{ext}" for s in stems[:3] for ext in (".posegraph", ".data"))
    remaining = sorted(p.name for p in tmp_path.iterdir())
    assert remaining == sorted(
        [f"{s}{ext}" for s in stems[3:] for ext in (".posegraph", ".data")] + ["slam_map.data", "slam_map.posegraph"]
    )


def test_prune_with_fewer_backups_than_keep(tmp_path: Path) -> None:
    make_backups(tmp_path, 2)
    assert prune_backups(tmp_path, PREFIX, 5) == []
    assert len(list(tmp_path.iterdir())) == 4


def test_prune_missing_directory(tmp_path: Path) -> None:
    assert prune_backups(tmp_path / "nope", PREFIX, 5) == []


def test_prune_ignores_other_prefixes(tmp_path: Path) -> None:
    make_backups(tmp_path, 6)
    (tmp_path / "map_guard_backup_old_manual.txt").write_text("x")  # not a timestamped backup
    deleted = prune_backups(tmp_path, PREFIX, 5)
    assert len(deleted) == 2
    assert (tmp_path / "map_guard_backup_old_manual.txt").exists()
