"""Names and pruning of the serialized map backups (<prefix>_<UTC timestamp>.posegraph / .data)."""

import re
from datetime import UTC, datetime
from pathlib import Path

TIMESTAMP_FORMAT = "%Y%m%dT%H%M%SZ"
TIMESTAMP_PATTERN = r"\d{8}T\d{6}Z"


def backup_name(prefix: str, when: datetime) -> str:
    """File stem of a backup taken at when (slam_toolbox appends .posegraph and .data).

    Args:
        prefix (str): Stem prefix, e.g. "map_guard_backup".
        when (datetime): Timezone-aware backup time.

    Returns:
        str: "<prefix>_<YYYYMMDDTHHMMSSZ>" in UTC, so names sort by time.
    """
    return f"{prefix}_{when.astimezone(UTC).strftime(TIMESTAMP_FORMAT)}"


def prune_backups(directory: Path, prefix: str, keep: int) -> list[Path]:
    """Delete all but the newest keep backups (every file of an older stem).

    Args:
        directory (Path): Backup directory.
        prefix (str): Stem prefix of the backups.
        keep (int): Newest backups to keep.

    Returns:
        list[Path]: Deleted files.
    """
    if not directory.is_dir():
        return []
    pattern = re.compile(rf"^{re.escape(prefix)}_{TIMESTAMP_PATTERN}$")
    by_stem: dict[str, list[Path]] = {}
    for path in directory.iterdir():
        stem = path.name.split(".", 1)[0]
        if path.is_file() and pattern.match(stem):
            by_stem.setdefault(stem, []).append(path)
    deleted: list[Path] = []
    for stem in sorted(by_stem)[:-keep] if len(by_stem) > keep else []:
        for path in by_stem[stem]:
            path.unlink(missing_ok=True)
            deleted.append(path)
    return deleted
