"""Persisted session log: append, load on start, paging, reset and rotation."""

import json
from pathlib import Path

from claude_agent.events import EventLog


def make_log(tmp_path: Path, size: int = 5, max_bytes: int = 10_000_000) -> EventLog:
    return EventLog(size, path=tmp_path / "session" / "events.jsonl", max_bytes=max_bytes)


def fill(log: EventLog, count: int) -> None:
    for i in range(count):
        log.append("assistant_text", text=f"m{i + 1}")


def test_append_writes_one_json_line_per_event(tmp_path: Path) -> None:
    log = make_log(tmp_path)
    fill(log, 3)
    lines = (tmp_path / "session" / "events.jsonl").read_text().splitlines()
    assert [json.loads(line)["seq"] for line in lines] == [1, 2, 3]
    assert json.loads(lines[2])["text"] == "m3"


def test_load_continues_seq_and_keeps_last_events_in_ram(tmp_path: Path) -> None:
    fill(make_log(tmp_path, size=3), 10)
    reloaded = make_log(tmp_path, size=3)
    assert reloaded.seq == 10
    assert [e["seq"] for e in reloaded.history()] == [8, 9, 10]
    assert reloaded.append("state")["seq"] == 11


def test_load_skips_corrupt_lines(tmp_path: Path) -> None:
    fill(make_log(tmp_path), 2)
    path = tmp_path / "session" / "events.jsonl"
    path.write_text(path.read_text() + "{broken\n\n")
    reloaded = make_log(tmp_path)
    assert reloaded.seq == 2 and len(reloaded.history()) == 2


def test_page_newest_and_before_seq_with_has_more(tmp_path: Path) -> None:
    log = make_log(tmp_path, size=3)
    fill(log, 10)
    events, more = log.page(None, 4)
    assert [e["seq"] for e in events] == [7, 8, 9, 10] and more is True
    events, more = log.page(7, 4)
    assert [e["seq"] for e in events] == [3, 4, 5, 6] and more is True
    events, more = log.page(3, 4)
    assert [e["seq"] for e in events] == [1, 2] and more is False
    events, more = log.page(None, 10)
    assert [e["seq"] for e in events] == list(range(1, 11)) and more is False


def test_page_older_than_ram_reads_disk(tmp_path: Path) -> None:
    log = make_log(tmp_path, size=2)
    fill(log, 6)
    assert [e["seq"] for e in log.history()] == [5, 6]
    events, more = log.page(5, 2)
    assert [e["seq"] for e in events] == [3, 4] and more is True


def test_page_without_file_serves_ram_only() -> None:
    log = EventLog(3)
    fill(log, 5)
    events, more = log.page(None, 100)
    assert [e["seq"] for e in events] == [3, 4, 5] and more is False
    assert log.page(None, 2)[0][-1]["seq"] == 5


def test_empty_log_page(tmp_path: Path) -> None:
    assert make_log(tmp_path).page(None, 100) == ([], False)


def test_reset_deletes_file_buffer_and_seq(tmp_path: Path) -> None:
    log = make_log(tmp_path)
    fill(log, 4)
    log.reset()
    assert not (tmp_path / "session" / "events.jsonl").exists()
    assert log.history() == [] and log.seq == 0 and log.page(None, 10) == ([], False)
    assert log.append("state")["seq"] == 1
    assert make_log(tmp_path).seq == 1


def test_rotation_drops_oldest_half(tmp_path: Path) -> None:
    log = make_log(tmp_path, size=100, max_bytes=2000)
    fill(log, 100)
    path = tmp_path / "session" / "events.jsonl"
    assert path.stat().st_size <= 2000
    seqs = [json.loads(line)["seq"] for line in path.read_text().splitlines()]
    assert seqs[-1] == 100 and seqs[0] > 1 and seqs == list(range(seqs[0], 101))
    events, more = log.page(None, 500)
    assert events[-1]["seq"] == 100 and more is False
    assert log.append("state")["seq"] == 101
    assert make_log(tmp_path, size=100, max_bytes=2000).seq == 101


def test_unwritable_path_does_not_break_append(tmp_path: Path) -> None:
    blocker = tmp_path / "file"
    blocker.write_text("x")
    log = EventLog(5, path=blocker / "sub" / "events.jsonl")
    assert log.append("state")["seq"] == 1
