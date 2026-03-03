#!/usr/bin/env python3
"""Poll topic_scraper_api instances and emit merged NDJSON stream.

The output is newline-delimited JSON (one record per source/topic sample).
Each record includes timing metadata (`received_at_ns`, `header_stamp_ns`,
`sample_seq`) and the selected `value`.
"""

import argparse
import json
import subprocess
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Any

DEFAULT_INTERVAL_S = 0.1
HELP_EPILOG = """Examples
========
Basic
-----
1) List one value from client leader joint states:
   python scripts/topic_scraper_collect.py \\
     --source client=http://192.168.1.34:18100 \\
     --select /leader/joint_states:.position[5] \\
     --interval 0.2

2) Poll once (single snapshot) and exit:
   python scripts/topic_scraper_collect.py \\
     --source client=http://192.168.1.34:18100 \\
     --select /follower/joint_states:.position[5] \\
     --once

Advanced
--------
3) Merge client + server streams for the same topic:
   python scripts/topic_scraper_collect.py \\
     --source client=http://192.168.1.34:18100 \\
     --source server=http://192.168.1.33:18100 \\
     --select /leader/joint_states:.position[5] \\
     --interval 0.1

4) Track multiple topics in one run (position + effort):
   python scripts/topic_scraper_collect.py \\
     --source client=http://192.168.1.34:18100 \\
     --select /filter/input_joint_updates:.position[5] \\
     --select /follower/joint_states:.position[5] \\
     --select /follower/joint_states:.effort[5] \\
     --interval 0.1

5) Build compact analysis records (jq object output):
   python scripts/topic_scraper_collect.py \\
     --source client=http://192.168.1.34:18100 \\
     --select '/follower/joint_states:{joint5_pos: .position[5], joint5_effort: .effort[5]}' \\
     --interval 0.1

Expert
------
6) Leader-follower skew telemetry across hosts:
   python scripts/topic_scraper_collect.py \\
     --source client=http://192.168.1.34:18100 \\
     --source server=http://192.168.1.33:18100 \\
     --select /filter/input_joint_updates:.position[5] \\
     --select /follower/joint_states:.position[5] \\
     --interval 0.05

7) Pipe NDJSON to jq for live field filtering:
   python scripts/topic_scraper_collect.py \\
     --source client=http://192.168.1.34:18100 \\
     --source server=http://192.168.1.33:18100 \\
     --select /filter/input_joint_updates:.position[5] \\
     --select /follower/joint_states:.position[5] \\
     --interval 0.05 | jq -c '{t: .timestamp_ns, src: .source, topic: .topic, v: .value, seq: .sample_seq}'

8) Save a capture for offline comparison:
   python scripts/topic_scraper_collect.py \\
     --source client=http://192.168.1.34:18100 \\
     --source server=http://192.168.1.33:18100 \\
     --select /filter/input_joint_updates:.position[5] \\
     --select /follower/joint_states:.position[5] \\
     --interval 0.05 > scrape_capture.ndjson

Notes
-----
- `--source` format: name=url
- `--select` format: /topic:jq-filter (applies to payload.message)
- `jq` CLI must be installed (the script runs jq as a subprocess)
"""


@dataclass(frozen=True)
class Source:
    """Source endpoint definition.

    Attributes:
        name: Source label used in output records.
        base_url: API base URL.
    """

    name: str
    base_url: str


@dataclass(frozen=True)
class Selector:
    """Topic selector with jq expression.

    Attributes:
        topic: ROS topic path.
        jq_filter: jq filter expression, evaluated against payload.message.
    """

    topic: str
    jq_filter: str


def parse_source(text: str) -> Source:
    """Parse source in `name=url` form."""

    if "=" not in text:
        raise ValueError("source must be in name=url format")
    name, base_url = text.split("=", 1)
    name = name.strip()
    base_url = base_url.strip().rstrip("/")
    if not name or not base_url:
        raise ValueError("source requires both non-empty name and url")
    return Source(name=name, base_url=base_url)


def parse_selector(text: str) -> Selector:
    """Parse selector in `/topic:jq-filter` form."""

    if ":" not in text:
        raise ValueError("selector must be in /topic:jq-filter format")
    topic, jq_filter = text.split(":", 1)
    topic = topic.strip()
    jq_filter = jq_filter.strip()
    if not topic.startswith("/"):
        raise ValueError("selector topic must start with /")
    if not jq_filter:
        raise ValueError("selector jq-filter must be non-empty")
    return Selector(topic=topic, jq_filter=jq_filter)


def topic_endpoint(base_url: str, topic: str) -> str:
    """Build endpoint URL from base URL and topic path."""

    return f"{base_url.rstrip('/')}/topics{topic}"


def http_get_json(url: str, timeout_s: float = 2.0) -> dict[str, Any] | None:
    """GET JSON object from URL.

    Returns None on non-200/network/decode errors.
    """

    req = urllib.request.Request(url, method="GET")
    try:
        with urllib.request.urlopen(req, timeout=timeout_s) as resp:
            if resp.status != 200:
                return None
            body = resp.read().decode("utf-8")
            if not body:
                return None
            data = json.loads(body)
            if not isinstance(data, dict):
                return None
            return data
    except (urllib.error.URLError, urllib.error.HTTPError, ValueError, OSError):
        return None


def apply_jq_filter(jq_filter: str, value: Any) -> Any:
    """Apply jq filter to value using jq CLI.

    Raises RuntimeError when jq command fails.
    """

    proc = subprocess.run(
        ["jq", "-c", jq_filter],
        input=json.dumps(value),
        text=True,
        capture_output=True,
        check=False,
    )
    if proc.returncode != 0:
        raise RuntimeError(proc.stderr.strip() or "jq command failed")
    output = proc.stdout.strip()
    if not output:
        return None
    try:
        return json.loads(output)
    except ValueError:
        return output


def build_record(source: str, topic: str, payload: dict[str, Any], selected_value: Any) -> dict[str, Any]:
    """Build output NDJSON record."""

    return {
        "timestamp_ns": time.time_ns(),
        "source": source,
        "topic": topic,
        "received_at_ns": payload.get("received_at_ns"),
        "header_stamp_ns": payload.get("header_stamp_ns"),
        "sample_seq": payload.get("sample_seq"),
        "value": selected_value,
    }


def run_collect(sources: list[Source], selectors: list[Selector], interval_s: float, once: bool) -> int:
    """Collect stream and print NDJSON records to stdout."""

    seen_seq: dict[tuple[str, str], Any] = {}
    while True:
        for source in sources:
            for selector in selectors:
                payload = http_get_json(topic_endpoint(source.base_url, selector.topic))
                if not payload:
                    continue
                seq = payload.get("sample_seq")
                key = (source.name, selector.topic)
                if seq is not None and seen_seq.get(key) == seq:
                    continue
                seen_seq[key] = seq
                message = payload.get("message")
                try:
                    selected_value = apply_jq_filter(selector.jq_filter, message)
                except RuntimeError as exc:
                    print(
                        json.dumps(
                            {
                                "timestamp_ns": time.time_ns(),
                                "source": source.name,
                                "topic": selector.topic,
                                "error": str(exc),
                            }
                        ),
                        flush=True,
                    )
                    continue
                record = build_record(source.name, selector.topic, payload, selected_value)
                print(json.dumps(record, separators=(",", ":")), flush=True)
        if once:
            break
        time.sleep(interval_s)
    return 0


def main() -> int:
    """CLI entrypoint."""

    parser = argparse.ArgumentParser(
        description="Poll topic_scraper_api and emit merged NDJSON records.",
        epilog=HELP_EPILOG,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--source",
        action="append",
        required=True,
        help="Source in name=url format (repeat for multiple hosts).",
    )
    parser.add_argument(
        "--select",
        action="append",
        required=True,
        help="Selector in /topic:jq-filter format (repeat for multiple topics).",
    )
    parser.add_argument("--interval", type=float, default=DEFAULT_INTERVAL_S, help="Polling interval in seconds.")
    parser.add_argument("--once", action="store_true", help="Poll once and exit.")
    args = parser.parse_args()

    try:
        sources = [parse_source(raw) for raw in args.source]
        selectors = [parse_selector(raw) for raw in args.select]
    except ValueError as exc:
        print(f"Invalid arguments: {exc}", file=sys.stderr)
        return 1
    if args.interval <= 0.0:
        print("Interval must be > 0", file=sys.stderr)
        return 1
    return run_collect(sources=sources, selectors=selectors, interval_s=args.interval, once=args.once)


if __name__ == "__main__":
    sys.exit(main())
