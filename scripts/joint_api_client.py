#!/usr/bin/env python3
"""CLI client for the test joint API: GET current joint updates or POST joint updates (single or batch).

The API is served by the test_joint_api node on the client. POSTed updates go through the same
path as master2master: filter_node -> feetech. For physical tests, restrict to gripper joints only.

Examples:
  # GET current joint updates (default base URL http://localhost:8080)
  python scripts/joint_api_client.py get
  python scripts/joint_api_client.py get --base-url http://192.168.1.34:8080

  # POST single joint (gripper)
  python scripts/joint_api_client.py post --joint joint_6 0.15

  # POST multiple joints (gripper only for safety)
  python scripts/joint_api_client.py post --joint joint_5 -0.1 --joint joint_6 0.2

  # POST from JSON file
  python scripts/joint_api_client.py post --file updates.json
"""

import argparse
import json
import os
import sys
from typing import Any

try:
    import urllib.request
    import urllib.error
except ImportError:
    urllib = None  # type: ignore[assignment]


DEFAULT_BASE_URL = "http://localhost:18080"
ENV_BASE_URL = "JOINT_API_BASE_URL"


def _base_url() -> str:
    return os.environ.get(ENV_BASE_URL, "").strip() or DEFAULT_BASE_URL


def _request(method: str, url: str, data: dict[str, Any] | None = None) -> dict[str, Any]:
    """Perform HTTP request and return JSON body."""
    if urllib is None:
        raise RuntimeError("urllib not available")
    req_data = None
    if data is not None:
        req_data = json.dumps(data).encode("utf-8")
    req = urllib.request.Request(
        url,
        data=req_data,
        method=method,
        headers={"Content-Type": "application/json"} if req_data else {},
    )
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            body = resp.read().decode("utf-8")
            return json.loads(body) if body else {}
    except urllib.error.HTTPError as e:
        body = e.read().decode("utf-8") if e.fp else ""
        try:
            err = json.loads(body)
            print(json.dumps(err, indent=2), file=sys.stderr)
        except Exception:
            print(body or str(e), file=sys.stderr)
        sys.exit(1)
    except (OSError, ValueError) as e:
        print(f"Request failed: {e}", file=sys.stderr)
        sys.exit(1)


def cmd_get(base_url: str) -> None:
    """GET /joint-updates and print result."""
    url = f"{base_url.rstrip('/')}/joint-updates"
    out = _request("GET", url)
    print(json.dumps(out, indent=2))


def cmd_post(base_url: str, joints: list[tuple[str, float]], file_path: str | None) -> None:
    """POST joint updates (from --joint args or --file)."""
    url = f"{base_url.rstrip('/')}/joint-updates"
    if file_path:
        with open(file_path, encoding="utf-8") as f:
            payload = json.load(f)
        if not isinstance(payload, dict):
            print("JSON file must be an object (joint name -> radians)", file=sys.stderr)
            sys.exit(1)
    else:
        payload = dict(joints)
    if not payload:
        print("No joints to post; use --joint NAME VALUE or --file PATH", file=sys.stderr)
        sys.exit(1)
    out = _request("POST", url, payload)
    print(json.dumps(out, indent=2))


def main() -> int:
    parser = argparse.ArgumentParser(
        description="GET or POST joint updates to the test joint API (filter -> feetech path).",
        epilog="Restrict physical tests to gripper joints (e.g. joint_5, joint_6) only.",
    )
    parser.add_argument(
        "--base-url",
        default=_base_url(),
        help=f"API base URL (default: {DEFAULT_BASE_URL}, or set {ENV_BASE_URL})",
    )
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("get", help="GET current joint updates")
    post_parser = sub.add_parser("post", help="POST joint updates (single or batch)")
    post_parser.add_argument(
        "--joint",
        metavar=("NAME", "VALUE"),
        nargs=2,
        action="append",
        default=[],
        help="Joint name and value in radians (repeat for multiple)",
    )
    post_parser.add_argument(
        "--file",
        metavar="PATH",
        help="Path to JSON file with joint name -> radians map",
    )
    args = parser.parse_args()
    base = args.base_url
    if args.command == "get":
        cmd_get(base)
    else:
        file_path = getattr(args, "file", None)
        joints_list: list[tuple[str, float]] = []
        if not file_path and args.joint:
            for name, val_str in args.joint:
                try:
                    joints_list.append((name.strip(), float(val_str)))
                except ValueError:
                    print(f"Invalid value for {name}: {val_str!r}", file=sys.stderr)
                    sys.exit(1)
        if not file_path and not joints_list:
            print("Error: provide --joint NAME VALUE (repeat for batch) or --file PATH", file=sys.stderr)
            sys.exit(1)
        cmd_post(base, joints_list, file_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
