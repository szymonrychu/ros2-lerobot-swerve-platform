"""Compute the ground board fit of one ground session (runs as a niced child process of the ground ComputeJob).

Telemetry: the calib_event lines of flow `ground_compute` (ground_results.py) go to stderr in CHILD_LOG_FORMAT;
jobs.py forwards them into the node's journal.
"""

import argparse
import logging
from collections.abc import Sequence
from datetime import datetime

from head_rgbd_calibration.config import load_config
from head_rgbd_calibration.ground_results import GroundStore, compute_ground
from head_rgbd_calibration.store import RESULT_FILE
from head_rgbd_calibration.telemetry import CHILD_LOG_FORMAT


def main(argv: Sequence[str] | None = None) -> int:
    """Run one ground board computation.

    Args:
        argv: --config <node config path> --session <sid> --result <rid>.

    Returns:
        int: 0 when result.json was written (also for a failed or implausible fit), 1 otherwise.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--session", required=True)
    parser.add_argument("--result", required=True)
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format=CHILD_LOG_FORMAT)
    cfg = load_config(args.config)
    store = GroundStore(cfg)
    compute_ground(cfg, store, args.session, args.result, datetime.now().astimezone())
    return 0 if (store.result_dir(args.session, args.result) / RESULT_FILE).is_file() else 1


if __name__ == "__main__":
    raise SystemExit(main())
