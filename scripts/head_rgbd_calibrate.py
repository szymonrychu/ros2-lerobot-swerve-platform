"""Head RGB-D calibration: capture, calibrate, URDF hand-off and validation, driven by scripts/head_rgbd_calibrate.yaml.

Run: uv run python -m scripts.head_rgbd_calibrate [--config FILE] {capture,capture-validation,calibrate,urdf,validate}
"""

import argparse
import subprocess
import sys
from datetime import datetime
from pathlib import Path

from pydantic import ValidationError
from ros2_common.head_rgbd_calibration.calibrate import failed_limits
from ros2_common.head_rgbd_calibration.extrinsics import load_extrinsics_yaml
from ros2_common.head_rgbd_calibration.urdf_handoff import check_drift, read_chain, rewrite_tof_joint, tof_joint_origin

from scripts.head_rgbd_calibration.config import DEFAULT_CONFIG, CalibrationConfig, load_calibration_config
from scripts.head_rgbd_calibration.frames import TopicNames, capture_bag, read_frames
from scripts.head_rgbd_calibration.pipeline import (
    bag_path,
    resolve_path,
    run_calibration,
    run_validation,
    session_name,
    write_calibration,
)

MODES = ("capture", "capture-validation", "calibrate", "urdf", "validate")
VALIDATION_SUFFIXES = ("0.5m", "1.0m", "2.0m", "3.0m", "sweep", "wall")
CAPTURE_NAME_FORMAT = "calib-%Y%m%d-%H%M%S"
RUNNER = subprocess.run
ERRORS = (ValueError, FileNotFoundError, RuntimeError, ValidationError)


def parse_args(argv: list[str] | None) -> argparse.Namespace:
    """Parse the command line: optional --config, the mode, the validation suffix or --check."""
    parser = argparse.ArgumentParser(description="Head RGB-D calibration tool.")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="calibration YAML")
    parser.add_argument("mode", choices=MODES)
    parser.add_argument("suffix", nargs="?", choices=VALIDATION_SUFFIXES, help="capture-validation: bag name suffix")
    parser.add_argument("--check", action="store_true", help="urdf: print the drift only")
    args = parser.parse_args(argv)
    if (args.mode == "capture-validation") != (args.suffix is not None):
        parser.error("a suffix is required for capture-validation and only valid there")
    if args.check and args.mode != "urdf":
        parser.error("--check is only valid for urdf")
    return args


def capture(cfg: CalibrationConfig, suffix: str | None) -> int:
    """Record a calibration bag, or a validation bag named <session>-<suffix>, and print its local path."""
    local = cfg.capture.model_copy(update={"local_dir": str(resolve_path(cfg.capture.local_dir))})
    if suffix is None:
        name = cfg.session or datetime.now().strftime(CAPTURE_NAME_FORMAT)
        path = capture_bag(local, name, run=RUNNER)
    else:
        path = capture_bag(local, f"{session_name(cfg)}-{suffix}", cfg.capture.validation_topics, run=RUNNER)
    print(path)
    return 0


def calibrate(cfg: CalibrationConfig) -> int:
    """Calibrate from the session bag and write the three files and the report; 1 when any limit fails."""
    session = session_name(cfg)
    run = run_calibration(read_frames(bag_path(cfg, session), TopicNames()), cfg)
    failures = failed_limits(run, cfg.limits)
    if failures:
        print("calibration rejected, nothing written:\n  " + "\n  ".join(failures), file=sys.stderr)
        return 1
    for path in write_calibration(run, cfg, session, datetime.now().astimezone()):
        print(f"wrote {path}")
    return 0


def urdf(cfg: CalibrationConfig, check_only: bool) -> int:
    """Print the URDF drift against the calibration and, unless check_only, rewrite tof_joint."""
    extrinsic = load_extrinsics_yaml(resolve_path(cfg.outputs.extrinsics))
    path = resolve_path(cfg.outputs.urdf)
    text = path.read_text()
    chain = read_chain(text)
    drift = check_drift(chain, extrinsic)
    print(f"drift: {drift.translation_mm:.3f} mm, {drift.rotation_deg:.3f} deg")
    if check_only:
        return 0
    note = f"{datetime.now().astimezone().date()}, stereo RMS {extrinsic.stereo_rms:.3f} px"
    path.write_text(rewrite_tof_joint(text, tof_joint_origin(chain, extrinsic), note))
    print(f"rewrote tof_joint in {path}")
    return 0


def validate(cfg: CalibrationConfig) -> int:
    """Print and write the validation target table; 1 when any target fails."""
    table, ok = run_validation(cfg, session_name(cfg))
    print(table)
    return 0 if ok else 1


def main(argv: list[str] | None = None) -> int:
    """Run one mode of the calibration tool.

    Args:
        argv: Command line without the program name.

    Returns:
        0 on success, 1 on an invalid config, a missing bag or file, a failed limit or a failed validation target.
    """
    args = parse_args(argv)
    try:
        cfg = load_calibration_config(args.config)
        if args.mode in ("capture", "capture-validation"):
            return capture(cfg, args.suffix)
        if args.mode == "calibrate":
            return calibrate(cfg)
        if args.mode == "urdf":
            return urdf(cfg, args.check)
        return validate(cfg)
    except ERRORS as exc:
        print(f"{args.mode} failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
