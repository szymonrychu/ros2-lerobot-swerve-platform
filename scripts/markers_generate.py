"""Generate the QR metadata markers and the ChArUco calibration board as print-ready files.

Run: uv run python -m scripts.markers_generate [config]   (default scripts/markers_generate.yaml)
"""

import argparse
import logging
import sys
from pathlib import Path

from pydantic import ValidationError

from scripts.markers.charuco import CharucoSpec, write_charuco
from scripts.markers.config import GeneratorConfig, load_generator_config
from scripts.markers.mesh import marker_bodies, write_bodies
from scripts.markers.models import MarkerSpec
from scripts.markers.pdf import write_marker_pdf
from scripts.markers.symbol import build_symbol

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = REPO_ROOT / "scripts" / "markers_generate.yaml"
MESH_SUFFIXES = {"stl": ("-base.stl", "-modules.stl"), "3mf": (".3mf",)}

LOGGER = logging.getLogger(__name__)


def marker_stem(spec: MarkerSpec) -> str:
    """File stem of a marker.

    Args:
        spec: The marker.

    Returns:
        "<kind>-<name>" in lower case.
    """
    return f"{spec.kind_code}-{spec.name}".lower()


def charuco_stem(spec: CharucoSpec) -> str:
    """File stem of the board.

    Args:
        spec: The board.

    Returns:
        "charuco-<x>x<y>-<square>mm".
    """
    return f"charuco-{spec.squares_x}x{spec.squares_y}-{spec.square_mm:g}mm"


def write_marker(spec: MarkerSpec, config: GeneratorConfig, out_dir: Path) -> list[Path]:
    """Write the requested outputs of one marker; nothing is left on disk when it fails.

    Args:
        spec: The marker.
        config: The whole config (symbol defaults and STL options).
        out_dir: Output directory.

    Returns:
        The written paths.

    Raises:
        ValueError: when the marker cannot be produced (payload too long, too small for FDM, does not fit the page).
    """
    symbol = build_symbol(spec, config.defaults)
    stem = out_dir / marker_stem(spec)
    wanted_meshes = {s for fmt in ("stl", "3mf") if fmt in spec.outputs for s in MESH_SUFFIXES[fmt]}
    bodies = marker_bodies(symbol, config.stl) if wanted_meshes else None
    out_dir.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []
    try:
        if "pdf" in spec.outputs:
            pdf_path = stem.with_suffix(".pdf")
            written.append(pdf_path)
            label = f"{spec.kind_code} {spec.name}"
            write_marker_pdf(symbol, spec.page or config.defaults.page, pdf_path, label)
        if bodies is not None:
            for path in write_bodies(bodies[0], bodies[1], stem):
                if any(path.name.endswith(s) for s in wanted_meshes):
                    written.append(path)
                else:
                    path.unlink()
    except Exception:
        for path in written:
            path.unlink(missing_ok=True)
        raise
    return written


def main(argv: list[str] | None = None) -> int:
    """Generate every marker and the board of the config.

    Args:
        argv: Command line without the program name; one optional config path.

    Returns:
        0 when everything was written, 1 on an invalid config or when any marker could not be produced.
    """
    parser = argparse.ArgumentParser(description="Generate printable QR markers and the ChArUco board.")
    parser.add_argument("config", nargs="?", type=Path, default=DEFAULT_CONFIG, help="generator YAML")
    args = parser.parse_args(argv)
    try:
        config = load_generator_config(args.config)
    except ValidationError as exc:
        print(f"invalid config {args.config}:\n{exc}", file=sys.stderr)
        return 1
    out_dir = Path(config.output_dir)
    if not out_dir.is_absolute():
        out_dir = REPO_ROOT / out_dir
    failures: list[str] = []
    files = 0
    for spec in config.markers:
        try:
            written = write_marker(spec, config, out_dir)
        except ValueError as exc:
            failures.append(f"{spec.kind_code} {spec.name}")
            print(f"FAILED marker {spec.kind_code} {spec.name}: {exc}", file=sys.stderr)
            continue
        files += len(written)
        for path in written:
            print(path)
    if config.charuco is not None:
        try:
            written = write_charuco(config.charuco, config.stl, out_dir / charuco_stem(config.charuco))
        except ValueError as exc:
            failures.append("charuco")
            print(f"FAILED charuco board: {exc}", file=sys.stderr)
        else:
            files += len(written)
            for path in written:
                print(path)
    failed = f": {', '.join(failures)}" if failures else ""
    print(f"wrote {files} files to {out_dir}; {len(failures)} failed{failed}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
