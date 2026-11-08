"""Render the ARM board from eight directions and the board seated in the gripper (grip fit) to PNG files.

Run: uv run --group render python -m scripts.markers_render [config]   (default scripts/markers_render.yaml)
Needs the OSMesa libs: eval "$(bash scripts/markers/setup_render_libs.sh | grep '^export')"
The module imports pyvista (scripts.markers.render), so it needs the render dependency group; the config model lives
in scripts.markers.config and loads without it.
"""

import argparse
import json
import sys
from pathlib import Path

import trimesh
from pydantic import ValidationError

from scripts.markers.config import RenderConfig, load_render_config
from scripts.markers.jaw_fit import load_gripper
from scripts.markers.render import render_views
from scripts.markers.views import GRIP_VIEW, VIEW_DIRECTIONS, grip_scene

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = REPO_ROOT / "scripts" / "markers_render.yaml"
GRIP_VIEW_DIRECTION = (0.0, 1.0, 0.0)  # along the finger width: the jaw gap and the stacked handle side on
BOARD_COLOURS = ("base", "modules")


def repo_path(value: str) -> Path:
    """A config path, relative ones resolved against the repo root.

    Args:
        value (str): Path from the config.

    Returns:
        Path: The absolute path.
    """
    path = Path(value)
    return path if path.is_absolute() else REPO_ROOT / path


def render_all(config: RenderConfig) -> list[Path]:
    """Render the eight board views and the grip fit.

    Args:
        config (RenderConfig): The config.

    Returns:
        list[Path]: Nine PNG paths: eight "<board>-<view>.png" and "<board>-grip_fit.png".
    """
    markers = repo_path(config.markers_dir)
    base = trimesh.load(markers / f"{config.board}-base.stl")
    modules = trimesh.load(markers / f"{config.board}-modules.stl")
    meta = json.loads((markers / f"{config.board}.json").read_text())
    out_dir = repo_path(config.out_dir)
    size = config.size
    paths = render_views([(base, "base"), (modules, "modules")], out_dir, size, VIEW_DIRECTIONS, config.board)
    scene = grip_scene(load_gripper(repo_path(config.arm_urdf_file)), meta, base, modules)
    paths += render_views(scene, out_dir, size, {GRIP_VIEW: GRIP_VIEW_DIRECTION}, config.board)
    return paths


def main(argv: list[str] | None = None) -> int:
    """Render the boards.

    Args:
        argv (list[str] | None): Command line without the program name; one optional config path.

    Returns:
        int: 0 when the PNGs were written, 1 on an invalid config.
    """
    parser = argparse.ArgumentParser(description="Offscreen renders of the ChArUco boards and the grip fit.")
    parser.add_argument("config", nargs="?", type=Path, default=DEFAULT_CONFIG, help="render YAML")
    args = parser.parse_args(argv)
    try:
        config = load_render_config(args.config)
    except ValidationError as exc:
        print(f"invalid config {args.config}:\n{exc}", file=sys.stderr)
        return 1
    for path in render_all(config):
        print(path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
