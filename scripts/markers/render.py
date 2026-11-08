"""Offscreen pyvista rendering of trimesh bodies (needs the render dependency group and the OSMesa libs)."""

from pathlib import Path

import numpy as np
import pyvista as pv
import trimesh

from scripts.markers.views import COLOUR_BASE, COLOUR_FIXED_JAW, COLOUR_MODULES, COLOUR_MOVING_JAW, camera_for

BACKGROUND = "gray"
COLOURS = {
    "base": COLOUR_BASE,
    "modules": COLOUR_MODULES,
    "fixed_jaw": COLOUR_FIXED_JAW,
    "moving_jaw": COLOUR_MOVING_JAW,
}


def render_views(
    meshes: list[tuple[trimesh.Trimesh, str]],
    out_dir: Path,
    size: tuple[int, int],
    views: dict[str, tuple[float, float, float]],
    prefix: str,
) -> list[Path]:
    """Render the meshes from each view direction to PNG files.

    Args:
        meshes (list[tuple[trimesh.Trimesh, str]]): Bodies with a colour key (a name of COLOURS or a colour name).
        out_dir (Path): Output directory, created when missing.
        size (tuple[int, int]): Image (width, height) in pixels.
        views (dict[str, tuple[float, float, float]]): View name to direction (centre toward camera).
        prefix (str): File name prefix; files are "<prefix>-<view>.png".

    Returns:
        list[Path]: The written PNG paths, in the order of `views`.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    bounds = np.array(
        [np.min([m.bounds[0] for m, _ in meshes], axis=0), np.max([m.bounds[1] for m, _ in meshes], axis=0)]
    )
    plotter = pv.Plotter(off_screen=True, window_size=list(size))
    plotter.set_background(BACKGROUND)
    for mesh, name in meshes:
        poly = pv.wrap(mesh)
        plotter.add_mesh(poly, color=COLOURS.get(name, name), smooth_shading=False, show_edges=False)
    paths: list[Path] = []
    for view, direction in views.items():
        position, focal, up = camera_for(direction, bounds)
        plotter.camera_position = [position, focal, up]
        plotter.reset_camera_clipping_range()
        plotter.render()
        path = out_dir / f"{prefix}-{view}.png"
        plotter.screenshot(str(path))
        paths.append(path)
    plotter.close()
    return paths
