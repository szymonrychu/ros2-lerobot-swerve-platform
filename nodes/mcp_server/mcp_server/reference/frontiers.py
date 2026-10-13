"""Python references of the frontiers kernel family (registry read by tests/regenerate_core_golden.py).

The kernels ported to mcp_core_cpp.frontiers (bench/report.json verdict "port"): traversability, geodesic_from,
frontier_field and tracker_update (FrontierTracker.update). Their original Python bodies live here, moved verbatim
from mcp_server/frontiers.py, and are used only to write the goldens. Each ``*_case`` function runs one golden case
through a given implementation, so the golden generator passes the reference and the parity test passes the
mcp_server entry point (which calls the C++ kernel) through the very same case code.
"""

import math
import time
from collections.abc import Callable, Mapping, Sequence
from functools import partial
from typing import Any

import numpy as np
from scipy import ndimage
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

from mcp_server.config import FrontierSettings
from mcp_server.frontiers import (
    EPOCH_RESET_NOTE,
    Frontier,
    FrontierField,
    FrontierSnapshot,
    FrontierTracker,
    MapGrid,
    RawFrontier,
    extract_frontiers,
    free_mask,
    known_bbox,
    match_ids,
    min_cells_for,
    snap_to_traversable,
)
from mcp_server.perception import OCCUPIED_THRESHOLD
from mcp_server.reference import ReferenceKernel, Tolerance

# Grid placement of the golden maps (the kernels are invariant to it; cells_to_map uses it for centroids).
CASE_ORIGIN = (-1.0, -2.0, 0.0)
EXACT = Tolerance()
DISTANCE_TOLERANCE = Tolerance(atol=1e-9, rtol=1e-12)  # Dijkstra may sum equal-length paths in another order


def traversability_reference(grid: MapGrid, pass_clearance_m: float) -> tuple[np.ndarray, np.ndarray]:
    """Traversable cells and obstacle clearance; unknown cells and cells outside the map count as obstacles.

    Args:
        grid (MapGrid): The map.
        pass_clearance_m (float): Clearance a traversable cell needs (footprint half width + margin) (m).

    Returns:
        tuple[np.ndarray, np.ndarray]: (trav bool H x W: known free with clearance >= pass_clearance_m,
        clearance_m float32 H x W: cell centre to the nearest obstacle cell edge (m), 0 on obstacles).
    """
    data = grid.data
    clearance = np.zeros(data.shape, dtype=np.float32)
    box = known_bbox(data)
    if box is not None:
        r0, c0, r1, c1 = box
        sub = data[r0:r1, c0:c1]
        open_cells = np.pad((sub >= 0) & (sub < OCCUPIED_THRESHOLD), 1, constant_values=False)
        cells = ndimage.distance_transform_edt(open_cells)[1:-1, 1:-1]
        clearance[r0:r1, c0:c1] = np.maximum(cells - 0.5, 0.0) * grid.resolution
    return free_mask(data) & (clearance >= pass_clearance_m), clearance


def geodesic_from_reference(
    trav: np.ndarray, start_rc: tuple[int, int], downsample: int, resolution: float
) -> np.ndarray:
    """Shortest path length through traversable space from a start cell to every cell (8-neighbour Dijkstra).

    The search runs on a grid downsample x coarser (a coarse cell is traversable when any of its cells is). A start
    outside traversable space snaps to the nearest traversable cell within SNAP_RADIUS_M (the snap is added to every
    distance); without one, nothing is reachable.

    Args:
        trav (np.ndarray): bool H x W traversable cells.
        start_rc (tuple[int, int]): Start (row, col).
        downsample (int): Coarsening factor (>= 1).
        resolution (float): Cell size (m).

    Returns:
        np.ndarray: float H x W distances (m), inf where unreachable or not traversable.
    """
    out = np.full(trav.shape, np.inf)
    snapped = snap_to_traversable(trav, start_rc, resolution)
    if snapped is None:
        return out
    (sr, sc), snap_m = snapped
    ds = int(downsample)
    rows = np.flatnonzero(trav.any(axis=1))
    cols = np.flatnonzero(trav.any(axis=0))
    r0, r1, c0, c1 = int(rows[0]), int(rows[-1]) + 1, int(cols[0]), int(cols[-1]) + 1
    sub = trav[r0:r1, c0:c1]
    h, w = -(-sub.shape[0] // ds), -(-sub.shape[1] // ds)
    padded = np.zeros((h * ds, w * ds), dtype=bool)
    padded[: sub.shape[0], : sub.shape[1]] = sub
    coarse = padded.reshape(h, ds, w, ds).any(axis=(1, 3))
    index = np.full((h, w), -1, dtype=np.int64)
    nodes = np.flatnonzero(coarse)
    index.flat[nodes] = np.arange(nodes.size)
    src_parts: list[np.ndarray] = []
    dst_parts: list[np.ndarray] = []
    cost_parts: list[np.ndarray] = []
    for dr, dc, cost in ((0, 1, 1.0), (1, 0, 1.0), (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0))):
        src_win = (slice(0, h - dr), slice(max(0, -dc), w - max(0, dc)))
        dst_win = (slice(dr, h), slice(max(0, dc), w - max(0, -dc)))
        both = coarse[src_win] & coarse[dst_win]
        src_parts.append(index[src_win][both])
        dst_parts.append(index[dst_win][both])
        cost_parts.append(np.full(int(both.sum()), cost))
    graph = csr_matrix(
        (np.concatenate(cost_parts), (np.concatenate(src_parts), np.concatenate(dst_parts))),
        shape=(nodes.size, nodes.size),
    )
    start = int(index[(sr - r0) // ds, (sc - c0) // ds])
    reach = dijkstra(graph, directed=False, indices=start)
    coarse_dist = np.full(h * w, np.inf)
    coarse_dist[nodes] = reach * ds * resolution + snap_m
    fine = np.repeat(np.repeat(coarse_dist.reshape(h, w), ds, axis=0), ds, axis=1)
    out[r0:r1, c0:c1] = fine[: sub.shape[0], : sub.shape[1]]
    out[~trav] = np.inf
    return out


def frontier_field_reference(grid: MapGrid, found: Sequence[RawFrontier]) -> FrontierField:
    """One EDT from every frontier cell over the known area, with the nearest frontier cell and its label.

    Args:
        grid (MapGrid): The map.
        found (Sequence[RawFrontier]): Frontiers of this map (their label values).

    Returns:
        FrontierField: The field (empty arrays when there is no frontier or nothing is known).
    """
    box = known_bbox(grid.data)
    if box is None or not found:
        empty = np.zeros((0, 0), dtype=np.int32)
        return FrontierField(0, 0, np.zeros((0, 0)), empty, empty, empty)
    r0, c0, r1, c1 = box
    label = np.zeros((r1 - r0, c1 - c0), dtype=np.int32)
    for f in found:
        label[f.cells[:, 0] - r0, f.cells[:, 1] - c0] = f.label
    cells, (near_r, near_c) = ndimage.distance_transform_edt(label == 0, return_indices=True)
    return FrontierField(
        row0=r0,
        col0=c0,
        distance_m=cells * grid.resolution,
        label=label[near_r, near_c],
        nearest_row=(near_r + r0).astype(np.int32),
        nearest_col=(near_c + c0).astype(np.int32),
    )


class ReferenceFrontierTracker(FrontierTracker):
    """FrontierTracker whose update is the original Python body (Python traversability and frontier field)."""

    def update(self, grid: MapGrid) -> FrontierSnapshot:
        """Frontiers of a map, recomputed only when its version differs from the last one.

        Args:
            grid (MapGrid): The current map.

        Returns:
            FrontierSnapshot: The snapshot of grid.version.
        """
        with self.lock:
            prev = self.snapshot
            if prev is not None and prev.version == grid.version:
                return prev
            notes: list[str] = []
            previous: tuple[Frontier, ...] = () if prev is None else prev.frontiers
            if prev is not None and (
                not math.isclose(grid.resolution, prev.grid.resolution)
                or grid.known_cells < self.settings.epoch_reset_known_fraction * prev.grid.known_cells
            ):
                self.epoch += 1
                self.next_id = 1
                self.retired_ids.clear()
                self.unreported.clear()
                previous = ()
                notes.append(EPOCH_RESET_NOTE)
            t0 = time.perf_counter()
            found = extract_frontiers(grid, min_cells_for(self.settings.min_length_m, grid.resolution))
            t1 = time.perf_counter()
            trav, clearance = traversability_reference(grid, self.pass_clearance_m)
            t2 = time.perf_counter()
            field = frontier_field_reference(grid, found)
            t3 = time.perf_counter()
            current, retired_entries, self.next_id = match_ids(
                previous, found, self.settings.match_radius_m, self.next_id
            )
            retired = tuple(self.retire(entry, grid.version) for entry in retired_entries)
            t4 = time.perf_counter()
            self.snapshot = FrontierSnapshot(
                version=grid.version,
                epoch=self.epoch,
                grid=grid,
                frontiers=tuple(current),
                trav=trav,
                clearance_m=clearance,
                field=field,
                timings_ms={
                    "extract": (t1 - t0) * 1000.0,
                    "traversability": (t2 - t1) * 1000.0,
                    "field": (t3 - t2) * 1000.0,
                    "match": (t4 - t3) * 1000.0,
                },
                notes=tuple(notes),
                retired=retired,
            )
            self.geodesic_cache.clear()
            self.compute_count += 1
            return self.snapshot


def case_grid(data: np.ndarray, resolution: float, version: tuple = (1,)) -> MapGrid:
    """MapGrid of a golden case map at CASE_ORIGIN.

    Args:
        data (np.ndarray): int16 H x W cell values.
        resolution (float): Cell size (m).
        version (tuple): Map version.

    Returns:
        MapGrid: The grid.
    """
    return MapGrid.from_array(data, resolution, *CASE_ORIGIN, version)


def field_outputs(field: FrontierField) -> dict[str, Any]:
    """Named outputs of a FrontierField.

    Args:
        field (FrontierField): The field.

    Returns:
        dict[str, Any]: row0, col0, distance_m, label, nearest_row, nearest_col.
    """
    return {
        "row0": field.row0,
        "col0": field.col0,
        "distance_m": field.distance_m,
        "label": field.label,
        "nearest_row": field.nearest_row,
        "nearest_col": field.nearest_col,
    }


def traversability_case(
    impl: Callable[[MapGrid, float], tuple[np.ndarray, np.ndarray]],
    data: np.ndarray,
    resolution: float,
    pass_clearance_m: float,
) -> dict[str, Any]:
    """One traversability golden case.

    Args:
        impl (Callable[[MapGrid, float], tuple[np.ndarray, np.ndarray]]): traversability implementation.
        data (np.ndarray): int16 H x W map.
        resolution (float): Cell size (m).
        pass_clearance_m (float): Clearance a traversable cell needs (m).

    Returns:
        dict[str, Any]: trav (bool H x W), clearance_m (float32 H x W).
    """
    trav, clearance = impl(case_grid(data, resolution), pass_clearance_m)
    return {"trav": trav, "clearance_m": clearance}


def geodesic_case(
    impl: Callable[[np.ndarray, tuple[int, int], int, float], np.ndarray],
    trav: np.ndarray,
    start_rc: Sequence[int],
    downsample: int,
    resolution: float,
) -> dict[str, Any]:
    """One geodesic_from golden case.

    Args:
        impl (Callable[[np.ndarray, tuple[int, int], int, float], np.ndarray]): geodesic_from implementation.
        trav (np.ndarray): bool H x W traversable cells.
        start_rc (Sequence[int]): Start (row, col).
        downsample (int): Coarsening factor.
        resolution (float): Cell size (m).

    Returns:
        dict[str, Any]: distance_m (float H x W, inf where unreachable).
    """
    return {"distance_m": impl(trav, (int(start_rc[0]), int(start_rc[1])), downsample, resolution)}


def field_case(
    impl: Callable[[MapGrid, Sequence[RawFrontier]], FrontierField],
    data: np.ndarray,
    resolution: float,
    min_cells: int,
) -> dict[str, Any]:
    """One frontier_field golden case: the frontiers of extract_frontiers (Python, not ported), then the field.

    Args:
        impl (Callable[[MapGrid, Sequence[RawFrontier]], FrontierField]): frontier_field implementation.
        data (np.ndarray): int16 H x W map.
        resolution (float): Cell size (m).
        min_cells (int): Smallest frontier cluster kept.

    Returns:
        dict[str, Any]: The field outputs (field_outputs).
    """
    grid = case_grid(data, resolution)
    return field_outputs(impl(grid, extract_frontiers(grid, min_cells)))


def snapshot_outputs(snapshot: FrontierSnapshot) -> dict[str, Any]:
    """Named outputs of one tracker snapshot (everything but the timings).

    Args:
        snapshot (FrontierSnapshot): The snapshot.

    Returns:
        dict[str, Any]: epoch, notes, frontiers (id, label, cells and geometry), retired, trav, clearance_m, field.
    """
    return {
        "epoch": snapshot.epoch,
        "notes": list(snapshot.notes),
        "frontiers": [
            {
                "id": f.id,
                "label": f.label,
                "cells": f.cells,
                "centroid_xy": list(f.centroid_xy),
                "target_xy": list(f.target_xy),
                "target_rc": list(f.target_rc),
                "length_m": f.length_m,
                "bbox": list(f.bbox),
            }
            for f in snapshot.frontiers
        ],
        "retired": [{"id": r.id, "reason": r.reason, "successor": r.successor} for r in snapshot.retired],
        "trav": snapshot.trav,
        "clearance_m": snapshot.clearance_m,
        "field": field_outputs(snapshot.field),
    }


def tracker_case(
    tracker_type: type[FrontierTracker],
    maps: Sequence[Mapping[str, Any]],
    settings: Mapping[str, Any],
    footprint_width_m: float,
) -> dict[str, Any]:
    """One tracker golden case: a sequence of map versions through one tracker (ids carried across versions).

    Args:
        tracker_type (type[FrontierTracker]): Tracker class (FrontierTracker or ReferenceFrontierTracker).
        maps (Sequence[Mapping[str, Any]]): Map versions in order, each {"data": int16 H x W, "resolution": float}.
        settings (Mapping[str, Any]): FrontierSettings fields.
        footprint_width_m (float): Robot width (m).

    Returns:
        dict[str, Any]: versions (snapshot_outputs per map version).
    """
    tracker = tracker_type(FrontierSettings(**settings), footprint_width_m=footprint_width_m)
    versions = [
        snapshot_outputs(tracker.update(case_grid(m["data"], m["resolution"], (i,)))) for i, m in enumerate(maps)
    ]
    return {"versions": versions}


# Kernel name -> reference (bench/inputs.py builds the cases).
KERNELS: dict[str, ReferenceKernel] = {
    "traversability": ReferenceKernel(
        run=partial(traversability_case, traversability_reference),
        inputs="frontier_traversability_cases",
        tolerances={"default": EXACT},
    ),
    "geodesic_from": ReferenceKernel(
        run=partial(geodesic_case, geodesic_from_reference),
        inputs="frontier_geodesic_cases",
        tolerances={"distance_m": DISTANCE_TOLERANCE},
    ),
    "frontier_field": ReferenceKernel(
        run=partial(field_case, frontier_field_reference),
        inputs="frontier_field_cases",
        tolerances={"default": EXACT},
    ),
    "tracker_update": ReferenceKernel(
        run=partial(tracker_case, ReferenceFrontierTracker),
        inputs="frontier_tracker_cases",
        tolerances={"default": EXACT},
    ),
}
