"""ChArUco calibration board: spec, OpenCV board, exact-pixel dark mask, PDF and two-colour STL/3MF output."""

import logging
from math import isclose
from pathlib import Path
from typing import Literal, Self

import cv2
import numpy as np
from pydantic import Field, field_validator, model_validator

from scripts.markers.mesh import mask_bodies, write_bodies
from scripts.markers.models import OUTPUT_FORMATS, PageSpec, StlOptions, StrictModel
from scripts.markers.pdf import draw_mask_pdf, page_mm

LOGGER = logging.getLogger(__name__)
MM_PER_M = 1000.0
MIN_THICKNESS_MM = 4.0
PX_PER_MM_STEP = 0.05
PX_PER_MM_MAX = 20.0
EXACT_TOL = 1e-6
DARK_BELOW = 128
PDF_TOP_MM = 2.5
LABEL_BELOW_BOARD_MM = 5.0
MESH_SUFFIX = {"stl": ("-base.stl", "-modules.stl"), "3mf": (".3mf",)}


class CharucoSpec(StrictModel):
    """ChArUco board definition shared by the generator and the calibration tool.

    squares_x / squares_y: chessboard squares (squares_y odd). square_mm / marker_mm: square and ArUco marker
    edge. dictionary: any cv2.aruco.DICT_* name. margin_mm: plate border around the board. thickness_mm: STL base
    thickness (>= 4). relief_mm: module height above the base. inlay_mm: module depth sunk into the base (flush inlay when relief_mm is 0). bed_mm: print bed edge the plate must fit. page: PDF
    media. outputs: subset of pdf/stl/3mf.
    """

    squares_x: int = Field(default=6, ge=2)
    squares_y: int = Field(default=5, ge=2)
    square_mm: float = Field(default=40.0, gt=0)
    marker_mm: float = Field(default=30.0, gt=0)
    dictionary: str = "DICT_5X5_100"
    margin_mm: float = Field(default=5.0, ge=0)
    thickness_mm: float = Field(default=MIN_THICKNESS_MM, ge=MIN_THICKNESS_MM)
    relief_mm: float = Field(default=0.0, ge=0)
    inlay_mm: float = Field(default=1.2, ge=0)
    bed_mm: float = Field(default=256.0, gt=0)
    page: PageSpec = Field(default_factory=lambda: PageSpec(size="A4", orientation="landscape"))
    outputs: list[Literal["pdf", "stl", "3mf"]] = Field(default_factory=lambda: list(OUTPUT_FORMATS), min_length=1)

    @field_validator("dictionary")
    @classmethod
    def check_dictionary(cls, value: str) -> str:
        """The dictionary is a cv2.aruco.DICT_* name."""
        if not value.startswith("DICT_") or not hasattr(cv2.aruco, value):
            raise ValueError(f"dictionary {value!r} is not a cv2.aruco.DICT_* name")
        return value

    @field_validator("squares_y")
    @classmethod
    def check_odd_rows(cls, value: int) -> int:
        """An even row count flips the legacy corner layout, so it is refused."""
        if value % 2 == 0:
            raise ValueError(f"squares_y {value} must be odd (an even row count changes the legacy board layout)")
        return value

    @model_validator(mode="after")
    def check_geometry(self) -> Self:
        """marker_mm < square_mm, modules have height, inlay < thickness, plate fits bed_mm in both directions."""
        if self.relief_mm + self.inlay_mm <= 0:
            raise ValueError("relief_mm + inlay_mm must be positive")
        if self.inlay_mm >= self.thickness_mm:
            raise ValueError(f"inlay_mm {self.inlay_mm} must be below thickness_mm {self.thickness_mm}")
        if self.marker_mm >= self.square_mm:
            raise ValueError(f"marker_mm {self.marker_mm} must be below square_mm {self.square_mm}")
        for field, squares in (("squares_x", self.squares_x), ("squares_y", self.squares_y)):
            plate = squares * self.square_mm + 2 * self.margin_mm
            if plate > self.bed_mm:
                raise ValueError(f"{field} {squares}: plate {plate:g} mm exceeds bed_mm {self.bed_mm:g}")
        return self

    @property
    def plate_mm(self) -> tuple[float, float]:
        """Plate (width, height) in mm."""
        return (
            self.squares_x * self.square_mm + 2 * self.margin_mm,
            self.squares_y * self.square_mm + 2 * self.margin_mm,
        )

    @property
    def marker_cell_mm(self) -> float:
        """Edge of one ArUco marker cell (marker bits plus the 1-cell border) in mm."""
        size = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self.dictionary)).markerSize
        return self.marker_mm / (size + 2)


def make_charuco_board(spec: CharucoSpec) -> cv2.aruco.CharucoBoard:
    """Build the OpenCV board (lengths in metres).

    Args:
        spec: The board definition.

    Returns:
        The cv2.aruco.CharucoBoard.
    """
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, spec.dictionary))
    return cv2.aruco.CharucoBoard(
        (spec.squares_x, spec.squares_y), spec.square_mm / MM_PER_M, spec.marker_mm / MM_PER_M, dictionary
    )


def is_whole(value: float) -> bool:
    """Whether value is an integer within EXACT_TOL."""
    return isclose(value, round(value), abs_tol=EXACT_TOL)


def exact_px_per_mm(spec: CharucoSpec) -> float:
    """Smallest resolution where squares, marker cells and the square-to-marker gap are whole pixels.

    Args:
        spec: The board definition.

    Returns:
        Pixels per mm.

    Raises:
        ValueError: when no resolution up to PX_PER_MM_MAX is exact.
    """
    lengths = (spec.square_mm, spec.marker_mm, spec.marker_cell_mm, (spec.square_mm - spec.marker_mm) / 2)
    steps = int(PX_PER_MM_MAX / PX_PER_MM_STEP)
    for step in range(1, steps + 1):
        ppm = step * PX_PER_MM_STEP
        if all(is_whole(length * ppm) for length in lengths):
            return ppm
    raise ValueError("no exact raster resolution for this square and marker size")


def board_mask(spec: CharucoSpec) -> tuple[np.ndarray, float]:
    """Dark mask of the board and its cell size.

    Args:
        spec: The board definition.

    Returns:
        (mask, cell_mm): bool array, True = dark, row 0 on top; cell_mm is the edge of one mask pixel.
    """
    ppm = exact_px_per_mm(spec)
    width, height = round(spec.squares_x * spec.square_mm * ppm), round(spec.squares_y * spec.square_mm * ppm)
    assert is_whole(spec.squares_x * spec.square_mm * ppm) and is_whole(spec.squares_y * spec.square_mm * ppm)
    image = make_charuco_board(spec).generateImage((width, height), marginSize=0, borderBits=1)
    assert image.shape == (height, width), f"board image {image.shape} differs from the exact size {(height, width)}"
    return image < DARK_BELOW, 1.0 / ppm


def write_charuco(spec: CharucoSpec, opts: StlOptions, stem: Path) -> list[Path]:
    """Write the requested outputs for the board.

    PDF: <stem>.pdf at 100 % scale. Meshes: <stem>-base.stl, <stem>-modules.stl, <stem>.3mf.

    Args:
        spec: The board definition.
        opts: 3D print options (min_module_mm and warn_module_mm are checked against the marker cell).
        stem: Output path without suffix.

    Returns:
        The written paths.

    Raises:
        ValueError: when the marker cell is below opts.min_module_mm.
    """
    mask, cell_mm = board_mask(spec)
    written: list[Path] = []
    stem.parent.mkdir(parents=True, exist_ok=True)
    if "pdf" in spec.outputs:
        width_mm, height_mm = spec.squares_x * spec.square_mm, spec.squares_y * spec.square_mm
        label = (
            f"ChArUco {spec.squares_x}x{spec.squares_y} {spec.dictionary} square {spec.square_mm:g} mm "
            f"marker {spec.marker_mm:g} mm - Print at 100 %"
        )
        origin_x = (page_mm(spec.page)[0] - width_mm) / 2
        pdf_path = stem.with_suffix(".pdf")
        draw_mask_pdf(
            mask,
            cell_mm,
            spec.page,
            pdf_path,
            (origin_x, PDF_TOP_MM),
            label,
            label_y_mm=PDF_TOP_MM + height_mm + LABEL_BELOW_BOARD_MM,
        )
        written.append(pdf_path)
    wanted = [suffix for fmt in ("stl", "3mf") if fmt in spec.outputs for suffix in MESH_SUFFIX[fmt]]
    if wanted:
        if spec.marker_cell_mm < opts.min_module_mm:
            raise ValueError(f"marker cell {spec.marker_cell_mm:.2f} mm is below min_module_mm {opts.min_module_mm}")
        mesh_opts = opts.model_copy(
            update={
                "base_thickness_mm": spec.thickness_mm,
                "relief_mm": spec.relief_mm,
                "inlay_mm": spec.inlay_mm,
                "min_module_mm": min(opts.min_module_mm, cell_mm),
                "warn_module_mm": min(opts.warn_module_mm, cell_mm),
            }
        )
        base, modules = mask_bodies(mask, cell_mm, spec.plate_mm, (spec.margin_mm, spec.margin_mm), mesh_opts)
        for path in write_bodies(base, modules, stem):
            if any(path.name.endswith(suffix) for suffix in wanted):
                written.append(path)
            else:
                path.unlink()
    return written
