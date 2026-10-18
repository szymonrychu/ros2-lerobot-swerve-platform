"""Build a QR Model 2 symbol (segno) for a marker spec, with version/ECC chosen and checked up front."""

from dataclasses import dataclass

import numpy as np
import segno

from scripts.markers.models import MarkerSpec, SymbolDefaults
from shared.ros2_common.marker_payload import encode_payload

ALNUM_CAPACITY = {(2, "m"): 38, (2, "q"): 29, (3, "m"): 61, (3, "q"): 47, (4, "m"): 90, (4, "q"): 67}
LIGHT = 255
DARK = 0


@dataclass(frozen=True)
class Symbol:
    """A QR symbol.

    payload_text: encoded text. version: QR version. ecc: "M" or "Q" as reported by segno. matrix: bool modules
    without quiet zone (True = dark). modules: 17 + 4 * version. module_mm: size_mm / modules. size_mm: symbol edge
    excluding quiet zone. quiet_zone_modules: quiet zone width in modules.
    """

    payload_text: str
    version: int
    ecc: str
    matrix: np.ndarray
    modules: int
    module_mm: float
    size_mm: float
    quiet_zone_modules: int


def build_symbol(spec: MarkerSpec, defaults: SymbolDefaults) -> Symbol:
    """Encode a marker as the smallest fitting QR version in the configured range.

    Args:
        spec: The marker.
        defaults: Symbol defaults (ecc, version range, quiet zone).

    Returns:
        The Symbol.

    Raises:
        ValueError: naming payload length and capacity when it exceeds max_version.
    """
    text = encode_payload(spec.payload())
    capacity = ALNUM_CAPACITY[(defaults.max_version, defaults.ecc)]
    if len(text) > capacity:
        raise ValueError(
            f"payload {text!r} is {len(text)} characters, over the version {defaults.max_version}-"
            f"{defaults.ecc.upper()} capacity {capacity}"
        )
    version = next(
        v
        for v in range(defaults.min_version, defaults.max_version + 1)
        if len(text) <= ALNUM_CAPACITY[(v, defaults.ecc)]
    )
    qr = segno.make_qr(text, mode="alphanumeric", error=defaults.ecc, boost_error=False, version=version)
    matrix = np.array([[bool(cell) for cell in row] for row in qr.matrix], dtype=bool)
    modules = matrix.shape[0]
    return Symbol(
        payload_text=text,
        version=qr.version,
        ecc=qr.error,
        matrix=matrix,
        modules=modules,
        module_mm=spec.size_mm / modules,
        size_mm=spec.size_mm,
        quiet_zone_modules=defaults.quiet_zone_modules,
    )


def render_raster(symbol: Symbol, px_per_module: int) -> np.ndarray:
    """Render the symbol with its quiet zone as a grayscale image.

    Args:
        symbol: The symbol.
        px_per_module: Pixels per module (>= 1).

    Returns:
        uint8 array, 0 = dark, 255 = light, side (modules + 2 * quiet zone) * px_per_module.
    """
    quiet = symbol.quiet_zone_modules
    padded = np.pad(symbol.matrix, quiet, constant_values=False)
    scaled = np.kron(padded, np.ones((px_per_module, px_per_module), dtype=bool))
    return np.where(scaled, DARK, LIGHT).astype(np.uint8)
