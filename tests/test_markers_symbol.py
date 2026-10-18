"""Tests for scripts/markers/config.py and symbol.py: generator config loading and the QR symbol builder."""

from pathlib import Path

import pytest
import segno
import yaml
import zxingcpp
from pydantic import ValidationError

from scripts.markers.config import GeneratorConfig, MarkerSpec, SymbolDefaults, load_generator_config
from scripts.markers.symbol import ALNUM_CAPACITY, build_symbol, render_raster
from shared.ros2_common.marker_payload import KNOWN_KINDS

SHIPPED = Path(__file__).resolve().parent.parent / "scripts" / "markers_generate.yaml"


def spec(name: str = "DOCK-1", kind: str = "CHARGER", size: float = 150.0, **params: str) -> MarkerSpec:
    """Build a MarkerSpec."""
    return MarkerSpec(kind_code=kind, name=name, size_mm=size, params=params)


def write_config(tmp_path: Path, **overrides: object) -> Path:
    """Write a minimal config with overrides and return its path."""
    data: dict[str, object] = {"markers": [{"kind_code": "CHARGER", "name": "DOCK-1", "size_mm": 150}]}
    data.update(overrides)
    path = tmp_path / "cfg.yaml"
    path.write_text(yaml.safe_dump(data))
    return path


def test_shipped_yaml_covers_known_kinds() -> None:
    config = load_generator_config(SHIPPED)
    assert len(config.markers) == 6
    assert {m.kind_code for m in config.markers} == set(KNOWN_KINDS)
    assert config.defaults.ecc == "q"
    assert config.stl.relief_mm == 0.6
    for marker in config.markers:
        assert marker.outputs == ["pdf", "stl", "3mf"]


def test_unknown_key_names_it(tmp_path: Path) -> None:
    with pytest.raises(ValidationError, match="bogus_key"):
        load_generator_config(write_config(tmp_path, bogus_key=1))


def test_invalid_marker_name_names_it(tmp_path: Path) -> None:
    bad = [{"kind_code": "CHARGER", "name": "bad name!", "size_mm": 150}]
    with pytest.raises(ValueError, match="bad name!"):
        load_generator_config(write_config(tmp_path, markers=bad))


def test_charuco_slot_is_free_form(tmp_path: Path) -> None:
    config = load_generator_config(write_config(tmp_path, charuco={"anything": 1}))
    assert config.charuco == {"anything": 1}
    assert isinstance(config, GeneratorConfig)


def test_defaults_are_validated() -> None:
    with pytest.raises(ValidationError):
        SymbolDefaults(max_version=5)
    with pytest.raises(ValidationError):
        SymbolDefaults(quiet_zone_modules=1)
    with pytest.raises(ValidationError):
        SymbolDefaults(ecc="h")


def test_charger_builds_v2_q_6mm_modules() -> None:
    symbol = build_symbol(spec(), SymbolDefaults())
    assert symbol.version == 2
    assert symbol.ecc == "Q"
    assert symbol.modules == 25
    assert symbol.matrix.shape == (25, 25)
    assert symbol.module_mm == pytest.approx(6.0)
    assert symbol.size_mm == 150.0
    assert symbol.quiet_zone_modules == 4
    assert symbol.payload_text == "P1/K:CHARGER/N:DOCK-1/S:150"


def test_longer_payload_picks_v3() -> None:
    symbol = build_symbol(spec(name="DOCK-NAME-1234", A="ABC"), SymbolDefaults())
    assert len(symbol.payload_text) > 29
    assert symbol.version == 3
    assert symbol.modules == 29


def test_over_capacity_raises_naming_capacity() -> None:
    long = spec(name="LONG-DOCK-NAME-12345", A="A" * 24, B="B" * 24)
    with pytest.raises(ValueError, match=r"capacity 47"):
        build_symbol(long, SymbolDefaults())


def test_ecc_m() -> None:
    symbol = build_symbol(spec(), SymbolDefaults(ecc="m"))
    assert symbol.ecc == "M"


@pytest.mark.parametrize("marker", load_generator_config(SHIPPED).markers, ids=lambda m: m.name)
def test_raster_decodes_to_payload(marker: MarkerSpec) -> None:
    symbol = build_symbol(marker, SymbolDefaults())
    image = render_raster(symbol, 8)
    assert image.dtype.name == "uint8"
    assert image.shape == ((symbol.modules + 8) * 8,) * 2
    results = zxingcpp.read_barcodes(image)
    assert [r.text for r in results] == [symbol.payload_text]


@pytest.mark.parametrize("key", sorted(ALNUM_CAPACITY))
def test_capacity_table_matches_segno(key: tuple[int, str]) -> None:
    version, ecc = key
    capacity = ALNUM_CAPACITY[key]
    segno.make_qr("A" * capacity, mode="alphanumeric", error=ecc, boost_error=False, version=version)
    with pytest.raises(segno.DataOverflowError):
        segno.make_qr("A" * (capacity + 1), mode="alphanumeric", error=ecc, boost_error=False, version=version)
