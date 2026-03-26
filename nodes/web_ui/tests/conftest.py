"""Shared pytest fixtures for web_ui tests."""

from __future__ import annotations

from pathlib import Path

import pytest


@pytest.fixture
def config_yaml(tmp_path: Path) -> Path:
    """Write a minimal valid config YAML and return its path."""
    content = """
bridge:
  host: localhost
  port: 9090
tabs: []
overlays: []
"""
    p = tmp_path / "config.yaml"
    p.write_text(content)
    return p


@pytest.fixture
def urdf_dir(tmp_path: Path) -> Path:
    """Return a temporary URDF directory with a minimal robot.urdf."""
    d = tmp_path / "urdf"
    d.mkdir()
    (d / "robot.urdf").write_text('<?xml version="1.0"?><robot name="test"><link name="base_link"/></robot>')
    return d
