"""Ensure the bridge package is importable when running pytest from any directory."""

import sys
from pathlib import Path

# Add nodes/steamdeck_ui/ to sys.path so `import bridge.config` resolves
_UI_DIR = Path(__file__).parent.parent.parent
if str(_UI_DIR) not in sys.path:
    sys.path.insert(0, str(_UI_DIR))
