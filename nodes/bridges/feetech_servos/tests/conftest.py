"""Pytest hooks and fixtures for feetech_servos tests.

Mocks the st3215 module so script tests (calibrate_servos, set_servo_id) can be
imported and run without the hardware library installed. Individual tests may
patch ST3215 or other symbols as needed.
"""

import sys
from unittest.mock import MagicMock

# Inject mock st3215 before any test imports calibrate_servos or set_servo_id
if "st3215" not in sys.modules:
    _st3215_mock = MagicMock()
    _st3215_mock.ST3215 = MagicMock()  # class used by scripts
    sys.modules["st3215"] = _st3215_mock
