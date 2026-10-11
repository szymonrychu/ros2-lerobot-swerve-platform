"""Pytest hooks and fixtures for feetech_servos tests.

Mocks the st3215 module so script tests (calibrate_servos, set_servo_id) can be
imported and run without opening a serial device. Individual tests may
patch ST3215 or other symbols as needed.

The pure protocol modules (packet handler, GroupSyncRead, GroupSyncWrite) stay the real
library code: they are imported before the package entry is replaced by the mock, so
``from st3215.group_sync_write import GroupSyncWrite`` resolves to the real class and the
bus tests (tests/sts_bus.py) exercise the packets the library actually builds.
"""

import sys
from unittest.mock import MagicMock

import st3215.group_sync_read  # noqa: F401 - keep the real module in sys.modules
import st3215.group_sync_write  # noqa: F401 - keep the real module in sys.modules
import st3215.protocol_packet_handler  # noqa: F401 - keep the real module in sys.modules
import st3215.values  # noqa: F401 - keep the real module in sys.modules

# Replace the package entry (ST3215 opens a serial port) with a mock before any test imports the scripts.
_st3215_mock = MagicMock()
_st3215_mock.ST3215 = MagicMock()  # class used by scripts
sys.modules["st3215"] = _st3215_mock
