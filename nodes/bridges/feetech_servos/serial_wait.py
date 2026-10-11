"""Blocking serial reads for the st3215 servo bus.

The st3215 library opens the port with ``timeout=0`` (non-blocking) and its
``rxPacket`` loop calls ``readPort`` until the reply is complete or the packet
timeout (``LATENCY_TIMER`` + transmit time, ~50 ms) expires, without sleeping.
Every bus round trip therefore spins a CPU core while the reply is in flight.

Giving the pyserial port a short read timeout makes each ``read(n)`` block in
the kernel (select) until the requested bytes arrive or the timeout passes.
A reply is returned as soon as its bytes arrive (no added latency), and the
library's own packet timeout still decides when a missing servo times out; the
overall timeout grows by at most one read timeout.
"""

from typing import Any

# Upper bound of one blocking read; also the worst-case overshoot of the library's packet timeout.
SERIAL_READ_TIMEOUT_S = 0.001


def use_blocking_reads(servo: Any, timeout_s: float = SERIAL_READ_TIMEOUT_S) -> None:
    """Make the st3215 bus reads block in the kernel instead of busy-waiting.

    Call it once after the library opened the port (e.g. after ``ST3215(device)``).
    The vendored library is not modified; only the pyserial timeout of its open port is set.

    :param servo: st3215 ``ST3215`` / ``protocol_packet_handler`` instance with a ``portHandler``
        whose ``ser`` is the open ``serial.Serial``. Without an open port it does nothing.
    :param timeout_s: read timeout in seconds (> 0).
    :return: None
    """
    port = getattr(servo, "portHandler", None)
    ser = getattr(port, "ser", None)
    if ser is not None:
        ser.timeout = timeout_s
