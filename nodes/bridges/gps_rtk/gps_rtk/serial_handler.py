"""Serial handler: mixed NMEA + RTCM3 + binary stream over native UART."""

import logging
import threading
import time
from typing import Callable

import serial

from .rtcm3 import is_valid_rtcm3_frame, parse_rtcm3_length

LOG = logging.getLogger(__name__)

NMEA_PREAMBLE = ord("$")
RTCM3_PREAMBLE = 0xD3
MAX_NMEA_LEN = 256
MAX_RTCM3_PAYLOAD = 1023

SERIAL_OPEN_MAX_RETRIES = 5
SERIAL_OPEN_RETRY_DELAY_S = 2.0
CONFIGURE_MAX_RETRIES = 3


def nmea_checksum(sentence: str) -> str:
    """Compute NMEA XOR checksum for sentence after $ and before *.

    Args:
        sentence: e.g. "GNGGA,..." (no leading $).

    Returns:
        Two-char hex string, e.g. "1A".
    """
    checksum = 0
    for c in sentence:
        checksum ^= ord(c)
    return f"{checksum:02X}"


def append_checksum_if_missing(cmd: str) -> str:
    """Append *XX checksum if not present.

    Args:
        cmd: NMEA command with or without *XX.

    Returns:
        Command with checksum.
    """
    cmd = cmd.strip()
    if "*" in cmd:
        return cmd
    if cmd.startswith("$"):
        cmd = cmd[1:]
    return f"${cmd}*{nmea_checksum(cmd)}"


class SerialStreamParser:
    """Byte-level state machine: split NMEA lines and RTCM3 frames from mixed serial stream."""

    def __init__(
        self,
        on_nmea: Callable[[str], None],
        on_rtcm3: Callable[[bytes], None],
        log_unknown: bool = False,
    ) -> None:
        self.on_nmea = on_nmea
        self.on_rtcm3 = on_rtcm3
        self.log_unknown = log_unknown
        self._nmea_buf: list[int] = []
        self._rtcm3_buf: list[int] = []
        self._state: str = "search"  # search, nmea, rtcm3
        self._rtcm3_payload_len: int | None = None
        self._unknown_count = 0

    def feed(self, data: bytes) -> None:
        """Feed raw bytes; call on_nmea/on_rtcm3 as complete messages are found."""
        for byte in data:
            self._feed_byte(byte)

    def _feed_byte(self, byte: int) -> None:
        if self._state == "rtcm3" and self._rtcm3_payload_len is not None:
            self._rtcm3_buf.append(byte)
            need = 1 + 2 + self._rtcm3_payload_len + 3
            if len(self._rtcm3_buf) >= need:
                frame = bytes(self._rtcm3_buf[:need])
                self._rtcm3_buf = self._rtcm3_buf[need:]
                self._state = "search"
                self._rtcm3_payload_len = None
                if is_valid_rtcm3_frame(frame):
                    self.on_rtcm3(frame)
                elif self.log_unknown:
                    LOG.debug("RTCM3 frame CRC/length invalid, dropped")
            return

        if self._state == "rtcm3":
            self._rtcm3_buf.append(byte)
            if len(self._rtcm3_buf) >= 3:
                length = parse_rtcm3_length(bytes(self._rtcm3_buf[1:3]))
                if length is not None and length <= MAX_RTCM3_PAYLOAD:
                    self._rtcm3_payload_len = length
                else:
                    self._rtcm3_buf.clear()
                    self._state = "search"
            return

        if byte == RTCM3_PREAMBLE:
            self._flush_nmea_invalid()
            self._rtcm3_buf = [byte]
            self._state = "rtcm3"
            self._rtcm3_payload_len = None
            return

        if byte == NMEA_PREAMBLE:
            self._flush_nmea_invalid()
            self._nmea_buf = [byte]
            self._state = "nmea"
            return

        if self._state == "nmea":
            if byte in (ord("\r"), ord("\n")):
                if self._nmea_buf:
                    line = bytes(self._nmea_buf).decode("ascii", errors="replace").strip()
                    self._nmea_buf = []
                    self._state = "search"
                    if line.startswith("$") and self._validate_nmea_checksum(line):
                        self.on_nmea(line)
            else:
                self._nmea_buf.append(byte)
                if len(self._nmea_buf) > MAX_NMEA_LEN:
                    self._nmea_buf.clear()
                    self._state = "search"
            return

        self._unknown_count += 1
        if self.log_unknown and self._unknown_count <= 10:
            LOG.debug("Discarding unknown byte 0x%02x (count=%d)", byte, self._unknown_count)

    def _flush_nmea_invalid(self) -> None:
        if self._nmea_buf:
            self._nmea_buf.clear()
        self._state = "search"

    @staticmethod
    def _validate_nmea_checksum(line: str) -> bool:
        if "*" not in line or len(line) < 10:
            return False
        try:
            idx = line.index("*")
            body = line[1:idx]
            expected = line[idx + 1 : idx + 3].upper()  # noqa: E203 (black adds space)
            computed = nmea_checksum(body)
            return computed == expected
        except (ValueError, IndexError):
            return False


# Base station configure commands (LC29H-BS): enable MSM7, ref 1005, ephemeris, NMEA sentences.
# The LC29H-BS outputs a mixed stream (NMEA + RTCM3 + proprietary binary). The SerialStreamParser
# handles the mixed stream correctly; these commands ensure the required RTCM message types and
# NMEA sentences are enabled. $PQTMSAVEPAR persists settings across power cycles.
LC29HBS_CONFIGURE_COMMANDS = [
    "$PAIR432,1",  # Enable RTCM MSM7 observation messages
    "$PAIR434,1",  # Enable RTCM 1005 reference station ARP
    "$PAIR436,1",  # Enable RTCM ephemeris messages
    "$PAIR062,0,1",  # Enable GGA
    "$PAIR062,1,1",  # Enable GLL
    "$PAIR062,2,1",  # Enable GSA
    "$PAIR062,3,1",  # Enable GSV
    "$PAIR062,4,1",  # Enable RMC
    "$PAIR062,5,1",  # Enable VTG
    "$PAIR062,6,1",  # Enable ZDA
    "$PAIR062,7,1",
    "$PAIR062,8,1",
]


class SerialHandler:
    """Open serial port, run parser in background thread, send NMEA commands."""

    def __init__(
        self,
        port: str,
        baud_rate: int,
        on_nmea: Callable[[str], None],
        on_rtcm3: Callable[[bytes], None],
        log_unknown_bytes: bool = False,
    ) -> None:
        self.port = port
        self.baud_rate = baud_rate
        self.on_nmea = on_nmea
        self.on_rtcm3 = on_rtcm3
        self.log_unknown_bytes = log_unknown_bytes
        self._ser: serial.Serial | None = None
        self._parser: SerialStreamParser | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()

    def open(self) -> None:
        """Open serial port with retry and start read thread.

        Retries up to SERIAL_OPEN_MAX_RETRIES times with SERIAL_OPEN_RETRY_DELAY_S
        backoff if the port fails to open (e.g. GNSS module not ready on cold boot).

        Raises:
            serial.SerialException: If all retries are exhausted.
        """
        last_exc: Exception | None = None
        for attempt in range(1, SERIAL_OPEN_MAX_RETRIES + 1):
            try:
                self._ser = serial.Serial(
                    port=self.port,
                    baudrate=self.baud_rate,
                    timeout=0.1,
                    write_timeout=2.0,
                )
                break
            except (OSError, serial.SerialException) as e:
                last_exc = e
                LOG.warning("Serial open failed (attempt %d/%d): %s", attempt, SERIAL_OPEN_MAX_RETRIES, e)
                if attempt < SERIAL_OPEN_MAX_RETRIES:
                    self._stop.wait(SERIAL_OPEN_RETRY_DELAY_S)
        else:
            raise serial.SerialException(
                f"Failed to open {self.port} after {SERIAL_OPEN_MAX_RETRIES} attempts: {last_exc}"
            )

        self._parser = SerialStreamParser(
            on_nmea=self.on_nmea,
            on_rtcm3=self.on_rtcm3,
            log_unknown=self.log_unknown_bytes,
        )
        self._stop.clear()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def close(self) -> None:
        """Stop read thread and close port."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._ser is not None and self._ser.is_open:
            self._ser.close()
            self._ser = None

    def _read_loop(self) -> None:
        error_backoff = 0.01
        while not self._stop.is_set() and self._ser is not None and self._ser.is_open:
            try:
                waiting = self._ser.in_waiting
                if waiting > 0:
                    data = self._ser.read(min(waiting, 4096))
                    if data and self._parser is not None:
                        self._parser.feed(data)
                    error_backoff = 0.01
                else:
                    self._stop.wait(0.02)
            except (OSError, serial.SerialException) as e:
                if error_backoff < 0.5:
                    LOG.warning("Serial read error: %s", e)
                error_backoff = min(error_backoff * 2, 2.0)
                self._stop.wait(error_backoff)

    def send_nmea(self, cmd: str) -> None:
        """Send NMEA command with checksum and \\r\\n."""
        if self._ser is None or not self._ser.is_open:
            raise RuntimeError("Serial port not open")
        cmd = append_checksum_if_missing(cmd)
        self._ser.write((cmd + "\r\n").encode("ascii"))

    def write(self, data: bytes) -> None:
        """Write raw bytes (e.g. RTCM3 to rover's serial)."""
        if self._ser is None or not self._ser.is_open:
            raise RuntimeError("Serial port not open")
        self._ser.write(data)

    def wait_for_serial_ready(self, timeout_s: float = 30.0) -> bool:
        """Block until the serial port starts producing data, or timeout.

        After a cold power-on the GNSS module needs a few seconds before the
        UART becomes active. This avoids sending configure commands into the
        void and accumulating serial I/O errors.

        Args:
            timeout_s: Maximum seconds to wait.

        Returns:
            True if data was detected, False on timeout.
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._ser is None or not self._ser.is_open:
                return False
            try:
                if self._ser.in_waiting > 0:
                    return True
            except (OSError, serial.SerialException):
                pass
            self._stop.wait(0.5)
        return False

    def send_base_configure(self) -> None:
        """Send LC29H-BS configure commands (MSM7, ref 1005, ephemeris, NMEA) and save to flash.

        Waits for the module to be ready, then sends all PAIR/NMEA enable commands and
        persists with $PQTMSAVEPAR. Retries up to CONFIGURE_MAX_RETRIES times with
        exponential backoff if serial write errors occur (e.g. module not ready yet on
        cold boot). Does not raise — logs error and returns if all retries fail so the
        node can still relay RTCM even without fresh configure.
        """
        for attempt in range(1, CONFIGURE_MAX_RETRIES + 1):
            LOG.info(
                "Waiting for GNSS module serial to become ready... (attempt %d/%d)",
                attempt,
                CONFIGURE_MAX_RETRIES,
            )
            if not self.wait_for_serial_ready(timeout_s=30.0):
                LOG.warning("Serial not ready after 30 s (attempt %d/%d)", attempt, CONFIGURE_MAX_RETRIES)
            try:
                for cmd in LC29HBS_CONFIGURE_COMMANDS:
                    self.send_nmea(cmd)
                    time.sleep(0.05)
                time.sleep(0.5)
                self.send_nmea("$PQTMSAVEPAR")
                LOG.info("Configure commands sent and saved to flash")
                return
            except (OSError, serial.SerialException) as e:
                LOG.warning(
                    "Serial write error during configure (attempt %d/%d): %s",
                    attempt,
                    CONFIGURE_MAX_RETRIES,
                    e,
                )
                if attempt < CONFIGURE_MAX_RETRIES:
                    backoff = 2**attempt
                    LOG.info("Retrying configure in %ds", backoff)
                    time.sleep(backoff)
        LOG.error(
            "Failed to send configure commands after %d attempts — node will continue without configure",
            CONFIGURE_MAX_RETRIES,
        )
