"""Prometheus metrics of the feetech bridge (default registry, created once at import).

The exporter only serves them when metrics_port is set, so importing this module is harmless on the server
(lerobot_leader) where the port stays unset.
"""

from collections.abc import Mapping, Sequence

from prometheus_client import Counter, Gauge, Histogram

from .battery import raw_to_volts
from .registers import decode_present_load

# Control loop at ~100 Hz: resolve 2 ms to 100 ms.
CYCLE_DURATION_BUCKETS = (0.002, 0.005, 0.01, 0.02, 0.05, 0.1)
# Reasons a set_register message is rejected or fails.
SET_REGISTER_FAILURE_REASONS = (
    "no_bus",
    "invalid_json",
    "missing_field",
    "unknown_register",
    "unknown_joint",
    "bad_value",
    "eprom_rejected",
    "write_failed",
)

BUS_UP = Gauge("servo_bus_up", "1 when at least one servo answered the last state read, else 0")
READ_FAILURES = Counter("servo_bus_read_failures_total", "State reads that got no answer from a servo", ["joint"])
CYCLE_DURATION = Histogram(
    "servo_cycle_duration_seconds", "Work time of one bridge loop iteration", buckets=CYCLE_DURATION_BUCKETS
)
PRESENT_LOAD = Gauge("servo_present_load", "Signed present load from the last register dump", ["joint"])
TEMPERATURE = Gauge("servo_temperature_celsius", "Servo temperature from the last register dump", ["joint"])
VOLTAGE = Gauge("servo_voltage_volts", "Servo supply voltage from the last register dump", ["joint"])
TORQUE_ENABLED = Gauge("servo_torque_enabled", "1 when torque is enabled (last register dump)", ["joint"])
SET_REGISTER_FAILURES = Counter(
    "servo_set_register_failures_total", "set_register messages that did not result in a write", ["reason"]
)
COMMANDS = Counter("servo_commands_total", "Joint command targets processed by the bridge loop")

# Counters at 0 are true values, so their children exist from the start.
for reason in SET_REGISTER_FAILURE_REASONS:
    SET_REGISTER_FAILURES.labels(reason=reason)


def init_joints(joint_names: Sequence[str]) -> None:
    """Precreate the read-failure counter of every joint so the loop never allocates children.

    Args:
        joint_names (Sequence[str]): Names of all joints on the bus.
    """
    for name in joint_names:
        READ_FAILURES.labels(joint=name)


def record_read_cycle(joints: Sequence[tuple[str, int]], readings: Mapping[int, object]) -> None:
    """Count joints that did not answer the state read and set the bus-up gauge.

    Args:
        joints (Sequence[tuple[str, int]]): (joint name, servo id) of every joint expected to answer.
        readings (Mapping[int, object]): Servo ids that answered (read_positions_and_speeds result).
    """
    answered = 0
    for name, servo_id in joints:
        if servo_id in readings:
            answered += 1
        else:
            READ_FAILURES.labels(joint=name).inc()
    BUS_UP.set(1.0 if answered else 0.0)


def record_register_dump(dump: Mapping[str, Mapping[str, int]], inverted: Mapping[str, bool]) -> None:
    """Update the per-joint gauges from a completed full register dump; unread registers stay untouched.

    Args:
        dump (Mapping[str, Mapping[str, int]]): joint name -> register name -> raw value.
        inverted (Mapping[str, bool]): joint name -> direction inverted (flips the load sign); missing = False.
    """
    for joint, registers in dump.items():
        if "present_load" in registers:
            PRESENT_LOAD.labels(joint=joint).set(
                decode_present_load(registers["present_load"], inverted.get(joint, False))
            )
        if "present_temperature" in registers:
            TEMPERATURE.labels(joint=joint).set(registers["present_temperature"])
        volts = raw_to_volts(registers.get("present_voltage"))
        if volts is not None:
            VOLTAGE.labels(joint=joint).set(volts)
        if "torque_enable" in registers:
            TORQUE_ENABLED.labels(joint=joint).set(1.0 if registers["torque_enable"] else 0.0)
