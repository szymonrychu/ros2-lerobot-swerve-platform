"""Configuration loading for Feetech servos bridge (namespace, joints with name+id, device)."""

import logging
import math
import os
import re
from dataclasses import dataclass, field
from pathlib import Path

import yaml

from .command_mapping import STEP_CENTER, STEPS_PER_RADIAN

LOGGER = logging.getLogger(__name__)
DEFAULT_CONFIG_PATH = Path("/etc/ros2/feetech_servos/config.yaml")
ENV_CONFIG_PATH_KEY = "FEETECH_SERVOS_CONFIG"
NODE_NAME_PATTERN = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")  # valid ROS node name

# Valid servo ID range for Feetech STS (0-253).
SERVO_ID_MIN = 0
SERVO_ID_MAX = 253

# Valid step range for position/limit config (Feetech STS 0-4095).
STEPS_MIN = 0
STEPS_MAX = 4095

JOINT_MODES = ("position", "velocity")
# ST3215 no-load speed at 12 V: 0.222 s / 60 deg -> ~4.71 rad/s.
DEFAULT_MAX_VELOCITY_RAD_S = 4.71
DEFAULT_VELOCITY_COMMAND_TIMEOUT_S = 0.3
DEFAULT_BATTERY_TOPIC = "/battery_state"
DEFAULT_BATTERY_INTERVAL_S = 1.0
DEFAULT_BATTERY_CELLS = 3
DEFAULT_BATTERY_FRAME_ID = "base_link"
# One servo is read per interval, so a full round over ~14 servos takes ~14 s at 1 Hz.
DEFAULT_BATTERY_STALE_S = 30.0
DEFAULT_TORQUE_STATE_INTERVAL_S = 1.0
DEFAULT_GOAL_REFRESH_INTERVAL_S = 1.0
DEFAULT_READ_FAILURE_LIMIT = 3
DEFAULT_READ_RETRY_INTERVAL_S = 5.0
EXECUTOR_KINDS = ("events", "single_threaded")
DEFAULT_EXECUTOR = "single_threaded"


def _parse_optional_steps(value: object) -> int | None:
    """Parse optional step value; must be int in [STEPS_MIN, STEPS_MAX]. Returns None if missing/invalid."""
    if value is None:
        return None
    try:
        v = int(value)
    except (TypeError, ValueError):
        return None
    if not (STEPS_MIN <= v <= STEPS_MAX):
        return None
    return v


@dataclass
class JointEntry:
    """Single joint: ROS name and Feetech servo ID, optional range mapping.

    Attributes:
        name: Joint name for JointState messages (str).
        id: Servo ID on the bus (int, 0-253).
        source_min_steps: Optional; leader/source range min (0-4095). None => 0.
        source_max_steps: Optional; leader/source range max (0-4095). None => 4095.
        source_inverted: Optional; if True, invert source progress before mapping to command range.
        command_min_steps: Optional; follower command range min. None => read from servo.
        command_max_steps: Optional; follower command range max. None => read from servo.
        mode: "position" (goal_position from JointState.position) or "velocity" (wheel mode,
            goal_speed from JointState.velocity in rad/s).
        inverted: If True, joint positive direction is opposite to the servo's (positions without
            source range mapping, and velocities).
        max_velocity_rad_s: Velocity command magnitude limit for velocity-mode joints, rad/s.
        max_speed_rad_s: Optional speed cap of a position joint, rad/s (None = servo default, unlimited).
        max_acceleration_rad_s2: Optional acceleration cap of a position joint, rad/s^2 (None = unlimited).
        startup_position_rad: Optional goal of a position joint (centred rad, follower joint frame) driven at every
            bridge start once torque is on, with the joint's caps. None = hold the present position.
    """

    name: str
    id: int  # noqa: A003
    source_min_steps: int | None = None
    source_max_steps: int | None = None
    source_inverted: bool = False
    command_min_steps: int | None = None
    command_max_steps: int | None = None
    mode: str = "position"
    inverted: bool = False
    max_velocity_rad_s: float = DEFAULT_MAX_VELOCITY_RAD_S
    max_speed_rad_s: float | None = None
    max_acceleration_rad_s2: float | None = None
    startup_position_rad: float | None = None


@dataclass
class JointGroup:
    """Joints published under one topic namespace.

    Attributes:
        namespace: Topic prefix (e.g. "swerve_drive" -> /swerve_drive/joint_states).
        joints: Ordered joints of this group.
        neutral_enabled: True creates /<namespace>/servo/record_neutral and save_neutral for this group.
        neutral_exclude_joints: Position joints of this group left out of the neutral record/save (e.g. gripper).
        neutral_stale_marker: Path of the stale marker JSON written after a neutral save; None = no marker.
    """

    namespace: str
    joints: list[JointEntry]
    neutral_enabled: bool = False
    neutral_exclude_joints: list[str] = field(default_factory=list)
    neutral_stale_marker: str | None = None

    @property
    def neutral_joints(self) -> list[JointEntry]:
        """Joints a neutral record/save covers: position joints minus neutral_exclude_joints, in config order."""
        excluded = set(self.neutral_exclude_joints)
        return [j for j in self.joints if j.mode == "position" and j.name not in excluded]

    @property
    def joint_names(self) -> list[str]:
        """Ordered joint names for JointState (same order as joints)."""
        return [j.name for j in self.joints]

    def joint_entry_by_name(self, name: str) -> JointEntry | None:
        """Return the JointEntry for a joint name in this group, or None if not found."""
        for j in self.joints:
            if j.name == name:
                return j
        return None


@dataclass
class BridgeConfig:
    """Bridge config: namespace, joints (name + servo id each), optional serial.

    Attributes:
        namespace: Topic prefix (e.g. "leader" -> /leader/joint_states).
        joints: List of JointEntry (name + servo id); order defines joint order in messages.
        device: Optional serial device path (e.g. /dev/ttyUSB0).
        baudrate: Optional baud rate for serial; None if not set.
        log_joint_updates: If True, print one line per update with changing joint names and values (silent by default).
        enable_torque_on_start: If True, set torque_enable=1 for all configured servos on startup.
        disable_torque_on_start: If True, set torque_enable=0 for all configured servos on startup.
        control_loop_hz: Main bridge loop frequency in Hz for state publish and command processing.
        register_publish_interval_s: Interval in seconds for full servo_registers dump (0 = disabled).
            Use >= 10 to avoid blocking the control loop and causing visible stutter; 1 Hz is not recommended.
        publish_effort_joints: Optional list of joint names for which to read present_load and publish
            in JointState.effort at control-loop rate (for haptic/force-feedback use). Empty/omit = no effort.
        publish_current_joints: Optional list of main-group joint names for which Present_Current is published in A on
            <namespace>/joint_currents. Non-empty lengthens the state sync read to 15 bytes for every servo. Empty = off.
        publish_only_on_change: If True, only publish joint_states when at least one joint position changed
            by more than publish_change_epsilon since the last publish. Useful for leader arms to avoid
            flooding the bus with identical messages and conflicting with external command sources.
        publish_change_epsilon: Minimum absolute position delta (radians) to count as a change.
            Only used when publish_only_on_change is True. Default 1e-3 (~0.06 degrees).
        extra_groups: Further namespaces served from the same serial bus (e.g. swerve drive servos
            sharing the follower arm bus). Each has its own joint_states / joint_commands topics.
        velocity_command_timeout_s: Velocity-mode joints are stopped when no command arrives for this long.
        battery_topic: Global (not namespaced) sensor_msgs/BatteryState topic for the pack voltage.
        battery_interval_s: Seconds between battery reads (one servo per read, round-robin); 0 = disabled.
        battery_cells: Number of series cells of the pack (length of BatteryState.cell_voltage).
        battery_frame_id: header.frame_id of BatteryState.
        battery_stale_s: Per-servo voltage readings older than this are ignored.
        direct_command_sources: joint_commands header.frame_id values (filter_node source tags) whose positions
            are follower joint radians; joints with a source range mapping pass them through unmapped. Empty
            (default): the mapping applies to every command.
        metrics_port: Prometheus /metrics port on 127.0.0.1; None (default) disables the exporter.
        node_name: ROS node name and metrics node label of this instance (distinct per bridge on one host);
            None (default) keeps feetech_servos_bridge / lerobot_follower.
        goal_refresh_interval_s: An unchanged goal_position target is rewritten when its last sync write is this
            old (a broadcast has no reply, so a lost packet or a servo reset is otherwise never corrected); 0 = off.
        read_failure_limit: Consecutive failed state reads before a servo is left out of the sync read.
        read_retry_interval_s: Seconds between single-read probes of a servo left out of the sync read.
        executor: rclpy executor draining the callbacks, "events" (EventsExecutor) or "single_threaded".
        torque_state_interval_s: Seconds between repeats of each group's torque_state; 0 = publish on change only.
        neutral_enabled: Neutral record/save services for the primary group (extra groups carry their own).
        neutral_exclude_joints: Primary-group position joints left out of the neutral record/save.
        neutral_stale_marker: Stale marker path written after a primary-group neutral save; None = no marker.
    """

    namespace: str
    joints: list[JointEntry]
    device: str | None = None
    baudrate: int | None = None
    log_joint_updates: bool = False
    enable_torque_on_start: bool = False
    disable_torque_on_start: bool = False
    control_loop_hz: float = 100.0
    register_publish_interval_s: float = 10.0
    publish_effort_joints: list[str] = ()
    publish_current_joints: list[str] = field(default_factory=list)
    publish_only_on_change: bool = False
    publish_change_epsilon: float = 1e-3
    extra_groups: list[JointGroup] = field(default_factory=list)
    velocity_command_timeout_s: float = DEFAULT_VELOCITY_COMMAND_TIMEOUT_S
    battery_topic: str = DEFAULT_BATTERY_TOPIC
    battery_interval_s: float = DEFAULT_BATTERY_INTERVAL_S
    battery_cells: int = DEFAULT_BATTERY_CELLS
    battery_frame_id: str = DEFAULT_BATTERY_FRAME_ID
    battery_stale_s: float = DEFAULT_BATTERY_STALE_S
    direct_command_sources: list[str] = field(default_factory=list)
    metrics_port: int | None = None
    node_name: str | None = None
    goal_refresh_interval_s: float = DEFAULT_GOAL_REFRESH_INTERVAL_S
    read_failure_limit: int = DEFAULT_READ_FAILURE_LIMIT
    read_retry_interval_s: float = DEFAULT_READ_RETRY_INTERVAL_S
    executor: str = DEFAULT_EXECUTOR
    torque_state_interval_s: float = DEFAULT_TORQUE_STATE_INTERVAL_S
    neutral_enabled: bool = False
    neutral_exclude_joints: list[str] = field(default_factory=list)
    neutral_stale_marker: str | None = None

    @property
    def groups(self) -> list[JointGroup]:
        """All joint groups: the primary namespace first, then extra_groups."""
        primary = JointGroup(
            namespace=self.namespace,
            joints=self.joints,
            neutral_enabled=self.neutral_enabled,
            neutral_exclude_joints=list(self.neutral_exclude_joints),
            neutral_stale_marker=self.neutral_stale_marker,
        )
        return [primary, *self.extra_groups]

    @property
    def all_joints(self) -> list[JointEntry]:
        """Joints of every group, in group order."""
        return [j for g in self.groups for j in g.joints]

    @property
    def joint_names(self) -> list[str]:
        """Ordered joint names for JointState (same order as joints)."""
        return [j.name for j in self.joints]

    def servo_id_for_joint_name(self, name: str) -> int | None:
        """Return servo ID for a joint name in any group, or None if not found."""
        for j in self.all_joints:
            if j.name == name:
                return j.id
        return None

    def joint_entry_by_name(self, name: str) -> JointEntry | None:
        """Return the full JointEntry for a joint name, or None if not found."""
        for j in self.joints:
            if j.name == name:
                return j
        return None


def parse_position_cap(item: dict, key: str, name: str, mode: str) -> tuple[bool, float | None]:
    """Parse an optional position-joint cap (speed or acceleration); only valid on position joints and > 0.

    Args:
        item (dict): Raw joint entry.
        key (str): Cap key in the entry.
        name (str): Joint name for the log message.
        mode (str): Joint mode.

    Returns:
        tuple[bool, float | None]: (valid, cap); cap is None when the key is absent.
    """
    if item.get(key) is None:
        return True, None
    try:
        cap = float(item[key])
    except (TypeError, ValueError):
        LOGGER.warning("joint %r: %s must be a number, got %r", name, key, item[key])
        return False, None
    if not math.isfinite(cap) or cap <= 0:
        LOGGER.warning("joint %r: %s must be > 0, got %r", name, key, item[key])
        return False, None
    if mode != "position":
        LOGGER.warning("joint %r: %s is only allowed on mode: position joints", name, key)
        return False, None
    return True, cap


def parse_startup_position(
    item: dict, name: str, mode: str, inverted: bool, source_mapped: bool, cmd_min: int | None, cmd_max: int | None
) -> tuple[bool, float | None]:
    """Parse the optional startup_position_rad of a position joint.

    Args:
        item (dict): Raw joint entry.
        name (str): Joint name for the log message.
        mode (str): Joint mode.
        inverted (bool): Joint inverted flag.
        source_mapped (bool): True when the joint has a source range mapping.
        cmd_min (int | None): command_min_steps.
        cmd_max (int | None): command_max_steps.

    Returns:
        tuple[bool, float | None]: (valid, radians); radians is None when the key is absent. Invalid when not a finite
            number, on a velocity or source-mapped joint, or when its steps fall outside 0..4095 or the command range.
    """
    key = "startup_position_rad"
    if item.get(key) is None:
        return True, None
    try:
        rad = float(item[key])
    except (TypeError, ValueError):
        LOGGER.warning("joint %r: %s must be a number, got %r", name, key, item[key])
        return False, None
    if not math.isfinite(rad):
        LOGGER.warning("joint %r: %s must be finite, got %r", name, key, item[key])
        return False, None
    if mode != "position" or source_mapped:
        LOGGER.warning("joint %r: %s is only allowed on mode: position joints without a source range", name, key)
        return False, None
    steps = round((-rad if inverted else rad) * STEPS_PER_RADIAN + STEP_CENTER)
    low = STEPS_MIN if cmd_min is None else max(STEPS_MIN, cmd_min)
    high = STEPS_MAX if cmd_max is None else min(STEPS_MAX, cmd_max)
    if not low <= steps <= high:
        LOGGER.warning("joint %r: %s=%r is step %d, outside the allowed range %d..%d", name, key, rad, steps, low, high)
        return False, None
    return True, rad


def parse_joints(raw_joints: object, seen_ids: set[int]) -> list[JointEntry] | None:
    """Parse a joint_names list of { name, id, ... } entries.

    Args:
        raw_joints: Raw YAML value of joint_names.
        seen_ids: Servo IDs already used (across groups); updated in place.

    Returns:
        list[JointEntry] | None: Parsed joints, or None if any entry is invalid, an ID is duplicated
            or a joint name repeats within the list.
    """
    if not isinstance(raw_joints, list) or not raw_joints:
        return None
    joints: list[JointEntry] = []
    for item in raw_joints:
        if not isinstance(item, dict):
            return None
        name = (item.get("name") or "").strip()
        if not name:
            return None
        raw_id = item.get("id")
        if raw_id is None:
            return None
        try:
            sid = int(raw_id)
        except (TypeError, ValueError):
            return None
        if not (SERVO_ID_MIN <= sid <= SERVO_ID_MAX):
            return None
        if sid in seen_ids:
            return None  # duplicate servo id
        seen_ids.add(sid)
        # Optional range-mapping: source (leader) and command (follower) steps; invalid => ignore, use None.
        source_min = _parse_optional_steps(item.get("source_min_steps"))
        source_max = _parse_optional_steps(item.get("source_max_steps"))
        source_inverted = bool(item.get("source_inverted", False))
        cmd_min = _parse_optional_steps(item.get("command_min_steps"))
        cmd_max = _parse_optional_steps(item.get("command_max_steps"))
        mode = str(item.get("mode", "position")).strip().lower()
        if mode not in JOINT_MODES:
            return None
        try:
            max_velocity = float(item.get("max_velocity_rad_s", DEFAULT_MAX_VELOCITY_RAD_S))
        except (TypeError, ValueError):
            return None
        if max_velocity <= 0:
            return None
        speed_ok, max_speed = parse_position_cap(item, "max_speed_rad_s", name, mode)
        accel_ok, max_accel = parse_position_cap(item, "max_acceleration_rad_s2", name, mode)
        if not (speed_ok and accel_ok):
            return None
        if source_min is not None and source_max is not None and source_min > source_max:
            source_min, source_max = None, None
        if cmd_min is not None and cmd_max is not None and cmd_min > cmd_max:
            cmd_min, cmd_max = None, None
        inverted = bool(item.get("inverted", False))
        startup_ok, startup_rad = parse_startup_position(
            item, name, mode, inverted, source_min is not None or source_max is not None, cmd_min, cmd_max
        )
        if not startup_ok:
            return None
        joints.append(
            JointEntry(
                name=name,
                id=sid,
                source_min_steps=source_min,
                source_max_steps=source_max,
                source_inverted=source_inverted,
                command_min_steps=cmd_min,
                command_max_steps=cmd_max,
                mode=mode,
                inverted=inverted,
                max_velocity_rad_s=max_velocity,
                max_speed_rad_s=max_speed,
                max_acceleration_rad_s2=max_accel,
                startup_position_rad=startup_rad,
            )
        )
    if len({j.name for j in joints}) != len(joints):
        return None
    return joints


def parse_neutral(item: dict, namespace: str, joints: list[JointEntry]) -> tuple[bool, list[str], str | None] | None:
    """Parse and validate the neutral keys of a group (top level for the primary group, or an extra_groups entry).

    Args:
        item (dict): Raw mapping holding neutral_enabled / neutral_exclude_joints / neutral_stale_marker.
        namespace (str): Group namespace, for the log message.
        joints (list[JointEntry]): The group's joints.

    Returns:
        tuple[bool, list[str], str | None] | None: (neutral_enabled, neutral_exclude_joints, neutral_stale_marker), or
            None when a value has the wrong type or an excluded name is not a position joint of the group.
    """
    enabled = item.get("neutral_enabled", False)
    if not isinstance(enabled, bool):
        LOGGER.warning("/%s: neutral_enabled must be true or false, got %r", namespace, enabled)
        return None
    raw_exclude = item.get("neutral_exclude_joints") or []
    if not isinstance(raw_exclude, list) or not all(isinstance(n, str) for n in raw_exclude):
        LOGGER.warning("/%s: neutral_exclude_joints must be a list of joint names, got %r", namespace, raw_exclude)
        return None
    position_names = {j.name for j in joints if j.mode == "position"}
    exclude = [n.strip() for n in raw_exclude]
    unknown = [n for n in exclude if n not in position_names]
    if unknown:
        LOGGER.warning("/%s: neutral_exclude_joints %s are not position joints of the group", namespace, unknown)
        return None
    marker = item.get("neutral_stale_marker")
    if marker is not None and not isinstance(marker, str):
        LOGGER.warning("/%s: neutral_stale_marker must be a path string, got %r", namespace, marker)
        return None
    return enabled, exclude, (marker.strip() or None) if marker is not None else None


def parse_extra_groups(raw_groups: object, primary_namespace: str, seen_ids: set[int]) -> list[JointGroup] | None:
    """Parse extra_groups: list of { namespace, joint_names } sharing the bus with the primary group.

    Args:
        raw_groups: Raw YAML value of extra_groups (None or list).
        primary_namespace: Namespace of the primary group (must not be reused).
        seen_ids: Servo IDs already used; updated in place.

    Returns:
        list[JointGroup] | None: Parsed groups ([] when absent), or None if invalid.
    """
    if raw_groups is None:
        return []
    if not isinstance(raw_groups, list):
        return None
    groups: list[JointGroup] = []
    namespaces = {primary_namespace}
    for item in raw_groups:
        if not isinstance(item, dict):
            return None
        namespace = str(item.get("namespace") or "").strip()
        if not namespace or "/" in namespace or namespace in namespaces:
            return None
        namespaces.add(namespace)
        joints = parse_joints(item.get("joint_names"), seen_ids)
        if joints is None:
            return None
        neutral = parse_neutral(item, namespace, joints)
        if neutral is None:
            return None
        enabled, exclude, marker = neutral
        groups.append(
            JointGroup(
                namespace=namespace,
                joints=joints,
                neutral_enabled=enabled,
                neutral_exclude_joints=exclude,
                neutral_stale_marker=marker,
            )
        )
    return groups


def load_config(path: Path | None = None) -> BridgeConfig | None:
    """Load bridge config from YAML file.

    Expects joint_names as list of { name: str, id: int } (explicit servo ID per joint).
    Does not assume servo IDs start from 1 or are sequential.

    Args:
        path: Path to YAML file. If None, uses DEFAULT_CONFIG_PATH.

    Returns:
        BridgeConfig | None: Parsed config, or None if file missing/invalid,
            namespace/joint_names missing, or any joint missing name/id or invalid id.
    """
    if path is None:
        path = DEFAULT_CONFIG_PATH
    if not path.exists():
        return None
    data = yaml.safe_load(path.read_text())
    if not data or not isinstance(data, dict):
        return None
    namespace = (data.get("namespace") or "").strip()
    raw_joints = data.get("joint_names") or []
    if not namespace or not raw_joints:
        return None
    if "/" in namespace:
        return None  # namespace is a topic segment, not a path
    if not isinstance(raw_joints, list):
        return None
    seen_ids: set[int] = set()
    joints = parse_joints(raw_joints, seen_ids)
    if joints is None:
        return None
    if not joints:
        return None
    neutral = parse_neutral(data, namespace, joints)
    if neutral is None:
        return None
    neutral_enabled, neutral_exclude_joints, neutral_stale_marker = neutral
    device = data.get("device")
    device = str(device).strip() if device else None
    baudrate = data.get("baudrate")
    if baudrate is not None:
        try:
            baudrate = int(baudrate)
        except (TypeError, ValueError):
            baudrate = None
    log_joint_updates = data.get("log_joint_updates", False)
    if not isinstance(log_joint_updates, bool):
        log_joint_updates = bool(log_joint_updates)
    enable_torque_on_start = data.get("enable_torque_on_start", False)
    if not isinstance(enable_torque_on_start, bool):
        enable_torque_on_start = bool(enable_torque_on_start)
    disable_torque_on_start = data.get("disable_torque_on_start", False)
    if not isinstance(disable_torque_on_start, bool):
        disable_torque_on_start = bool(disable_torque_on_start)
    raw_control_hz = data.get("control_loop_hz", 100.0)
    try:
        control_loop_hz = max(1.0, float(raw_control_hz))
    except (TypeError, ValueError):
        control_loop_hz = 100.0
    raw_reg_interval = data.get("register_publish_interval_s", 10.0)
    try:
        register_publish_interval_s = max(0.0, float(raw_reg_interval))
    except (TypeError, ValueError):
        register_publish_interval_s = 10.0
    raw_effort_joints = data.get("publish_effort_joints")
    if isinstance(raw_effort_joints, list):
        publish_effort_joints = [str(j).strip() for j in raw_effort_joints if j]
        # Only include names that exist in joints
        joint_name_set = {j.name for j in joints}
        publish_effort_joints = [n for n in publish_effort_joints if n in joint_name_set]
    else:
        publish_effort_joints = []
    publish_current_joints = []
    raw_current_joints = data.get("publish_current_joints")
    if isinstance(raw_current_joints, list):
        main_names = {j.name for j in joints}
        for name in (str(j).strip() for j in raw_current_joints if j):
            if name in main_names:
                publish_current_joints.append(name)
            else:
                LOGGER.warning("publish_current_joints: %r is not a main-group joint; dropped", name)
    publish_only_on_change = data.get("publish_only_on_change", False)
    if not isinstance(publish_only_on_change, bool):
        publish_only_on_change = bool(publish_only_on_change)
    raw_change_epsilon = data.get("publish_change_epsilon", 1e-3)
    try:
        publish_change_epsilon = max(0.0, float(raw_change_epsilon))
    except (TypeError, ValueError):
        publish_change_epsilon = 1e-3
    extra_groups = parse_extra_groups(data.get("extra_groups"), namespace, seen_ids)
    if extra_groups is None:
        return None
    raw_timeout = data.get("velocity_command_timeout_s", DEFAULT_VELOCITY_COMMAND_TIMEOUT_S)
    try:
        velocity_command_timeout_s = max(0.05, float(raw_timeout))
    except (TypeError, ValueError):
        velocity_command_timeout_s = DEFAULT_VELOCITY_COMMAND_TIMEOUT_S
    battery_topic = str(data.get("battery_topic") or DEFAULT_BATTERY_TOPIC).strip()
    try:
        battery_interval_s = max(0.0, float(data.get("battery_interval_s", DEFAULT_BATTERY_INTERVAL_S)))
    except (TypeError, ValueError):
        battery_interval_s = DEFAULT_BATTERY_INTERVAL_S
    try:
        battery_cells = int(data.get("battery_cells", DEFAULT_BATTERY_CELLS))
    except (TypeError, ValueError):
        battery_cells = DEFAULT_BATTERY_CELLS
    if battery_cells < 1:
        battery_cells = DEFAULT_BATTERY_CELLS
    battery_frame_id = str(data.get("battery_frame_id") or DEFAULT_BATTERY_FRAME_ID).strip()
    try:
        battery_stale_s = float(data.get("battery_stale_s", DEFAULT_BATTERY_STALE_S))
    except (TypeError, ValueError):
        battery_stale_s = DEFAULT_BATTERY_STALE_S
    if battery_stale_s <= 0:
        battery_stale_s = DEFAULT_BATTERY_STALE_S
    raw_direct_sources = data.get("direct_command_sources") or []
    if not isinstance(raw_direct_sources, list):
        return None
    direct_command_sources = [str(s).strip() for s in raw_direct_sources if str(s).strip()]
    raw_metrics_port = data.get("metrics_port")
    node_name = str(data.get("node_name") or "").strip() or None
    if node_name is not None and not NODE_NAME_PATTERN.match(node_name):
        return None
    try:
        goal_refresh_interval_s = max(0.0, float(data.get("goal_refresh_interval_s", DEFAULT_GOAL_REFRESH_INTERVAL_S)))
    except (TypeError, ValueError):
        goal_refresh_interval_s = DEFAULT_GOAL_REFRESH_INTERVAL_S
    try:
        read_failure_limit = int(data.get("read_failure_limit", DEFAULT_READ_FAILURE_LIMIT))
    except (TypeError, ValueError):
        read_failure_limit = DEFAULT_READ_FAILURE_LIMIT
    if read_failure_limit < 1:
        read_failure_limit = DEFAULT_READ_FAILURE_LIMIT
    try:
        read_retry_interval_s = float(data.get("read_retry_interval_s", DEFAULT_READ_RETRY_INTERVAL_S))
    except (TypeError, ValueError):
        read_retry_interval_s = DEFAULT_READ_RETRY_INTERVAL_S
    if read_retry_interval_s <= 0:
        read_retry_interval_s = DEFAULT_READ_RETRY_INTERVAL_S
    try:
        torque_state_interval_s = float(data.get("torque_state_interval_s", DEFAULT_TORQUE_STATE_INTERVAL_S))
    except (TypeError, ValueError):
        torque_state_interval_s = DEFAULT_TORQUE_STATE_INTERVAL_S
    if not torque_state_interval_s >= 0:  # negative or NaN
        torque_state_interval_s = DEFAULT_TORQUE_STATE_INTERVAL_S
    executor = data.get("executor", DEFAULT_EXECUTOR)
    if executor not in EXECUTOR_KINDS:
        return None
    return BridgeConfig(
        namespace=namespace,
        joints=joints,
        device=device,
        baudrate=baudrate,
        log_joint_updates=log_joint_updates,
        enable_torque_on_start=enable_torque_on_start,
        disable_torque_on_start=disable_torque_on_start,
        control_loop_hz=control_loop_hz,
        register_publish_interval_s=register_publish_interval_s,
        publish_effort_joints=publish_effort_joints,
        publish_current_joints=publish_current_joints,
        publish_only_on_change=publish_only_on_change,
        publish_change_epsilon=publish_change_epsilon,
        extra_groups=extra_groups,
        velocity_command_timeout_s=velocity_command_timeout_s,
        battery_topic=battery_topic,
        battery_interval_s=battery_interval_s,
        battery_cells=battery_cells,
        battery_frame_id=battery_frame_id,
        battery_stale_s=battery_stale_s,
        direct_command_sources=direct_command_sources,
        metrics_port=int(raw_metrics_port) if raw_metrics_port is not None else None,
        node_name=node_name,
        goal_refresh_interval_s=goal_refresh_interval_s,
        read_failure_limit=read_failure_limit,
        read_retry_interval_s=read_retry_interval_s,
        executor=executor,
        torque_state_interval_s=torque_state_interval_s,
        neutral_enabled=neutral_enabled,
        neutral_exclude_joints=neutral_exclude_joints,
        neutral_stale_marker=neutral_stale_marker,
    )


def load_config_from_env() -> BridgeConfig | None:
    """Load config from path in FEETECH_SERVOS_CONFIG env, or default path.

    Returns:
        BridgeConfig | None: Result of load_config(path).
    """
    path_str = os.environ.get(ENV_CONFIG_PATH_KEY, "").strip()
    path = Path(path_str) if path_str else DEFAULT_CONFIG_PATH
    return load_config(path)
