"""Pydantic configuration for the claude_agent node, loaded from a YAML file named by CLAUDE_AGENT_CONFIG."""

from collections.abc import Mapping
from pathlib import Path

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

CONFIG_ENV_VAR = "CLAUDE_AGENT_CONFIG"
TOKEN_KEY = "MCP_SERVER_TOKEN"
DEFAULT_EFFECTOR_TOOLS = [
    "navigate_to_pose",
    "move_relative",
    "drive",
    "move_arm_joints",
    "move_arm_cartesian",
    "set_gripper",
    "arm_home",
    "arm_set_home",
    "look_around",
]
DEFAULT_UNCAPPED_TOOLS = ["stop", "acquire_control", "release_control"]
DEFAULT_SENSOR_TOOLS = [
    "get_robot_state",
    "get_camera_image",
    "get_map_summary",
    "get_arm_state",
    "get_body_state",
    "pixel_to_ground",
    "get_annotated_camera_image",
    "mark_candidate_points",
    "resolve_candidate",
    "capture_calibration_sample",
    "solve_camera_calibration",
    "clear_calibration_samples",
    "get_topdown_view",
    "remember_object",
    "list_objects",
    "forget_object",
    "list_pois",
    "add_poi",
    "update_poi",
    "delete_poi",
]
# Approximate maximum horizontal reach of the SO-101 from the shoulder_lift axis, in cm: the straight-line sum of the link
# offsets in nodes/web_ui/urdf/so101_arm.urdf (shoulder_lift -> elbow_flex 11.6 + elbow_flex -> wrist_flex 13.5 +
# wrist_flex -> wrist_roll 6.4 + wrist_roll -> gripper tip 9.8 = 41.3 cm), an upper bound with the arm fully stretched.
ARM_REACH_CM = 41.0
DEFAULT_ARM_BASE_HEIGHT_M = 0.165
DEFAULT_NAV_GOAL_XY_TOLERANCE_CM = 1.0
DEFAULT_NAV_GOAL_YAW_TOLERANCE_DEG = 2.0
DEFAULT_CAMERA_NOTE = (
    "The gripper camera is mounted at an angle on the arm and looks slightly from left to right; "
    "its images are delivered upright (rotated 180 degrees in software). "
    "The 'front' camera is an overhead camera (640x480, autofocus) looking down at the front of the robot, the arm "
    "and the floor in front of it: take it first for an overview (overview first), then judge the arm-to-object "
    "distance and the gripper position relative to the object from it before aiming with the gripper camera. "
    "It has no depth; judge distances from the image."
)
# Model turns on top of max_turn_cap that the SDK's absolute max_turns allows: the runner enforces the agent-set turn_cap
# itself, so this only backstops a runner that failed to.
TURN_MARGIN = 10
DEFAULT_ROBOT_EVENTS_TOPIC = "/robot_events"
DEFAULT_STATE_DIR = "/var/lib/claude_agent"
DEFAULT_WORKDIR = "/var/lib/claude_agent/workspace"
DEFAULT_SESSION_LOG_MAX_BYTES = 50 * 1024 * 1024


class MissingTokenError(RuntimeError):
    """The MCP token file is missing, unreadable or empty."""


class ClaudeAgentConfig(BaseModel):
    """Settings of the claude_agent node.

    Attributes:
        model: Claude model alias or id passed to the Agent SDK.
        max_ro_cap: Hard maximum of the agent-chosen read-only (sensor) call budget per instruction.
        max_rw_cap: Hard maximum of the agent-chosen read-write (effector) call budget per instruction.
        max_turn_cap: Hard maximum of the agent-chosen model-turn budget per instruction (SDK ``max_turns`` is this plus
            TURN_MARGIN, see the ``max_turns`` property).
        max_phase_ro_cap: Largest read-only (sensor) call cap the agent may give ONE phase of its plan.
        max_phase_rw_cap: Largest read-write (effector) call cap of one phase.
        max_phase_turn_cap: Largest model-turn cap of one phase. The caps of all phases together must also stay within
            the three instruction maxima above.
        effector_tools: Short names (no mcp__robot__ prefix) of the motion tools; keep in sync with mcp_server MOTION_TOOLS.
        uncapped_tools: Short names of safety/control tools that are never capped (stop never counts).
        sensor_tools: Short names of read-only tools (never capped).
        mcp_url: Robot MCP server Streamable HTTP URL.
        mcp_token_file: File holding the MCP bearer token (``MCP_SERVER_TOKEN=<token>`` or the bare token); read at each session start.
        http_host: Bind address of the API (loopback only; the web UI proxies it).
        http_port: Port of the HTTP/WebSocket API.
        history_size: Number of most recent events kept in RAM (older ones are read from the session log on demand).
        image_thumbnail_max_px: Longest edge of image thumbnails in tool_result events.
        system_prompt_extra: Optional text appended to the system prompt.
        workdir: The agent's persistent workspace (notes, NOTES.md): cwd of the Claude CLI and the only place its file
            tools may touch. Survives deploys and session resets.
        state_dir: Directory of the node's state; the session log is ``<state_dir>/session/events.jsonl``.
        session_log_max_bytes: Size at which the session log is rotated (the oldest half of the events is dropped).
        arm_reach_cm: Approximate maximum horizontal reach of the arm from the shoulder_lift axis, stated in the prompt.
        arm_base_height_m: Height of the arm base above the floor in metres, stated in the prompt.
        nav_goal_xy_tolerance_cm: Nav2 goal position precision in cm, stated in the prompt (keep equal to the Nav2 goal checker).
        nav_goal_yaw_tolerance_deg: Nav2 goal heading precision in degrees, stated in the prompt.
        camera_note: Description of the gripper camera mounting and image orientation, stated in the prompt.
        instruction_timeout_s: Watchdog per instruction; on expiry the model is interrupted, the robot stopped and the turn ends as "timeout".
        connect_timeout_s: Bound for starting the Claude session (SDK connect/initialize).
        stop_timeout_s: Bound for the robot ``stop`` call and for the model interrupt, each.
        robot_events_topic: ROS2 topic (std_msgs/String JSON) of the mcp_server event monitor.
        robot_events_history: Number of most recent robot events kept in memory.
        robot_event_debounce_s: A critical event does not interrupt the model again within this many seconds of the last interrupt.
    """

    model_config = ConfigDict(extra="forbid")

    model: str = "opus"
    max_ro_cap: int = Field(default=300, ge=1)
    max_rw_cap: int = Field(default=100, ge=1)
    max_turn_cap: int = Field(default=150, ge=1)
    max_phase_ro_cap: int = Field(default=60, ge=1)
    max_phase_rw_cap: int = Field(default=40, ge=1)
    max_phase_turn_cap: int = Field(default=40, ge=1)
    effector_tools: list[str] = Field(default_factory=lambda: list(DEFAULT_EFFECTOR_TOOLS))
    uncapped_tools: list[str] = Field(default_factory=lambda: list(DEFAULT_UNCAPPED_TOOLS))
    sensor_tools: list[str] = Field(default_factory=lambda: list(DEFAULT_SENSOR_TOOLS))
    mcp_url: str = "http://127.0.0.1:18200/mcp"
    mcp_token_file: str = "/etc/ros2/mcp_server/token"
    http_host: str = "127.0.0.1"
    http_port: int = Field(default=18300, ge=1, le=65535)
    history_size: int = Field(default=500, ge=1)
    image_thumbnail_max_px: int = Field(default=480, ge=16)
    system_prompt_extra: str = ""
    workdir: str = DEFAULT_WORKDIR
    state_dir: str = DEFAULT_STATE_DIR
    session_log_max_bytes: int = Field(default=DEFAULT_SESSION_LOG_MAX_BYTES, ge=1024)
    arm_reach_cm: float = Field(default=ARM_REACH_CM, gt=0)
    arm_base_height_m: float = Field(default=DEFAULT_ARM_BASE_HEIGHT_M, ge=0)
    nav_goal_xy_tolerance_cm: float = Field(default=DEFAULT_NAV_GOAL_XY_TOLERANCE_CM, gt=0)
    nav_goal_yaw_tolerance_deg: float = Field(default=DEFAULT_NAV_GOAL_YAW_TOLERANCE_DEG, gt=0)
    camera_note: str = DEFAULT_CAMERA_NOTE
    instruction_timeout_s: float = Field(default=900.0, gt=0)
    connect_timeout_s: float = Field(default=240.0, gt=0)
    stop_timeout_s: float = Field(default=5.0, gt=0)
    robot_events_topic: str = DEFAULT_ROBOT_EVENTS_TOPIC
    robot_events_history: int = Field(default=50, ge=1)
    robot_event_debounce_s: float = Field(default=2.0, ge=0)

    @property
    def max_turns(self) -> int:
        """Absolute SDK ``max_turns``: the hard turn-cap maximum plus a small margin.

        Returns:
            int: ``max_turn_cap + TURN_MARGIN``.
        """
        return self.max_turn_cap + TURN_MARGIN

    @model_validator(mode="after")
    def check_tool_lists(self) -> "ClaudeAgentConfig":
        """Reject a tool listed in more than one class (stop must never be capped).

        Returns:
            ClaudeAgentConfig: The validated config.
        """
        lists = {
            "effector_tools": self.effector_tools,
            "uncapped_tools": self.uncapped_tools,
            "sensor_tools": self.sensor_tools,
        }
        names = list(lists)
        for i, first in enumerate(names):
            for second in names[i + 1 :]:
                overlap = set(lists[first]) & set(lists[second])
                if overlap:
                    raise ValueError(f"{sorted(overlap)} listed in both {first} and {second}")
        return self


def config_path_from_env(env: Mapping[str, str]) -> Path | None:
    """Read the config file path from the environment.

    Args:
        env (Mapping[str, str]): Environment mapping.

    Returns:
        Path | None: Path named by CLAUDE_AGENT_CONFIG, or None when unset or empty.
    """
    value = env.get(CONFIG_ENV_VAR, "")
    return Path(value) if value else None


def load_config(path: Path | None) -> ClaudeAgentConfig:
    """Load and validate the YAML config.

    Args:
        path (Path | None): YAML file, or None for all defaults.

    Returns:
        ClaudeAgentConfig: Validated config (an empty file means defaults).
    """
    if path is None:
        return ClaudeAgentConfig()
    data = yaml.safe_load(path.read_text()) or {}
    return ClaudeAgentConfig.model_validate(data)


def read_mcp_token(path: Path | str) -> str:
    """Read the MCP bearer token; the value is never logged.

    Args:
        path (Path | str): Token file, ``MCP_SERVER_TOKEN=<token>`` line or the bare token.

    Returns:
        str: The token.

    Raises:
        MissingTokenError: When the file cannot be read or holds no token (message names the path only).
    """
    try:
        raw = Path(path).read_text().strip()
    except OSError as exc:
        raise MissingTokenError(f"cannot read MCP token file {path}: {exc.strerror}") from exc
    token = raw.split("=", 1)[1].strip() if raw.startswith(f"{TOKEN_KEY}=") else raw
    if not token:
        raise MissingTokenError(f"MCP token file {path} is empty")
    return token
