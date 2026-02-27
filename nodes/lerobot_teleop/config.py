"""Environment-based configuration for teleop node (no ROS deps)."""

import os

DEFAULT_LEADER_JOINT_STATES_TOPIC = "/leader/joint_states"
DEFAULT_FOLLOWER_JOINT_COMMANDS_TOPIC = "/follower/joint_commands"
ENV_LEADER_TOPIC_KEY = "TELEOP_LEADER_JOINT_STATES_TOPIC"
ENV_FOLLOWER_TOPIC_KEY = "TELEOP_FOLLOWER_JOINT_COMMANDS_TOPIC"


def get_config() -> tuple[str, str]:
    """Read leader and follower topics from environment.

    Returns:
        tuple[str, str]: (leader joint_states topic, follower joint_commands topic).
    """
    leader = os.environ.get(ENV_LEADER_TOPIC_KEY, DEFAULT_LEADER_JOINT_STATES_TOPIC)
    follower = os.environ.get(ENV_FOLLOWER_TOPIC_KEY, DEFAULT_FOLLOWER_JOINT_COMMANDS_TOPIC)
    return (
        (leader or "").strip() or DEFAULT_LEADER_JOINT_STATES_TOPIC,
        (follower or "").strip() or DEFAULT_FOLLOWER_JOINT_COMMANDS_TOPIC,
    )
