"""The packaged robot body (arm_collision/robot_body.yaml): the one body definition every node loads."""

from arm_collision.settings import BodySettings, load_robot_body

EXPECTED_BOXES = [
    ("frame_left", (0.0, 0.1333, 0.15), (0.425, 0.035, 0.06)),
    ("frame_right", (0.0, -0.1333, 0.15), (0.425, 0.035, 0.06)),
    ("crossbar", (-0.025, 0.0, 0.08), (0.25, 0.2316, 0.04)),
]
EXPECTED_CYLINDERS = [
    ("wheel_fl", 0.1525, 0.1333, 0.0625, 0.0, 0.12),
    ("wheel_fr", 0.1525, -0.1333, 0.0625, 0.0, 0.12),
    ("wheel_rl", -0.1525, 0.1333, 0.0625, 0.0, 0.12),
    ("wheel_rr", -0.1525, -0.1333, 0.0625, 0.0, 0.12),
    ("lidar", 0.150, 0.040, 0.037, 0.10, 0.21),
]


def test_load_robot_body_returns_the_deployed_boxes_and_cylinders() -> None:
    """load_robot_body() returns exactly the body mcp_server checked against (client.yml 2026-11-01), in order."""
    body = load_robot_body()
    assert isinstance(body, BodySettings)
    assert [(b.name, b.center, b.size) for b in body.boxes] == EXPECTED_BOXES
    assert [(c.name, c.x, c.y, c.radius, c.z_min, c.z_max) for c in body.cylinders] == EXPECTED_CYLINDERS
