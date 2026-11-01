"""The packaged robot body (arm_collision/robot_body.yaml): the one body definition every node loads."""

from arm_collision.settings import BodySettings, footprint_body, load_robot_body

EXPECTED_BOXES = [
    ("frame_left", (0.0, 0.1333, 0.15), (0.425, 0.035, 0.06)),
    ("frame_right", (0.0, -0.1333, 0.15), (0.425, 0.035, 0.06)),
    ("crossbar", (-0.025, 0.0, 0.08), (0.25, 0.2316, 0.04)),
    ("head_arm", (0.12, 0.10, 0.30), (0.08, 0.024, 0.024)),
]
EXPECTED_CYLINDERS = [
    ("wheel_fl", 0.1525, 0.1333, 0.0625, 0.0, 0.15),
    ("wheel_fr", 0.1525, -0.1333, 0.0625, 0.0, 0.15),
    ("wheel_rl", -0.1525, 0.1333, 0.0625, 0.0, 0.15),
    ("wheel_rr", -0.1525, -0.1333, 0.0625, 0.0, 0.15),
    ("lidar", 0.150, 0.040, 0.037, 0.10, 0.21),
    ("head_post", 0.09, 0.10, 0.012, 0.0, 0.30),
    ("head", 0.15, 0.10, 0.075, 0.287, 0.396),
]


def test_load_robot_body_returns_the_deployed_boxes_and_cylinders() -> None:
    """load_robot_body() returns the deployed body (client.yml 2026-11-01) with the wheels 3 cm above the tyres and the head, in
    order."""
    body = load_robot_body()
    assert isinstance(body, BodySettings)
    assert [(b.name, b.center, b.size) for b in body.boxes] == EXPECTED_BOXES
    assert [(c.name, c.x, c.y, c.radius, c.z_min, c.z_max) for c in body.cylinders] == EXPECTED_CYLINDERS


def test_footprint_body_is_the_floor_level_body_without_the_head() -> None:
    """footprint_body() drops the head post, head arm and head envelope: the nav footprint keeps the floor-level H."""
    body = footprint_body(load_robot_body())
    assert [b.name for b in body.boxes] == ["frame_left", "frame_right", "crossbar"]
    assert [c.name for c in body.cylinders] == ["wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr", "lidar"]
