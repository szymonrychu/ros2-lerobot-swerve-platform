"""arm_collision stays rclpy-free: importing the checker pulls in no ROS, OpenCV or node package."""

import json
import subprocess
import sys

FORBIDDEN_MODULES = ("rclpy", "cv2", "mcp_server", "ros2_common")
PROBE = (
    "import json, sys\n"
    "import arm_collision.self_collision\n"
    "print(json.dumps(sorted({name.split('.')[0] for name in sys.modules})))\n"
)


def test_self_collision_import_loads_no_ros_opencv_or_node_package() -> None:
    """A fresh interpreter importing arm_collision.self_collision has none of FORBIDDEN_MODULES in sys.modules."""
    result = subprocess.run([sys.executable, "-c", PROBE], capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stderr
    loaded = set(json.loads(result.stdout.strip().splitlines()[-1]))
    assert loaded.isdisjoint(FORBIDDEN_MODULES), sorted(loaded & set(FORBIDDEN_MODULES))
