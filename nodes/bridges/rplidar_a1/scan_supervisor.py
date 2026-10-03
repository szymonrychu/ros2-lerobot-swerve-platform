"""Run the RPLidar launch file and restart it (via systemd) when /scan never starts or goes silent.

The RPLidar A1 driver sometimes wedges in its device handshake after a restart: the process keeps running but never
creates the /scan publisher. This supervisor starts `ros2 launch .../launch/rplidar_a1.launch.py` as a child process,
watches /scan, and on a missing or silent scan stream stops the child and exits with status 1 so systemd
(Restart=on-failure) starts a fresh driver.
"""

import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scan_watch import ScanWatch
from sensor_msgs.msg import LaserScan

LAUNCH_FILE = Path(__file__).resolve().parent / "launch" / "rplidar_a1.launch.py"
SCAN_TOPIC = "/scan"
STARTUP_GRACE_S = 30.0
SILENCE_TIMEOUT_S = 5.0
CHILD_STOP_TIMEOUT_S = 10.0
EXIT_RESTART = 1


def stop_child(child: subprocess.Popen) -> None:
    """Stop the launch process: SIGINT (clean launch shutdown), then SIGKILL after a timeout.

    Args:
        child: The `ros2 launch` process.
    """
    if child.poll() is not None:
        return
    child.send_signal(signal.SIGINT)
    try:
        child.wait(timeout=CHILD_STOP_TIMEOUT_S)
    except subprocess.TimeoutExpired:
        child.kill()
        child.wait()


def main() -> int:
    """Supervise the lidar driver.

    Returns:
        int: Exit status: the child's status if it exited by itself, EXIT_RESTART when the watchdog fired, 0 on SIGTERM.
    """
    child = subprocess.Popen(["ros2", "launch", str(LAUNCH_FILE)])
    watch = ScanWatch(time.monotonic(), STARTUP_GRACE_S, SILENCE_TIMEOUT_S)
    rclpy.init()
    node = Node("rplidar_scan_supervisor")
    node.create_subscription(
        LaserScan, SCAN_TOPIC, lambda _msg: watch.on_scan(time.monotonic()), qos_profile_sensor_data
    )
    stopping = {"requested": False}
    signal.signal(signal.SIGTERM, lambda *_: stopping.__setitem__("requested", True))
    status = 0
    try:
        while not stopping["requested"]:
            rclpy.spin_once(node, timeout_sec=0.2)
            if child.poll() is not None:
                node.get_logger().error(f"rplidar launch exited with {child.returncode}")
                status = child.returncode or EXIT_RESTART
                break
            if watch.should_restart(time.monotonic()):
                node.get_logger().error(
                    f"no {SCAN_TOPIC} within {STARTUP_GRACE_S:.0f} s of start or silent for {SILENCE_TIMEOUT_S:.0f} s;"
                    " restarting the driver"
                )
                status = EXIT_RESTART
                break
    finally:
        stop_child(child)
        node.destroy_node()
        rclpy.try_shutdown()
    return status


if __name__ == "__main__":
    sys.exit(main())
