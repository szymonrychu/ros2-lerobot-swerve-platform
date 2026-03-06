# Swerve drive controller: cmd_vel -> joint commands, odometry (FK/IK).
# Config via SWERVE_DRIVE_CONTROLLER_CONFIG or /etc/ros2/swerve_drive_controller/config.yaml.
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-numpy \
    python3-pip \
    python3-venv \
    ros-jazzy-rclpy \
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
RUN python3 -m venv /app/venv && /app/venv/bin/pip install --no-cache-dir poetry
ENV PATH="/app/venv/bin:$PATH"
RUN poetry config virtualenvs.in-project true

COPY pyproject.toml poetry.lock* ./
RUN poetry install --only main --no-interaction --no-root
ENV PATH="/app/.venv/bin:$PATH"

COPY swerve_drive_controller ./swerve_drive_controller

CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && exec python3 -m swerve_drive_controller"]
