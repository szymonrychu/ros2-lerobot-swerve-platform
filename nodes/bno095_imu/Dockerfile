# BNO095 IMU bridge — publishes sensor_msgs/Imu for Nav2.
# Config via BNO095_IMU_CONFIG or /etc/ros2/bno095_imu/config.yaml.
# Pass I2C device e.g. --device=/dev/i2c-1:/dev/i2c-1 when running.
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-venv \
    ros-jazzy-rclpy \
    ros-jazzy-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
RUN python3 -m venv /app/venv && /app/venv/bin/pip install --no-cache-dir poetry
ENV PATH="/app/venv/bin:$PATH"
RUN poetry config virtualenvs.in-project true

COPY pyproject.toml poetry.lock* ./
RUN poetry install --only main --no-interaction --no-root
ENV PATH="/app/.venv/bin:$PATH"

COPY bno095_imu ./bno095_imu

CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && exec python3 -m bno095_imu"]
