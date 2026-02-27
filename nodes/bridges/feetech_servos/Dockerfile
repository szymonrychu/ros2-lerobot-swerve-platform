# Feetech servos bridge — one container per arm (e.g. leader or follower).
# Config via FEETECH_SERVOS_CONFIG (YAML path) or /etc/ros2/feetech_servos/config.yaml.
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
RUN poetry config virtualenvs.create false

COPY pyproject.toml poetry.lock ./
RUN poetry install --no-dev --no-interaction

COPY . ./feetech_servos/

ENV PYTHONPATH=/app
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && exec python3 -m feetech_servos"]
