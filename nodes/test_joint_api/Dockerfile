# test_joint_api: REST API for joint updates (publishes to filter input topic)
# Config via TEST_JOINT_API_CONFIG or /etc/ros2/test_joint_api/config.yaml
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

COPY test_joint_api ./test_joint_api

CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && exec python3 -m test_joint_api"]
