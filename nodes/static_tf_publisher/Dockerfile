# Static TF publisher: base_link -> imu_link, camera, lidar (configurable offsets).
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-venv \
    ros-jazzy-rclpy \
    ros-jazzy-tf2-ros \
    ros-jazzy-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
RUN python3 -m venv /app/venv && /app/venv/bin/pip install --no-cache-dir poetry
ENV PATH="/app/venv/bin:$PATH"
RUN poetry config virtualenvs.in-project true

COPY pyproject.toml poetry.lock* ./
RUN poetry install --only main --no-interaction --no-root
ENV PATH="/app/.venv/bin:$PATH"

COPY static_tf_publisher ./static_tf_publisher

CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && exec python3 -m static_tf_publisher"]
