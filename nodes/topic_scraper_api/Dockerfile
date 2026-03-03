# topic_scraper_api: dynamic ROS2 topic scraper and HTTP API
# Config via TOPIC_SCRAPER_API_CONFIG or /etc/ros2/topic_scraper_api/config.yaml
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-numpy \
    python3-pip \
    python3-venv \
    ros-jazzy-rclpy \
    ros-jazzy-sensor-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-rosidl-runtime-py \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
RUN python3 -m venv /app/venv && /app/venv/bin/pip install --no-cache-dir poetry
ENV PATH="/app/venv/bin:$PATH"
RUN poetry config virtualenvs.in-project true

COPY pyproject.toml poetry.lock* ./
RUN poetry install --only main --no-interaction --no-root
ENV PATH="/app/.venv/bin:$PATH"

COPY topic_scraper_api ./topic_scraper_api

CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && exec python3 -m topic_scraper_api"]
