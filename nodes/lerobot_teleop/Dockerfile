# Lerobot teleoperation node: reads leader topics (from Server via master2master), commands follower
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

COPY pyproject.toml poetry.lock ./
RUN poetry install --only main --no-interaction --no-root
ENV PATH="/app/.venv/bin:$PATH"

COPY . ./lerobot_teleop/

ENV PYTHONPATH=/app
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && exec python3 -m lerobot_teleop"]
