# Reusable UVC camera bridge — one container per camera instance
# Env: UVC_DEVICE, UVC_TOPIC. Device mapping via compose device:.
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-venv \
    ros-jazzy-rclpy \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
RUN python3 -m venv /app/venv && /app/venv/bin/pip install --no-cache-dir poetry
ENV PATH="/app/venv/bin:$PATH"
RUN poetry config virtualenvs.create false

COPY pyproject.toml poetry.lock ./
RUN poetry install --no-dev --no-interaction

COPY . ./uvc_camera/

ENV PYTHONPATH=/app
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && exec python3 -m uvc_camera"]
