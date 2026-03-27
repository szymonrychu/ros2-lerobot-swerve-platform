# Stage 1 — build React frontend
FROM node:22-slim AS frontend-builder
WORKDIR /app
COPY frontend/package.json frontend/package-lock.json* ./
RUN npm ci
COPY frontend/ ./
RUN npm run build

# Stage 2 — final image
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-numpy \
    python3-pip \
    python3-venv \
    libgl1 \
    libglib2.0-0 \
    ros-jazzy-rclpy \
    ros-jazzy-sensor-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-rosidl-runtime-py \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
RUN python3 -m venv /app/venv && /app/venv/bin/pip install --no-cache-dir poetry
ENV PATH="/app/venv/bin:$PATH"
RUN poetry config virtualenvs.in-project true

COPY pyproject.toml poetry.lock* ./
RUN poetry install --only main --no-interaction --no-root
ENV PATH="/app/.venv/bin:$PATH"
ENV PYTHONPATH="/app"

COPY web_ui/ ./web_ui/
COPY config/ ./config/
COPY urdf/ ./urdf/

# Embed the pre-built React app
COPY --from=frontend-builder /app/dist/ ./web_ui/static/

CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && exec python3 -m web_ui"]
