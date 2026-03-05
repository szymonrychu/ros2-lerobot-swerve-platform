# GPS RTK bridge — LC29H-BS (base) / LC29H-DA (rover). Publishes NavSatFix, RTCM3 over TCP.
# Config via GPS_RTK_CONFIG or /etc/ros2/gps_rtk/config.yaml.
# Pass serial device e.g. --device=/dev/ttyS0:/dev/ttyS0 when running.
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

COPY gps_rtk ./gps_rtk

CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && exec python3 -m gps_rtk"]
