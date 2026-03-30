# RGBD Camera Tab Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the first tab of the web_ui dashboard with an RGBD camera view showing RGB preview, depth preview (red-green colormap), and a live 3D triangulated mesh from the RealSense D435i.

**Architecture:** Two parallel tracks — backend (Track A: Python serializer + bridge + config) and frontend (Track B: React + Three.js components) — followed by config/Ansible changes and deployment. The backend serializes depth images (16UC1→colormap + downscaled raw bytes) and CameraInfo into WebSocket JSON; the frontend reconstructs a 3D mesh in the browser from the depth+color arrays.

**Tech Stack:** Python + OpenCV + NumPy (backend), React + Three.js + @react-three/fiber (frontend), RealSense D435i + ros-jazzy-realsense2-camera (sensor)

---

## Track A — Backend (Tasks 1–5)

### Task 1: RealSense Dockerfile — enable aligned depth

**Files:**
- Modify: `nodes/bridges/realsense_d435i/Dockerfile`

- [ ] **Step 1: Add `align_depth:=true` to CMD**

Replace the CMD line in `nodes/bridges/realsense_d435i/Dockerfile`:

```dockerfile
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && exec ros2 launch realsense2_camera rs_launch.py enable_accel:=true enable_gyro:=true unite_imu_method:=linear_interpolation align_depth:=true"]
```

This causes the driver to publish `/camera/aligned_depth_to_color/image_raw` (16UC1 depth aligned pixel-for-pixel to color) and `/camera/aligned_depth_to_color/camera_info`.

- [ ] **Step 2: Commit**

```bash
git add nodes/bridges/realsense_d435i/Dockerfile
git commit -m "feat(realsense): enable aligned depth to color"
```

---

### Task 2: bridge.py — add CameraInfo and RGBD topic hints

**Files:**
- Modify: `nodes/web_ui/web_ui/bridge.py`

- [ ] **Step 1: Import CameraInfo and add to MSG_TYPE_MAP, SENSOR_TYPES, TOPIC_TYPE_HINTS**

In `nodes/web_ui/web_ui/bridge.py`, apply these changes:

Change the sensor_msgs import line (currently line 16):
```python
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu, JointState, LaserScan, NavSatFix
```

Add `"sensor_msgs/CameraInfo": CameraInfo,` to `MSG_TYPE_MAP` (after line 31):
```python
MSG_TYPE_MAP: dict[str, type] = {
    "sensor_msgs/CameraInfo": CameraInfo,
    "sensor_msgs/Imu": Imu,
    "sensor_msgs/JointState": JointState,
    "sensor_msgs/NavSatFix": NavSatFix,
    "sensor_msgs/LaserScan": LaserScan,
    "sensor_msgs/Image": Image,
    "sensor_msgs/CompressedImage": CompressedImage,
    "nav_msgs/OccupancyGrid": OccupancyGrid,
    "nav_msgs/Odometry": Odometry,
    "geometry_msgs/PoseStamped": PoseStamped,
}
```

Add `CameraInfo` to `SENSOR_TYPES` (currently line 53):
```python
SENSOR_TYPES: frozenset[type] = frozenset(
    {CameraInfo, Imu, JointState, NavSatFix, LaserScan, OccupancyGrid, Odometry, Image, CompressedImage}
)
```

Add three entries to `TOPIC_TYPE_HINTS` (after line 44):
```python
TOPIC_TYPE_HINTS: dict[str, type] = {
    "/controller/imu/data": Imu,
    "/controller/follower/joint_states": JointState,
    "/controller/swerve_drive/joint_states": JointState,
    "/controller/gps/fix": NavSatFix,
    "/controller/scan": LaserScan,
    "/controller/local_costmap": OccupancyGrid,
    "/controller/odom": Odometry,
    "/controller/camera_0/image_raw": Image,
    "/controller/camera_0/image_compressed": CompressedImage,
    "/controller/goal_pose": PoseStamped,
    "/camera/color/image_raw": Image,
    "/camera/aligned_depth_to_color/image_raw": Image,
    "/camera/aligned_depth_to_color/camera_info": CameraInfo,
}
```

- [ ] **Step 2: Pass `topic` to `msg_to_dict` in the callback**

In `_make_callback` (currently lines 88–102), change the `msg_to_dict` call to pass the topic:

```python
def _make_callback(self, topic: str) -> Any:
    def callback(msg: Any) -> None:
        try:
            data = msg_to_dict(msg, topic=topic)
            envelope = {"topic": topic, "data": data}
        except Exception as exc:
            self.get_logger().warning(f"Serialize error on {topic}: {exc}")
            return
        log.debug("ros2_msg_rx", topic=topic, msg_type=type(msg).__name__)
        with self._lock:
            self._latest[topic] = envelope
            self._dirty.add(topic)
            self._topic_last_rx[topic] = time.monotonic()

    return callback
```

- [ ] **Step 3: Update conftest.py to stub CameraInfo**

In `nodes/web_ui/tests/conftest.py`, add `"CameraInfo"` to the stub list (currently line 49):

```python
for msg_class in ("Imu", "JointState", "NavSatFix", "LaserScan", "Image", "CompressedImage", "CameraInfo"):
    sensor_mod = sys.modules["sensor_msgs.msg"]
    if not hasattr(sensor_mod, msg_class):
        setattr(sensor_mod, msg_class, MagicMock())
```

- [ ] **Step 4: Run existing tests to verify nothing breaks**

```bash
cd nodes/web_ui && poetry run pytest tests/ -v
```

Expected: All existing tests PASS.

- [ ] **Step 5: Commit**

```bash
git add nodes/web_ui/web_ui/bridge.py nodes/web_ui/tests/conftest.py
git commit -m "feat(web_ui): add CameraInfo support and RGBD topic hints to bridge"
```

---

### Task 3: msg_serializer.py — depth serialization, CameraInfo, color downscale

**Files:**
- Modify: `nodes/web_ui/web_ui/msg_serializer.py`
- Modify: `nodes/web_ui/tests/test_msg_serializer.py`

- [ ] **Step 1: Write failing tests for new serializer behavior**

Add to `nodes/web_ui/tests/test_msg_serializer.py`:

```python
import base64
import numpy as np


def _make_depth_image_msg(width: int = 640, height: int = 480, fill_mm: int = 1000) -> MagicMock:
    """Return a mock 16UC1 depth Image message."""
    arr = np.full((height, width), fill_mm, dtype=np.uint16)
    msg = MagicMock()
    msg.__class__.__name__ = "Image"
    msg.encoding = "16UC1"
    msg.width = width
    msg.height = height
    msg.step = width * 2
    msg.data = arr.tobytes()
    return msg


def _make_rgb_image_msg(width: int = 640, height: int = 480) -> MagicMock:
    """Return a mock rgb8 Image message."""
    arr = np.zeros((height, width, 3), dtype=np.uint8)
    arr[:, :, 0] = 200  # red channel
    msg = MagicMock()
    msg.__class__.__name__ = "Image"
    msg.encoding = "rgb8"
    msg.width = width
    msg.height = height
    msg.step = width * 3
    msg.data = arr.tobytes()
    return msg


def _make_camera_info_msg(width: int = 640, height: int = 480) -> MagicMock:
    """Return a mock CameraInfo message."""
    msg = MagicMock()
    msg.__class__.__name__ = "CameraInfo"
    msg.width = width
    msg.height = height
    msg.k = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
    return msg


def test_serialize_depth_image_returns_expected_keys() -> None:
    """16UC1 depth image returns depth_preview_b64, depth_b64, depth_width, depth_height."""
    result = msg_to_dict(_make_depth_image_msg(), topic="/camera/aligned_depth_to_color/image_raw")
    assert "depth_preview_b64" in result
    assert "depth_b64" in result
    assert result["depth_width"] == 160
    assert result["depth_height"] == 120
    assert result["depth_preview_b64"] is not None


def test_serialize_depth_image_raw_bytes_length() -> None:
    """Downscaled depth_b64 decodes to exactly 160*120*2 bytes (uint16 LE)."""
    result = msg_to_dict(_make_depth_image_msg(), topic="/camera/aligned_depth_to_color/image_raw")
    raw = base64.b64decode(result["depth_b64"])
    assert len(raw) == 160 * 120 * 2


def test_serialize_depth_image_values_preserved() -> None:
    """Downscaled depth values match the fill value from the source image."""
    fill_mm = 1500
    result = msg_to_dict(_make_depth_image_msg(fill_mm=fill_mm), topic="/camera/aligned_depth_to_color/image_raw")
    raw = base64.b64decode(result["depth_b64"])
    arr = np.frombuffer(raw, dtype=np.uint16)
    # All non-zero values should be close to fill_mm (INTER_NEAREST preserves exact values for uniform fill)
    assert int(arr[0]) == fill_mm


def test_serialize_camera_info() -> None:
    """CameraInfo returns fx, fy, cx, cy, width, height from K matrix."""
    result = msg_to_dict(_make_camera_info_msg())
    assert result == {"fx": 600.0, "fy": 600.0, "cx": 320.0, "cy": 240.0, "width": 640, "height": 480}


def test_color_image_rgbd_topic_includes_color_small() -> None:
    """Color image on RGBD color topic includes color_small_b64."""
    result = msg_to_dict(_make_rgb_image_msg(), topic="/camera/color/image_raw")
    assert "jpeg_b64" in result
    assert "color_small_b64" in result
    assert result["color_small_b64"] is not None


def test_color_image_non_rgbd_topic_no_color_small() -> None:
    """Color image on non-RGBD topic does not include color_small_b64."""
    result = msg_to_dict(_make_rgb_image_msg(), topic="/controller/camera_0/image_raw")
    assert "jpeg_b64" in result
    assert "color_small_b64" not in result
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd nodes/web_ui && poetry run pytest tests/test_msg_serializer.py -v -k "depth or camera_info or color_small"
```

Expected: 6 FAILED tests (functions not yet implemented).

- [ ] **Step 3: Implement the changes in msg_serializer.py**

Replace the entire content of `nodes/web_ui/web_ui/msg_serializer.py` with:

```python
"""Serialize ROS2 messages to JSON-serializable dicts for the WebSocket bridge."""

from __future__ import annotations

import array
import base64
from typing import Any

import cv2
import numpy as np

# Topics that produce downscaled color alongside the full-res JPEG (for RGBD mesh coloring).
RGBD_COLOR_TOPICS: frozenset[str] = frozenset({"/camera/color/image_raw"})

# Mesh downscale target resolution.
_MESH_W = 160
_MESH_H = 120

# Depth colormap clip range in millimetres.
_DEPTH_MIN_MM = 200.0
_DEPTH_MAX_MM = 5000.0


def msg_to_dict(msg: Any, topic: str = "") -> dict[str, Any]:
    """Convert a ROS2 message object to a JSON-serializable dict.

    Handles nested messages, arrays, and Image/CameraInfo types with special encoding.
    Uses __slots__ introspection to walk the message tree without depending on
    rosidl_runtime_py (which requires PYTHONPATH from sourced ROS2 setup.bash).

    Args:
        msg: ROS2 message object (rclpy message).
        topic: ROS2 topic string the message arrived on (used to select RGBD-specific handling).

    Returns:
        dict[str, Any]: JSON-serializable representation.
    """
    msg_type = type(msg).__name__
    if msg_type == "CameraInfo":
        return _serialize_camera_info(msg)
    if msg_type in ("Image", "CompressedImage"):
        return _serialize_image(msg, topic)
    return _serialize_msg(msg)


def _serialize_msg(msg: Any) -> dict[str, Any]:
    """Recursively serialize a ROS2 message to a plain dict.

    Args:
        msg: ROS2 message with __slots__.

    Returns:
        dict[str, Any]: JSON-serializable dict.
    """
    result: dict[str, Any] = {}
    slots = getattr(msg, "__slots__", None)
    if slots is None:
        return result
    for slot in slots:
        # rclpy slot names start with underscore; strip it to get field name
        field = slot.lstrip("_")
        value = getattr(msg, field, None)
        result[field] = _serialize_value(value)
    return result


def _serialize_value(value: Any) -> Any:
    """Convert a single ROS2 field value to a JSON-serializable type.

    Args:
        value: Field value from a ROS2 message.

    Returns:
        Any: JSON-serializable representation.
    """
    if isinstance(value, array.array):
        return list(value)
    if isinstance(value, (bytes, bytearray)):
        return list(value)
    if isinstance(value, (list, tuple)):
        return [_serialize_value(v) for v in value]
    if isinstance(value, (int, float, bool, str)) or value is None:
        return value
    if hasattr(value, "__slots__"):
        return _serialize_msg(value)
    # numpy scalar / array
    try:
        return value.tolist()
    except AttributeError:
        pass
    return str(value)


def _serialize_camera_info(msg: Any) -> dict[str, Any]:
    """Serialize a CameraInfo message to intrinsics dict.

    Extracts fx, fy, cx, cy from the K (camera matrix) field.

    Args:
        msg: sensor_msgs/CameraInfo ROS2 message.

    Returns:
        dict[str, Any]: Dict with fx, fy, cx, cy, width, height.
    """
    k = list(msg.k)
    return {
        "fx": float(k[0]),
        "fy": float(k[4]),
        "cx": float(k[2]),
        "cy": float(k[5]),
        "width": int(msg.width),
        "height": int(msg.height),
    }


def _serialize_image(msg: Any, topic: str = "") -> dict[str, Any]:
    """Serialize Image or CompressedImage message to a dict with JPEG base64.

    For 16UC1 depth images: returns red-green colormap preview JPEG and downscaled
    raw uint16 bytes for 3D mesh construction.
    For color/mono Image: returns full-res JPEG. Also returns a downscaled JPEG
    (color_small_b64) when the topic is a known RGBD color topic.
    For CompressedImage: decodes then re-encodes to JPEG.

    Args:
        msg: Image or CompressedImage ROS2 message.
        topic: Topic name used to decide whether to add color_small_b64.

    Returns:
        dict[str, Any]: Dict with image data keys.
    """
    try:
        msg_type = type(msg).__name__
        if msg_type == "Image" and msg.encoding.lower() == "16uc1":
            return _serialize_depth_image(msg)

        if msg_type == "CompressedImage":
            raw = bytes(msg.data)
            arr = np.frombuffer(raw, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        else:
            encoding = msg.encoding.lower()
            raw = bytes(msg.data)
            arr = np.frombuffer(raw, dtype=np.uint8)
            bpp = _bytes_per_pixel(encoding)
            if bpp == 1:
                arr = arr.reshape((msg.height, msg.step))[:, : msg.width]
            else:
                arr = arr.reshape((msg.height, msg.step // bpp, bpp))[:, : msg.width, :]
            if encoding in ("rgb8", "rgb16"):
                img = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            elif encoding in ("bgr8", "bgr16"):
                img = arr
            elif encoding in ("mono8", "mono16"):
                img = arr
            else:
                img = arr

        if img is None:
            return {"jpeg_b64": None, "error": "decode_failed"}

        success, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not success:
            return {"jpeg_b64": None, "error": "encode_failed"}

        result: dict[str, Any] = {"jpeg_b64": base64.b64encode(buf.tobytes()).decode("ascii")}

        if topic in RGBD_COLOR_TOPICS:
            img_small = cv2.resize(img, (_MESH_W, _MESH_H))
            ok_s, buf_s = cv2.imencode(".jpg", img_small, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok_s:
                result["color_small_b64"] = base64.b64encode(buf_s.tobytes()).decode("ascii")

        return result
    except Exception as exc:  # pylint: disable=broad-except
        return {"jpeg_b64": None, "error": str(exc)}


def _serialize_depth_image(msg: Any) -> dict[str, Any]:
    """Serialize a 16UC1 depth Image to colormap preview + downscaled raw bytes.

    Produces:
    - depth_preview_b64: Full-resolution red-green colormap JPEG (close=red, far=green).
    - depth_b64: 160x120 downscaled uint16 LE raw bytes, base64-encoded.
    - depth_width, depth_height: Dimensions of depth_b64 grid (always 160, 120).

    Args:
        msg: sensor_msgs/Image with encoding '16UC1'.

    Returns:
        dict[str, Any]: Dict with depth_preview_b64, depth_b64, depth_width, depth_height.
    """
    try:
        raw = bytes(msg.data)
        depth = np.frombuffer(raw, dtype=np.uint16).reshape((msg.height, msg.width))

        # --- Red-green colormap preview (full resolution) ---
        depth_f = depth.astype(np.float32)
        normalized = np.clip(
            (depth_f - _DEPTH_MIN_MM) / (_DEPTH_MAX_MM - _DEPTH_MIN_MM), 0.0, 1.0
        )
        norm_u8 = (normalized * 255).astype(np.uint8)
        # OpenCV uses BGR: close=red → B=0, G=norm, R=255-norm (but in BGR: B=0, G=norm, R=255-norm)
        # Desired: close=red (R channel high), far=green (G channel high)
        b = np.zeros_like(norm_u8)
        g = norm_u8
        r = (255 - norm_u8)
        colormap_bgr = cv2.merge([b, g, r])
        success_p, buf_p = cv2.imencode(".jpg", colormap_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
        depth_preview_b64 = (
            base64.b64encode(buf_p.tobytes()).decode("ascii") if success_p else None
        )

        # --- Downscaled raw depth (160x120) for mesh construction ---
        depth_small = cv2.resize(depth, (_MESH_W, _MESH_H), interpolation=cv2.INTER_NEAREST)
        depth_b64 = base64.b64encode(depth_small.astype(np.uint16).tobytes()).decode("ascii")

        return {
            "depth_preview_b64": depth_preview_b64,
            "depth_b64": depth_b64,
            "depth_width": _MESH_W,
            "depth_height": _MESH_H,
        }
    except Exception as exc:  # pylint: disable=broad-except
        return {"depth_preview_b64": None, "depth_b64": None, "error": str(exc)}


def extract_field_from_dict(data: dict[str, Any], path: str) -> float | None:
    """Extract a nested numeric field from a dict using dot-notation + array indexing.

    This is the Python equivalent of the TypeScript extractField utility, used for
    testing message serialization independently of the renderer.

    Args:
        data: Dict (e.g. serialized ROS2 message).
        path: Dot-notation path, e.g. "linear_acceleration.x" or "position[0]".

    Returns:
        float | None: Extracted numeric value, or None if not found / not numeric.
    """
    current: Any = data
    parts = path.replace("[", ".[").split(".")
    for part in parts:
        if current is None:
            return None
        if part.startswith("[") and part.endswith("]"):
            try:
                idx = int(part[1:-1])
                current = current[idx]
            except (IndexError, TypeError, ValueError):
                return None
        else:
            if not isinstance(current, dict):
                return None
            current = current.get(part)
    if isinstance(current, (int, float)) and not isinstance(current, bool):
        return float(current)
    return None


def _bytes_per_pixel(encoding: str) -> int:
    """Return bytes per pixel for a ROS2 image encoding.

    Args:
        encoding: ROS2 image encoding string (e.g. 'rgb8', 'mono16').

    Returns:
        int: Bytes per pixel.
    """
    encoding = encoding.lower()
    if "16" in encoding:
        return 2
    if encoding in ("rgba8", "bgra8"):
        return 4
    if encoding in ("rgb8", "bgr8"):
        return 3
    return 1
```

- [ ] **Step 4: Run new tests to verify they pass**

```bash
cd nodes/web_ui && poetry run pytest tests/test_msg_serializer.py -v
```

Expected: All tests PASS (both old and new).

- [ ] **Step 5: Commit**

```bash
git add nodes/web_ui/web_ui/msg_serializer.py nodes/web_ui/tests/test_msg_serializer.py
git commit -m "feat(web_ui): add depth/CameraInfo serialization and RGBD color downscale"
```

---

### Task 4: config.py — add rgbd_camera tab type and topic fields

**Files:**
- Modify: `nodes/web_ui/web_ui/config.py`
- Modify: `nodes/web_ui/tests/test_config.py`

- [ ] **Step 1: Write failing tests**

Add to `nodes/web_ui/tests/test_config.py`:

```python
def test_rgbd_camera_tab_type_is_valid(tmp_path: Path) -> None:
    """rgbd_camera is a valid tab type."""
    p = tmp_path / "cfg.yaml"
    p.write_text("""
tabs:
  - id: rgbd
    type: rgbd_camera
    label: RGBD Cam
    color_topic: /camera/color/image_raw
    depth_topic: /camera/aligned_depth_to_color/image_raw
    camera_info_topic: /camera/aligned_depth_to_color/camera_info
overlays: []
""")
    cfg = load_config(p)
    assert cfg.tabs[0].type == "rgbd_camera"
    assert cfg.tabs[0].color_topic == "/camera/color/image_raw"
    assert cfg.tabs[0].depth_topic == "/camera/aligned_depth_to_color/image_raw"
    assert cfg.tabs[0].camera_info_topic == "/camera/aligned_depth_to_color/camera_info"


def test_rgbd_topics_in_all_subscribed_topics(tmp_path: Path) -> None:
    """color_topic, depth_topic, camera_info_topic are included in subscribed topics."""
    p = tmp_path / "cfg.yaml"
    p.write_text("""
tabs:
  - id: rgbd
    type: rgbd_camera
    label: RGBD Cam
    color_topic: /camera/color/image_raw
    depth_topic: /camera/aligned_depth_to_color/image_raw
    camera_info_topic: /camera/aligned_depth_to_color/camera_info
overlays: []
""")
    cfg = load_config(p)
    topics = cfg.all_subscribed_topics()
    assert "/camera/color/image_raw" in topics
    assert "/camera/aligned_depth_to_color/image_raw" in topics
    assert "/camera/aligned_depth_to_color/camera_info" in topics
```

- [ ] **Step 2: Run to verify they fail**

```bash
cd nodes/web_ui && poetry run pytest tests/test_config.py -v -k "rgbd"
```

Expected: 2 FAILED (ValueError for unknown tab type).

- [ ] **Step 3: Implement config.py changes**

In `nodes/web_ui/web_ui/config.py`:

Add `"rgbd_camera"` to the `valid` set in `check_type` (line 77–86):
```python
valid = {
    "camera",
    "sensor_graph",
    "effector_graph",
    "imu_orientation",
    "nav_local",
    "nav_gps",
    "scene3d",
    "robot_status",
    "rgbd_camera",
}
```

Add three new optional fields to `TabConfig` (after `arm_command_topic` on line 72):
```python
color_topic: str | None = None
depth_topic: str | None = None
camera_info_topic: str | None = None
```

Add the new fields to `all_subscribed_topics` (line 113):
```python
for attr in (
    "scan_topic", "costmap_topic", "odom_topic", "fix_topic", "arm_joint_topic",
    "color_topic", "depth_topic", "camera_info_topic",
):
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd nodes/web_ui && poetry run pytest tests/test_config.py -v
```

Expected: All tests PASS.

- [ ] **Step 5: Commit**

```bash
git add nodes/web_ui/web_ui/config.py nodes/web_ui/tests/test_config.py
git commit -m "feat(web_ui): add rgbd_camera tab type and color/depth/camera_info_topic fields"
```

---

### Task 5: default.yaml — replace first tab with rgbd_camera

**Files:**
- Modify: `nodes/web_ui/config/default.yaml`

- [ ] **Step 1: Replace the first tab**

In `nodes/web_ui/config/default.yaml`, replace lines 9–13:

```yaml
  - id: gripper_camera
    type: camera
    label: "Gripper Cam"
    topic: /controller/camera_0/image_raw
```

With:

```yaml
  - id: rgbd_camera
    type: rgbd_camera
    label: "RGBD Cam"
    color_topic: /camera/color/image_raw
    depth_topic: /camera/aligned_depth_to_color/image_raw
    camera_info_topic: /camera/aligned_depth_to_color/camera_info
```

- [ ] **Step 2: Run lint**

```bash
cd nodes/web_ui && poetry run poe lint
```

Expected: No errors.

- [ ] **Step 3: Commit**

```bash
git add nodes/web_ui/config/default.yaml
git commit -m "feat(web_ui): replace gripper camera tab with RGBD camera tab in default config"
```

---

## Track B — Frontend (Tasks 6–9)

> Tasks 6–9 are independent of Track A and can be implemented in parallel.

### Task 6: types.ts — add new TabConfig fields

**Files:**
- Modify: `nodes/web_ui/frontend/src/types.ts`

- [ ] **Step 1: Add three new optional fields to TabConfig**

In `nodes/web_ui/frontend/src/types.ts`, add after `arm_command_topic`:

```typescript
export interface TabConfig {
  id: string
  type: string
  label: string
  topic?: string
  topics?: TabTopicSpec[]
  window_s?: number
  max_points?: number
  scan_topic?: string
  costmap_topic?: string
  odom_topic?: string
  goal_topic?: string
  fix_topic?: string
  tile_url?: string
  default_zoom?: number
  urdf_file?: string
  arm_urdf_file?: string
  arm_joint_topic?: string
  arm_offset?: [number, number, number]
  arm_command_topic?: string
  color_topic?: string
  depth_topic?: string
  camera_info_topic?: string
}
```

- [ ] **Step 2: Commit**

```bash
git add nodes/web_ui/frontend/src/types.ts
git commit -m "feat(web_ui): add color_topic, depth_topic, camera_info_topic to TabConfig"
```

---

### Task 7: App.tsx — register RgbdCameraTab

**Files:**
- Modify: `nodes/web_ui/frontend/src/App.tsx`

- [ ] **Step 1: Add lazy import, renderTab case, and allTopics collection**

In `nodes/web_ui/frontend/src/App.tsx`:

Add lazy import after existing ones (line 15):
```typescript
const RgbdCameraTab = lazy(() => import('./tabs/RgbdCameraTab'))
```

Add case to `renderTab` switch (after `case 'robot_status'`):
```typescript
case 'rgbd_camera': return <RgbdCameraTab {...props} />
```

Extend the `allTopics` flatMap to include the three new fields (after `arm_joint_topic`):
```typescript
const allTopics = config
  ? [
      ...config.tabs.flatMap((t) => [
        t.topic,
        ...(t.topics?.map((ts) => ts.topic) ?? []),
        t.scan_topic, t.costmap_topic, t.odom_topic, t.fix_topic, t.arm_joint_topic,
        t.color_topic, t.depth_topic, t.camera_info_topic,
      ]),
      ...config.overlays.map((o) => o.topic),
    ].filter((t): t is string => Boolean(t))
  : []
```

- [ ] **Step 2: Commit**

```bash
git add nodes/web_ui/frontend/src/App.tsx
git commit -m "feat(web_ui): register RgbdCameraTab in App"
```

---

### Task 8: RgbdCameraTab.tsx — tab layout component

**Files:**
- Create: `nodes/web_ui/frontend/src/tabs/RgbdCameraTab.tsx`

- [ ] **Step 1: Create the component**

Create `nodes/web_ui/frontend/src/tabs/RgbdCameraTab.tsx`:

```tsx
import { Suspense } from 'react'
import { Canvas } from '@react-three/fiber'
import { OrbitControls } from '@react-three/drei'
import { TabConfig } from '../types'
import DepthMesh from '../components3d/DepthMesh'

interface ColorData {
  jpeg_b64: string | null
  color_small_b64?: string | null
}

interface DepthData {
  depth_preview_b64: string | null
  depth_b64: string | null
  depth_width: number
  depth_height: number
}

interface CameraInfoData {
  fx: number
  fy: number
  cx: number
  cy: number
  width: number
  height: number
}

interface Props {
  tab: TabConfig
  topicData: Record<string, unknown>
  publish: (topic: string, msgType: string, data: unknown) => void
}

export default function RgbdCameraTab({ tab, topicData }: Props) {
  const colorData = tab.color_topic
    ? (topicData[tab.color_topic] as ColorData | undefined)
    : undefined
  const depthData = tab.depth_topic
    ? (topicData[tab.depth_topic] as DepthData | undefined)
    : undefined
  const cameraInfo = tab.camera_info_topic
    ? (topicData[tab.camera_info_topic] as CameraInfoData | undefined)
    : undefined

  const hasMesh =
    depthData?.depth_b64 != null &&
    colorData?.color_small_b64 != null &&
    cameraInfo != null

  return (
    <div style={{ width: '100%', height: '100%', display: 'flex', flexDirection: 'column', background: '#000' }}>
      {/* Top row: RGB preview + Depth preview side by side */}
      <div style={{ display: 'flex', height: '25%', minHeight: 120, borderBottom: '1px solid #1a1a1a' }}>
        <div style={{
          flex: 1,
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          borderRight: '1px solid #1a1a1a',
          overflow: 'hidden',
        }}>
          {colorData?.jpeg_b64 ? (
            <img
              src={`data:image/jpeg;base64,${colorData.jpeg_b64}`}
              alt="RGB"
              style={{ maxWidth: '100%', maxHeight: '100%', objectFit: 'contain' }}
            />
          ) : (
            <span style={{ color: '#444', fontSize: 11 }}>Waiting for {tab.color_topic}…</span>
          )}
        </div>
        <div style={{
          flex: 1,
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          overflow: 'hidden',
        }}>
          {depthData?.depth_preview_b64 ? (
            <img
              src={`data:image/jpeg;base64,${depthData.depth_preview_b64}`}
              alt="Depth"
              style={{ maxWidth: '100%', maxHeight: '100%', objectFit: 'contain' }}
            />
          ) : (
            <span style={{ color: '#444', fontSize: 11 }}>Waiting for {tab.depth_topic}…</span>
          )}
        </div>
      </div>

      {/* Bottom: 3D RGBD mesh */}
      <div style={{ flex: 1 }}>
        {hasMesh ? (
          <Canvas
            camera={{ position: [0, 0, -0.5], fov: 60, near: 0.01, far: 20 }}
            style={{ background: '#0a0a0a' }}
          >
            <Suspense fallback={null}>
              <DepthMesh
                depthB64={depthData!.depth_b64!}
                depthWidth={depthData!.depth_width}
                depthHeight={depthData!.depth_height}
                colorSmallB64={colorData!.color_small_b64!}
                intrinsics={cameraInfo!}
              />
            </Suspense>
            <OrbitControls makeDefault />
          </Canvas>
        ) : (
          <div style={{ width: '100%', height: '100%', display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
            <span style={{ color: '#444', fontSize: 11 }}>Waiting for depth + color + camera info…</span>
          </div>
        )}
      </div>
    </div>
  )
}
```

- [ ] **Step 2: Build to verify no TypeScript errors**

```bash
cd nodes/web_ui/frontend && npm run build 2>&1 | head -40
```

Expected: No TypeScript errors (may fail if DepthMesh not yet created — create a stub first if needed):

Stub to unblock build (create `nodes/web_ui/frontend/src/components3d/DepthMesh.tsx` temporarily):
```tsx
export default function DepthMesh(_props: unknown) { return null }
```

- [ ] **Step 3: Commit**

```bash
git add nodes/web_ui/frontend/src/tabs/RgbdCameraTab.tsx nodes/web_ui/frontend/src/components3d/DepthMesh.tsx
git commit -m "feat(web_ui): add RgbdCameraTab layout component"
```

---

### Task 9: DepthMesh.tsx — Three.js RGBD mesh component

**Files:**
- Modify: `nodes/web_ui/frontend/src/components3d/DepthMesh.tsx` (replace stub)

- [ ] **Step 1: Implement the full DepthMesh component**

Replace `nodes/web_ui/frontend/src/components3d/DepthMesh.tsx` with:

```tsx
import { useEffect, useRef } from 'react'
import * as THREE from 'three'

interface Intrinsics {
  fx: number
  fy: number
  cx: number
  cy: number
  width: number
  height: number
}

interface Props {
  depthB64: string
  depthWidth: number
  depthHeight: number
  colorSmallB64: string
  intrinsics: Intrinsics
}

const DISCONTINUITY_MM = 200

function base64ToUint8Array(b64: string): Uint8Array {
  const binary = atob(b64)
  const out = new Uint8Array(binary.length)
  for (let i = 0; i < binary.length; i++) out[i] = binary.charCodeAt(i)
  return out
}

function decodeJpegPixels(b64: string, w: number, h: number): Promise<Uint8ClampedArray> {
  return new Promise((resolve) => {
    const img = new Image()
    img.onload = () => {
      const canvas = document.createElement('canvas')
      canvas.width = w
      canvas.height = h
      const ctx = canvas.getContext('2d')!
      ctx.drawImage(img, 0, 0, w, h)
      resolve(ctx.getImageData(0, 0, w, h).data)
    }
    img.src = `data:image/jpeg;base64,${b64}`
  })
}

export default function DepthMesh({
  depthB64, depthWidth, depthHeight, colorSmallB64, intrinsics,
}: Props) {
  const geoRef = useRef<THREE.BufferGeometry>(null!)

  useEffect(() => {
    if (!depthB64 || !colorSmallB64 || !geoRef.current) return

    const W = depthWidth
    const H = depthHeight

    // Scale intrinsics from original resolution to mesh resolution
    const sx = W / intrinsics.width
    const sy = H / intrinsics.height
    const fx = intrinsics.fx * sx
    const fy = intrinsics.fy * sy
    const cx = intrinsics.cx * sx
    const cy = intrinsics.cy * sy

    // Decode raw uint16 depth bytes
    const bytes = base64ToUint8Array(depthB64)
    const depth = new Uint16Array(bytes.buffer, bytes.byteOffset, bytes.byteLength / 2)

    decodeJpegPixels(colorSmallB64, W, H).then((rgba) => {
      const nVerts = W * H
      const positions = new Float32Array(nVerts * 3)
      const colors = new Float32Array(nVerts * 3)

      // Build vertex positions (RealSense Z-forward, Three.js Y-up)
      for (let v = 0; v < H; v++) {
        for (let u = 0; u < W; u++) {
          const i = v * W + u
          const d = depth[i]
          const z = d / 1000.0  // mm → metres
          if (d > 0) {
            positions[i * 3 + 0] = (u - cx) * z / fx      // x: right
            positions[i * 3 + 1] = -((v - cy) * z / fy)   // y: up (flip image Y)
            positions[i * 3 + 2] = z                        // z: depth (forward)
          } else {
            positions[i * 3 + 0] = 0
            positions[i * 3 + 1] = 0
            positions[i * 3 + 2] = 0
          }
          colors[i * 3 + 0] = rgba[i * 4 + 0] / 255
          colors[i * 3 + 1] = rgba[i * 4 + 1] / 255
          colors[i * 3 + 2] = rgba[i * 4 + 2] / 255
        }
      }

      // Triangulate: two triangles per 2x2 quad, skip at depth discontinuities
      const indices: number[] = []
      for (let v = 0; v < H - 1; v++) {
        for (let u = 0; u < W - 1; u++) {
          const i00 = v * W + u
          const i10 = v * W + (u + 1)
          const i01 = (v + 1) * W + u
          const i11 = (v + 1) * W + (u + 1)
          const d00 = depth[i00], d10 = depth[i10]
          const d01 = depth[i01], d11 = depth[i11]

          // Upper triangle: top-left → top-right → bottom-left
          if (
            d00 > 0 && d10 > 0 && d01 > 0 &&
            Math.max(d00, d10, d01) - Math.min(d00, d10, d01) < DISCONTINUITY_MM
          ) {
            indices.push(i00, i10, i01)
          }
          // Lower triangle: top-right → bottom-right → bottom-left
          if (
            d10 > 0 && d11 > 0 && d01 > 0 &&
            Math.max(d10, d11, d01) - Math.min(d10, d11, d01) < DISCONTINUITY_MM
          ) {
            indices.push(i10, i11, i01)
          }
        }
      }

      const geo = geoRef.current
      geo.setAttribute('position', new THREE.BufferAttribute(positions, 3))
      geo.setAttribute('color', new THREE.BufferAttribute(colors, 3))
      geo.setIndex(indices)
      geo.computeVertexNormals()
      geo.attributes.position.needsUpdate = true
      geo.attributes.color.needsUpdate = true
    })
  }, [depthB64, colorSmallB64, depthWidth, depthHeight, intrinsics])

  return (
    <mesh>
      <bufferGeometry ref={geoRef} />
      <meshBasicMaterial vertexColors side={THREE.DoubleSide} />
    </mesh>
  )
}
```

- [ ] **Step 2: Build to verify TypeScript compiles cleanly**

```bash
cd nodes/web_ui/frontend && npm run build 2>&1 | tail -20
```

Expected: Build succeeds with no TypeScript errors.

- [ ] **Step 3: Commit**

```bash
git add nodes/web_ui/frontend/src/components3d/DepthMesh.tsx
git commit -m "feat(web_ui): add DepthMesh Three.js component for RGBD point cloud mesh"
```

---

## Task 10: Ansible — update web_ui config in client.yml

**Files:**
- Modify: `ansible/group_vars/client.yml`

- [ ] **Step 1: Replace the first tab in the web_ui config section**

In `ansible/group_vars/client.yml`, find the web_ui tabs section (around line 542–546):

```yaml
        - id: gripper_camera
          type: camera
          label: "Gripper Cam"
          topic: /controller/camera_0/image_raw
```

Replace with:

```yaml
        - id: rgbd_camera
          type: rgbd_camera
          label: "RGBD Cam"
          color_topic: /camera/color/image_raw
          depth_topic: /camera/aligned_depth_to_color/image_raw
          camera_info_topic: /camera/aligned_depth_to_color/camera_info
```

- [ ] **Step 2: Run Ansible lint and syntax check**

```bash
poetry run poe lint-ansible && poetry run poe test-ansible
```

Expected: No errors.

- [ ] **Step 3: Commit**

```bash
git add ansible/group_vars/client.yml
git commit -m "feat(ansible): update web_ui config to use rgbd_camera tab"
```

---

## Task 11: Commit spec + push + deploy

- [ ] **Step 1: Commit spec document (from brainstorming phase)**

```bash
git add docs/superpowers/specs/2026-03-30-rgbd-camera-tab-design.md docs/superpowers/plans/2026-03-30-rgbd-camera-tab.md
git commit -m "docs: add RGBD camera tab design spec and implementation plan"
```

- [ ] **Step 2: Run all backend tests and lint**

```bash
cd nodes/web_ui && poetry run pytest tests/ -v && poetry run poe lint
```

Expected: All tests PASS, no lint errors.

- [ ] **Step 3: Push to main**

```bash
git push
```

- [ ] **Step 4: Deploy realsense_d435i and web_ui**

```bash
./scripts/deploy-nodes.sh client realsense_d435i web_ui
```

Wait for both containers to come up successfully.

- [ ] **Step 5: Verify in browser**

Open `http://client.ros2.lan:8080` in a browser. On the first tab ("RGBD Cam"):
- Top-left: RGB color feed from RealSense
- Top-right: Depth preview in red-green colormap (close objects red, far green)
- Bottom: 3D mesh updates live; OrbitControls allows rotation and zoom
- No console errors in browser DevTools

---

## Parallel Execution Note

**Track A** (Tasks 1–5: Dockerfile + Backend) and **Track B** (Tasks 6–9: Frontend) are independent and can be executed simultaneously in separate git worktrees. Task 10 (Ansible) and Task 11 (deploy) must run after both tracks are merged to main.
