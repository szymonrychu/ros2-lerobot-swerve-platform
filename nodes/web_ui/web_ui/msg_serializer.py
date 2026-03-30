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
    if len(k) < 9:
        return {"error": f"CameraInfo K matrix has {len(k)} elements, expected 9"}
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
        depth = np.frombuffer(raw, dtype=np.uint16).reshape((msg.height, msg.step // 2))[:, : msg.width]

        # --- Red-green colormap preview (full resolution) ---
        depth_f = depth.astype(np.float32)
        valid = depth > 0
        normalized = np.clip((depth_f - _DEPTH_MIN_MM) / (_DEPTH_MAX_MM - _DEPTH_MIN_MM), 0.0, 1.0)
        norm_u8 = (normalized * 255).astype(np.uint8)
        b = np.zeros_like(norm_u8)
        g = np.where(valid, norm_u8, 0).astype(np.uint8)
        r = np.where(valid, 255 - norm_u8, 0).astype(np.uint8)
        colormap_bgr = cv2.merge([b, g, r])
        success_p, buf_p = cv2.imencode(".jpg", colormap_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
        depth_preview_b64 = base64.b64encode(buf_p.tobytes()).decode("ascii") if success_p else None

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
