# RGBD Camera Tab — Design Spec

## Context

The web_ui dashboard's first tab currently shows a single gripper camera feed. We want to replace it with an RGBD camera tab that displays the RealSense D435i's color and depth streams, plus a 3D triangulated mesh reconstructed from the aligned depth+color data. This provides spatial awareness of the robot's environment directly in the browser.

## RealSense Dockerfile Change

Add `align_depth:=true` to `nodes/bridges/realsense_d435i/Dockerfile` CMD so the driver publishes depth aligned to the color frame.

**New topics of interest:**

| Topic | Type | Resolution | Purpose |
|---|---|---|---|
| `/camera/color/image_raw` | Image (rgb8) | 640x480 | RGB preview + downscaled mesh color |
| `/camera/aligned_depth_to_color/image_raw` | Image (16UC1) | 640x480 | Depth preview + downscaled mesh depth |
| `/camera/aligned_depth_to_color/camera_info` | CameraInfo | — | Intrinsics for 3D projection |

Depth alignment runs on the D435i ASIC — minimal RPi CPU impact.

## Backend Changes

### `bridge.py`

- Import `CameraInfo` from `sensor_msgs.msg`
- Add to `MSG_TYPE_MAP`: `"sensor_msgs/CameraInfo": CameraInfo`
- Add to `SENSOR_TYPES`: `CameraInfo`
- Add to `TOPIC_TYPE_HINTS`:
  - `/camera/color/image_raw` → `Image`
  - `/camera/aligned_depth_to_color/image_raw` → `Image`
  - `/camera/aligned_depth_to_color/camera_info` → `CameraInfo`
- Pass `topic` string to `msg_to_dict()` in the callback

### `msg_serializer.py`

Add `topic: str = ""` parameter to `msg_to_dict()`.

**Color image serialization** (when topic matches RGBD color):

```python
{
    "jpeg_b64": "<full 640x480 JPEG base64>",
    "color_small_b64": "<160x120 downscaled JPEG base64>"
}
```

The `color_small_b64` field is produced by `cv2.resize(..., (160, 120))` then JPEG-encoding. Added only when `topic` is in a module-level constant `RGBD_COLOR_TOPICS: frozenset[str]` defined in `msg_serializer.py` (e.g. `{"/camera/color/image_raw"}`). Regular camera topics continue returning only `jpeg_b64`.

**Depth image serialization** (16UC1 encoding):

New function `_serialize_depth_image(msg)`:

1. Parse raw bytes as `np.uint16`, reshape to `(height, width)`
2. **Colorized preview** (full 640x480):
   - Clip depth to 200–5000 mm range, normalize to 0–255
   - Red channel = `255 - normalized` (close = red), Green channel = `normalized` (far = green), Blue = 0
   - Encode as JPEG → `depth_preview_b64`
3. **Downscaled depth for mesh** (160x120):
   - `cv2.resize` with `INTER_NEAREST` to 160x120
   - Raw uint16 bytes → base64 → `depth_b64`

```python
{
    "depth_preview_b64": "<red-green colormapped 640x480 JPEG base64>",
    "depth_b64": "<160x120 raw uint16 LE bytes, base64>",
    "depth_width": 160,
    "depth_height": 120
}
```

**CameraInfo serialization**:

Detect `CameraInfo` type in `msg_to_dict`, extract from K matrix:

```python
{"fx": K[0], "fy": K[4], "cx": K[2], "cy": K[5], "width": msg.width, "height": msg.height}
```

### `config.py`

- Add `"rgbd_camera"` to valid tab types
- Add optional fields to `TabConfig`: `color_topic`, `depth_topic`, `camera_info_topic`
- Update topic collection to include these new fields

## Frontend Changes

### `types.ts`

Add to `TabConfig`:

```typescript
color_topic?: string
depth_topic?: string
camera_info_topic?: string
```

### `App.tsx`

- Lazy import `RgbdCameraTab`
- Add `case 'rgbd_camera'` to `renderTab()`
- Add `t.color_topic`, `t.depth_topic`, `t.camera_info_topic` to `allTopics` collection

### New: `RgbdCameraTab.tsx`

Layout:

```
┌────────────────┬─────────────────────────┐
│  RGB Preview   │  Depth Preview          │  ~25% height
│  (640x480 jpg) │  (red-green colormap)   │
├────────────────┴─────────────────────────┤
│                                          │
│   3D RGBD Mesh (Three.js Canvas)         │  ~75% height
│   OrbitControls for rotation/zoom        │
│                                          │
└──────────────────────────────────────────┘
```

- Top row: two `<img>` elements side by side on black background
  - Left: `data:image/jpeg;base64,${colorData.jpeg_b64}`
  - Right: `data:image/jpeg;base64,${depthData.depth_preview_b64}`
- Bottom: `<Canvas>` from @react-three/fiber containing `<DepthMesh>` + `<OrbitControls>`

Data sources from `topicData`:
- `topicData[tab.color_topic]` → `{jpeg_b64, color_small_b64}`
- `topicData[tab.depth_topic]` → `{depth_preview_b64, depth_b64, depth_width, depth_height}`
- `topicData[tab.camera_info_topic]` → `{fx, fy, cx, cy, width, height}`

### New: `DepthMesh.tsx` (Three.js component)

Props: `depthB64`, `colorSmallB64`, `intrinsics: {fx, fy, cx, cy, width, height}`

**Mesh construction algorithm:**

1. Decode `depthB64` from base64 → `Uint16Array` (19,200 values for 160x120)
2. Decode `colorSmallB64` JPEG → RGB pixels via offscreen `<canvas>` + `getImageData`
3. Scale intrinsics to 160x120: `fx_s = fx * (160 / width)`, same for fy, cx, cy
4. For each pixel (u, v) with depth d (in mm):
   - `z = d / 1000.0` (meters)
   - `x = (u - cx_s) * z / fx_s`
   - `y = -(v - cy_s) * z / fy_s` (negate Y for Three.js Y-up)
5. Build indexed `BufferGeometry`:
   - For each 2x2 quad of adjacent pixels → 2 triangles
   - **Skip** triangle if any vertex has `d == 0` (no depth reading)
   - **Skip** triangle if max depth delta between vertices > 200 mm (discontinuity)
6. Set `position` (Float32Array), `color` (Float32Array, normalized 0-1 RGB) attributes
7. Render: `<mesh><bufferGeometry /><meshBasicMaterial vertexColors /></mesh>`

**Performance:**
- Use `useRef` for geometry, update attribute arrays in-place with `needsUpdate = true`
- 160x120 grid = 19,200 vertices, up to ~38,000 triangles — well within Three.js budget
- Reuse `Float32Array` buffers across frames (allocate once, overwrite)

**Camera setup:**
- Position: `[0, 0, -0.5]` looking along +Z (into scene)
- FOV: 60
- OrbitControls for interactive rotation/zoom

## Config Changes

### `default.yaml`

Replace `gripper_camera` tab with:

```yaml
- id: rgbd_camera
  type: rgbd_camera
  label: "RGBD Cam"
  color_topic: /camera/color/image_raw
  depth_topic: /camera/aligned_depth_to_color/image_raw
  camera_info_topic: /camera/aligned_depth_to_color/camera_info
```

### Ansible `client.yml`

Update the web_ui node's config section with the same tab replacement. Add the 3 new topics to TOPIC_TYPE_HINTS if the Ansible config overrides them (check current client.yml web_ui config structure).

## Bandwidth Estimate

Per frame at 20 Hz:
- Color JPEG (full): ~30 KB
- Color JPEG (160x120): ~8 KB
- Depth preview JPEG (full): ~20 KB
- Depth raw (160x120 uint16 base64): ~51 KB

**Total: ~109 KB/frame × 20 Hz ≈ 2.2 MB/s** — acceptable on local WiFi network.

## Files to Modify

| File | Change |
|---|---|
| `nodes/bridges/realsense_d435i/Dockerfile` | Add `align_depth:=true` to CMD |
| `nodes/web_ui/web_ui/bridge.py` | CameraInfo import, type hints, pass topic to serializer |
| `nodes/web_ui/web_ui/msg_serializer.py` | Depth serialization, color downscale, CameraInfo, topic param |
| `nodes/web_ui/web_ui/config.py` | New tab type, new topic fields |
| `nodes/web_ui/config/default.yaml` | Replace first tab with rgbd_camera |
| `nodes/web_ui/frontend/src/types.ts` | New TabConfig fields |
| `nodes/web_ui/frontend/src/App.tsx` | Lazy import + renderTab case + allTopics |
| `nodes/web_ui/frontend/src/tabs/RgbdCameraTab.tsx` | **New** — tab layout component |
| `nodes/web_ui/frontend/src/components3d/DepthMesh.tsx` | **New** — Three.js mesh from depth+color |
| `ansible/group_vars/client.yml` | Update web_ui tab config |

## Verification

1. **Unit tests**: Add tests for `_serialize_depth_image()` and CameraInfo serialization in `test_msg_serializer.py`
2. **Backend smoke test**: Start web_ui with RealSense running, check `/ws` messages contain `depth_preview_b64`, `depth_b64`, `color_small_b64`, and CameraInfo fields
3. **Frontend visual test**: Open browser, verify:
   - RGB preview shows color feed
   - Depth preview shows red-green colormap (red = close, green = far)
   - 3D mesh renders with correct geometry and colors
   - OrbitControls allow rotation/zoom
   - No visible tearing or frame sync issues
4. **Deploy**: Rebuild realsense_d435i and web_ui containers, deploy via `scripts/deploy-nodes.sh`
