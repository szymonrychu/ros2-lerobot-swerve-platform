import { Suspense, useMemo } from 'react'
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

  const stableIntrinsics = useMemo(
    () =>
      cameraInfo
        ? {
            fx: cameraInfo.fx,
            fy: cameraInfo.fy,
            cx: cameraInfo.cx,
            cy: cameraInfo.cy,
            width: cameraInfo.width,
            height: cameraInfo.height,
          }
        : null,
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [cameraInfo?.fx, cameraInfo?.fy, cameraInfo?.cx, cameraInfo?.cy, cameraInfo?.width, cameraInfo?.height]
  )

  const hasMesh =
    depthData?.depth_b64 != null &&
    colorData?.color_small_b64 != null &&
    stableIntrinsics != null

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
                intrinsics={stableIntrinsics!}
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
