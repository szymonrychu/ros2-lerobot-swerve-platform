import { Suspense, useMemo } from 'react'
import { Canvas } from '@react-three/fiber'
import { OrbitControls } from '@react-three/drei'
import Box from '@mui/material/Box'
import Stack from '@mui/material/Stack'
import { CANVAS_BG } from '../theme'
import { TabConfig } from '../types'
import { WaitingMessage } from './WaitingMessage'
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

const PREVIEW_SX = {
  flex: 1,
  minWidth: 0,
  minHeight: 0,
  display: 'flex',
  alignItems: 'center',
  justifyContent: 'center',
  overflow: 'hidden',
} as const

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
    <Box sx={{ width: '100%', height: '100%', display: 'flex', flexDirection: 'column', bgcolor: CANVAS_BG }}>
      {/* Top: RGB + depth previews, side by side (stacked on phones) */}
      <Stack
        direction={{ xs: 'column', sm: 'row' }}
        divider={<Box sx={{ flexShrink: 0, bgcolor: 'divider', width: { xs: '100%', sm: '1px' }, height: { xs: '1px', sm: 'auto' } }} />}
        sx={{ height: { xs: '40%', sm: '25%' }, minHeight: 120, flexShrink: 0, borderBottom: 1, borderColor: 'divider' }}
      >
        <Box sx={PREVIEW_SX}>
          {colorData?.jpeg_b64 ? (
            <img
              src={`data:image/jpeg;base64,${colorData.jpeg_b64}`}
              alt="RGB"
              style={{ maxWidth: '100%', maxHeight: '100%', objectFit: 'contain' }}
            />
          ) : (
            <WaitingMessage>Waiting for {tab.color_topic}…</WaitingMessage>
          )}
        </Box>
        <Box sx={PREVIEW_SX}>
          {depthData?.depth_preview_b64 ? (
            <img
              src={`data:image/jpeg;base64,${depthData.depth_preview_b64}`}
              alt="Depth"
              style={{ maxWidth: '100%', maxHeight: '100%', objectFit: 'contain' }}
            />
          ) : (
            <WaitingMessage>Waiting for {tab.depth_topic}…</WaitingMessage>
          )}
        </Box>
      </Stack>

      {/* Bottom: 3D RGBD mesh */}
      <Box sx={{ flex: 1, minHeight: 0 }}>
        {hasMesh ? (
          <Canvas
            camera={{ position: [0, 0, 0.5], fov: 60, near: 0.01, far: 20 }}
            style={{ background: CANVAS_BG }}
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
          <WaitingMessage>Waiting for depth + color + camera info…</WaitingMessage>
        )}
      </Box>
    </Box>
  )
}
