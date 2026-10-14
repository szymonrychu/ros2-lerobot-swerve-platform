import Box from '@mui/material/Box'
import Stack from '@mui/material/Stack'
import { CANVAS_BG } from '../theme'
import { TabConfig } from '../types'
import { WaitingMessage } from './WaitingMessage'

interface ColorData {
  jpeg_b64: string | null
}

interface DepthData {
  depth_preview_b64: string | null
  depth_b64: string | null
  depth_width: number
  depth_height: number
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
  return (
    <Box sx={{ width: '100%', height: '100%', display: 'flex', flexDirection: 'column', bgcolor: CANVAS_BG }}>
      {/* Top: RGB + depth previews, side by side (stacked on phones) */}
      <Stack
        direction={{ xs: 'column', sm: 'row' }}
        divider={<Box sx={{ flexShrink: 0, bgcolor: 'divider', width: { xs: '100%', sm: '1px' }, height: { xs: '1px', sm: 'auto' } }} />}
        sx={{ height: { xs: '40dvh', sm: '25dvh' }, minHeight: 'clamp(7.5rem, 20dvh, 15rem)', flexShrink: 0, borderBottom: 1, borderColor: 'divider' }}
      >
        <Box sx={PREVIEW_SX}>
          {colorData?.jpeg_b64 ? (
            <img
              src={`data:image/jpeg;base64,${colorData.jpeg_b64}`}
              alt="RGB"
              style={{ maxWidth: '100%', maxHeight: '100%', objectFit: 'contain' }}
            />
          ) : (
            <WaitingMessage>Waiting for {tab.color_topic}...</WaitingMessage>
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
            <WaitingMessage>Waiting for {tab.depth_topic}...</WaitingMessage>
          )}
        </Box>
      </Stack>
    </Box>
  )
}
