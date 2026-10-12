import { useState } from 'react'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import FormControlLabel from '@mui/material/FormControlLabel'
import Slider from '@mui/material/Slider'
import Stack from '@mui/material/Stack'
import Switch from '@mui/material/Switch'
import Typography from '@mui/material/Typography'
import { CameraStream } from '../components/CameraStream'
import { useLayout } from '../layout/useLayout'
import { CANVAS_BG } from '../theme'
import { TabConfig } from '../types'
import { WaitingMessage } from './WaitingMessage'

interface Props {
  tab: TabConfig
  topicData: Record<string, unknown>
  publish: (topic: string, msgType: string, data: unknown) => void
}

const CELL_SX = { flex: '1 1 0', minWidth: 0, minHeight: 0, overflow: 'hidden', position: 'relative' } as const
const PAN_TILT_LABEL = 'pan/tilt: no head mount configured'
const PAN_TILT_RANGE_DEG = 90

interface PreviewProps {
  topic: string | undefined
  /** Base64 JPEG of the latest preview, or undefined before the first message. */
  jpeg: string | null | undefined
  alt: string
}

/**
 * One greyscale preview image, or the waiting line while the topic has sent no preview (no placeholder image).
 *
 * @param topic - topic name shown in the waiting line
 * @param jpeg - base64 JPEG of the latest preview
 * @param alt - image alt text
 */
function Preview({ topic, jpeg, alt }: PreviewProps) {
  if (!jpeg) return <WaitingMessage>{`Waiting for ${topic}...`}</WaitingMessage>
  return (
    <Box sx={{ width: '100%', height: '100%', display: 'flex', alignItems: 'center', justifyContent: 'center', bgcolor: CANVAS_BG }}>
      <img src={`data:image/jpeg;base64,${jpeg}`} alt={alt} style={{ maxWidth: '100%', maxHeight: '100%', objectFit: 'contain' }} />
    </Box>
  )
}

/**
 * Head tab: head RGB stream and depth preview (16UC1 mm: stereo_depth or a ToF camera), optional amplitude (only
 * when the tab names an amplitude topic), and a disabled pan/tilt block.
 * The pan/tilt controls hold the place for a future head mount: they are disabled and never publish.
 * Phone stacks the previews vertically; tablet and desktop put them side by side.
 *
 * @param tab - tab config (rgb_camera_topic, tof_depth_topic, tof_amplitude_topic, tof_range_m)
 * @param topicData - latest payload per topic
 */
export default function HeadTab({ tab, topicData }: Props) {
  const layout = useLayout()
  const [showAmplitude, setShowAmplitude] = useState(false)
  const direction = layout === 'phone' ? 'column' : 'row'
  const cellMinHeight = layout === 'phone' ? 'clamp(10dvh, 28dvh, 40dvh)' : 0
  const depth = topicData[tab.tof_depth_topic ?? ''] as { depth_preview_b64?: string | null } | undefined
  const amplitude = topicData[tab.tof_amplitude_topic ?? ''] as { jpeg_b64?: string | null } | undefined

  return (
    <Box sx={{ width: '100%', height: '100%', display: 'flex', flexDirection: 'column', gap: 1, overflowY: layout === 'phone' ? 'auto' : 'hidden' }}>
      <Box data-testid="head-layout" data-direction={direction} sx={{ flex: '1 1 auto', minHeight: 0, display: 'flex', flexDirection: direction, gap: 1 }}>
        <Box sx={{ ...CELL_SX, minHeight: cellMinHeight }}>
          <CameraStream topic={tab.rgb_camera_topic} data={topicData[tab.rgb_camera_topic ?? '']} alt="head camera" />
        </Box>
        <Box sx={{ ...CELL_SX, minHeight: cellMinHeight }}>
          <Preview topic={tab.tof_depth_topic} jpeg={depth?.depth_preview_b64} alt="depth" />
        </Box>
        {showAmplitude && tab.tof_amplitude_topic && (
          <Box sx={{ ...CELL_SX, minHeight: cellMinHeight }}>
            <Preview topic={tab.tof_amplitude_topic} jpeg={amplitude?.jpeg_b64} alt="ToF amplitude" />
          </Box>
        )}
      </Box>
      <Stack direction="row" spacing={2} sx={{ flex: '0 0 auto', alignItems: 'center', flexWrap: 'wrap', px: 1 }}>
        {tab.tof_amplitude_topic && (
          <FormControlLabel
            control={<Switch checked={showAmplitude} onChange={(e) => setShowAmplitude(e.target.checked)} />}
            label="amplitude"
          />
        )}
        {tab.tof_range_m !== undefined && (
          <Typography variant="body2" color="text.secondary">{`depth range ${tab.tof_range_m} m`}</Typography>
        )}
      </Stack>
      <Stack data-testid="head-pan-tilt" spacing={1} sx={{ flex: '0 0 auto', p: 1 }}>
        <Typography variant="body2" color="text.secondary">{PAN_TILT_LABEL}</Typography>
        <Stack direction="row" spacing={2} sx={{ alignItems: 'center' }}>
          <Slider disabled aria-label="pan" value={0} min={-PAN_TILT_RANGE_DEG} max={PAN_TILT_RANGE_DEG} sx={{ flex: 1 }} />
          <Slider disabled aria-label="tilt" value={0} min={-PAN_TILT_RANGE_DEG} max={PAN_TILT_RANGE_DEG} sx={{ flex: 1 }} />
          <Button disabled variant="outlined" size="small">center</Button>
        </Stack>
      </Stack>
    </Box>
  )
}
