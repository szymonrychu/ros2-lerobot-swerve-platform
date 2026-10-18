import { useEffect, useRef, useState } from 'react'
import Alert from '@mui/material/Alert'
import ToggleButton from '@mui/material/ToggleButton'
import ToggleButtonGroup from '@mui/material/ToggleButtonGroup'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import FormControlLabel from '@mui/material/FormControlLabel'
import Slider from '@mui/material/Slider'
import Stack from '@mui/material/Stack'
import Switch from '@mui/material/Switch'
import Typography from '@mui/material/Typography'
import { requestZoom, ZoomFactor } from '../camera/zoomApi'
import { CameraStream } from '../components/CameraStream'
import { useLayout } from '../layout/useLayout'
import { CANVAS_BG, TOUCH_TARGET_PX } from '../theme'
import { TabConfig } from '../types'
import { WaitingMessage } from './WaitingMessage'

interface Props {
  tab: TabConfig
  topicData: Record<string, unknown>
  publish: (topic: string, msgType: string, data: unknown) => void
}

const CELL_SX = { flex: '1 1 0', minWidth: 0, minHeight: 0, overflow: 'hidden', position: 'relative' } as const
const HEAD_JOINTS = ['head_pan', 'head_tilt']
const HEAD_MSG_TYPE = 'sensor_msgs/JointState'
const HEAD_PUBLISH_INTERVAL_MS = 200
const DEFAULT_PAN_RANGE_DEG: [number, number] = [-90, 90]
const DEFAULT_TILT_RANGE_DEG: [number, number] = [-30, 45]
const DEFAULT_HOME_DEG: [number, number] = [0, 30]
const DEG_PER_RAD = 180 / Math.PI
const ZOOM_FACTORS: ZoomFactor[] = [2, 4, 8]
const DEFAULT_ZOOM_FACTOR: ZoomFactor = 4

/** A shown crop: base64 JPEG, factor and whether autofocus converged. */
interface Crop {
  jpeg: string
  factor: ZoomFactor
  focused: boolean
}

/**
 * One short user message for a failed zoom request.
 *
 * @param err - error thrown by requestZoom (message carries the HTTP status)
 * @returns the message shown under the view
 */
function zoomErrorText(err: unknown): string {
  const msg = err instanceof Error ? err.message : ''
  if (msg.includes('HTTP 409')) return 'Camera busy'
  if (msg.includes('HTTP 504') || /HTTP \d+, timeout\)/.test(msg)) return 'Zoom timed out'
  return 'Camera error'
}

/**
 * Measured head angles from a JointState payload.
 *
 * @param sample - latest sample of head_joint_states_topic (name/position arrays), or undefined
 * @returns [pan, tilt] in degrees, or null before a sample carrying both joints
 */
function measuredDeg(sample: unknown): [number, number] | null {
  const js = sample as { name?: string[]; position?: number[] } | undefined
  if (!js?.name || !js.position) return null
  const out = HEAD_JOINTS.map((j) => js.position?.[js.name?.indexOf(j) ?? -1])
  return out.every((v) => typeof v === 'number' && Number.isFinite(v)) ? [(out[0] as number) * DEG_PER_RAD, (out[1] as number) * DEG_PER_RAD] : null
}

interface PanTiltProps {
  tab: TabConfig
  sample: unknown
  publish: Props['publish']
}

/**
 * Pan and tilt sliders plus Home. Disabled until the first measured sample. Dragging publishes at most every
 * 200 ms (leading edge, then the latest value on a timer), release publishes once more.
 *
 * @param tab - tab config (head_command_topic, ranges, head_home_deg, head_joint_states_topic)
 * @param sample - latest head joint states payload
 * @param publish - publishes a message on a topic
 */
function PanTiltControls({ tab, sample, publish }: PanTiltProps) {
  const topic = tab.head_command_topic as string
  const panRange = tab.head_pan_range_deg ?? DEFAULT_PAN_RANGE_DEG
  const tiltRange = tab.head_tilt_range_deg ?? DEFAULT_TILT_RANGE_DEG
  const home = tab.head_home_deg ?? DEFAULT_HOME_DEG
  const measured = measuredDeg(sample)
  const [target, setTarget] = useState<[number, number] | null>(null)
  const lastSent = useRef(-Infinity)
  const timer = useRef<ReturnType<typeof setTimeout> | null>(null)
  const latest = useRef<[number, number] | null>(null)
  const shown = target ?? measured

  function send(deg: [number, number]): void {
    if (timer.current) clearTimeout(timer.current)
    timer.current = null
    lastSent.current = Date.now()
    publish(topic, HEAD_MSG_TYPE, { name: HEAD_JOINTS, position: deg.map((d) => d / DEG_PER_RAD) })
  }

  function onDrag(deg: [number, number]): void {
    setTarget(deg)
    latest.current = deg
    const wait = lastSent.current + HEAD_PUBLISH_INTERVAL_MS - Date.now()
    if (wait <= 0) send(deg)
    else if (!timer.current) {
      timer.current = setTimeout(() => {
        timer.current = null
        if (latest.current) send(latest.current)
      }, wait)
    }
  }

  function onRelease(deg: [number, number]): void {
    latest.current = null
    send(deg)
    setTarget(null)
  }

  useEffect(() => () => {
    if (timer.current) clearTimeout(timer.current)
  }, [])

  function slider(index: 0 | 1, label: string, range: [number, number]) {
    const update = (v: number): [number, number] => (index === 0 ? [v, (shown as [number, number])[1]] : [(shown as [number, number])[0], v])
    return (
      <Stack spacing={0} sx={{ flex: 1 }}>
        <Typography variant="body2" color="text.secondary">{label}</Typography>
        <Slider
          aria-label={label}
          disabled={!shown}
          value={shown ? shown[index] : 0}
          min={range[0]}
          max={range[1]}
          valueLabelDisplay="auto"
          onChange={(_, v) => onDrag(update(v as number))}
          onChangeCommitted={(_, v) => onRelease(update(v as number))}
        />
      </Stack>
    )
  }

  return (
    <Stack data-testid="head-pan-tilt" spacing={1} sx={{ flex: '0 0 auto', p: 1 }}>
      {!measured && (
        <Typography variant="body2" color="text.secondary">{`waiting for ${tab.head_joint_states_topic}`}</Typography>
      )}
      <Stack direction="row" spacing={2} sx={{ alignItems: 'center' }}>
        {slider(0, 'pan', panRange)}
        {slider(1, 'tilt', tiltRange)}
        <Button variant="outlined" size="small" sx={{ minHeight: TOUCH_TARGET_PX }} onClick={() => onRelease(home)}>Home</Button>
      </Stack>
      {measured && (
        <Typography data-testid="head-measured" variant="body2">
          {`pan ${measured[0].toFixed(1)} deg, tilt ${measured[1].toFixed(1)} deg`}
        </Typography>
      )}
    </Stack>
  )
}

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
 * Head tab: head RGB stream and depth preview (16UC1 mm from a ToF camera), optional amplitude (only
 * when the tab names an amplitude topic), and pan/tilt sliders with Home when the tab has head_command_topic.
 * Phone stacks the previews vertically; tablet and desktop put them side by side.
 *
 * @param tab - tab config (rgb_camera_topic, tof_depth_topic, tof_amplitude_topic, tof_range_m)
 * @param topicData - latest payload per topic
 * @param publish - publishes a head command
 */
export default function HeadTab({ tab, topicData, publish }: Props) {
  const layout = useLayout()
  const zoomEnabled = Boolean(tab.zoom_command_topic)
  const [factor, setFactor] = useState<ZoomFactor>(DEFAULT_ZOOM_FACTOR)
  const [pending, setPending] = useState(false)
  const [crop, setCrop] = useState<Crop | null>(null)
  const [zoomError, setZoomError] = useState<string | null>(null)
  const [showAmplitude, setShowAmplitude] = useState(false)
  const direction = layout === 'phone' ? 'column' : 'row'
  const cellMinHeight = layout === 'phone' ? 'clamp(10dvh, 28dvh, 40dvh)' : 0
  const depthTopic = tab.tof_depth_topic
  const depth = topicData[depthTopic ?? ''] as { depth_preview_b64?: string | null } | undefined
  const amplitude = topicData[tab.tof_amplitude_topic ?? ''] as { jpeg_b64?: string | null } | undefined

  function onPointSelect(u: number, v: number): void {
    if (pending) return
    setPending(true)
    setZoomError(null)
    requestZoom(tab.id, u, v, factor)
      .then((reply) => setCrop({ jpeg: reply.jpeg_b64, factor, focused: reply.status !== 'af_failed' }))
      .catch((err) => setZoomError(zoomErrorText(err)))
      .finally(() => setPending(false))
  }

  return (
    <Box sx={{ width: '100%', height: '100%', display: 'flex', flexDirection: 'column', gap: 1, overflowY: layout === 'phone' ? 'auto' : 'hidden' }}>
      <Box data-testid="head-layout" data-direction={direction} sx={{ flex: '1 1 auto', minHeight: 0, display: 'flex', flexDirection: direction, gap: 1 }}>
        <Box sx={{ ...CELL_SX, minHeight: cellMinHeight }}>
          {crop ? (
            <Box sx={{ width: '100%', height: '100%', display: 'flex', alignItems: 'center', justifyContent: 'center', bgcolor: CANVAS_BG }}>
              <img src={`data:image/jpeg;base64,${crop.jpeg}`} alt="head camera zoom" style={{ maxWidth: '100%', maxHeight: '100%', objectFit: 'contain' }} />
            </Box>
          ) : (
            <CameraStream
              topic={tab.rgb_camera_topic}
              data={topicData[tab.rgb_camera_topic ?? '']}
              alt="head camera"
              onPointSelect={zoomEnabled ? onPointSelect : undefined}
            />
          )}
        </Box>
        <Box sx={{ ...CELL_SX, minHeight: cellMinHeight }}>
          <Preview topic={depthTopic} jpeg={depth?.depth_preview_b64} alt="depth" />
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
        {zoomEnabled && (
          <ToggleButtonGroup
            exclusive
            size="small"
            role="radiogroup"
            aria-label="zoom factor"
            value={factor}
            onChange={(_, v: ZoomFactor | null) => v && setFactor(v)}
          >
            {ZOOM_FACTORS.map((f) => (
              <ToggleButton key={f} value={f} role="radio" aria-checked={f === factor} aria-label={`${f}x`}>{`${f}x`}</ToggleButton>
            ))}
          </ToggleButtonGroup>
        )}
        {pending && <Typography variant="body2" color="text.secondary">Zooming...</Typography>}
        {crop && (
          <>
            <Typography data-testid="zoom-info" variant="body2">{`${crop.factor}x zoom`}</Typography>
            {!crop.focused && <Typography variant="body2" color="warning.main">focus not confirmed</Typography>}
            <Button variant="outlined" size="small" onClick={() => setCrop(null)}>Back to live</Button>
          </>
        )}
        {zoomError && <Alert severity="warning" sx={{ py: 0 }}>{zoomError}</Alert>}
        {tab.tof_range_m !== undefined && (
          <Typography variant="body2" color="text.secondary">{`depth range ${tab.tof_range_m} m`}</Typography>
        )}
      </Stack>
      {tab.head_command_topic && <PanTiltControls tab={tab} sample={topicData[tab.head_joint_states_topic ?? '']} publish={publish} />}
    </Box>
  )
}
