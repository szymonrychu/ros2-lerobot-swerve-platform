import Box from '@mui/material/Box'
import { CameraStream } from '../components/CameraStream'
import { useLayout } from '../layout/useLayout'
import { TabConfig } from '../types'

/** Arm tab fields defined by the backend config (T28); optional until the shared type carries them. */
type ArmTabConfig = TabConfig & { gripper_camera_topic?: string; overview_camera_topic?: string }

interface Props {
  tab: ArmTabConfig
  topicData: Record<string, unknown>
  publish: (topic: string, msgType: string, data: unknown) => void
}

const CELL_SX = { flex: '1 1 0', minWidth: 0, minHeight: 0, overflow: 'hidden', position: 'relative' } as const

/**
 * Arm tab: gripper and overview camera streams, a slot for the 3D view and a slot for the jog panel.
 * Phone stacks everything vertically; tablet and desktop put the streams side by side with the 3D view.
 *
 * @param tab - tab config (gripper_camera_topic, overview_camera_topic)
 * @param topicData - latest payload per topic
 */
export default function ArmTab({ tab, topicData }: Props) {
  const layout = useLayout()
  const direction = layout === 'phone' ? 'column' : 'row'
  const streamMinHeight = layout === 'phone' ? 'clamp(10dvh, 28dvh, 40dvh)' : 0

  return (
    <Box sx={{ width: '100%', height: '100%', display: 'flex', flexDirection: 'column', gap: 1, overflowY: layout === 'phone' ? 'auto' : 'hidden' }}>
      <Box
        data-testid="arm-layout"
        data-direction={direction}
        sx={{ flex: '1 1 auto', minHeight: 0, display: 'flex', flexDirection: direction, gap: 1 }}
      >
        <Box sx={{ ...CELL_SX, minHeight: streamMinHeight }}>
          <CameraStream topic={tab.gripper_camera_topic} data={topicData[tab.gripper_camera_topic ?? '']} alt="gripper camera" />
        </Box>
        <Box sx={{ ...CELL_SX, minHeight: streamMinHeight }}>
          <CameraStream topic={tab.overview_camera_topic} data={topicData[tab.overview_camera_topic ?? '']} alt="overview camera" />
        </Box>
        <Box data-testid="arm-view-slot" sx={{ ...CELL_SX, minHeight: streamMinHeight }} />
      </Box>
      <Box data-testid="arm-jog-slot" sx={{ flex: '0 0 auto' }} />
    </Box>
  )
}
