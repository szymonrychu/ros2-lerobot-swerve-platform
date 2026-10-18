import Box from '@mui/material/Box'
import { lazy, Suspense } from 'react'
import ArmJogPanel from '../arm/ArmJogPanel'
import { CameraStream } from '../components/CameraStream'
import { useLayout } from '../layout/useLayout'
import { TabConfig } from '../types'

const ArmView = lazy(() => import('../arm/ArmView.tsx'))

interface Props {
  tab: TabConfig
  topicData: Record<string, unknown>
  publish: (topic: string, msgType: string, data: unknown) => void
}

const CELL_SX = { flex: '1 1 0', minWidth: 0, minHeight: 0, overflow: 'hidden', position: 'relative' } as const

/**
 * Arm tab: gripper and head camera streams, a slot for the 3D view and a slot for the jog panel.
 * Phone stacks everything vertically; tablet and desktop put the streams side by side with the 3D view.
 *
 * @param tab - tab config (gripper_camera_topic, head_camera_topic)
 * @param topicData - latest payload per topic
 * @param publish - ROS publish helper
 */
export default function ArmTab({ tab, topicData, publish }: Props) {
  const layout = useLayout()
  const direction = layout === 'phone' ? 'column' : 'row'
  const streamMinHeight = layout === 'phone' ? 'clamp(10dvh, 28dvh, 40dvh)' : 0

  return (
    <Box sx={{ width: '100%', height: '100%', position: 'relative', display: 'flex', flexDirection: 'column', gap: 1, overflowY: layout === 'phone' ? 'auto' : 'hidden' }}>
      <Box
        data-testid="arm-layout"
        data-direction={direction}
        sx={{ flex: '1 1 auto', minHeight: 0, display: 'flex', flexDirection: direction, gap: 1 }}
      >
        <Box sx={{ ...CELL_SX, minHeight: streamMinHeight }}>
          <CameraStream topic={tab.gripper_camera_topic} data={topicData[tab.gripper_camera_topic ?? '']} alt="gripper camera" />
        </Box>
        <Box sx={{ ...CELL_SX, minHeight: streamMinHeight }}>
          <CameraStream topic={tab.head_camera_topic} data={topicData[tab.head_camera_topic ?? '']} alt="head camera" />
        </Box>
        <Box data-testid="arm-view-slot" sx={{ ...CELL_SX, minHeight: streamMinHeight }}>
          <Suspense fallback={null}>
            <ArmView tab={tab} topicData={topicData} publish={publish} />
          </Suspense>
        </Box>
      </Box>
      <Box data-testid="arm-jog-slot" sx={{ flex: '0 0 auto' }}>
        <ArmJogPanel tab={tab} topicData={topicData} />
      </Box>
    </Box>
  )
}
