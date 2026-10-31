import Box from '@mui/material/Box'
import { lazy, Suspense, useState } from 'react'
import { ReleasedBannerView, releasedLabel } from '../calibrate/ReleasedBanner'
import { useServoGroups } from '../calibrate/servoApi'
import ArmJogPanel from '../arm/ArmJogPanel'
import ArmActions from '../arm/ArmActions'
import { activeSourceOf, armControlState, releasedJoints } from '../arm/armControls'
import { CameraStream } from '../components/CameraStream'
import { useLayout } from '../layout/useLayout'
import type { ActionResult } from '../map/mapActions'
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
  const servo = useServoGroups()
  const released = releasedLabel(servo, 'arm') !== null
  const [armReady, setArmReady] = useState(false)
  const [action, setAction] = useState<ActionResult>({ state: 'idle', message: '' })
  const controls = armControlState({
    released,
    releasedJoints: releasedJoints(servo, 'arm'),
    busy: action.state === 'busy',
    armReady,
    activeSource: activeSourceOf(tab, topicData),
    graspBusy: false,
  })
  const drag = controls.drag
  // The draggable arm in the 3D view is a command source too: drop its goals while the arm is limp.
  const guardedPublish: Props['publish'] = (topic, msgType, data) => {
    if (!released) publish(topic, msgType, data)
  }
  const direction = layout === 'phone' ? 'column' : 'row'
  const streamMinHeight = layout === 'phone' ? 'clamp(10dvh, 28dvh, 40dvh)' : 0

  return (
    <Box sx={{ width: '100%', height: '100%', position: 'relative', display: 'flex', flexDirection: 'column', gap: 1, overflowY: layout === 'phone' ? 'auto' : 'hidden' }}>
      <ReleasedBannerView groupId="arm" servo={servo} />
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
            <ArmView
              tab={tab}
              topicData={topicData}
              publish={guardedPublish}
              dragDisabledReason={drag.disabled ? drag.reason : null}
              onArmReady={setArmReady}
            />
          </Suspense>
        </Box>
      </Box>
      <Box data-testid="arm-actions-slot" sx={{ flex: '0 0 auto' }}>
        <ArmActions tabId={tab.id} controls={controls} action={action} onAction={setAction} />
      </Box>
      <Box data-testid="arm-jog-slot" sx={{ flex: '0 0 auto' }}>
        <ArmJogPanel tab={tab} topicData={topicData} released={released} />
      </Box>
    </Box>
  )
}
