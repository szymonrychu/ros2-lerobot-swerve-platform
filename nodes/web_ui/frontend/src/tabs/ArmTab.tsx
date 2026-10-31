import Box from '@mui/material/Box'
import { lazy, Suspense, useCallback, useEffect, useMemo, useState } from 'react'
import { ReleasedBannerView, releasedLabel } from '../calibrate/ReleasedBanner'
import { useServoGroups } from '../calibrate/servoApi'
import ArmJogPanel from '../arm/ArmJogPanel'
import ArmActions from '../arm/ArmActions'
import { BASE_LINK_POSE } from '../arm/armViewPick'
import { activeSourceOf, armControlState, releasedJoints } from '../arm/armControls'
import { CameraStream } from '../components/CameraStream'
import { DEFAULT_ARM_OFFSET } from '../grasp/grasp'
import { GraspPanel } from '../grasp/GraspPanel'
import { useGrasp } from '../grasp/useGrasp'
import { useLayout } from '../layout/useLayout'
import type { ActionResult } from '../map/mapActions'
import type { Vec2 } from '../map/mapMath'
import { TabConfig } from '../types'

const ArmView = lazy(() => import('../arm/ArmView.tsx'))

interface Props {
  tab: TabConfig
  topicData: Record<string, unknown>
  publish: (topic: string, msgType: string, data: unknown) => void
}

const CELL_SX = { flex: '1 1 0', minWidth: 0, minHeight: 0, overflow: 'hidden', position: 'relative' } as const

/**
 * Arm tab: gripper and head camera streams, the draggable 3D arm, Arm home / Set home / Grasp and the jog panel.
 * Phone stacks everything vertically; tablet and desktop put the streams side by side with the 3D view.
 *
 * @param tab - tab config (gripper_camera_topic, head_camera_topic, arm_offset, grasp_command_topic)
 * @param topicData - latest payload per topic
 * @param publish - ROS publish helper
 */
export default function ArmTab({ tab, topicData, publish }: Props) {
  const layout = useLayout()
  const servo = useServoGroups()
  const released = releasedLabel(servo, 'arm') !== null
  const [armReady, setArmReady] = useState(false)
  const [action, setAction] = useState<ActionResult>({ state: 'idle', message: '' })
  const armMount = useMemo(() => tab.arm_offset ?? DEFAULT_ARM_OFFSET, [tab.arm_offset])
  const grasp = useGrasp({ tabId: tab.id, topicData, mount: armMount, notify: setAction })
  const [graspOpen, setGraspOpen] = useState(false)
  const { cancelPick, applyClick } = grasp
  // Closing the grasp panel (or a limp arm, which takes no grasp) ends the pick.
  useEffect(() => {
    if (!graspOpen) cancelPick()
  }, [graspOpen, cancelPick])
  useEffect(() => {
    if (released) {
      setGraspOpen(false)
      cancelPick()
    }
  }, [released, cancelPick])
  // The picked point is already in base_link: the identity pose keeps it unchanged.
  const onPick = useCallback((point: Vec2) => applyClick(point, BASE_LINK_POSE), [applyClick])
  const controls = armControlState({
    released,
    releasedJoints: releasedJoints(servo, 'arm'),
    busy: action.state === 'busy',
    armReady,
    activeSource: activeSourceOf(tab, topicData),
    graspBusy: grasp.executing || grasp.planning,
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
              pickMode={grasp.pickMode}
              onPick={onPick}
              graspPreview={graspOpen ? grasp.preview : null}
            />
          </Suspense>
        </Box>
      </Box>
      <Box data-testid="arm-actions-slot" sx={{ flex: '0 0 auto' }}>
        <ArmActions
          tabId={tab.id}
          controls={controls}
          action={action}
          onAction={setAction}
          grasp={tab.grasp_command_topic ? grasp : null}
          graspOpen={graspOpen}
          onGraspOpenChange={setGraspOpen}
        />
      </Box>
      {graspOpen && <GraspPanel state={grasp} onClose={() => setGraspOpen(false)} />}
      <Box data-testid="arm-jog-slot" sx={{ flex: '0 0 auto' }}>
        <ArmJogPanel tab={tab} topicData={topicData} released={released} />
      </Box>
    </Box>
  )
}
