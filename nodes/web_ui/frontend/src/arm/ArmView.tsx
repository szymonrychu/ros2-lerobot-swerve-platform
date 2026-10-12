import { useCallback, useEffect, useRef, useState } from 'react'
import { Canvas, useFrame, useThree } from '@react-three/fiber'
import { OrbitControls } from '@react-three/drei'
import type { OrbitControls as OrbitControlsImpl } from 'three-stdlib'
import Box from '@mui/material/Box'
import ToggleButton from '@mui/material/ToggleButton'
import ToggleButtonGroup from '@mui/material/ToggleButtonGroup'
import type { URDFRobot } from 'urdf-loader'
import { InteractiveArm } from '../components3d/InteractiveArm'
import { RobotModel } from '../components3d/RobotModel'
import type { ArmCameraMode, ArmCameraMount } from '../map3d/armCamera'
import { TOKENS } from '../theme'
import { TabConfig } from '../types'
import { ARM_CAMERA_MODES, applyArmCamera } from './armView.ts'

const MODE_LABELS: Record<ArmCameraMode, string> = {
  'side-left': 'Side L',
  'side-right': 'Side R',
  above: 'Above',
  gripper: 'Gripper',
}
const DEFAULT_MODE: ArmCameraMode = 'side-left'
const DEFAULT_MOUNT: ArmCameraMount = { xyz: [0, 0, 0], rpy: [0, 0, 0] }
const CAMERA_FOV = 50
const FREE_CAMERA_POSITION: [number, number, number] = [0.5, 0.4, 0.5]
const FREE_CAMERA_TARGET: [number, number, number] = [0, 0.15, 0]

interface JointStates {
  name?: string[]
  position?: number[]
}

interface SceneProps {
  tab: TabConfig
  armJoints: JointStates | undefined
  mode: ArmCameraMode | null
  publish: (topic: string, msgType: string, data: unknown) => void
}

/** Scene contents: the arm, the camera glue and the orbit controls (inside the Canvas). */
function ArmScene({ tab, armJoints, mode, publish }: SceneProps) {
  const { invalidate, camera } = useThree()
  const orbitRef = useRef<OrbitControlsImpl | null>(null)
  const robotRef = useRef<URDFRobot | null>(null)
  const mount = tab.gripper_camera_mount ?? DEFAULT_MOUNT

  const onRobotLoaded = useCallback(
    (robot: URDFRobot | null) => {
      robotRef.current = robot
      invalidate()
    },
    [invalidate],
  )

  // Joint updates: RobotModel applies setJointValue in its effect, the next frame glues the camera to the result.
  useEffect(() => invalidate(), [armJoints, mode, invalidate])

  // Leaving a glued mode hands the camera back to the orbit controls with the world up vector.
  useEffect(() => {
    if (mode !== null) return
    camera.up.set(0, 1, 0)
    orbitRef.current?.target.set(...FREE_CAMERA_TARGET)
    orbitRef.current?.update()
    invalidate()
  }, [mode, camera, invalidate])

  // Default priority (0): a positive priority would switch off the automatic render.
  useFrame(() => {
    const robot = robotRef.current
    const controls = orbitRef.current
    if (mode === null || !robot || !controls) return
    applyArmCamera(camera, controls.target, robot, mode, mount)
  })

  return (
    <>
      <ambientLight intensity={0.8} />
      <directionalLight position={[2, 4, 3]} intensity={1.2} />
      {tab.arm_urdf &&
        (tab.arm_command_topic ? (
          <InteractiveArm
            urdfFile={tab.arm_urdf}
            liveJointStates={armJoints}
            commandTopic={tab.arm_command_topic}
            publish={publish}
            orbitRef={orbitRef}
            onRobotLoaded={onRobotLoaded}
          />
        ) : (
          <RobotModel urdfFile={tab.arm_urdf} jointStates={armJoints} onRobotLoaded={onRobotLoaded} />
        ))}
      <OrbitControls ref={orbitRef} enabled={mode === null} enableDamping={false} />
    </>
  )
}

interface Props {
  tab: TabConfig
  topicData: Record<string, unknown>
  publish: (topic: string, msgType: string, data: unknown) => void
}

/**
 * The Arm tab's own 3D view: the follower arm from arm_joint_states_topic with camera modes glued to the arm
 * (side left, side right, above, gripper). Picking the active mode again frees the camera for orbiting.
 *
 * @param tab - tab config (arm_urdf, arm_joint_states_topic, arm_command_topic, gripper_camera_mount)
 * @param topicData - latest payload per topic
 * @param publish - ROS publish helper for the draggable arm
 */
export default function ArmView({ tab, topicData, publish }: Props) {
  const [mode, setMode] = useState<ArmCameraMode | null>(DEFAULT_MODE)
  const armJoints = topicData[tab.arm_joint_states_topic ?? ''] as JointStates | undefined
  return (
    <Box data-testid="arm-view" sx={{ position: 'relative', width: '100%', height: '100%' }}>
      <Canvas
        frameloop="demand"
        camera={{ position: FREE_CAMERA_POSITION, fov: CAMERA_FOV, near: 0.01, far: 50 }}
        style={{ background: TOKENS.bgCanvas, touchAction: 'none' }}
        dpr={[1, 2]}
      >
        <ArmScene tab={tab} armJoints={armJoints} mode={mode} publish={publish} />
      </Canvas>
      <ToggleButtonGroup
        exclusive
        size="small"
        value={mode}
        onChange={(_, next: ArmCameraMode | null) => setMode(next)}
        aria-label="arm camera"
        sx={{ position: 'absolute', top: 8, left: 8, flexDirection: 'column', bgcolor: 'background.paper' }}
      >
        {ARM_CAMERA_MODES.map((m) => (
          <ToggleButton key={m} value={m} sx={{ minHeight: 44, minWidth: 44 }}>
            {MODE_LABELS[m]}
          </ToggleButton>
        ))}
      </ToggleButtonGroup>
    </Box>
  )
}
