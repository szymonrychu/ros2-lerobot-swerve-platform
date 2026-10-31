import { useCallback, useEffect, useRef, useState, type ReactNode } from 'react'
import ChevronLeftIcon from '@mui/icons-material/ChevronLeft'
import ChevronRightIcon from '@mui/icons-material/ChevronRight'
import ExpandLessIcon from '@mui/icons-material/ExpandLess'
import PanToolIcon from '@mui/icons-material/PanTool'
import { Canvas, useFrame, useThree } from '@react-three/fiber'
import { OrbitControls } from '@react-three/drei'
import type { OrbitControls as OrbitControlsImpl } from 'three-stdlib'
import Box from '@mui/material/Box'
import type { URDFRobot } from 'urdf-loader'
import { ToolButton, ToolGroup } from '../components/ToolButton'
import { InteractiveArm } from '../components3d/InteractiveArm'
import { RobotModel } from '../components3d/RobotModel'
import { GraspPreview } from '../grasp/GraspPreview'
import { DEFAULT_ARM_OFFSET } from '../grasp/grasp'
import type { GraspPreviewData } from '../grasp/useGrasp'
import type { Vec2 } from '../map/mapMath'
import { isClick } from '../poi/editor'
import { rosToThree } from '../map3d/coords'
import { SCENE_COLORS } from '../map3d/palette'
import type { ArmCameraMode, ArmCameraMount } from '../map3d/armCamera'
import { TOKENS } from '../theme'
import { TabConfig } from '../types'
import { WAITING_REASON } from './armControls'
import { ARM_CAMERA_MODES, applyArmCamera } from './armView.ts'
import { armViewGroundY, pickArmViewGround } from './armViewPick'

const MODE_LABELS: Record<ArmCameraMode, string> = {
  'side-left': 'Side L',
  'side-right': 'Side R',
  above: 'Above',
  gripper: 'Gripper',
}
const MODE_ICONS: Record<ArmCameraMode, ReactNode> = {
  'side-left': <ChevronLeftIcon />,
  'side-right': <ChevronRightIcon />,
  above: <ExpandLessIcon />,
  gripper: <PanToolIcon />,
}
const DEFAULT_MODE: ArmCameraMode = 'side-left'
const DEFAULT_MOUNT: ArmCameraMount = { xyz: [0, 0, 0], rpy: [0, 0, 0] }
const CAMERA_FOV = 50
const FREE_CAMERA_POSITION: [number, number, number] = [0.5, 0.4, 0.5]
const FREE_CAMERA_TARGET: [number, number, number] = [0, 0.15, 0]
const FLOOR_GRID_SIZE_M = 2
const FLOOR_GRID_DIVISIONS = 20
const PICK_CURSOR = 'crosshair'

interface JointStates {
  name?: string[]
  position?: number[]
}

interface PickLayerProps {
  armOffset: readonly [number, number, number]
  onPick: (point: Vec2) => void
}

/**
 * Grasp pick inside the Canvas: a left click that does not move picks the base_link floor point under it; a press
 * that moves is an orbit and picks nothing.
 */
function PickLayer({ armOffset, onPick }: PickLayerProps) {
  const { gl, camera } = useThree()
  useEffect(() => {
    const el = gl.domElement
    let down: Vec2 | null = null
    const screen = (e: PointerEvent | MouseEvent): Vec2 => ({ x: e.clientX, y: e.clientY })
    const onDown = (e: Event) => {
      const ev = e as PointerEvent
      down = ev.button === 0 ? screen(ev) : null
    }
    const onUp = (e: Event) => {
      const ev = e as PointerEvent
      const start = down
      down = null
      if (!start || !isClick(start, screen(ev))) return
      const rect = el.getBoundingClientRect()
      camera.updateMatrixWorld()
      const at = pickArmViewGround(camera, ev.clientX - rect.left, ev.clientY - rect.top, rect.width, rect.height, armOffset)
      if (at) onPick(at)
    }
    const cursor = el.style.cursor
    el.style.cursor = PICK_CURSOR
    el.addEventListener('pointerdown', onDown)
    window.addEventListener('pointerup', onUp)
    return () => {
      el.style.cursor = cursor
      el.removeEventListener('pointerdown', onDown)
      window.removeEventListener('pointerup', onUp)
    }
  }, [gl, camera, armOffset, onPick])
  return null
}

interface SceneProps {
  tab: TabConfig
  armJoints: JointStates | undefined
  mode: ArmCameraMode | null
  publish: (topic: string, msgType: string, data: unknown) => void
  /** The arm is draggable: a command topic is configured and no reason blocks dragging. */
  draggable: boolean
  onArmReady: (ready: boolean) => void
  pickMode: boolean
  onPick: (point: Vec2) => void
  graspPreview: GraspPreviewData | null
}

/** Scene contents: the arm, the floor, the grasp preview, the camera glue and the orbit controls (inside the Canvas). */
function ArmScene({ tab, armJoints, mode, publish, draggable, onArmReady, pickMode, onPick, graspPreview }: SceneProps) {
  const { invalidate, camera } = useThree()
  const orbitRef = useRef<OrbitControlsImpl | null>(null)
  const robotRef = useRef<URDFRobot | null>(null)
  const mount = tab.gripper_camera_mount ?? DEFAULT_MOUNT
  const armOffset = tab.arm_offset ?? DEFAULT_ARM_OFFSET
  const baseLinkOrigin = rosToThree({ x: -armOffset[0], y: -armOffset[1], z: -armOffset[2] })

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
        (draggable && !pickMode && tab.arm_command_topic ? (
          <InteractiveArm
            urdfFile={tab.arm_urdf}
            liveJointStates={armJoints}
            commandTopic={tab.arm_command_topic}
            publish={publish}
            orbitRef={orbitRef}
            onReady={onArmReady}
            onRobotLoaded={onRobotLoaded}
          />
        ) : (
          <RobotModel urdfFile={tab.arm_urdf} jointStates={armJoints} onRobotLoaded={onRobotLoaded} />
        ))}
      <gridHelper args={[FLOOR_GRID_SIZE_M, FLOOR_GRID_DIVISIONS, SCENE_COLORS.gridCentre, SCENE_COLORS.gridLine]} position={[baseLinkOrigin[0], armViewGroundY(armOffset), baseLinkOrigin[2]]} />
      <group position={baseLinkOrigin}>
        <GraspPreview data={graspPreview} />
      </group>
      {pickMode && <PickLayer armOffset={armOffset} onPick={onPick} />}
      <OrbitControls ref={orbitRef} enabled={mode === null} enableDamping={false} />
    </>
  )
}

interface Props {
  tab: TabConfig
  topicData: Record<string, unknown>
  publish: (topic: string, msgType: string, data: unknown) => void
  /** Why the arm cannot be dragged (shown on the view), or null while it can. */
  dragDisabledReason: string | null
  /** Called with true once the draggable arm has its first joint states. */
  onArmReady: (ready: boolean) => void
  /** A left click on the floor picks the grasp object; the arm is display-only meanwhile. */
  pickMode: boolean
  /** Called with the picked floor point in base_link. */
  onPick: (point: Vec2) => void
  /** Grasp object box and planned tool path (base_link), or null. */
  graspPreview: GraspPreviewData | null
}

/**
 * The Arm tab's own 3D view: the follower arm from arm_joint_states_topic with camera modes glued to the arm
 * (side left, side right, above, gripper). Picking the active mode again frees the camera for orbiting.
 *
 * @param tab - tab config (arm_urdf, arm_joint_states_topic, arm_command_topic, gripper_camera_mount)
 * @param topicData - latest payload per topic
 * @param publish - ROS publish helper for the draggable arm
 * @param dragDisabledReason - why dragging is off, or null; the display-only arm is drawn for any reason but waiting
 * @param onArmReady - readiness of the draggable arm (first joint states received)
 * @param pickMode - a floor click picks the grasp object
 * @param onPick - receives the picked base_link floor point
 * @param graspPreview - grasp preview drawn in base_link coordinates, or null
 */
export default function ArmView({ tab, topicData, publish, dragDisabledReason, onArmReady, pickMode, onPick, graspPreview }: Props) {
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
        <ArmScene
          tab={tab}
          armJoints={armJoints}
          mode={mode}
          publish={publish}
          draggable={dragDisabledReason === null || dragDisabledReason === WAITING_REASON}
          onArmReady={onArmReady}
          pickMode={pickMode}
          onPick={onPick}
          graspPreview={graspPreview}
        />
      </Canvas>
      {dragDisabledReason !== null && (
        <Box
          data-testid="arm-drag-reason"
          sx={{ position: 'absolute', bottom: 8, left: 8, right: 8, px: 1, py: 0.5, bgcolor: TOKENS.bgPanel, typography: 'caption' }}
        >
          {dragDisabledReason}
        </Box>
      )}
      <Box sx={{ position: 'absolute', top: 8, left: 8, bgcolor: TOKENS.bgPanel }}>
        <ToolGroup label="arm camera">
          {ARM_CAMERA_MODES.map((m) => (
            <ToolButton key={m} label={MODE_LABELS[m]} icon={MODE_ICONS[m]} pressed={mode === m} onClick={() => setMode(mode === m ? null : m)} />
          ))}
        </ToolGroup>
      </Box>
    </Box>
  )
}
