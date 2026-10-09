/**
 * The 3D map scene (lazy-loaded chunk): SLAM map, costmap, GPS tiles, Nav2 overlays and the robot (base + arm)
 * in the ROS map frame. Rendering is on demand; the component is memoised so unrelated topic updates do not
 * re-render the canvas.
 */
import { memo, Suspense, useCallback, useEffect, useMemo, useRef } from 'react'
import type { MutableRefObject } from 'react'
import { Canvas, useThree } from '@react-three/fiber'
import { OrbitControls } from '@react-three/drei'
import type { OrbitControls as OrbitControlsImpl } from 'three-stdlib'
import * as THREE from 'three'
import { RobotModel } from '../components3d/RobotModel'
import { InteractiveArm } from '../components3d/InteractiveArm'
import { Pose2D, Vec2 } from '../map/mapMath'
import { CANVAS_BG } from '../theme'
import { rosToThree, rosYawToThreeY, ThreeTuple } from './coords'
import { GpsAnchor } from './geo'
import { Bounds, classifyBaseLink, fitDistance } from './groundMath'
import { LayerState } from './layers'
import { PoiLayer } from '../poi/PoiLayer'
import type { Poi } from '../poi/types'
import { pickGround } from './picking'
import {
  FootprintLayer,
  GoalMarker,
  GpsTilesLayer,
  GridImageLayer,
  GridImageMsg,
  LIFT,
  PathLayer,
  PathMsg,
  SCENE_COLORS,
} from './sceneLayers'

const CAMERA_FOV = 50
const INITIAL_CAMERA: ThreeTuple = [-3, 3, 3]
const ORBIT_MAX_POLAR = Math.PI / 2 - 0.05 // keep the orbit camera above the ground
const OBLIQUE_ELEVATION = Math.PI / 4 // camera elevation when leaving the top view
const COSTMAP_OPACITY = 0.55
const DEFAULT_ARM_OFFSET: [number, number, number] = [0.25, 0, 0]

/** Imperative handle the tab uses for camera buttons and ground picking. */
export interface SceneController {
  /** Map-frame point under a client (viewport) position, or null when it misses the ground. */
  pick: (clientX: number, clientY: number) => Vec2 | null
  /** Keep the camera offset, move the view centre to p. */
  centerOn: (p: Vec2) => void
  /** Frame the given map-frame bounds. */
  fit: (b: Bounds) => void
}

export interface MapSceneProps {
  tileVersion?: string | null // /api/config tile_version, appended to tile URLs to bust stale browser caches
  baseUrdf?: string
  armUrdf?: string
  armOffset?: [number, number, number]
  armCommandTopic?: string
  publish: (topic: string, msgType: string, data: unknown) => void
  onArmReady: (ready: boolean) => void
  map: GridImageMsg | null | undefined
  costmap: GridImageMsg | null | undefined
  globalPath: PathMsg | null | undefined
  localPath: PathMsg | null | undefined
  goal: Pose2D | null | undefined
  draftGoal: Pose2D | null
  footprint: PathMsg | null | undefined
  robotPose: Pose2D | null
  baseJoints: JointStates | undefined
  armJoints: JointStates | undefined
  anchor: GpsAnchor | null
  layers: LayerState
  topView: boolean
  /** A one-finger/left-button gesture belongs to the app (goal setting, adding a POI), not to camera panning. */
  goalMode: boolean
  pois: Poi[]
  selectedPoiId: string | null
  poiDraft: Vec2[]
  controllerRef: MutableRefObject<SceneController | null>
}

interface JointStates {
  name?: string[]
  position?: number[]
}

/** Configure OrbitControls for free orbit or the locked top view, and expose the controller. */
function CameraRig({
  orbitRef,
  topView,
  goalMode,
  controllerRef,
}: {
  orbitRef: MutableRefObject<OrbitControlsImpl | null>
  topView: boolean
  goalMode: boolean
  controllerRef: MutableRefObject<SceneController | null>
}) {
  const { camera, gl, invalidate } = useThree()

  useEffect(() => {
    const c = orbitRef.current
    if (!c) return
    const target = c.target
    const dist = Math.max(camera.position.distanceTo(target), 1)
    if (topView) {
      c.enableRotate = false
      c.minPolarAngle = 0
      c.maxPolarAngle = 0
      c.minAzimuthAngle = 0
      c.maxAzimuthAngle = 0
      c.screenSpacePanning = true
      // Goal mode: one finger / left button draws the goal; two fingers, wheel and right button still pan + zoom.
      c.mouseButtons = goalMode
        ? { MIDDLE: THREE.MOUSE.DOLLY, RIGHT: THREE.MOUSE.PAN }
        : { LEFT: THREE.MOUSE.PAN, MIDDLE: THREE.MOUSE.DOLLY, RIGHT: THREE.MOUSE.PAN }
      c.touches = goalMode ? { TWO: THREE.TOUCH.DOLLY_PAN } : { ONE: THREE.TOUCH.PAN, TWO: THREE.TOUCH.DOLLY_PAN }
      camera.position.set(target.x, target.y + dist, target.z)
    } else {
      c.enableRotate = true
      c.minPolarAngle = 0
      c.maxPolarAngle = ORBIT_MAX_POLAR
      c.minAzimuthAngle = -Infinity
      c.maxAzimuthAngle = Infinity
      c.screenSpacePanning = true
      c.mouseButtons = { LEFT: THREE.MOUSE.ROTATE, MIDDLE: THREE.MOUSE.DOLLY, RIGHT: THREE.MOUSE.PAN }
      c.touches = { ONE: THREE.TOUCH.ROTATE, TWO: THREE.TOUCH.DOLLY_PAN }
      const offset = camera.position.clone().sub(target)
      if (offset.y > 0.95 * offset.length()) {
        // Coming out of the top view: tilt to an oblique view behind the current heading.
        camera.position.set(
          target.x,
          target.y + dist * Math.sin(OBLIQUE_ELEVATION),
          target.z + dist * Math.cos(OBLIQUE_ELEVATION),
        )
      }
    }
    c.update()
    invalidate()
  }, [topView, goalMode, camera, orbitRef, invalidate])

  useEffect(() => {
    controllerRef.current = {
      pick: (clientX, clientY) => {
        const rect = gl.domElement.getBoundingClientRect()
        camera.updateMatrixWorld()
        return pickGround(camera, clientX - rect.left, clientY - rect.top, rect.width, rect.height)
      },
      centerOn: (p) => {
        const c = orbitRef.current
        if (!c) return
        const [x, , z] = rosToThree(p)
        const delta = new THREE.Vector3(x - c.target.x, 0 - c.target.y, z - c.target.z)
        c.target.add(delta)
        camera.position.add(delta)
        c.update()
        invalidate()
      },
      fit: (b) => {
        const c = orbitRef.current
        if (!c) return
        const rect = gl.domElement.getBoundingClientRect()
        const aspect = rect.height > 0 ? rect.width / rect.height : 1
        const fov = camera instanceof THREE.PerspectiveCamera ? camera.fov : CAMERA_FOV
        const dist = fitDistance(b.maxX - b.minX, b.maxY - b.minY, fov, aspect)
        const [x, , z] = rosToThree({ x: (b.minX + b.maxX) / 2, y: (b.minY + b.maxY) / 2 })
        const dir = camera.position.clone().sub(c.target)
        if (dir.lengthSq() < 1e-9) dir.set(0, 1, 0)
        dir.normalize()
        c.target.set(x, 0, z)
        camera.position.copy(c.target).addScaledVector(dir, dist)
        if (camera instanceof THREE.PerspectiveCamera) {
          camera.far = Math.max(1000, dist * 4)
          camera.updateProjectionMatrix()
        }
        c.update()
        invalidate()
      },
    }
    return () => {
      controllerRef.current = null
    }
  }, [camera, gl, orbitRef, controllerRef, invalidate])

  return null
}

/** Base URDF at the robot pose with the arm mounted on it; robot-part layers toggle link visibility. */
const RobotLayer = memo(function RobotLayer({
  pose,
  baseUrdf,
  armUrdf,
  armOffset,
  armCommandTopic,
  baseJoints,
  armJoints,
  showBody,
  showWheels,
  showArm,
  publish,
  orbitRef,
  onArmReady,
}: {
  pose: Pose2D | null
  baseUrdf?: string
  armUrdf?: string
  armOffset: [number, number, number]
  armCommandTopic?: string
  baseJoints: JointStates | undefined
  armJoints: JointStates | undefined
  showBody: boolean
  showWheels: boolean
  showArm: boolean
  publish: (topic: string, msgType: string, data: unknown) => void
  orbitRef: MutableRefObject<OrbitControlsImpl | null>
  onArmReady: (ready: boolean) => void
}) {
  const { invalidate } = useThree()
  // Without a map pose the robot is drawn at the map origin (arm control still works without SLAM).
  const p = pose ?? { x: 0, y: 0, yaw: 0 }
  const linkVisible = useCallback(
    (link: string) => (classifyBaseLink(link) === 'wheels' ? showWheels : showBody),
    [showBody, showWheels],
  )
  const armPos = useMemo(
    () => rosToThree({ x: armOffset[0], y: armOffset[1], z: armOffset[2] }),
    [armOffset],
  )
  useEffect(() => invalidate(), [p.x, p.y, p.yaw, invalidate])
  return (
    <group position={rosToThree(p)} rotation={[0, rosYawToThreeY(p.yaw), 0]}>
      {baseUrdf && (
        <RobotModel
          urdfFile={baseUrdf}
          jointStates={baseJoints}
          visible={showBody || showWheels}
          linkVisible={linkVisible}
        />
      )}
      {armUrdf &&
        (armCommandTopic ? (
          <InteractiveArm
            urdfFile={armUrdf}
            liveJointStates={armJoints}
            position={armPos}
            commandTopic={armCommandTopic}
            publish={publish}
            orbitRef={orbitRef}
            onReady={onArmReady}
            visible={showArm}
          />
        ) : (
          <RobotModel urdfFile={armUrdf} jointStates={armJoints} position={armPos} visible={showArm} />
        ))}
    </group>
  )
})

function inMapFrame<T extends { frame_id?: string }>(msg: T | null | undefined, mapFrame?: string): msg is T {
  return Boolean(msg) && (!mapFrame || !msg!.frame_id || msg!.frame_id === mapFrame)
}

/** Scene contents (inside the Canvas). */
function SceneContents(props: MapSceneProps & { mapFrame?: string }) {
  const orbitRef = useRef<OrbitControlsImpl | null>(null)
  const { layers } = props
  const armOffset = props.armOffset ?? DEFAULT_ARM_OFFSET
  return (
    <>
      <ambientLight intensity={0.7} />
      <directionalLight position={[5, 10, 5]} intensity={1} />
      <gridHelper args={[20, 20, '#2a3038', '#1a1f26']} position={[0, LIFT.gps - 0.002, 0]} />

      {layers.gpsMap && props.anchor && (
        <GpsTilesLayer
          anchor={props.anchor}
          aroundX={props.robotPose?.x ?? 0}
          aroundY={props.robotPose?.y ?? 0}
          tileVersion={props.tileVersion}
        />
      )}
      <GridImageLayer msg={props.map} lift={LIFT.map} opacity={1} visible={layers.slamMap} />
      <GridImageLayer
        msg={inMapFrame(props.costmap, props.mapFrame) ? props.costmap : null}
        lift={LIFT.costmap}
        opacity={COSTMAP_OPACITY}
        visible={layers.localCostmap}
      />
      {layers.globalPlan && inMapFrame(props.globalPath, props.mapFrame) && (
        <PathLayer path={props.globalPath} color={SCENE_COLORS.globalPath} width={3} lift={LIFT.plan} />
      )}
      {layers.localPlan && inMapFrame(props.localPath, props.mapFrame) && (
        <PathLayer path={props.localPath} color={SCENE_COLORS.localPath} width={2.5} lift={LIFT.plan + 0.002} />
      )}
      {layers.goal && props.goal && <GoalMarker pose={props.goal} color={SCENE_COLORS.goal} />}
      {props.draftGoal && <GoalMarker pose={props.draftGoal} color={SCENE_COLORS.draft} dashed />}
      {layers.footprint && inMapFrame(props.footprint, props.mapFrame) && (
        <FootprintLayer footprint={props.footprint} pose={props.robotPose} />
      )}
      <PoiLayer pois={props.pois} selectedId={props.selectedPoiId} draft={props.poiDraft} visible={layers.pois} />
      <Suspense fallback={null}>
        <RobotLayer
          pose={props.robotPose}
          baseUrdf={props.baseUrdf}
          armUrdf={props.armUrdf}
          armOffset={armOffset}
          armCommandTopic={props.armCommandTopic}
          baseJoints={props.baseJoints}
          armJoints={props.armJoints}
          showBody={layers.robotBase}
          showWheels={layers.robotWheels}
          showArm={layers.robotArm}
          publish={props.publish}
          orbitRef={orbitRef}
          onArmReady={props.onArmReady}
        />
      </Suspense>
      <OrbitControls ref={orbitRef} makeDefault enableDamping={false} />
      <CameraRig
        orbitRef={orbitRef}
        topView={props.topView}
        goalMode={props.goalMode}
        controllerRef={props.controllerRef}
      />
    </>
  )
}

/** The canvas; memoised so the tab's re-renders on unrelated topics never reach it. */
export const MapScene = memo(function MapScene(props: MapSceneProps & { mapFrame?: string }) {
  return (
    <Canvas
      frameloop="demand"
      camera={{ position: INITIAL_CAMERA, fov: CAMERA_FOV, near: 0.02, far: 1000 }}
      style={{ background: CANVAS_BG, touchAction: 'none' }}
      dpr={[1, 2]}
    >
      <SceneContents {...props} />
    </Canvas>
  )
})

export default MapScene
