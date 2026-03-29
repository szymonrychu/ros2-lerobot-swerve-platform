import { useEffect, useRef, useCallback, useState } from 'react'
import { useThree } from '@react-three/fiber'
import type { OrbitControls as OrbitControlsImpl } from 'three-stdlib'
import * as THREE from 'three'
import type { URDFRobot, URDFJoint } from 'urdf-loader'
import { RobotModel } from './RobotModel'
import log from '../logging'
import { solveCCDIK } from './ccdIKSolver'

const PUBLISH_INTERVAL_MS = 50 // throttle to ~20Hz
const RING_RADIUS = 0.045 // world-space metres — slightly larger than arm cross-section
const RING_TUBE = 0.006
const COLOR_IDLE = 0x44aaff
const COLOR_HOVER = 0x66ccff
const COLOR_DRAG = 0xffffff
const OPACITY_IDLE = 0.45
const OPACITY_HOVER = 0.85
const OPACITY_DRAG = 1.0
const GHOST_BODY_OPACITY = 0.3
const EE_LINK_NAME = 'gripper_frame_link'
const IK_CHAIN = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll'] as const
const EE_SPHERE_RADIUS = 0.018
const EE_COLOR_IDLE = 0xff6633
const EE_COLOR_HOVER = 0xff8855
const EE_COLOR_DRAG = 0xffaa77
const EE_OPACITY_IDLE = 0.75
const EE_OPACITY_HOVER = 0.95
const EE_OPACITY_DRAG = 1.0

interface JointStates {
  name?: string[]
  position?: number[]
}

interface RingEntry {
  mesh: THREE.Mesh
  joint: URDFJoint
  jointName: string
}

interface ActiveRingDrag {
  type: 'ring'
  entry: RingEntry
  startAngle: number
  startJointValue: number
  ringScreenCenter: { x: number; y: number }
}

interface ActiveIKDrag {
  type: 'ik'
  dragPlane: THREE.Plane
}

type ActiveDrag = ActiveRingDrag | ActiveIKDrag

interface Props {
  urdfFile: string
  liveJointStates?: JointStates
  position?: [number, number, number]
  commandTopic: string
  publish: (topic: string, msgType: string, data: unknown) => void
  orbitRef: React.RefObject<OrbitControlsImpl | null>
  onReady?: (ready: boolean) => void
}

export function InteractiveArm({ urdfFile, liveJointStates, position, commandTopic, publish, orbitRef, onReady }: Props) {
  const { gl, camera, invalidate } = useThree()
  const cameraRef = useRef<THREE.Camera>(camera)
  cameraRef.current = camera

  // Ghost robot ref — shows commanded position, hosts rings
  const ghostRobotRef = useRef<URDFRobot | null>(null)
  // Body meshes of the ghost (from loadMeshCb) — opacity toggled on drag
  const ghostBodyMeshesRef = useRef<THREE.Mesh[]>([])

  const ringsRef = useRef<RingEntry[]>([])
  const ringByUuidRef = useRef<Map<string, RingEntry>>(new Map())
  const activeDragRef = useRef<ActiveDrag | null>(null)
  const hoveredRingRef = useRef<RingEntry | null>(null)
  const eeSphereRef = useRef<THREE.Mesh | null>(null)
  const commandedRef = useRef<Record<string, number>>({})
  const lastPublishRef = useRef<number>(0)
  const hasInitialStateRef = useRef<boolean>(false)

  // Ghost joint states — frozen during drag so live updates don't overwrite setJointValue()
  const [ghostJointStates, setGhostJointStates] = useState<JointStates | undefined>(undefined)

  // Sync commanded positions + ghost states from live when not dragging
  useEffect(() => {
    if (activeDragRef.current) return
    if (!liveJointStates?.name || !liveJointStates?.position) return
    for (let i = 0; i < liveJointStates.name.length; i++) {
      commandedRef.current[liveJointStates.name[i]] = liveJointStates.position[i]
    }
    setGhostJointStates(liveJointStates)
    if (!hasInitialStateRef.current) {
      hasInitialStateRef.current = true
      log.info('[interactive] initial servo positions received — arm control enabled')
      onReady?.(true)
    }
  }, [liveJointStates, onReady])

  const setGhostBodyOpacity = useCallback((opacity: number) => {
    for (const mesh of ghostBodyMeshesRef.current) {
      (mesh.material as THREE.MeshStandardMaterial).opacity = opacity
    }
  }, [])

  const buildJointStateMsg = useCallback(() => {
    const robot = ghostRobotRef.current
    if (!robot) return null
    const names = Object.keys(robot.joints)
    const positions = names.map((n) => commandedRef.current[n] ?? 0)
    return { name: names, position: positions, velocity: [], effort: [] }
  }, [])

  const publishNow = useCallback(() => {
    const msg = buildJointStateMsg()
    if (msg) {
      publish(commandTopic, 'sensor_msgs/JointState', msg)
      lastPublishRef.current = Date.now()
    }
  }, [buildJointStateMsg, publish, commandTopic])

  // Create torus rings on ghost robot's joints
  const createRings = useCallback((robot: URDFRobot) => {
    for (const entry of ringsRef.current) {
      entry.mesh.parent?.remove(entry.mesh)
      entry.mesh.geometry.dispose()
      ;(entry.mesh.material as THREE.Material).dispose()
    }
    ringsRef.current = []
    ringByUuidRef.current.clear()

    const geom = new THREE.TorusGeometry(RING_RADIUS, RING_TUBE, 32, 48)

    for (const [name, joint] of Object.entries(robot.joints)) {
      const j = joint as unknown as URDFJoint
      if (j.jointType === 'fixed') continue

      const mat = new THREE.MeshBasicMaterial({
        color: COLOR_IDLE,
        transparent: true,
        opacity: OPACITY_IDLE,
        depthTest: false,
        side: THREE.DoubleSide,
      })
      const mesh = new THREE.Mesh(geom, mat)
      mesh.renderOrder = 999
      j.add(mesh)

      const entry: RingEntry = { mesh, joint: j, jointName: name }
      ringsRef.current.push(entry)
      ringByUuidRef.current.set(mesh.uuid, entry)
    }

    log.info(`[interactive] created ${ringsRef.current.length} joint rings`)
  }, [])

  const createEESphere = useCallback((robot: URDFRobot) => {
    const eeLink = (robot as any).frames?.[EE_LINK_NAME] ?? robot.links[EE_LINK_NAME]
    if (!eeLink) {
      log.warn('[interactive] end-effector link not found:', EE_LINK_NAME)
      return
    }
    const geom = new THREE.SphereGeometry(EE_SPHERE_RADIUS, 16, 16)
    const mat = new THREE.MeshBasicMaterial({
      color: EE_COLOR_IDLE,
      transparent: true,
      opacity: EE_OPACITY_IDLE,
      depthTest: false,
    })
    const sphere = new THREE.Mesh(geom, mat)
    sphere.renderOrder = 1000
    eeLink.add(sphere)
    eeSphereRef.current = sphere
    log.info('[interactive] EE sphere created at', EE_LINK_NAME)
  }, [])

  const setRingAppearance = useCallback((entry: RingEntry, color: number, opacity: number) => {
    const mat = entry.mesh.material as THREE.MeshBasicMaterial
    mat.color.setHex(color)
    mat.opacity = opacity
  }, [])

  const getRingScreenCenter = useCallback((entry: RingEntry, canvasRect: DOMRect) => {
    const worldPos = entry.joint.getWorldPosition(new THREE.Vector3())
    const ndc = worldPos.project(cameraRef.current)
    return {
      x: ((ndc.x + 1) / 2) * canvasRect.width + canvasRect.left,
      y: ((-ndc.y + 1) / 2) * canvasRect.height + canvasRect.top,
    }
  }, [])

  useEffect(() => {
    const canvas = gl.domElement

    type HitResult =
      | { kind: 'ring'; entry: RingEntry }
      | { kind: 'ee' }
      | null

    const hitInteractables = (e: PointerEvent): HitResult => {
      const rect = canvas.getBoundingClientRect()
      const ndc = new THREE.Vector2(
        ((e.clientX - rect.left) / rect.width) * 2 - 1,
        -((e.clientY - rect.top) / rect.height) * 2 + 1,
      )
      const raycaster = new THREE.Raycaster()
      raycaster.setFromCamera(ndc, cameraRef.current)

      const targets: THREE.Object3D[] = [...ringsRef.current.map((r) => r.mesh)]
      if (eeSphereRef.current) targets.push(eeSphereRef.current)

      const hits = raycaster.intersectObjects(targets, false)
      if (!hits.length) return null

      const hit = hits[0].object
      if (eeSphereRef.current && hit.uuid === eeSphereRef.current.uuid) return { kind: 'ee' }
      const entry = ringByUuidRef.current.get(hit.uuid)
      return entry ? { kind: 'ring', entry } : null
    }

    const onPointerDown = (e: PointerEvent) => {
      if (e.button !== 0) return

      const hit = hitInteractables(e)
      if (!hit) return

      e.stopPropagation()

      if (hit.kind === 'ring') {
        if (!hasInitialStateRef.current) return
        const { entry } = hit
        const rect = canvas.getBoundingClientRect()
        const center = getRingScreenCenter(entry, rect)
        const startAngle = Math.atan2(e.clientY - center.y, e.clientX - center.x)
        activeDragRef.current = {
          type: 'ring',
          entry,
          startAngle,
          startJointValue: commandedRef.current[entry.jointName] ?? 0,
          ringScreenCenter: center,
        }
        setRingAppearance(entry, COLOR_DRAG, OPACITY_DRAG)
        setGhostBodyOpacity(GHOST_BODY_OPACITY)
        if (orbitRef.current) orbitRef.current.enabled = false
        canvas.style.cursor = 'grabbing'
        invalidate()
        log.debug('[interactive] ring drag start:', entry.jointName)
      } else if (hit.kind === 'ee') {
        if (!hasInitialStateRef.current) return
        const sphereWorldPos = new THREE.Vector3()
        eeSphereRef.current!.getWorldPosition(sphereWorldPos)
        const camForward = new THREE.Vector3(0, 0, -1).applyQuaternion(cameraRef.current.quaternion)
        const dragPlane = new THREE.Plane().setFromNormalAndCoplanarPoint(camForward, sphereWorldPos)
        activeDragRef.current = { type: 'ik', dragPlane }
        const mat = eeSphereRef.current!.material as THREE.MeshBasicMaterial
        mat.color.setHex(EE_COLOR_DRAG)
        mat.opacity = EE_OPACITY_DRAG
        setGhostBodyOpacity(GHOST_BODY_OPACITY)
        if (orbitRef.current) orbitRef.current.enabled = false
        canvas.style.cursor = 'grabbing'
        invalidate()
        log.debug('[interactive] IK drag start')
      }
    }

    const onPointerMove = (e: PointerEvent) => {
      const drag = activeDragRef.current
      const robot = ghostRobotRef.current

      if (drag && robot) {
        if (drag.type === 'ring') {
          const { ringScreenCenter, startAngle, startJointValue, entry } = drag
          const currentAngle = Math.atan2(e.clientY - ringScreenCenter.y, e.clientX - ringScreenCenter.x)
          let delta = currentAngle - startAngle
          delta = ((delta + Math.PI) % (2 * Math.PI)) - Math.PI

          const joint = entry.joint
          let newValue = startJointValue + delta
          if (joint.limit) {
            newValue = Math.max(joint.limit.lower, Math.min(joint.limit.upper, newValue))
          }
          robot.setJointValue(entry.jointName, newValue)
          commandedRef.current[entry.jointName] = newValue
          invalidate()

          if (Date.now() - lastPublishRef.current >= PUBLISH_INTERVAL_MS) {
            publishNow()
          }
        } else if (drag.type === 'ik') {
          const rect = canvas.getBoundingClientRect()
          const ndc = new THREE.Vector2(
            ((e.clientX - rect.left) / rect.width) * 2 - 1,
            -((e.clientY - rect.top) / rect.height) * 2 + 1,
          )
          const raycaster = new THREE.Raycaster()
          raycaster.setFromCamera(ndc, cameraRef.current)
          const intersection = new THREE.Vector3()
          if (raycaster.ray.intersectPlane(drag.dragPlane, intersection)) {
            const eeLink = (robot as any).frames?.[EE_LINK_NAME] ?? robot.links[EE_LINK_NAME]
            if (eeLink) {
              solveCCDIK(robot, intersection, {
                chainJointNames: [...IK_CHAIN],
                endEffector: eeLink,
                maxIterations: 10,
                tolerance: 0.001,
              })
              for (const name of IK_CHAIN) {
                const joint = robot.joints[name] as unknown as any
                if (joint) commandedRef.current[name] = (joint.jointValue as number[])[0] ?? 0
              }
              invalidate()
              if (Date.now() - lastPublishRef.current >= PUBLISH_INTERVAL_MS) publishNow()
            }
          }
        }
        return
      }

      // Hover detection
      const hit = hitInteractables(e)
      const prevRing = hoveredRingRef.current
      if (hit?.kind === 'ring') {
        if (hit.entry !== prevRing) {
          if (prevRing) setRingAppearance(prevRing, COLOR_IDLE, OPACITY_IDLE)
          setRingAppearance(hit.entry, COLOR_HOVER, OPACITY_HOVER)
          hoveredRingRef.current = hit.entry
          canvas.style.cursor = hasInitialStateRef.current ? 'grab' : 'not-allowed'
          invalidate()
        }
      } else {
        if (prevRing) {
          setRingAppearance(prevRing, COLOR_IDLE, OPACITY_IDLE)
          hoveredRingRef.current = null
          invalidate()
        }
        if (eeSphereRef.current) {
          const mat = eeSphereRef.current.material as THREE.MeshBasicMaterial
          if (hit?.kind === 'ee') {
            mat.color.setHex(EE_COLOR_HOVER)
            mat.opacity = EE_OPACITY_HOVER
            canvas.style.cursor = hasInitialStateRef.current ? 'grab' : 'not-allowed'
            invalidate()
          } else if (mat.color.getHex() === EE_COLOR_HOVER) {
            mat.color.setHex(EE_COLOR_IDLE)
            mat.opacity = EE_OPACITY_IDLE
            canvas.style.cursor = ''
            invalidate()
          } else {
            canvas.style.cursor = ''
          }
        } else {
          canvas.style.cursor = ''
        }
      }
    }

    const onPointerUp = () => {
      const drag = activeDragRef.current
      if (!drag) return

      if (drag.type === 'ring') {
        setRingAppearance(drag.entry, COLOR_HOVER, OPACITY_HOVER)
      } else if (drag.type === 'ik' && eeSphereRef.current) {
        const mat = eeSphereRef.current.material as THREE.MeshBasicMaterial
        mat.color.setHex(EE_COLOR_IDLE)
        mat.opacity = EE_OPACITY_IDLE
      }

      setGhostBodyOpacity(0)
      publishNow()
      activeDragRef.current = null
      if (orbitRef.current) orbitRef.current.enabled = true
      canvas.style.cursor = ''
      invalidate()
      log.debug('[interactive] drag end')
    }

    canvas.addEventListener('pointerdown', onPointerDown, { capture: true })
    window.addEventListener('pointermove', onPointerMove)
    window.addEventListener('pointerup', onPointerUp)

    return () => {
      canvas.removeEventListener('pointerdown', onPointerDown, { capture: true } as EventListenerOptions)
      window.removeEventListener('pointermove', onPointerMove)
      window.removeEventListener('pointerup', onPointerUp)
      if (orbitRef.current) orbitRef.current.enabled = true
    }
  }, [gl, invalidate, publishNow, setRingAppearance, getRingScreenCenter, setGhostBodyOpacity, orbitRef, eeSphereRef])

  return (
    <>
      {/* Solid arm — always tracks live servo position */}
      <RobotModel
        urdfFile={urdfFile}
        jointStates={liveJointStates}
        position={position}
      />
      {/* Ghost arm — always in scene; body opacity 0 normally, ghost during drag; hosts rings */}
      <RobotModel
        urdfFile={urdfFile}
        jointStates={ghostJointStates}
        position={position}
        ghost
        onBodyMeshesLoaded={(meshes) => {
          ghostBodyMeshesRef.current = meshes
        }}
        onRobotLoaded={(robot) => {
          ghostRobotRef.current = robot
          if (robot) {
            createRings(robot)
            createEESphere(robot)
          } else {
            if (eeSphereRef.current) {
              eeSphereRef.current.geometry.dispose()
              ;(eeSphereRef.current.material as THREE.Material).dispose()
              eeSphereRef.current = null
            }
          }
        }}
      />
    </>
  )
}
