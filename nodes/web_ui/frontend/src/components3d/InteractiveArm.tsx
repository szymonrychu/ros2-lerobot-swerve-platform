import { useEffect, useRef, useCallback, useState } from 'react'
import { useThree } from '@react-three/fiber'
import type { OrbitControls as OrbitControlsImpl } from 'three-stdlib'
import * as THREE from 'three'
import type { URDFRobot, URDFJoint } from 'urdf-loader'
import { RobotModel } from './RobotModel'
import log from '../logging'

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

interface JointStates {
  name?: string[]
  position?: number[]
}

interface RingEntry {
  mesh: THREE.Mesh
  joint: URDFJoint
  jointName: string
}

interface ActiveDrag {
  entry: RingEntry
  startAngle: number
  startJointValue: number
  ringScreenCenter: { x: number; y: number }
}

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

    const hitRings = (e: PointerEvent): RingEntry | null => {
      const rect = canvas.getBoundingClientRect()
      const ndc = new THREE.Vector2(
        ((e.clientX - rect.left) / rect.width) * 2 - 1,
        -((e.clientY - rect.top) / rect.height) * 2 + 1,
      )
      const raycaster = new THREE.Raycaster()
      raycaster.setFromCamera(ndc, cameraRef.current)
      const ringMeshes = ringsRef.current.map((r) => r.mesh)
      const hits = raycaster.intersectObjects(ringMeshes, false)
      if (!hits.length) return null
      return ringByUuidRef.current.get(hits[0].object.uuid) ?? null
    }

    const onPointerDown = (e: PointerEvent) => {
      if (e.button !== 0) return
      if (!hasInitialStateRef.current) return

      const entry = hitRings(e)
      if (!entry) return

      e.stopPropagation()

      const rect = canvas.getBoundingClientRect()
      const center = getRingScreenCenter(entry, rect)
      const startAngle = Math.atan2(e.clientY - center.y, e.clientX - center.x)

      activeDragRef.current = {
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
      log.debug('[interactive] drag start:', entry.jointName, 'at', (commandedRef.current[entry.jointName] ?? 0).toFixed(3))
    }

    const onPointerMove = (e: PointerEvent) => {
      const drag = activeDragRef.current
      const robot = ghostRobotRef.current

      if (drag && robot) {
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
        return
      }

      // Hover detection
      const hit = hitRings(e)
      const prev = hoveredRingRef.current

      if (hit !== prev) {
        if (prev) setRingAppearance(prev, COLOR_IDLE, OPACITY_IDLE)
        if (hit) {
          setRingAppearance(hit, COLOR_HOVER, OPACITY_HOVER)
          canvas.style.cursor = hasInitialStateRef.current ? 'grab' : 'not-allowed'
        } else {
          canvas.style.cursor = ''
        }
        hoveredRingRef.current = hit
        invalidate()
      }
    }

    const onPointerUp = () => {
      const drag = activeDragRef.current
      if (!drag) return

      setRingAppearance(drag.entry, COLOR_HOVER, OPACITY_HOVER)
      setGhostBodyOpacity(0)
      publishNow()

      activeDragRef.current = null
      if (orbitRef.current) orbitRef.current.enabled = true
      canvas.style.cursor = ''
      invalidate()
      log.debug('[interactive] drag end:', drag.entry.jointName, 'at', (commandedRef.current[drag.entry.jointName] ?? 0).toFixed(3))
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
  }, [gl, invalidate, publishNow, setRingAppearance, getRingScreenCenter, setGhostBodyOpacity, orbitRef])

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
          if (robot) createRings(robot)
        }}
      />
    </>
  )
}
