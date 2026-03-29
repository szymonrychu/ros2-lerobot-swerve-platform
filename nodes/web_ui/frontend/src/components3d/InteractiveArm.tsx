import { useEffect, useRef, useCallback } from 'react'
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
  startAngle: number          // screen-space angle at drag start
  startJointValue: number     // joint angle at drag start
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

  const robotRef = useRef<URDFRobot | null>(null)
  const ringsRef = useRef<RingEntry[]>([])
  const ringByUuidRef = useRef<Map<string, RingEntry>>(new Map())
  const activeDragRef = useRef<ActiveDrag | null>(null)
  const hoveredRingRef = useRef<RingEntry | null>(null)
  const commandedRef = useRef<Record<string, number>>({})
  const lastPublishRef = useRef<number>(0)
  const hasInitialStateRef = useRef<boolean>(false)

  // Sync commanded positions from live state when not dragging
  useEffect(() => {
    if (activeDragRef.current) return
    if (!liveJointStates?.name || !liveJointStates?.position) return
    for (let i = 0; i < liveJointStates.name.length; i++) {
      commandedRef.current[liveJointStates.name[i]] = liveJointStates.position[i]
    }
    if (!hasInitialStateRef.current) {
      hasInitialStateRef.current = true
      log.info('[interactive] initial servo positions received — arm control enabled')
      onReady?.(true)
    }
  }, [liveJointStates, onReady])

  const buildJointStateMsg = useCallback(() => {
    const robot = robotRef.current
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

  // Create torus rings for every non-fixed joint and attach as children
  const createRings = useCallback((robot: URDFRobot) => {
    // Remove any old rings first
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
      mesh.renderOrder = 999 // draw on top
      // Torus lies in XY plane → ring perpendicular to local Z (the joint rotation axis)
      // No extra rotation needed since all joints rotate around local Z
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

  // Compute screen-space center of a ring given its joint's world position
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

      // Prevent OrbitControls from consuming this event
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
      if (orbitRef.current) orbitRef.current.enabled = false
      canvas.style.cursor = 'grabbing'
      log.debug('[interactive] drag start:', entry.jointName, 'at', (commandedRef.current[entry.jointName] ?? 0).toFixed(3))
    }

    const onPointerMove = (e: PointerEvent) => {
      const drag = activeDragRef.current
      const robot = robotRef.current

      if (drag && robot) {
        // Arc-based drag: compute angle of pointer relative to ring center
        const { ringScreenCenter, startAngle, startJointValue, entry } = drag
        const currentAngle = Math.atan2(e.clientY - ringScreenCenter.y, e.clientX - ringScreenCenter.x)
        let delta = currentAngle - startAngle
        // Normalize to [-π, π] to handle wrap-around
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

      // Hover detection (no active drag)
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

      setRingAppearance(drag.entry, COLOR_HOVER, OPACITY_HOVER) // restore to hover since cursor is still over it
      publishNow()

      activeDragRef.current = null
      if (orbitRef.current) orbitRef.current.enabled = true
      canvas.style.cursor = ''
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
  }, [gl, invalidate, publishNow, setRingAppearance, getRingScreenCenter, orbitRef])

  return (
    <RobotModel
      urdfFile={urdfFile}
      // Always pass live joint states so the arm tracks real servo positions.
      // During drag, visual updates happen directly via robot.setJointValue() above.
      jointStates={liveJointStates}
      position={position}
      onRobotLoaded={(robot) => {
        robotRef.current = robot
        if (robot) createRings(robot)
      }}
    />
  )
}
