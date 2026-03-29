import { useEffect, useRef, useCallback } from 'react'
import { useThree } from '@react-three/fiber'
import type { OrbitControls as OrbitControlsImpl } from 'three-stdlib'
import * as THREE from 'three'
import type { URDFRobot, URDFJoint } from 'urdf-loader'
import { RobotModel } from './RobotModel'
import log from '../logging'

const DRAG_SENSITIVITY = 0.012 // radians per pixel
const PUBLISH_INTERVAL_MS = 50 // throttle to ~20Hz

interface JointStates {
  name?: string[]
  position?: number[]
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

interface ActiveJoint {
  joint: URDFJoint
  name: string
  startAngle: number
  startX: number
  originalMaterials: Map<string, THREE.Material | THREE.Material[]>
}

export function InteractiveArm({ urdfFile, liveJointStates, position, commandTopic, publish, orbitRef, onReady }: Props) {
  const { gl, camera, invalidate } = useThree()
  const cameraRef = useRef<THREE.Camera>(camera)
  cameraRef.current = camera
  const robotRef = useRef<URDFRobot | null>(null)
  const activeJointRef = useRef<ActiveJoint | null>(null)
  const commandedRef = useRef<Record<string, number>>({})
  const lastPublishRef = useRef<number>(0)
  // Guard: do not allow drag/publish until at least one live servo reading has been received.
  // This prevents sending zero positions on startup before real values are known.
  const hasInitialStateRef = useRef<boolean>(false)

  // Sync commanded positions from live state when not dragging.
  // The first update also sets hasInitialStateRef so interaction becomes safe.
  useEffect(() => {
    if (activeJointRef.current) return
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

  const highlightJoint = useCallback((joint: URDFJoint, originalMaterials: Map<string, THREE.Material | THREE.Material[]>) => {
    joint.traverse((obj) => {
      if ((obj as THREE.Mesh).isMesh) {
        const mesh = obj as THREE.Mesh
        originalMaterials.set(mesh.uuid, mesh.material)
        mesh.material = new THREE.MeshStandardMaterial({ color: 0x44aaff })
      }
    })
  }, [])

  const restoreJoint = useCallback((joint: URDFJoint, originalMaterials: Map<string, THREE.Material | THREE.Material[]>) => {
    joint.traverse((obj) => {
      if ((obj as THREE.Mesh).isMesh) {
        const mesh = obj as THREE.Mesh
        const orig = originalMaterials.get(mesh.uuid)
        if (orig !== undefined) mesh.material = orig
      }
    })
  }, [])

  useEffect(() => {
    const canvas = gl.domElement

    const onPointerDown = (e: PointerEvent) => {
      const robot = robotRef.current
      if (!robot) return
      if (e.button !== 0) return
      // Block interaction until live servo positions have been received at least once
      if (!hasInitialStateRef.current) {
        log.debug('[interactive] drag blocked — waiting for initial servo positions')
        return
      }

      // Build raycaster from pointer position
      const rect = canvas.getBoundingClientRect()
      const ndc = new THREE.Vector2(
        ((e.clientX - rect.left) / rect.width) * 2 - 1,
        -((e.clientY - rect.top) / rect.height) * 2 + 1,
      )
      const raycaster = new THREE.Raycaster()
      raycaster.setFromCamera(ndc, cameraRef.current)

      const hits = raycaster.intersectObject(robot, true)
      if (!hits.length) return

      // Walk up to find nearest URDFJoint
      let obj: THREE.Object3D | null = hits[0].object
      let foundJoint: URDFJoint | null = null
      let foundName = ''
      while (obj) {
        const j = obj as unknown as URDFJoint & { isURDFJoint?: boolean }
        if (j.isURDFJoint && j.jointType !== 'fixed') {
          foundJoint = j
          foundName = j.urdfName
          break
        }
        obj = obj.parent
      }
      if (!foundJoint || !foundName) return

      // Prevent OrbitControls from starting orbit when we're dragging a joint
      e.stopPropagation()

      const currentAngle = (commandedRef.current[foundName] ?? 0)
      const originalMaterials = new Map<string, THREE.Material | THREE.Material[]>()
      highlightJoint(foundJoint, originalMaterials)

      activeJointRef.current = {
        joint: foundJoint,
        name: foundName,
        startAngle: currentAngle,
        startX: e.clientX,
        originalMaterials,
      }

      if (orbitRef.current) orbitRef.current.enabled = false
      canvas.style.cursor = 'grabbing'
      log.debug('[interactive] drag start:', foundName, 'at', currentAngle.toFixed(3))
    }

    const onPointerMove = (e: PointerEvent) => {
      const active = activeJointRef.current
      const robot = robotRef.current
      if (!active || !robot) return

      const deltaX = e.clientX - active.startX
      const deltaAngle = deltaX * DRAG_SENSITIVITY

      const joint = active.joint
      let newAngle = active.startAngle + deltaAngle
      if (joint.limit) {
        newAngle = Math.max(joint.limit.lower, Math.min(joint.limit.upper, newAngle))
      }

      robot.setJointValue(active.name, newAngle)
      commandedRef.current[active.name] = newAngle
      invalidate()

      const now = Date.now()
      if (now - lastPublishRef.current >= PUBLISH_INTERVAL_MS) {
        publishNow()
      }
    }

    const onPointerUp = () => {
      const active = activeJointRef.current
      if (!active) return

      restoreJoint(active.joint, active.originalMaterials)
      publishNow()

      activeJointRef.current = null
      if (orbitRef.current) orbitRef.current.enabled = true
      canvas.style.cursor = ''
      log.debug('[interactive] drag end:', active.name, 'at', (commandedRef.current[active.name] ?? 0).toFixed(3))
    }

    // Use capture phase so our handler fires before OrbitControls' listener
    canvas.addEventListener('pointerdown', onPointerDown, { capture: true })
    window.addEventListener('pointermove', onPointerMove)
    window.addEventListener('pointerup', onPointerUp)

    return () => {
      canvas.removeEventListener('pointerdown', onPointerDown, { capture: true } as EventListenerOptions)
      window.removeEventListener('pointermove', onPointerMove)
      window.removeEventListener('pointerup', onPointerUp)
      if (orbitRef.current) orbitRef.current.enabled = true
    }
  }, [gl, invalidate, publishNow, highlightJoint, restoreJoint, orbitRef])

  // While dragging, use commanded positions; otherwise pass live state
  const displayJointStates: JointStates | undefined = activeJointRef.current
    ? {
        name: Object.keys(commandedRef.current),
        position: Object.values(commandedRef.current),
      }
    : liveJointStates

  return (
    <RobotModel
      urdfFile={urdfFile}
      jointStates={displayJointStates}
      position={position}
      onRobotLoaded={(robot) => {
        robotRef.current = robot
      }}
    />
  )
}
