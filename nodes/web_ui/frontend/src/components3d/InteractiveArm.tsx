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
const RING_TUBE = 0.003
const COLOR_IDLE = 0x44aaff
const COLOR_HOVER = 0x66ccff
const COLOR_DRAG = 0xffffff
const OPACITY_IDLE = 0.45
const OPACITY_HOVER = 0.85
const OPACITY_DRAG = 1.0
const GHOST_BODY_OPACITY = 0.3
const EE_SPHERE_RADIUS = 0.012
const EE_COLOR_IDLE = 0xff6633
const EE_COLOR_HOVER = 0xff8855
const EE_COLOR_DRAG = 0xffaa77
const EE_OPACITY_IDLE = 0.75
const EE_OPACITY_HOVER = 0.95
const EE_OPACITY_DRAG = 1.0

const ARROW_SHAFT_RADIUS = 0.0025
const ARROW_HEAD_RADIUS = 0.005
const ARROW_LENGTH = 0.04
const ARROW_HEAD_LENGTH = 0.01
const ARROW_OPACITY_IDLE = 0.60
const ARROW_OPACITY_HOVER = 0.90
const ARROW_OPACITY_DRAG = 1.0
const ARROW_COLORS: Record<string, number> = { x: 0xff3333, y: 0x33cc33, z: 0x3399ff }

// Ordered IK chain: each entry is the joint name and the child link it drives.
// The sphere for joint[i] is placed at its child link; dragging it solves IK
// using joints[0..i] as the chain and child link as the end-effector.
const IK_JOINTS: Array<{ joint: string; childLink: string }> = [
  { joint: 'shoulder_pan', childLink: 'shoulder_link' },
  { joint: 'shoulder_lift', childLink: 'upper_arm_link' },
  { joint: 'elbow_flex', childLink: 'lower_arm_link' },
  { joint: 'wrist_flex', childLink: 'wrist_link' },
  { joint: 'wrist_roll', childLink: 'gripper_link' },
  { joint: 'gripper', childLink: 'gripper_frame_link' },
]

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
  axisSign: number
}

interface ActiveIKDrag {
  type: 'ik'
  dragPlane: THREE.Plane
  /** Index into IK_JOINTS for the grabbed sphere — chain is IK_JOINTS[0..ikJointIdx] */
  ikJointIdx: number
}

interface ActiveAxisDrag {
  type: 'axis'
  worldAxis: THREE.Vector3
  axisOrigin: THREE.Vector3
  ikJointIdx: number
}

type ActiveDrag = ActiveRingDrag | ActiveIKDrag | ActiveAxisDrag

interface AxisArrowEntry {
  shaftMesh: THREE.Mesh
  headMesh: THREE.Mesh
  ikJointIdx: number
  worldAxis: THREE.Vector3
  axisKey: string
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

/** Finds t such that lineOrigin + t*lineDir is closest to the given ray. */
function closestPointOnLine(
  lineOrigin: THREE.Vector3,
  lineDir: THREE.Vector3,
  ray: THREE.Ray,
): THREE.Vector3 {
  const w = new THREE.Vector3().subVectors(lineOrigin, ray.origin)
  const b = lineDir.dot(ray.direction)
  const denom = 1 - b * b
  if (Math.abs(denom) < 1e-8) return lineOrigin.clone()
  const c = lineDir.dot(w)
  const f = ray.direction.dot(w)
  const t = (b * f - c) / denom
  return new THREE.Vector3().copy(lineOrigin).addScaledVector(lineDir, t)
}

export function InteractiveArm({ urdfFile, liveJointStates, position, commandTopic, publish, orbitRef, onReady }: Props) {
  const { gl, camera, scene, invalidate } = useThree()
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
  // Map from joint name → sphere mesh (one per IK_JOINTS entry)
  const jointSpheresRef = useRef<Map<string, THREE.Mesh>>(new Map())
  // Map from sphere uuid → IK_JOINTS index (for hit testing)
  const sphereByUuidRef = useRef<Map<string, number>>(new Map())
  const commandedRef = useRef<Record<string, number>>({})
  const lastPublishRef = useRef<number>(0)
  const hasInitialStateRef = useRef<boolean>(false)

  const axisArrowsRef = useRef<AxisArrowEntry[]>([])
  const arrowMeshByUuidRef = useRef<Map<string, AxisArrowEntry>>(new Map())

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
    const robot = ghostRobotRef.current
    if (robot) updateArrowPositions(robot)
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

  const createJointSpheres = useCallback((robot: URDFRobot) => {
    // Dispose old spheres
    for (const sphere of jointSpheresRef.current.values()) {
      sphere.parent?.remove(sphere)
      sphere.geometry.dispose()
      ;(sphere.material as THREE.Material).dispose()
    }
    jointSpheresRef.current.clear()
    sphereByUuidRef.current.clear()

    const geom = new THREE.SphereGeometry(EE_SPHERE_RADIUS, 16, 16)

    for (let idx = 0; idx < IK_JOINTS.length; idx++) {
      const { joint: jointName, childLink: linkName } = IK_JOINTS[idx]
      const link = (robot as any).frames?.[linkName] ?? robot.links[linkName]
      if (!link) {
        log.warn('[interactive] link not found for sphere:', linkName)
        continue
      }
      const mat = new THREE.MeshBasicMaterial({
        color: EE_COLOR_IDLE,
        transparent: true,
        opacity: EE_OPACITY_IDLE,
        depthTest: false,
      })
      const sphere = new THREE.Mesh(geom, mat)
      sphere.renderOrder = 1000
      link.add(sphere)
      jointSpheresRef.current.set(jointName, sphere)
      sphereByUuidRef.current.set(sphere.uuid, idx)
    }

    log.info(`[interactive] created ${jointSpheresRef.current.size} joint spheres`)
  }, [])

  const updateArrowPositions = useCallback((robot: URDFRobot) => {
    robot.updateMatrixWorld(true)
    for (const entry of axisArrowsRef.current) {
      const { childLink: linkName } = IK_JOINTS[entry.ikJointIdx]
      const link = (robot as any).frames?.[linkName] ?? robot.links[linkName]
      if (!link) continue
      const wp = link.getWorldPosition(new THREE.Vector3())
      entry.shaftMesh.parent?.position.copy(wp)
    }
  }, [])

  const createAxisArrows = useCallback((robot: URDFRobot, threeScene: THREE.Scene) => {
    // Dispose previous arrows
    const seenGroups = new Set<THREE.Object3D>()
    for (const entry of axisArrowsRef.current) {
      const group = entry.shaftMesh.parent
      if (group && !seenGroups.has(group)) {
        seenGroups.add(group)
        threeScene.remove(group)
      }
      entry.shaftMesh.geometry.dispose()
      entry.headMesh.geometry.dispose()
      ;(entry.shaftMesh.material as THREE.Material).dispose()
      ;(entry.headMesh.material as THREE.Material).dispose()
    }
    axisArrowsRef.current = []
    arrowMeshByUuidRef.current.clear()

    robot.updateMatrixWorld(true)

    // J2..J6 = IK_JOINTS indices 1..5
    for (let idx = 1; idx <= 5; idx++) {
      const { childLink: linkName } = IK_JOINTS[idx]
      const link = (robot as any).frames?.[linkName] ?? robot.links[linkName]
      if (!link) continue

      const wp = link.getWorldPosition(new THREE.Vector3())

      const axes: Array<{ key: string; worldAxis: THREE.Vector3 }> = [
        { key: 'x', worldAxis: new THREE.Vector3(1, 0, 0) },
        { key: 'y', worldAxis: new THREE.Vector3(0, 1, 0) },
        { key: 'z', worldAxis: new THREE.Vector3(0, 0, 1) },
      ]

      for (const { key, worldAxis } of axes) {
        const color = ARROW_COLORS[key]

        const shaftGeom = new THREE.CylinderGeometry(ARROW_SHAFT_RADIUS, ARROW_SHAFT_RADIUS, ARROW_LENGTH, 8)
        const headGeom = new THREE.ConeGeometry(ARROW_HEAD_RADIUS, ARROW_HEAD_LENGTH, 8)

        const shaftMat = new THREE.MeshBasicMaterial({ color, transparent: true, opacity: ARROW_OPACITY_IDLE, depthTest: false })
        const headMat = new THREE.MeshBasicMaterial({ color, transparent: true, opacity: ARROW_OPACITY_IDLE, depthTest: false })

        const shaftMesh = new THREE.Mesh(shaftGeom, shaftMat)
        const headMesh = new THREE.Mesh(headGeom, headMat)
        shaftMesh.renderOrder = 998
        headMesh.renderOrder = 998

        if (key === 'x') {
          shaftMesh.rotation.z = -Math.PI / 2
          shaftMesh.position.set(ARROW_LENGTH / 2, 0, 0)
          headMesh.rotation.z = -Math.PI / 2
          headMesh.position.set(ARROW_LENGTH + ARROW_HEAD_LENGTH / 2, 0, 0)
        } else if (key === 'y') {
          shaftMesh.position.set(0, ARROW_LENGTH / 2, 0)
          headMesh.position.set(0, ARROW_LENGTH + ARROW_HEAD_LENGTH / 2, 0)
        } else {
          // z
          shaftMesh.rotation.x = Math.PI / 2
          shaftMesh.position.set(0, 0, ARROW_LENGTH / 2)
          headMesh.rotation.x = Math.PI / 2
          headMesh.position.set(0, 0, ARROW_LENGTH + ARROW_HEAD_LENGTH / 2)
        }

        const group = new THREE.Group()
        group.add(shaftMesh)
        group.add(headMesh)
        group.position.copy(wp)
        threeScene.add(group)

        const entry: AxisArrowEntry = { shaftMesh, headMesh, ikJointIdx: idx, worldAxis, axisKey: key }
        axisArrowsRef.current.push(entry)
        arrowMeshByUuidRef.current.set(shaftMesh.uuid, entry)
        arrowMeshByUuidRef.current.set(headMesh.uuid, entry)
      }
    }

    log.info(`[interactive] created ${axisArrowsRef.current.length} axis arrows`)
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
      | { kind: 'ik'; ikJointIdx: number }
      | { kind: 'axis'; ikJointIdx: number; worldAxis: THREE.Vector3; axisKey: string }
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
      for (const sphere of jointSpheresRef.current.values()) targets.push(sphere)
      for (const entry of axisArrowsRef.current) {
        targets.push(entry.shaftMesh, entry.headMesh)
      }

      const hits = raycaster.intersectObjects(targets, false)
      if (!hits.length) return null

      const hit = hits[0].object
      const ikIdx = sphereByUuidRef.current.get(hit.uuid)
      if (ikIdx !== undefined) return { kind: 'ik', ikJointIdx: ikIdx }
      const ringEntry = ringByUuidRef.current.get(hit.uuid)
      if (ringEntry) return { kind: 'ring', entry: ringEntry }
      const arrowEntry = arrowMeshByUuidRef.current.get(hit.uuid)
      if (arrowEntry) return { kind: 'axis', ikJointIdx: arrowEntry.ikJointIdx, worldAxis: arrowEntry.worldAxis, axisKey: arrowEntry.axisKey }
      return null
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

        // Compute axis sign: if joint axis points toward camera, flip delta
        const jointWorldQuat = new THREE.Quaternion()
        entry.joint.getWorldQuaternion(jointWorldQuat)
        const worldAxis = new THREE.Vector3(0, 0, 1).applyQuaternion(jointWorldQuat).normalize()
        const camDir = new THREE.Vector3().subVectors(
          entry.joint.getWorldPosition(new THREE.Vector3()),
          cameraRef.current.position,
        ).normalize()
        const axisSign = worldAxis.dot(camDir) >= 0 ? 1 : -1

        activeDragRef.current = {
          type: 'ring',
          entry,
          startAngle,
          startJointValue: commandedRef.current[entry.jointName] ?? 0,
          ringScreenCenter: center,
          axisSign,
        }
        setRingAppearance(entry, COLOR_DRAG, OPACITY_DRAG)
        setGhostBodyOpacity(GHOST_BODY_OPACITY)
        if (orbitRef.current) orbitRef.current.enabled = false
        canvas.style.cursor = 'grabbing'
        invalidate()
        log.debug('[interactive] ring drag start:', entry.jointName)
      } else if (hit.kind === 'ik') {
        if (!hasInitialStateRef.current) return
        const { ikJointIdx } = hit
        const jointName = IK_JOINTS[ikJointIdx].joint
        const sphere = jointSpheresRef.current.get(jointName)
        if (!sphere) return
        const sphereWorldPos = new THREE.Vector3()
        sphere.getWorldPosition(sphereWorldPos)
        const camForward = new THREE.Vector3(0, 0, -1).applyQuaternion(cameraRef.current.quaternion)
        const dragPlane = new THREE.Plane().setFromNormalAndCoplanarPoint(camForward, sphereWorldPos)
        activeDragRef.current = { type: 'ik', dragPlane, ikJointIdx }
        const mat = sphere.material as THREE.MeshBasicMaterial
        mat.color.setHex(EE_COLOR_DRAG)
        mat.opacity = EE_OPACITY_DRAG
        setGhostBodyOpacity(GHOST_BODY_OPACITY)
        if (orbitRef.current) orbitRef.current.enabled = false
        canvas.style.cursor = 'grabbing'
        invalidate()
        log.debug('[interactive] IK drag start on', jointName)
      } else if (hit.kind === 'axis') {
        if (!hasInitialStateRef.current) return
        const { ikJointIdx, worldAxis } = hit
        const chainEntry = IK_JOINTS[ikJointIdx]
        const linkName = chainEntry.childLink
        const robot = ghostRobotRef.current
        if (!robot) return
        const link = (robot as any).frames?.[linkName] ?? robot.links[linkName]
        if (!link) return
        const axisOrigin = link.getWorldPosition(new THREE.Vector3())
        activeDragRef.current = { type: 'axis', worldAxis: worldAxis.clone(), axisOrigin, ikJointIdx }
        for (const entry of axisArrowsRef.current) {
          if (entry.ikJointIdx === ikJointIdx && entry.axisKey === hit.axisKey) {
            ;(entry.shaftMesh.material as THREE.MeshBasicMaterial).opacity = ARROW_OPACITY_DRAG
            ;(entry.headMesh.material as THREE.MeshBasicMaterial).opacity = ARROW_OPACITY_DRAG
          }
        }
        setGhostBodyOpacity(GHOST_BODY_OPACITY)
        if (orbitRef.current) orbitRef.current.enabled = false
        canvas.style.cursor = 'grabbing'
        invalidate()
      }
    }

    const onPointerMove = (e: PointerEvent) => {
      const drag = activeDragRef.current
      const robot = ghostRobotRef.current

      if (drag && robot) {
        if (drag.type === 'ring') {
          const { ringScreenCenter, startAngle, startJointValue, entry, axisSign } = drag
          const currentAngle = Math.atan2(e.clientY - ringScreenCenter.y, e.clientX - ringScreenCenter.x)
          let delta = axisSign * (currentAngle - startAngle)
          delta = ((delta + Math.PI) % (2 * Math.PI)) - Math.PI

          const joint = entry.joint
          let newValue = startJointValue + delta
          if (joint.limit) {
            newValue = Math.max(joint.limit.lower, Math.min(joint.limit.upper, newValue))
          }
          robot.setJointValue(entry.jointName, newValue)
          commandedRef.current[entry.jointName] = newValue
          updateArrowPositions(robot)
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
            // Build sub-chain: joints 0..ikJointIdx (inclusive)
            const chainEntry = IK_JOINTS[drag.ikJointIdx]
            const chainJoints = IK_JOINTS.slice(0, drag.ikJointIdx + 1).map((e) => e.joint)
            const linkName = chainEntry.childLink
            const eeLink = (robot as any).frames?.[linkName] ?? robot.links[linkName]
            if (eeLink) {
              solveCCDIK(robot, intersection, {
                chainJointNames: chainJoints,
                endEffector: eeLink,
                maxIterations: 10,
                tolerance: 0.001,
              })
              for (const name of chainJoints) {
                const joint = robot.joints[name] as unknown as any
                if (joint) commandedRef.current[name] = (joint.jointValue as number[])[0] ?? 0
              }
              updateArrowPositions(robot)
              invalidate()
              if (Date.now() - lastPublishRef.current >= PUBLISH_INTERVAL_MS) publishNow()
            }
          }
        } else if (drag.type === 'axis') {
          const rect = canvas.getBoundingClientRect()
          const ndc = new THREE.Vector2(
            ((e.clientX - rect.left) / rect.width) * 2 - 1,
            -((e.clientY - rect.top) / rect.height) * 2 + 1,
          )
          const raycaster = new THREE.Raycaster()
          raycaster.setFromCamera(ndc, cameraRef.current)
          const target = closestPointOnLine(drag.axisOrigin, drag.worldAxis, raycaster.ray)
          const chainEntry = IK_JOINTS[drag.ikJointIdx]
          const chainJoints = IK_JOINTS.slice(0, drag.ikJointIdx + 1).map((e) => e.joint)
          const linkName = chainEntry.childLink
          const eeLink = (robot as any).frames?.[linkName] ?? robot.links[linkName]
          if (eeLink) {
            solveCCDIK(robot, target, {
              chainJointNames: chainJoints,
              endEffector: eeLink,
              maxIterations: 10,
              tolerance: 0.001,
            })
            for (const name of chainJoints) {
              const joint = robot.joints[name] as unknown as any
              if (joint) commandedRef.current[name] = (joint.jointValue as number[])[0] ?? 0
            }
            updateArrowPositions(robot)
            invalidate()
            if (Date.now() - lastPublishRef.current >= PUBLISH_INTERVAL_MS) publishNow()
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
        // Sphere hover: brighten hovered sphere, restore all others
        const hoveredIkIdx = hit?.kind === 'ik' ? hit.ikJointIdx : -1
        for (let idx = 0; idx < IK_JOINTS.length; idx++) {
          const sphere = jointSpheresRef.current.get(IK_JOINTS[idx].joint)
          if (!sphere) continue
          const mat = sphere.material as THREE.MeshBasicMaterial
          if (idx === hoveredIkIdx) {
            mat.color.setHex(EE_COLOR_HOVER)
            mat.opacity = EE_OPACITY_HOVER
            canvas.style.cursor = hasInitialStateRef.current ? 'grab' : 'not-allowed'
            invalidate()
          } else if (mat.color.getHex() === EE_COLOR_HOVER) {
            mat.color.setHex(EE_COLOR_IDLE)
            mat.opacity = EE_OPACITY_IDLE
            invalidate()
          }
        }
        if (hoveredIkIdx === -1 && hit?.kind !== 'axis') canvas.style.cursor = ''
      }

      // Axis arrow hover
      const hoveredAxisEntry = hit?.kind === 'axis'
        ? axisArrowsRef.current.find(e => e.ikJointIdx === hit.ikJointIdx && e.axisKey === hit.axisKey)
        : undefined
      for (const e of axisArrowsRef.current) {
        const isHovered = hoveredAxisEntry && e.ikJointIdx === hoveredAxisEntry.ikJointIdx && e.axisKey === hoveredAxisEntry.axisKey
        const sm = e.shaftMesh.material as THREE.MeshBasicMaterial
        const hm = e.headMesh.material as THREE.MeshBasicMaterial
        const targetOpacity = isHovered ? ARROW_OPACITY_HOVER : ARROW_OPACITY_IDLE
        if (sm.opacity !== targetOpacity) { sm.opacity = targetOpacity; hm.opacity = targetOpacity; invalidate() }
      }
      if (hit?.kind === 'axis') canvas.style.cursor = hasInitialStateRef.current ? 'grab' : 'not-allowed'
    }

    const onPointerUp = () => {
      const drag = activeDragRef.current
      if (!drag) return

      if (drag.type === 'ring') {
        setRingAppearance(drag.entry, COLOR_HOVER, OPACITY_HOVER)
      } else if (drag.type === 'ik') {
        const sphere = jointSpheresRef.current.get(IK_JOINTS[drag.ikJointIdx].joint)
        if (sphere) {
          const mat = sphere.material as THREE.MeshBasicMaterial
          mat.color.setHex(EE_COLOR_IDLE)
          mat.opacity = EE_OPACITY_IDLE
        }
      } else if (drag.type === 'axis') {
        for (const e of axisArrowsRef.current) {
          ;(e.shaftMesh.material as THREE.MeshBasicMaterial).opacity = ARROW_OPACITY_IDLE
          ;(e.headMesh.material as THREE.MeshBasicMaterial).opacity = ARROW_OPACITY_IDLE
        }
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
  }, [gl, invalidate, publishNow, setRingAppearance, getRingScreenCenter, setGhostBodyOpacity, orbitRef, jointSpheresRef, sphereByUuidRef, updateArrowPositions])

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
            createJointSpheres(robot)
            createAxisArrows(robot, scene)
          } else {
            // Cleanup axis arrows
            const seenGroups = new Set<THREE.Object3D>()
            for (const entry of axisArrowsRef.current) {
              const group = entry.shaftMesh.parent
              if (group && !seenGroups.has(group)) {
                seenGroups.add(group)
                scene.remove(group)
              }
              entry.shaftMesh.geometry.dispose()
              entry.headMesh.geometry.dispose()
              ;(entry.shaftMesh.material as THREE.Material).dispose()
              ;(entry.headMesh.material as THREE.Material).dispose()
            }
            axisArrowsRef.current = []
            arrowMeshByUuidRef.current.clear()
            // createJointSpheres disposes old spheres when called with next robot;
            // on final unmount, dispose manually
            for (const sphere of jointSpheresRef.current.values()) {
              sphere.parent?.remove(sphere)
              sphere.geometry.dispose()
              ;(sphere.material as THREE.Material).dispose()
            }
            jointSpheresRef.current.clear()
            sphereByUuidRef.current.clear()
          }
        }}
      />
    </>
  )
}
