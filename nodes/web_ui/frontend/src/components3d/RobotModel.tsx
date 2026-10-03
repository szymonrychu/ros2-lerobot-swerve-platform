import { useEffect, useRef, useState } from 'react'
import { useThree } from '@react-three/fiber'
import URDFLoader from 'urdf-loader'
import type { URDFRobot } from 'urdf-loader'
import * as THREE from 'three'
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js'
import log from '../logging'
import { URDF_TO_THREE_ROTATION_X } from '../map3d/coords'

const GHOST_COLOR = 0x66aaff
const MESH_COLOR = 0x888888

/**
 * Parsed STL geometry per mesh URL, shared by every model in the session: the four wheels of the base share one
 * (78 MB) download, and InteractiveArm's ghost arm reuses the solid arm's meshes instead of fetching them again.
 */
const GEOMETRY_CACHE = new Map<string, Promise<THREE.BufferGeometry>>()

/**
 * Fetch and parse an STL mesh once per URL.
 *
 * @param path - mesh URL
 * @returns shared geometry (never dispose it)
 */
export function loadStlGeometry(path: string): Promise<THREE.BufferGeometry> {
  let cached = GEOMETRY_CACHE.get(path)
  if (!cached) {
    cached = fetch(path)
      .then((r) => {
        if (!r.ok) throw new Error(`HTTP ${r.status}`)
        return r.arrayBuffer()
      })
      .then((buf) => new STLLoader().parse(buf))
    // A failed fetch may be retried by the next model load.
    cached.catch(() => GEOMETRY_CACHE.delete(path))
    GEOMETRY_CACHE.set(path, cached)
  }
  return cached
}

interface JointStates {
  name?: string[]
  position?: number[]
}

interface Props {
  urdfFile: string
  jointStates?: JointStates
  /** Group position in three coordinates (relative to the parent group). */
  position?: [number, number, number]
  /** Whole-model visibility. */
  visible?: boolean
  /** Per-link visibility of visual meshes; links without a decision stay visible. */
  linkVisible?: (linkName: string) => boolean
  onRobotLoaded?: (robot: URDFRobot | null) => void
  /** When true, body meshes are created with transparent material at opacity 0. */
  ghost?: boolean
  /** Called after URDF load with all body meshes (ghost only). */
  onBodyMeshesLoaded?: (meshes: THREE.Mesh[]) => void
}

/**
 * Apply per-link visibility to the URDF visuals (not to child links, which carry their own visuals).
 *
 * @param robot - loaded URDF robot
 * @param linkVisible - decision per link name, or undefined to show everything
 */
function applyLinkVisibility(robot: URDFRobot, linkVisible?: (linkName: string) => boolean): void {
  for (const [name, link] of Object.entries(robot.links)) {
    const show = linkVisible ? linkVisible(name) : true
    for (const child of link.children) {
      if ((child as unknown as { isURDFVisual?: boolean }).isURDFVisual) child.visible = show
    }
  }
}

/**
 * A URDF model inside its own group, oriented from URDF z-up to the y-up scene. Meshes are STL files served by
 * the backend under /api/urdf/.
 */
export function RobotModel({
  urdfFile,
  jointStates,
  position,
  visible = true,
  linkVisible,
  onRobotLoaded,
  ghost,
  onBodyMeshesLoaded,
}: Props) {
  const { invalidate } = useThree()
  const groupRef = useRef<THREE.Group>(null)
  const robotRef = useRef<URDFRobot | null>(null)
  const [robotReady, setRobotReady] = useState(0)
  const onRobotLoadedRef = useRef(onRobotLoaded)
  onRobotLoadedRef.current = onRobotLoaded
  const onBodyMeshesLoadedRef = useRef(onBodyMeshesLoaded)
  onBodyMeshesLoadedRef.current = onBodyMeshesLoaded
  const linkVisibleRef = useRef(linkVisible)
  linkVisibleRef.current = linkVisible

  useEffect(() => {
    const bodyMeshes: THREE.Mesh[] = []
    let robotObj: URDFRobot | null = null
    let cancelled = false

    // The manager's onLoad fires after every async mesh load (urdf-loader's own onComplete fires before).
    const manager = new THREE.LoadingManager()
    manager.onLoad = () => {
      const group = groupRef.current
      if (!robotObj || cancelled || !group) return
      const robot = robotObj

      robot.rotation.x = URDF_TO_THREE_ROTATION_X

      if (ghost) {
        robot.traverse((child) => {
          if ((child as THREE.Mesh).isMesh) {
            const m = child as THREE.Mesh
            m.material = new THREE.MeshStandardMaterial({
              color: GHOST_COLOR,
              transparent: true,
              opacity: 0,
              depthWrite: false,
            })
            bodyMeshes.push(m)
          }
        })
      }
      applyLinkVisibility(robot, linkVisibleRef.current)

      robotRef.current = robot
      group.add(robot)
      setRobotReady((n) => n + 1)
      invalidate()
      onRobotLoadedRef.current?.(robot)
      if (ghost) onBodyMeshesLoadedRef.current?.(bodyMeshes)

      const linkCount = Object.keys(robot.links).length
      const jointCount = Object.keys(robot.joints).length
      log.info(`[3d] URDF loaded: ${urdfFile}${ghost ? ' (ghost)' : ''} - ${linkCount} links, ${jointCount} joints`)
    }

    const loader = new URDFLoader(manager)
    loader.loadMeshCb = (path, mgr, done) => {
      mgr.itemStart(path)
      loadStlGeometry(path)
        .then((geom) => {
          done(new THREE.Mesh(geom, new THREE.MeshStandardMaterial({ color: MESH_COLOR })), undefined)
        })
        .catch((err: Error) => {
          log.warn('[3d] mesh load failed:', path, '-', err)
          done(new THREE.Object3D(), err)
          mgr.itemError(path)
        })
        .finally(() => mgr.itemEnd(path))
    }

    loader.load(
      `/api/urdf/${urdfFile}`,
      (robot) => {
        robotObj = robot
      },
      undefined,
      (err: unknown) => log.warn('[3d] URDF load failed:', urdfFile, '-', err),
    )

    return () => {
      cancelled = true
      if (robotRef.current) {
        robotRef.current.parent?.remove(robotRef.current)
        robotRef.current = null
        onRobotLoadedRef.current?.(null)
        invalidate()
      }
    }
  }, [urdfFile, ghost, invalidate])

  useEffect(() => {
    const robot = robotRef.current
    if (!robot) return
    applyLinkVisibility(robot, linkVisible)
    invalidate()
  }, [linkVisible, robotReady, invalidate])

  useEffect(() => {
    const robot = robotRef.current
    if (!robot || !jointStates?.name || !jointStates?.position) return
    let updated = 0
    for (let i = 0; i < jointStates.name.length; i++) {
      const jName = jointStates.name[i]
      if (jName in robot.joints) {
        robot.setJointValue(jName, jointStates.position[i])
        updated++
      }
    }
    if (updated > 0) invalidate()
  }, [jointStates, invalidate, robotReady])

  useEffect(() => {
    invalidate()
  }, [visible, invalidate])

  return <group ref={groupRef} position={position} visible={visible} />
}
