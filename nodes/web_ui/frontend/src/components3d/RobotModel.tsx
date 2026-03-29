import { useEffect, useRef, useState } from 'react'
import { useThree } from '@react-three/fiber'
import URDFLoader from 'urdf-loader'
import type { URDFRobot } from 'urdf-loader'
import * as THREE from 'three'
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js'
import log from '../logging'

const GHOST_COLOR = 0x66aaff

interface JointStates {
  name?: string[]
  position?: number[]
}

interface Props {
  urdfFile: string
  jointStates?: JointStates
  position?: [number, number, number]
  onRobotLoaded?: (robot: URDFRobot | null) => void
  /** When true, body meshes are created with transparent material at opacity 0. */
  ghost?: boolean
  /** Called after URDF load with all body meshes (created by loadMeshCb). */
  onBodyMeshesLoaded?: (meshes: THREE.Mesh[]) => void
}

export function RobotModel({ urdfFile, jointStates, position, onRobotLoaded, ghost, onBodyMeshesLoaded }: Props) {
  const { scene, invalidate } = useThree()
  const robotRef = useRef<URDFRobot | null>(null)
  const [robotReady, setRobotReady] = useState(0)
  const onRobotLoadedRef = useRef(onRobotLoaded)
  onRobotLoadedRef.current = onRobotLoaded
  const onBodyMeshesLoadedRef = useRef(onBodyMeshesLoaded)
  onBodyMeshesLoadedRef.current = onBodyMeshesLoaded

  useEffect(() => {
    const stlLoader = new STLLoader()
    const bodyMeshes: THREE.Mesh[] = []
    let robotObj: URDFRobot | null = null

    // Custom LoadingManager: its onLoad fires AFTER all async mesh fetches complete.
    // urdf-loader's onComplete fires after parse() returns, which is BEFORE meshes
    // are loaded (loadMeshCb uses async fetch). So ghost material overrides must
    // happen in the manager's onLoad callback, not in urdf-loader's onComplete.
    const manager = new THREE.LoadingManager()
    manager.onLoad = () => {
      if (!robotObj) return
      const robot = robotObj

      robot.rotation.x = -Math.PI / 2
      if (position) robot.position.set(...position)

      // Ghost: override materials AFTER all meshes are loaded and urdf-loader
      // has applied URDF <material> tags (line 556-558 of URDFLoader.js)
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

      robotRef.current = robot
      scene.add(robot)
      setRobotReady((n) => n + 1)
      invalidate()
      onRobotLoadedRef.current?.(robot)
      if (ghost) onBodyMeshesLoadedRef.current?.(bodyMeshes)

      const linkCount = Object.keys(robot.links).length
      const jointCount = Object.keys(robot.joints).length
      log.info(`[3d] URDF loaded: ${urdfFile}${ghost ? ' (ghost)' : ''} — ${linkCount} links, ${jointCount} joints, ${bodyMeshes.length} body meshes`)
    }

    const loader = new URDFLoader(manager)

    loader.loadMeshCb = (path, mgr, done) => {
      log.debug('[3d] fetching mesh:', path)
      // Notify manager so it waits for this mesh before firing onLoad
      mgr.itemStart(path)
      fetch(path)
        .then((r) => {
          if (!r.ok) throw new Error(`HTTP ${r.status}`)
          return r.arrayBuffer()
        })
        .then((buf) => {
          const geom = stlLoader.parse(buf)
          const mesh = new THREE.Mesh(geom, new THREE.MeshStandardMaterial({ color: 0x888888 }))
          done(mesh, undefined)
          mgr.itemEnd(path)
        })
        .catch((err: Error) => {
          log.warn('[3d] mesh load failed:', path, '—', err)
          done(new THREE.Object3D(), err)
          mgr.itemError(path)
          mgr.itemEnd(path)
        })
    }

    loader.load(
      `/api/urdf/${urdfFile}`,
      (robot) => {
        // Store robot ref — actual setup happens in manager.onLoad after all meshes load
        robotObj = robot
      },
      undefined,
      (err: unknown) => {
        log.warn('[3d] URDF load failed:', urdfFile, '—', err)
      },
    )

    return () => {
      if (robotRef.current) {
        scene.remove(robotRef.current)
        robotRef.current = null
        onRobotLoadedRef.current?.(null)
      }
    }
  }, [urdfFile, scene, invalidate])

  useEffect(() => {
    const robot = robotRef.current
    if (!robot || !jointStates?.name || !jointStates?.position) return
    let updated = 0
    for (let i = 0; i < jointStates.name.length; i++) {
      const jName = jointStates.name[i]
      const angle = jointStates.position[i]
      if (jName in robot.joints) {
        robot.setJointValue(jName, angle)
        updated++
      }
    }
    if (updated > 0) invalidate()
    log.debug('[3d] frame:', updated, 'joints updated', ghost ? '(ghost)' : '')
  }, [jointStates, invalidate, robotReady])

  return null
}
