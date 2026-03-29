import { useEffect, useRef, useState } from 'react'
import { useThree } from '@react-three/fiber'
import URDFLoader from 'urdf-loader'
import type { URDFRobot } from 'urdf-loader'
import * as THREE from 'three'
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js'
import log from '../logging'

interface JointStates {
  name?: string[]
  position?: number[]
}

interface Props {
  urdfFile: string
  jointStates?: JointStates
  position?: [number, number, number]
  onRobotLoaded?: (robot: URDFRobot | null) => void
}

export function RobotModel({ urdfFile, jointStates, position, onRobotLoaded }: Props) {
  const { scene, invalidate } = useThree()
  const robotRef = useRef<URDFRobot | null>(null)
  const [robotReady, setRobotReady] = useState(0)
  // Store callback in a ref so changing it doesn't retrigger URDF reload
  const onRobotLoadedRef = useRef(onRobotLoaded)
  onRobotLoadedRef.current = onRobotLoaded

  useEffect(() => {
    const loader = new URDFLoader()
    const stlLoader = new STLLoader()

    loader.loadMeshCb = (path, _manager, done) => {
      log.debug('[3d] fetching mesh:', path)
      const fullPath = path
      fetch(fullPath)
        .then((r) => {
          if (!r.ok) throw new Error(`HTTP ${r.status}`)
          return r.arrayBuffer()
        })
        .then((buf) => {
          const geom = stlLoader.parse(buf)
          const mesh = new THREE.Mesh(geom, new THREE.MeshStandardMaterial({ color: 0x888888 }))
          done(mesh, undefined)
        })
        .catch((err: Error) => {
          log.warn('[3d] mesh load failed:', path, '—', err)
          done(new THREE.Object3D(), err)
        })
    }

    loader.load(
      `/api/urdf/${urdfFile}`,
      (robot) => {
        // ROS/URDF uses Z-up; Three.js uses Y-up. Rotate -90° around X to correct orientation.
        robot.rotation.x = -Math.PI / 2
        if (position) robot.position.set(...position)
        robotRef.current = robot
        scene.add(robot)
        setRobotReady((n) => n + 1)
        invalidate()
        onRobotLoadedRef.current?.(robot)
        const linkCount = Object.keys(robot.links).length
        const jointCount = Object.keys(robot.joints).length
        log.info(`[3d] URDF loaded: ${urdfFile} — ${linkCount} links, ${jointCount} joints`)
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
    log.debug('[3d] frame:', updated, 'joints updated')
  }, [jointStates, invalidate, robotReady])

  return null
}
