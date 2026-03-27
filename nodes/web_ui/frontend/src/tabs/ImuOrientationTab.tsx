/**
 * IMU Orientation Tab — renders live XYZ orientation arrows in 3D.
 *
 * Reads orientation quaternion from the configured IMU topic and applies it
 * to a group of three axis arrows (X=red, Y=green, Z=blue).
 * The quaternion is sourced from sensor_msgs/Imu orientation field.
 */
import { useEffect, useRef } from 'react'
import { Canvas, useFrame } from '@react-three/fiber'
import { OrbitControls, Line } from '@react-three/drei'
import * as THREE from 'three'
import { TabConfig } from '../types'

interface Quaternion {
  x: number
  y: number
  z: number
  w: number
}

interface Props {
  tab: TabConfig
  topicData: Record<string, unknown>
  publish: (topic: string, msgType: string, data: unknown) => void
}

interface ArrowsProps {
  quatRef: React.MutableRefObject<Quaternion>
}

const AXIS_LENGTH = 0.8

function OrientationArrows({ quatRef }: ArrowsProps) {
  const groupRef = useRef<THREE.Group>(null)
  const q = useRef(new THREE.Quaternion())

  useFrame(() => {
    if (!groupRef.current) return
    const { x, y, z, w } = quatRef.current
    // ROS uses Z-up; convert to Three.js Y-up by swapping Y and Z with sign flip
    q.current.set(x, z, -y, w)
    groupRef.current.quaternion.copy(q.current)
  })

  const xEnd: [number, number, number] = [AXIS_LENGTH, 0, 0]
  const yEnd: [number, number, number] = [0, AXIS_LENGTH, 0]
  const zEnd: [number, number, number] = [0, 0, AXIS_LENGTH]
  const origin: [number, number, number] = [0, 0, 0]

  return (
    <group ref={groupRef}>
      {/* X axis — red */}
      <Line points={[origin, xEnd]} color="#e06c75" lineWidth={3} />
      <mesh position={xEnd}>
        <coneGeometry args={[0.04, 0.12, 8]} />
        <meshStandardMaterial color="#e06c75" />
      </mesh>

      {/* Y axis — green */}
      <Line points={[origin, yEnd]} color="#98c379" lineWidth={3} />
      <mesh position={yEnd} rotation={[0, 0, -Math.PI / 2]}>
        <coneGeometry args={[0.04, 0.12, 8]} />
        <meshStandardMaterial color="#98c379" />
      </mesh>

      {/* Z axis — blue */}
      <Line points={[origin, zEnd]} color="#61afef" lineWidth={3} />
      <mesh position={zEnd} rotation={[Math.PI / 2, 0, 0]}>
        <coneGeometry args={[0.04, 0.12, 8]} />
        <meshStandardMaterial color="#61afef" />
      </mesh>
    </group>
  )
}

export default function ImuOrientationTab({ tab, topicData }: Props) {
  const imuTopic = tab.topics?.[0]?.topic ?? tab.topic ?? ''
  const quatRef = useRef<Quaternion>({ x: 0, y: 0, z: 0, w: 1 })

  useEffect(() => {
    if (!imuTopic) return
    const msg = topicData[imuTopic] as Record<string, unknown> | undefined
    if (!msg) return
    const ori = msg['orientation'] as Quaternion | undefined
    if (!ori) return
    const { x, y, z, w } = ori
    if (typeof x === 'number' && typeof y === 'number' && typeof z === 'number' && typeof w === 'number') {
      quatRef.current = { x, y, z, w }
    }
  }, [topicData, imuTopic])

  return (
    <div style={{ width: '100%', height: '100%', background: '#050505', position: 'relative' }}>
      <Canvas camera={{ position: [1.5, 1.5, 1.5], fov: 45 }} style={{ background: '#050505' }}>
        <ambientLight intensity={0.8} />
        <directionalLight position={[3, 5, 3]} intensity={0.8} />
        <OrientationArrows quatRef={quatRef} />
        {/* Reference grid floor */}
        <gridHelper args={[2, 10, '#1a1a1a', '#111']} position={[0, -0.01, 0]} />
        <OrbitControls enableDamping dampingFactor={0.1} />
      </Canvas>
      {/* Axis legend */}
      <div style={{
        position: 'absolute', bottom: 16, left: 16,
        display: 'flex', flexDirection: 'column', gap: 4, pointerEvents: 'none',
      }}>
        {[['#e06c75', 'X (forward)'], ['#98c379', 'Y (left)'], ['#61afef', 'Z (up)']].map(([color, label]) => (
          <div key={label} style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
            <div style={{ width: 24, height: 3, background: color, borderRadius: 2 }} />
            <span style={{ color: '#888', fontSize: 11, fontFamily: 'monospace' }}>{label}</span>
          </div>
        ))}
      </div>
    </div>
  )
}
