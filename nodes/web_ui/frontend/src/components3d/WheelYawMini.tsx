import { Canvas } from '@react-three/fiber'
import { extractField } from '../utils/fieldExtract'

interface Props {
  topicData: Record<string, unknown>
}

const WHEEL_POSITIONS: [number, number][] = [
  [0.2, 0.15],
  [0.2, -0.15],
  [-0.2, 0.15],
  [-0.2, -0.15],
]
const STEER_FIELDS = ['position[1]', 'position[3]', 'position[5]', 'position[7]']
const TOPIC = '/controller/swerve_drive/joint_states'

function WheelMesh({ x, z, yaw }: { x: number; z: number; yaw: number }) {
  return (
    <mesh position={[x, 0, z]} rotation={[0, -yaw, 0]}>
      <boxGeometry args={[0.04, 0.04, 0.12]} />
      <meshStandardMaterial color="#888" />
    </mesh>
  )
}

export function WheelYawMini({ topicData }: Props) {
  const swervData = topicData[TOPIC] as Record<string, unknown> | undefined

  return (
    <Canvas
      camera={{ position: [0, 1.2, 0], fov: 40, up: [0, 0, -1] }}
      style={{ width: 64, height: 64, flexShrink: 0 }}
    >
      <ambientLight intensity={1} />
      {WHEEL_POSITIONS.map(([x, z], i) => {
        const yaw = swervData ? (extractField(swervData, STEER_FIELDS[i]) ?? 0) : 0
        return <WheelMesh key={i} x={x} z={z} yaw={yaw} />
      })}
      <mesh position={[0, 0, 0]}>
        <boxGeometry args={[0.4, 0.01, 0.3]} />
        <meshStandardMaterial color="#333" />
      </mesh>
    </Canvas>
  )
}
