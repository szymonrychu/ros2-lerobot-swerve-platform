import { Line } from '@react-three/drei'

interface TFFrame {
  translation?: { x: number; y: number; z: number }
  child_frame_id?: string
}

interface Props {
  frames: TFFrame[]
}

const AXIS_LEN = 0.15

export function TFFrames({ frames }: Props) {
  return (
    <>
      {frames.map((f, i) => {
        const pos: [number, number, number] = [
          f.translation?.x ?? 0,
          f.translation?.z ?? 0,
          -(f.translation?.y ?? 0),
        ]
        return (
          <group key={i} position={pos}>
            <Line points={[[0,0,0],[AXIS_LEN,0,0]]} color="red" lineWidth={2} />
            <Line points={[[0,0,0],[0,AXIS_LEN,0]]} color="green" lineWidth={2} />
            <Line points={[[0,0,0],[0,0,AXIS_LEN]]} color="blue" lineWidth={2} />
          </group>
        )
      })}
    </>
  )
}
