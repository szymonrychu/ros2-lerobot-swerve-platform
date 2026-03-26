import { Canvas } from '@react-three/fiber'
import { Grid, OrbitControls } from '@react-three/drei'
import { Suspense } from 'react'
import { RobotModel } from './RobotModel'

interface JointStates {
  name?: string[]
  position?: number[]
}

interface Props {
  urdfFile: string
  jointStates?: JointStates
  children?: React.ReactNode
}

export function RobotScene({ urdfFile, jointStates, children }: Props) {
  return (
    <Canvas
      camera={{ position: [1.5, 1.5, 1.5], fov: 50 }}
      style={{ background: '#050505' }}
    >
      <ambientLight intensity={0.6} />
      <directionalLight position={[5, 10, 5]} intensity={1} />
      <Grid
        args={[10, 10]}
        cellSize={0.1}
        cellColor="#222"
        sectionColor="#333"
        fadeDistance={8}
        position={[0, -0.051, 0]}
      />
      <Suspense fallback={null}>
        <RobotModel urdfFile={urdfFile} jointStates={jointStates} />
      </Suspense>
      {children}
      <OrbitControls makeDefault />
    </Canvas>
  )
}
