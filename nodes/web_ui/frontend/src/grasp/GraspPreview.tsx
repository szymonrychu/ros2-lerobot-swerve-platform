/**
 * 3D preview of the Grasp panel (inside the Canvas, robot group = base_link): a translucent box at the object pose,
 * a polyline through the planned tool-point waypoints and a small labelled marker per waypoint.
 */
import { memo, useMemo } from 'react'
import { Html, Line } from '@react-three/drei'
import * as THREE from 'three'
import { MapLabel } from '../map3d/MapLabel'
import { OBJECT_COLOR, SELECTED_OUTLINE } from '../map3d/palette'
import type { GraspPreviewData } from './useGrasp'

const PATH_COLOR = SELECTED_OUTLINE
const MARKER_RADIUS_M = 0.008

const Box = memo(function Box({ box }: { box: NonNullable<GraspPreviewData['box']> }) {
  const edges = useMemo(() => new THREE.EdgesGeometry(new THREE.BoxGeometry(...box.size)), [box.size])
  return (
    <group position={box.position} rotation={[0, box.rotationY, 0]}>
      <mesh renderOrder={3}>
        <boxGeometry args={box.size} />
        <meshBasicMaterial color={OBJECT_COLOR} transparent opacity={0.35} depthWrite={false} toneMapped={false} />
      </mesh>
      <lineSegments geometry={edges}>
        <lineBasicMaterial color={OBJECT_COLOR} toneMapped={false} />
      </lineSegments>
    </group>
  )
})

export const GraspPreview = memo(function GraspPreview({ data }: { data: GraspPreviewData | null }) {
  const path = useMemo(() => (data ? data.waypoints.map((w) => w.position) : []), [data])
  if (!data) return null
  return (
    <group>
      {data.box && <Box box={data.box} />}
      {path.length >= 2 && <Line points={path} color={PATH_COLOR} lineWidth={3} />}
      {data.waypoints.map((w, i) => (
        <group key={`${w.label}-${i}`} position={w.position}>
          <mesh renderOrder={4}>
            <sphereGeometry args={[MARKER_RADIUS_M, 12, 8]} />
            <meshBasicMaterial color={PATH_COLOR} toneMapped={false} />
          </mesh>
          <Html center zIndexRange={[5, 0]} style={{ pointerEvents: 'none' }}>
            <MapLabel>{w.label}</MapLabel>
          </Html>
        </group>
      ))}
    </group>
  )
})
