/** Scene objects of the map photo layer (inside the Canvas, map frame): one viewing-direction wedge per located photo. */
import { memo, useEffect, useMemo } from 'react'
import { useThree } from '@react-three/fiber'
import { Line } from '@react-three/drei'
import * as THREE from 'three'
import { rosToThree, ThreeTuple } from '../map3d/coords'
import type { Photo } from './api'
import { cameraColor, photoWedge } from './geometry'

/** Stacking heights (three y, metres), above the POI layer. */
const WEDGE_LIFT = { fill: 0.044, outline: 0.046 } as const
const SELECTED_COLOR = '#ffffff'
const FILL_OPACITY = 0.35
const SELECTED_FILL_OPACITY = 0.7

const Wedge = memo(function Wedge({ photo, selected, onSelect }: { photo: Photo; selected: boolean; onSelect: ((id: string) => void) | null }) {
  const wedge = photoWedge(photo)
  const geometry = useMemo(() => {
    if (wedge === null) return null
    return new THREE.ShapeGeometry(new THREE.Shape(wedge.vertices.map(([x, y]) => new THREE.Vector2(x, y))))
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [photo.view])
  useEffect(() => () => geometry?.dispose(), [geometry])
  if (wedge === null || geometry === null) return null
  const color = cameraColor(photo.camera)
  const outline: ThreeTuple[] = [...wedge.vertices, wedge.vertices[0]].map(([x, y]) => rosToThree({ x, y }, WEDGE_LIFT.outline))
  return (
    <>
      <mesh
        geometry={geometry}
        position={[0, WEDGE_LIFT.fill, 0]}
        rotation={[-Math.PI / 2, 0, 0]}
        renderOrder={4}
        onClick={
          onSelect === null
            ? undefined
            : (e) => {
                e.stopPropagation()
                onSelect(photo.id)
              }
        }
      >
        <meshBasicMaterial color={color} transparent opacity={selected ? SELECTED_FILL_OPACITY : FILL_OPACITY} depthWrite={false} side={THREE.DoubleSide} toneMapped={false} />
      </mesh>
      <Line points={outline} color={selected ? SELECTED_COLOR : color} lineWidth={selected ? 3 : 1.5} dashed={wedge.dashed} dashSize={0.03} gapSize={0.02} />
    </>
  )
})

/** The wedges of the given photos; `onSelect` is null while another tool owns the pointer. */
export const PhotoLayer = memo(function PhotoLayer({
  photos,
  selectedId,
  visible,
  onSelect,
}: {
  photos: Photo[]
  selectedId: string | null
  visible: boolean
  onSelect: ((id: string) => void) | null
}) {
  const { invalidate } = useThree()
  useEffect(() => invalidate(), [photos, selectedId, visible, invalidate])
  if (!visible) return null
  return (
    <>
      {photos.map((p) => (
        <Wedge key={p.id} photo={p} selected={p.id === selectedId} onSelect={onSelect} />
      ))}
    </>
  )
})
