/** Scene objects of the map photo layer (inside the Canvas, map frame): per located photo a viewing-direction wedge and a triangle marker (KIND_SHAPE.photo). */
import { memo, useEffect, useMemo } from 'react'
import { useThree } from '@react-three/fiber'
import { Line } from '@react-three/drei'
import * as THREE from 'three'
import { rosToThree, ThreeTuple } from '../map3d/coords'
import { SELECTED_OUTLINE } from '../map3d/palette'
import type { Photo } from './api'
import { cameraColor, photoMarker, photoWedge } from './geometry'

/** Stacking heights (three y, metres), above the POI layer. */
const WEDGE_LIFT = { fill: 0.044, outline: 0.046, marker: 0.048 } as const
const SELECTED_COLOR = SELECTED_OUTLINE
const FILL_OPACITY = 0.35
const SELECTED_FILL_OPACITY = 0.7

const Wedge = memo(function Wedge({ photo, selected, onSelect }: { photo: Photo; selected: boolean; onSelect: ((id: string) => void) | null }) {
  const wedge = photoWedge(photo)
  const marker = photoMarker(photo)
  const geometry = useMemo(() => {
    if (wedge === null) return null
    return new THREE.ShapeGeometry(new THREE.Shape(wedge.vertices.map(([x, y]) => new THREE.Vector2(x, y))))
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [photo.view])
  const markerGeometry = useMemo(
    () => (marker === null ? null : new THREE.ShapeGeometry(new THREE.Shape(marker.map(([x, y]) => new THREE.Vector2(x, y))))),
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [photo.view],
  )
  useEffect(() => () => geometry?.dispose(), [geometry])
  useEffect(() => () => markerGeometry?.dispose(), [markerGeometry])
  if (wedge === null || geometry === null || marker === null || markerGeometry === null) return null
  const color = cameraColor(photo.camera)
  const select =
    onSelect === null
      ? undefined
      : (e: { stopPropagation: () => void }) => {
          e.stopPropagation()
          onSelect(photo.id)
        }
  const outline: ThreeTuple[] = [...wedge.vertices, wedge.vertices[0]].map(([x, y]) => rosToThree({ x, y }, WEDGE_LIFT.outline))
  return (
    <>
      <mesh
        geometry={geometry}
        position={[0, WEDGE_LIFT.fill, 0]}
        rotation={[-Math.PI / 2, 0, 0]}
        renderOrder={4}
        onClick={select}
      >
        <meshBasicMaterial color={color} transparent opacity={selected ? SELECTED_FILL_OPACITY : FILL_OPACITY} depthWrite={false} side={THREE.DoubleSide} toneMapped={false} />
      </mesh>
      <mesh position={[0, WEDGE_LIFT.marker, 0]} rotation={[-Math.PI / 2, 0, 0]} geometry={markerGeometry} renderOrder={5} onClick={select}>
        <meshBasicMaterial color={selected ? SELECTED_COLOR : color} depthWrite={false} side={THREE.DoubleSide} toneMapped={false} />
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
