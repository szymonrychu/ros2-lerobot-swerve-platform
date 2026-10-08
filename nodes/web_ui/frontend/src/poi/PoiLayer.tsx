/**
 * Scene objects of the POI layer (inside the Canvas, map frame): point discs, translucent area polygons with
 * outlines, labels, vertex handles of the selected area and the area draft being drawn.
 */
import { memo, useEffect, useMemo } from 'react'
import { useThree } from '@react-three/fiber'
import { Html, Line } from '@react-three/drei'
import * as THREE from 'three'
import type { Vec2 } from '../map/mapMath'
import { rosToThree, ThreeTuple } from '../map3d/coords'
import { objectDetails, poiStyle, STATUS_COLORS } from './style'
import type { Poi } from './types'

/** Stacking heights (three y, metres), above the goal marker. */
const POI_LIFT = { fill: 0.034, outline: 0.036, handle: 0.04, draft: 0.042 } as const
const DISC_SEGMENTS = 40
const HANDLE_RADIUS_M = 0.06
const DRAFT_COLOR = '#ffdc00'
/** Half diagonal of the diamond that marks a remembered object, m. */
const OBJECT_MARKER_RADIUS_M = 0.07

function closedOutline(polygon: [number, number][], lift: number): ThreeTuple[] {
  const pts = polygon.map(([x, y]) => rosToThree({ x, y }, lift))
  return pts.length > 0 ? [...pts, pts[0]] : []
}

function PoiLabel({ poi, lift }: { poi: Poi; lift: number }) {
  const tag = poi.kind === 'object' ? 'obj' : poi.created_by === 'agent' ? 'A' : 'U'
  const seen = poi.kind === 'object' && (poi.times_seen ?? 0) > 1 ? ` x${poi.times_seen}` : ''
  return (
    <Html position={rosToThree(poi, lift)} center zIndexRange={[5, 0]} style={{ pointerEvents: 'none' }}>
      <div
        style={{
          transform: 'translateY(-22px)',
          whiteSpace: 'nowrap',
          font: '600 11px sans-serif',
          color: '#fff',
          background: 'rgba(13, 17, 23, 0.78)',
          border: `1px solid ${STATUS_COLORS[poi.status]}`,
          borderRadius: 4,
          padding: '1px 5px',
        }}
        title={objectDetails(poi, Date.now() / 1000) || undefined}
      >
        <span style={{ opacity: 0.7, marginRight: 4 }}>{tag}</span>
        {poi.name || '(unnamed)'}
        {seen}
      </div>
    </Html>
  )
}

const PointMarker = memo(function PointMarker({ poi, selected }: { poi: Poi; selected: boolean }) {
  const style = poiStyle(poi, selected)
  const ring = useMemo(() => {
    const pts: ThreeTuple[] = []
    for (let i = 0; i <= DISC_SEGMENTS; i++) {
      const a = (i / DISC_SEGMENTS) * Math.PI * 2
      pts.push([Math.cos(a) * poi.radius_m, 0, Math.sin(a) * poi.radius_m])
    }
    return pts
  }, [poi.radius_m])
  return (
    <group position={rosToThree(poi, POI_LIFT.fill)}>
      <mesh rotation={[-Math.PI / 2, 0, 0]} renderOrder={2}>
        <circleGeometry args={[poi.radius_m, DISC_SEGMENTS]} />
        <meshBasicMaterial color={style.fill} transparent opacity={style.opacity} depthWrite={false} toneMapped={false} />
      </mesh>
      <group position={[0, POI_LIFT.outline - POI_LIFT.fill, 0]}>
        <Line
          points={ring}
          color={style.outline}
          lineWidth={style.lineWidth}
          dashed={style.dashed}
          dashSize={0.04}
          gapSize={0.03}
        />
      </group>
      <PoiLabel poi={{ ...poi, x: 0, y: 0 }} lift={0} />
    </group>
  )
})

/** A remembered object: a diamond in the object colour with its label (times seen and last sighting in the tooltip). */
const ObjectMarker = memo(function ObjectMarker({ poi, selected }: { poi: Poi; selected: boolean }) {
  const style = poiStyle(poi, selected)
  const diamond = useMemo(() => {
    const r = OBJECT_MARKER_RADIUS_M
    const pts: ThreeTuple[] = [
      [r, 0, 0],
      [0, 0, r],
      [-r, 0, 0],
      [0, 0, -r],
    ]
    return [...pts, pts[0]]
  }, [])
  return (
    <group position={rosToThree(poi, POI_LIFT.fill)}>
      <mesh rotation={[-Math.PI / 2, 0, Math.PI / 4]} renderOrder={2}>
        <circleGeometry args={[OBJECT_MARKER_RADIUS_M * Math.SQRT2, 4]} />
        <meshBasicMaterial color={style.fill} transparent opacity={Math.min(1, style.opacity + 0.3)} depthWrite={false} toneMapped={false} />
      </mesh>
      <group position={[0, POI_LIFT.outline - POI_LIFT.fill, 0]}>
        <Line points={diamond} color={style.outline} lineWidth={style.lineWidth} dashed={style.dashed} dashSize={0.03} gapSize={0.02} />
      </group>
      <PoiLabel poi={{ ...poi, x: 0, y: 0 }} lift={0} />
    </group>
  )
})

const AreaMarker = memo(function AreaMarker({ poi, selected }: { poi: Poi; selected: boolean }) {
  const style = poiStyle(poi, selected)
  const geometry = useMemo(() => {
    const shape = new THREE.Shape(poi.polygon.map(([x, y]) => new THREE.Vector2(x, y)))
    return new THREE.ShapeGeometry(shape)
  }, [poi.polygon])
  useEffect(() => () => geometry.dispose(), [geometry])
  const outline = useMemo(() => closedOutline(poi.polygon, POI_LIFT.outline), [poi.polygon])
  return (
    <>
      <mesh geometry={geometry} position={[0, POI_LIFT.fill, 0]} rotation={[-Math.PI / 2, 0, 0]} renderOrder={2}>
        <meshBasicMaterial
          color={style.fill}
          transparent
          opacity={style.opacity * 0.6}
          depthWrite={false}
          side={THREE.DoubleSide}
          toneMapped={false}
        />
      </mesh>
      <Line
        points={outline}
        color={style.outline}
        lineWidth={style.lineWidth}
        dashed={style.dashed}
        dashSize={0.06}
        gapSize={0.04}
      />
      {selected &&
        poi.polygon.map(([x, y], i) => (
          <mesh key={i} position={rosToThree({ x, y }, POI_LIFT.handle)} rotation={[-Math.PI / 2, 0, 0]} renderOrder={3}>
            <circleGeometry args={[HANDLE_RADIUS_M, 20]} />
            <meshBasicMaterial color="#ffffff" depthTest={false} toneMapped={false} />
          </mesh>
        ))}
      <PoiLabel poi={poi} lift={POI_LIFT.fill} />
    </>
  )
})

/** All POIs of the layer plus the area being drawn. */
export const PoiLayer = memo(function PoiLayer({
  pois,
  selectedId,
  draft,
  visible,
}: {
  pois: Poi[]
  selectedId: string | null
  draft: Vec2[]
  visible: boolean
}) {
  const { invalidate } = useThree()
  useEffect(() => invalidate(), [pois, selectedId, draft, visible, invalidate])
  const draftLine = useMemo(
    () => draft.map((v) => rosToThree(v, POI_LIFT.draft)),
    [draft],
  )
  if (!visible) return null
  return (
    <>
      {pois.map((p) =>
        p.kind === 'point' ? (
          <PointMarker key={p.id} poi={p} selected={p.id === selectedId} />
        ) : p.kind === 'object' ? (
          <ObjectMarker key={p.id} poi={p} selected={p.id === selectedId} />
        ) : (
          <AreaMarker key={p.id} poi={p} selected={p.id === selectedId} />
        ),
      )}
      {draft.length >= 2 && <Line points={draftLine} color={DRAFT_COLOR} lineWidth={3} dashed dashSize={0.05} gapSize={0.03} />}
      {draft.map((v, i) => (
        <mesh key={i} position={rosToThree(v, POI_LIFT.draft)} rotation={[-Math.PI / 2, 0, 0]} renderOrder={3}>
          <circleGeometry args={[HANDLE_RADIUS_M, 20]} />
          <meshBasicMaterial color={DRAFT_COLOR} depthTest={false} toneMapped={false} />
        </mesh>
      ))}
    </>
  )
})
