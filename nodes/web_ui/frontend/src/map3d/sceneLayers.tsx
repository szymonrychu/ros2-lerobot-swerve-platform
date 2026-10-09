/**
 * Scene objects of the 3D map tab, all in the ROS map frame via rosToThree. Each is memoised so a topic update
 * only re-renders the object that draws it.
 */
import { memo, useEffect, useMemo, useState } from 'react'
import { useThree } from '@react-three/fiber'
import { Line } from '@react-three/drei'
import * as THREE from 'three'
import log from '../logging'
import { frontEdgeIndex, MapMeta, Pose2D } from '../map/mapMath'
import { rosToThree, rosYawToThreeY, ThreeTuple } from './coords'
import { GpsAnchor, tilesAround } from './geo'
import { mapPlacement } from './groundMath'

/** Stacking heights (three y, metres) so ground layers never z-fight. */
export const LIFT = {
  gps: -0.004,
  map: 0,
  costmap: 0.004,
  plan: 0.02,
  footprint: 0.025,
  goal: 0.03,
} as const

export const SCENE_COLORS = {
  robot: '#2f9bff',
  robotFront: '#ffdc00',
  globalPath: '#2ecc40',
  localPath: '#ff851b',
  goal: '#ff4136',
  draft: '#ffdc00',
  costmap: '#c158dc',
  free: '#fefefe',
  occupied: '#000000',
  unknown: '#cdcdcd',
} as const

export const GPS_TILE_ZOOM = 19
export const GPS_TILE_RADIUS = 2

/** OccupancyGrid image payload (msg_serializer.serialize_occupancy_grid; also the local costmap contract). */
export interface GridImageMsg extends MapMeta {
  png_b64: string
  frame_id: string
  stamp?: number
}

export interface PathMsg {
  frame_id: string
  points: [number, number][]
}

/**
 * Flat textured rectangle on the ground at a map-frame placement.
 * The plane faces +y; its local +y (texture top) points to ROS +y before the yaw rotation.
 */
const GroundRect = memo(function GroundRect({
  texture,
  center,
  width,
  height,
  yaw,
  lift,
  opacity = 1,
  visible = true,
}: {
  texture: THREE.Texture
  center: { x: number; y: number }
  width: number
  height: number
  yaw: number
  lift: number
  opacity?: number
  visible?: boolean
}) {
  return (
    <group position={rosToThree(center, lift)} rotation={[0, rosYawToThreeY(yaw), 0]} visible={visible}>
      <mesh rotation={[-Math.PI / 2, 0, 0]} renderOrder={lift > 0 ? 1 : 0}>
        <planeGeometry args={[width, height]} />
        <meshBasicMaterial
          map={texture}
          transparent={opacity < 1}
          opacity={opacity}
          depthWrite={opacity >= 1}
          side={THREE.DoubleSide}
          toneMapped={false}
        />
      </mesh>
    </group>
  )
})

/**
 * Decode a base64 PNG into a texture; re-decodes only when the image or its placement changes.
 *
 * @param msg - grid image payload or null/undefined
 * @returns texture (nearest filtering, sRGB) or null while decoding / absent
 */
function useGridTexture(msg: GridImageMsg | null | undefined): THREE.Texture | null {
  const { invalidate } = useThree()
  const [texture, setTexture] = useState<THREE.Texture | null>(null)
  const png = msg?.png_b64
  useEffect(() => {
    if (!png) {
      setTexture(null)
      return
    }
    let cancelled = false
    let loaded: THREE.Texture | null = null
    new THREE.TextureLoader().load(
      `data:image/png;base64,${png}`,
      (tex) => {
        if (cancelled) {
          tex.dispose()
          return
        }
        tex.magFilter = THREE.NearestFilter
        tex.minFilter = THREE.LinearFilter
        tex.generateMipmaps = false
        tex.colorSpace = THREE.SRGBColorSpace
        loaded = tex
        setTexture(tex)
        invalidate()
      },
      undefined,
      () => log.warn('[map3d] failed to decode grid image'),
    )
    return () => {
      cancelled = true
      loaded?.dispose()
    }
  }, [png, invalidate])
  return texture
}

/** SLAM map or local costmap drawn as a ground texture at its origin and resolution. */
export const GridImageLayer = memo(function GridImageLayer({
  msg,
  lift,
  opacity,
  visible,
}: {
  msg: GridImageMsg | null | undefined
  lift: number
  opacity: number
  visible: boolean
}) {
  const texture = useGridTexture(msg)
  const { invalidate } = useThree()
  useEffect(() => invalidate(), [visible, invalidate])
  if (!msg || !texture) return null
  const p = mapPlacement(msg)
  return (
    <GroundRect
      texture={texture}
      center={p.center}
      width={p.width}
      height={p.height}
      yaw={p.yaw}
      lift={lift}
      opacity={opacity}
      visible={visible}
    />
  )
})

/** Texture cache for GPS tiles (session lifetime; the tile set around the robot changes slowly). */
const TILE_TEXTURES = new Map<string, THREE.Texture>()

const GpsTile = memo(function GpsTile({
  url,
  center,
  width,
  height,
  yaw,
}: {
  url: string
  center: { x: number; y: number }
  width: number
  height: number
  yaw: number
}) {
  const { invalidate } = useThree()
  const [texture, setTexture] = useState<THREE.Texture | null>(() => TILE_TEXTURES.get(url) ?? null)
  useEffect(() => {
    const cached = TILE_TEXTURES.get(url)
    if (cached) {
      setTexture(cached)
      return
    }
    let cancelled = false
    new THREE.TextureLoader().load(
      url,
      (tex) => {
        tex.colorSpace = THREE.SRGBColorSpace
        TILE_TEXTURES.set(url, tex)
        if (!cancelled) {
          setTexture(tex)
          invalidate()
        }
      },
      undefined,
      () => log.debug('[map3d] tile load failed:', url),
    )
    return () => {
      cancelled = true
    }
  }, [url, invalidate])
  if (!texture) return null
  return <GroundRect texture={texture} center={center} width={width} height={height} yaw={yaw} lift={LIFT.gps} />
})

/** Map tiles from the backend tile proxy, placed around the robot with the GPS anchor (Web Mercator). */
export const GpsTilesLayer = memo(function GpsTilesLayer({
  anchor,
  aroundX,
  aroundY,
  tileVersion,
}: {
  anchor: GpsAnchor
  aroundX: number
  aroundY: number
  tileVersion?: string | null
}) {
  // Recomputing 25 placements per pose update is cheap; tiles are keyed by URL so meshes and textures persist.
  const tiles = useMemo(
    () => tilesAround(anchor, { x: aroundX, y: aroundY }, GPS_TILE_ZOOM, GPS_TILE_RADIUS, tileVersion),
    [anchor, aroundX, aroundY, tileVersion],
  )
  return (
    <>
      {tiles.map((t) => (
        <GpsTile key={t.url} url={t.url} center={t.center} width={t.width} height={t.height} yaw={t.yaw} />
      ))}
    </>
  )
})

/** Nav2 path as a ground line. */
export const PathLayer = memo(function PathLayer({
  path,
  color,
  width,
  lift,
}: {
  path: PathMsg
  color: string
  width: number
  lift: number
}) {
  const points = useMemo(() => path.points.map(([x, y]) => rosToThree({ x, y }, lift)), [path, lift])
  if (points.length < 2) return null
  return <Line points={points} color={color} lineWidth={width} />
})

/** Goal pose: a ring at the position and an arrow along the heading. */
export const GoalMarker = memo(function GoalMarker({
  pose,
  color,
  dashed,
}: {
  pose: Pose2D
  color: string
  dashed?: boolean
}) {
  const arrow: ThreeTuple[] = [
    [0, 0, 0],
    [0.45, 0, 0],
    [0.33, 0, -0.08],
    [0.45, 0, 0],
    [0.33, 0, 0.08],
  ]
  const ring = useMemo(() => {
    const pts: ThreeTuple[] = []
    for (let i = 0; i <= 32; i++) {
      const a = (i / 32) * Math.PI * 2
      pts.push([Math.cos(a) * 0.15, 0, Math.sin(a) * 0.15])
    }
    return pts
  }, [])
  return (
    <group position={rosToThree(pose, LIFT.goal)} rotation={[0, rosYawToThreeY(pose.yaw), 0]}>
      <Line points={ring} color={color} lineWidth={3} dashed={dashed} dashSize={0.05} gapSize={0.03} />
      <Line points={arrow} color={color} lineWidth={3} />
    </group>
  )
})

/** Nav2 footprint polygon (map frame) with the front edge highlighted. */
export const FootprintLayer = memo(function FootprintLayer({ footprint, pose }: { footprint: PathMsg; pose: Pose2D | null }) {
  const outline = useMemo(() => {
    const pts = footprint.points.map(([x, y]) => rosToThree({ x, y }, LIFT.footprint))
    return pts.length >= 3 ? [...pts, pts[0]] : []
  }, [footprint])
  const front = useMemo(() => {
    if (!pose || footprint.points.length < 3) return null
    const i = frontEdgeIndex(footprint.points, pose)
    if (i < 0) return null
    const [ax, ay] = footprint.points[i]
    const [bx, by] = footprint.points[(i + 1) % footprint.points.length]
    return [rosToThree({ x: ax, y: ay }, LIFT.footprint + 0.002), rosToThree({ x: bx, y: by }, LIFT.footprint + 0.002)]
  }, [footprint, pose])
  if (outline.length === 0) return null
  return (
    <>
      <Line points={outline} color={SCENE_COLORS.robot} lineWidth={2.5} />
      {front && <Line points={front} color={SCENE_COLORS.robotFront} lineWidth={5} />}
    </>
  )
})
