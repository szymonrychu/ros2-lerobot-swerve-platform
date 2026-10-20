/**
 * Scene objects of the 3D map tab, all in the ROS map frame via rosToThree. Each is memoised so a topic update
 * only re-renders the object that draws it.
 */
import { memo, useEffect, useMemo, useRef, useState } from 'react'
import { useThree } from '@react-three/fiber'
import { Html, Line } from '@react-three/drei'
import * as THREE from 'three'
import log from '../logging'
import { frontEdgeIndex, MapMeta, Pose2D, Vec2 } from '../map/mapMath'
import { decodeMap3dCloud, type DecodedMap3dCloud } from './cloudDecode'
import { rosToThree, rosYawToThreeY, ThreeTuple } from './coords'
import { GpsAnchor, tilesAround } from './geo'
import {
  decodeGroundCloud,
  GROUND_COLORS,
  groundVisible,
  planePatch,
  planeZ,
  regionEdges,
  warningPolygons,
  type GroundState,
  type Vec3Tuple,
} from './groundLayer'
import { mapPlacement } from './groundMath'
import { buildLidarWalls, measuredColumnsFromCloud, type WallGrid } from './lidarWalls'
import { MapLabel } from './MapLabel'
import { cloudToMapPoints, LIDAR_COLOR, rgbdCloudToMap, scanToMapPoints, tofColors } from './obstaclePoints'
import { SCENE_COLORS } from './palette'
import { cropUvs, type TileCrop } from './tileSources'
import { recordTileResult, tileFailureCaption, type TileResult, type TileResults } from './tileStatus'

/** Stacking heights (three y, metres) so ground layers never z-fight. */
export const LIFT = {
  gps: -0.004,
  map: 0,
  costmap: 0.004,
  plan: 0.02,
  footprint: 0.025,
  goal: 0.03,
} as const

export { SCENE_COLORS }

export const DEFAULT_GPS_TILE_ZOOM = 19 // preferred display zoom; above the source max zoom tiles are stretched
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
  crop,
}: {
  texture: THREE.Texture
  center: { x: number; y: number }
  width: number
  height: number
  yaw: number
  lift: number
  opacity?: number
  visible?: boolean
  crop?: TileCrop
}) {
  const geometry = useMemo(() => {
    const g = new THREE.PlaneGeometry(width, height)
    if (crop) g.setAttribute('uv', new THREE.Float32BufferAttribute(cropUvs(crop), 2))
    return g
  }, [width, height, crop?.u0, crop?.v0, crop?.u1, crop?.v1]) // eslint-disable-line react-hooks/exhaustive-deps
  useEffect(() => () => geometry.dispose(), [geometry])
  return (
    <group position={rosToThree(center, lift)} rotation={[0, rosYawToThreeY(yaw), 0]} visible={visible}>
      <mesh rotation={[-Math.PI / 2, 0, 0]} renderOrder={lift > 0 ? 1 : 0}>
        <primitive object={geometry} attach="geometry" />
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

/** Load outcomes of tile URLs; shared so the layers panel can show why the GPS layer is empty. */
let tileResults: TileResults = {}
const tileResultListeners = new Set<() => void>()

function reportTile(url: string, result: TileResult): void {
  tileResults = recordTileResult(tileResults, url, result)
  tileResultListeners.forEach((l) => l())
}

/** HTTP status of a tile URL (the image loader hides it), or null when the request itself fails. */
async function probeTileStatus(url: string): Promise<number | null> {
  try {
    return (await fetch(url)).status
  } catch {
    return null
  }
}

/** One tile: a textured ground rectangle showing a crop of its image; nothing is drawn while loading or on failure. */
const GpsTile = memo(function GpsTile({
  url,
  crop,
  center,
  width,
  height,
  yaw,
}: {
  url: string
  crop: TileCrop
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
      reportTile(url, { ok: true })
      return
    }
    setTexture(null)
    let cancelled = false
    new THREE.TextureLoader().load(
      url,
      (tex) => {
        tex.colorSpace = THREE.SRGBColorSpace
        TILE_TEXTURES.set(url, tex)
        reportTile(url, { ok: true })
        if (!cancelled) {
          setTexture(tex)
          invalidate()
        }
      },
      undefined,
      () => {
        log.debug('[map3d] tile load failed:', url)
        reportTile(url, { ok: false, status: null })
        void probeTileStatus(url).then((status) => reportTile(url, { ok: false, status }))
      },
    )
    return () => {
      cancelled = true
    }
  }, [url, invalidate])
  if (!texture) return null
  return (
    <GroundRect texture={texture} crop={crop} center={center} width={width} height={height} yaw={yaw} lift={LIFT.gps} />
  )
})

/**
 * Map tiles from the backend tile proxy, placed around the robot with the GPS anchor (Web Mercator). Above the
 * source's max zoom the parent tile at max zoom is fetched and each display tile shows its sub-rectangle.
 */
export const GpsTilesLayer = memo(function GpsTilesLayer({
  anchor,
  aroundX,
  aroundY,
  tileVersion,
  tileSource,
  zoom,
  maxZoom,
  onTileCaption,
}: {
  anchor: GpsAnchor
  aroundX: number
  aroundY: number
  tileVersion?: string | null
  tileSource?: string | null
  zoom: number
  maxZoom: number
  onTileCaption?: (caption: string | null) => void
}) {
  // Recomputing 25 placements per pose update is cheap; tiles are keyed by display tile so meshes persist.
  const tiles = useMemo(
    () => tilesAround(anchor, { x: aroundX, y: aroundY }, zoom, GPS_TILE_RADIUS, tileVersion, tileSource, maxZoom),
    [anchor, aroundX, aroundY, tileVersion, tileSource, zoom, maxZoom],
  )
  const urlKey = useMemo(() => [...new Set(tiles.map((t) => t.url))].sort().join('\n'), [tiles])
  useEffect(() => {
    if (!onTileCaption) return
    const urls = urlKey.split('\n')
    const report = () => onTileCaption(tileFailureCaption(tileResults, urls))
    tileResultListeners.add(report)
    report()
    return () => {
      tileResultListeners.delete(report)
      onTileCaption(null)
    }
  }, [urlKey, onTileCaption])
  return (
    <>
      {tiles.map((t) => (
        <GpsTile
          key={`${t.z}/${t.x}/${t.y}`}
          url={t.url}
          crop={t.crop}
          center={t.center}
          width={t.width}
          height={t.height}
          yaw={t.yaw}
        />
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

const BASE_POLE_HEIGHT_M = 0.6
const BASE_POLE_RADIUS_M = 0.03
const BASE_CONE_HEIGHT_M = 0.18
const BASE_CONE_RADIUS_M = 0.1
const BASE_RING_RADIUS_M = 0.3
const BASE_RING_SEGMENTS = 40

/** RTK base station marker: a pole with a cone on top, a ground ring and a label. */
export const BaseStationMarker = memo(function BaseStationMarker({ position, label }: { position: Vec2; label: string }) {
  const ring = useMemo(() => {
    const pts: ThreeTuple[] = []
    for (let i = 0; i <= BASE_RING_SEGMENTS; i++) {
      const a = (i / BASE_RING_SEGMENTS) * Math.PI * 2
      pts.push([Math.cos(a) * BASE_RING_RADIUS_M, 0, Math.sin(a) * BASE_RING_RADIUS_M])
    }
    return pts
  }, [])
  return (
    <group position={rosToThree(position, LIFT.gps + 0.002)}>
      <Line points={ring} color={SCENE_COLORS.rtkBase} lineWidth={3} />
      <mesh position={[0, BASE_POLE_HEIGHT_M / 2, 0]}>
        <cylinderGeometry args={[BASE_POLE_RADIUS_M, BASE_POLE_RADIUS_M, BASE_POLE_HEIGHT_M, 12]} />
        <meshStandardMaterial color={SCENE_COLORS.rtkBase} />
      </mesh>
      <mesh position={[0, BASE_POLE_HEIGHT_M + BASE_CONE_HEIGHT_M / 2, 0]}>
        <coneGeometry args={[BASE_CONE_RADIUS_M, BASE_CONE_HEIGHT_M, 16]} />
        <meshStandardMaterial color={SCENE_COLORS.rtkBase} />
      </mesh>
      <Html
        position={[0, BASE_POLE_HEIGHT_M + BASE_CONE_HEIGHT_M, 0]}
        center
        zIndexRange={[5, 0]}
        style={{ pointerEvents: 'none' }}
      >
        <MapLabel>{label}</MapLabel>
      </Html>
    </group>
  )
})

const OBSTACLE_POINT_SIZE = 0.04

/** Map-frame xyz (ROS) to the three.js y-up layout (x, z, -y). */
function rosXyzToThree(xyz: Float32Array): Float32Array {
  const out = new Float32Array(xyz.length)
  for (let i = 0; i < xyz.length; i += 3) {
    out[i] = xyz[i]
    out[i + 1] = xyz[i + 2]
    out[i + 2] = 0 - xyz[i + 1]
  }
  return out
}

/** One three.js point set; positions are three-layout xyz, colors optional (else the uniform color). */
function PointSet({
  positions,
  colors,
  color,
  size = OBSTACLE_POINT_SIZE,
}: {
  positions: Float32Array
  colors?: Float32Array
  color?: string
  size?: number
}) {
  const { invalidate } = useThree()
  const geometry = useMemo(() => {
    const g = new THREE.BufferGeometry()
    g.setAttribute('position', new THREE.BufferAttribute(positions, 3))
    if (colors) g.setAttribute('color', new THREE.BufferAttribute(colors, 3))
    return g
  }, [positions, colors])
  useEffect(() => {
    invalidate()
    return () => geometry.dispose()
  }, [geometry, invalidate])
  return (
    <points geometry={geometry} frustumCulled={false}>
      <pointsMaterial size={size} sizeAttenuation vertexColors={Boolean(colors)} color={colors ? undefined : color} />
    </points>
  )
}

/**
 * 3D obstacles layer: ToF cloud (coloured by height band) and lidar scan points, both already carrying the pose of
 * their sensor frame in the map frame. Malformed or cleared payloads draw nothing.
 */
export const ObstaclePointsLayer = memo(function ObstaclePointsLayer({
  cloud,
  scan,
  visible,
}: {
  cloud: unknown
  scan: unknown
  visible: boolean
}) {
  const tof = useMemo(() => {
    const xyz = visible ? cloudToMapPoints(cloud) : null
    return xyz && xyz.length > 0 ? { positions: rosXyzToThree(xyz), colors: tofColors(xyz) } : null
  }, [cloud, visible])
  const lidar = useMemo(() => {
    const xyz = visible ? scanToMapPoints(scan) : null
    return xyz && xyz.length > 0 ? rosXyzToThree(xyz) : null
  }, [scan, visible])
  if (!visible) return null
  return (
    <>
      {tof && <PointSet positions={tof.positions} colors={tof.colors} />}
      {lidar && <PointSet positions={lidar} color={LIDAR_COLOR} />}
    </>
  )
})

/**
 * Decode the map3d_cloud payload once per update.
 *
 * @param payload - raw map3d_cloud topic value
 * @param frameId - the tab's map3d_frame_id
 * @returns the decoded cloud, or null when absent, cleared or rejected
 */
export function useMap3dCloud(payload: unknown, frameId: string | undefined): DecodedMap3dCloud | null {
  return useMemo(() => decodeMap3dCloud(payload, frameId), [payload, frameId])
}

/** Measured 3D map: one point per voxel (point size = voxel), vertex colours when the payload has them. */
export const Map3dCloudLayer = memo(function Map3dCloudLayer({
  cloud,
  visible,
}: {
  cloud: DecodedMap3dCloud | null
  visible: boolean
}) {
  if (!visible || !cloud || cloud.count === 0) return null
  return <PointSet positions={cloud.positions} colors={cloud.colors} color={SCENE_COLORS.map3dPoints} size={cloud.voxel} />
})

/** Point size (m) of the head RGB-D cloud: the voxel edge the backend downsamples to (web_ui.bridge.RGBD_CLOUD_VOXEL_M). */
const RGBD_POINT_SIZE_M = 0.02

/**
 * Head RGB-D layer: the coloured head_rgbd cloud placed in the map frame by the pose its sensor frame had at the
 * message stamp. A payload without a pose or a malformed one draws nothing.
 */
export const HeadRgbdLayer = memo(function HeadRgbdLayer({ payload, visible }: { payload: unknown; visible: boolean }) {
  const cloud = useMemo(() => {
    const placed = visible ? rgbdCloudToMap(payload) : null
    return placed && placed.count > 0 ? { positions: rosXyzToThree(placed.positions), colors: placed.colors } : null
  }, [payload, visible])
  if (!visible || !cloud) return null
  return <PointSet positions={cloud.positions} colors={cloud.colors} color={SCENE_COLORS.map3dPoints} size={RGBD_POINT_SIZE_M} />
})

/** Point size (m) of the ground cloud: the voxel edge the backend downsamples to (web_ui.cloud_serializer.GROUND_CLOUD_VOXEL_M). */
const GROUND_POINT_SIZE_M = 0.02
const GROUND_PATCH_OPACITY = 0.25
const GROUND_WARNING_OPACITY = 0.35
/** Lift (m) of the plane overlays above the fitted plane, so they do not z-fight with it. */
const GROUND_OVERLAY_LIFT_M = 0.004
const GROUND_EDGE_WIDTH = 3

/** base_link xyz (ROS) to the three.js position inside the base group. */
function baseToThree([x, y, z]: Vec3Tuple): ThreeTuple {
  return [x, z, 0 - y]
}

/** A translucent convex polygon (triangle fan) from three-layout vertices. */
function FanMesh({ vertices, color, opacity }: { vertices: ThreeTuple[]; color: string; opacity: number }) {
  const { invalidate } = useThree()
  const geometry = useMemo(() => {
    const g = new THREE.BufferGeometry()
    const pos: number[] = []
    for (let i = 1; i + 1 < vertices.length; i++) pos.push(...vertices[0], ...vertices[i], ...vertices[i + 1])
    g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3))
    return g
  }, [vertices])
  useEffect(() => {
    invalidate()
    return () => geometry.dispose()
  }, [geometry, invalidate])
  return (
    <mesh geometry={geometry} frustumCulled={false}>
      <meshBasicMaterial color={color} transparent opacity={opacity} side={THREE.DoubleSide} depthWrite={false} />
    </mesh>
  )
}

/**
 * Ground layer: the labelled points (coloured per label, placed by the cloud frame pose), and, placed by the base_link
 * pose, the fitted plane patch, the obstacle/warning edge lines and the translucent warning areas. Nothing is drawn from
 * the plane, edges or warning areas when the state is absent, stale or has no plane; the points are hidden with a stale state.
 */
export const GroundLayer = memo(function GroundLayer({
  cloud,
  state,
  pose,
  visible,
}: {
  cloud: unknown
  state: GroundState | null | undefined
  pose: Pose2D | null
  visible: boolean
}) {
  const stale = state?.stale === true
  const points = useMemo(() => {
    const decoded = visible && !stale ? decodeGroundCloud(cloud) : null
    return decoded && decoded.count > 0 ? { positions: rosXyzToThree(decoded.positions), colors: decoded.labelColors } : null
  }, [cloud, visible, stale])
  const drawn = visible && groundVisible(state)
  const overlay = useMemo(() => {
    if (!drawn || !state?.plane) return null
    const plane = state.plane
    const lift = (x: number, y: number): Vec3Tuple => [x, y, (planeZ(plane, x, y) ?? plane.height_m) + GROUND_OVERLAY_LIFT_M]
    return {
      patch: planePatch(state)?.map(baseToThree) ?? null,
      edges: regionEdges(state).map((e) => ({ ...e, line: [baseToThree(lift(...e.a)), baseToThree(lift(...e.b))] })),
      areas: warningPolygons(state).map((w) => ({ ...w, vertices: w.polygon.map(([x, y]) => baseToThree(lift(x, y))) })),
    }
  }, [drawn, state])
  const p = pose ?? { x: 0, y: 0, yaw: 0 }
  return (
    <>
      {points && <PointSet positions={points.positions} colors={points.colors} size={GROUND_POINT_SIZE_M} />}
      {overlay && (
        <group position={rosToThree(p)} rotation={[0, rosYawToThreeY(p.yaw), 0]}>
          {overlay.patch && <FanMesh vertices={overlay.patch} color={GROUND_COLORS.plane} opacity={GROUND_PATCH_OPACITY} />}
          {overlay.areas.map((a, i) => (
            <FanMesh key={`${a.source}-${i}`} vertices={a.vertices} color={GROUND_COLORS.warningFill} opacity={GROUND_WARNING_OPACITY} />
          ))}
          {overlay.edges.map((e) => (
            <Line key={e.name} points={e.line} color={e.colour} lineWidth={GROUND_EDGE_WIDTH} />
          ))}
        </group>
      )}
    </>
  )
})

const WALL_MIN_INTERVAL_MS = 1000
const WALL_OPACITY = 0.35

/**
 * Hold back a changing value so it updates at most once per interval (the latest value always arrives, trailing).
 *
 * @param value - the live value
 * @param intervalMs - minimum time between updates
 * @returns the throttled value
 */
function useThrottled<T>(value: T, intervalMs: number): T {
  const [held, setHeld] = useState(value)
  const last = useRef(0)
  useEffect(() => {
    const wait = Math.max(0, last.current + intervalMs - Date.now())
    const t = setTimeout(() => {
      last.current = Date.now()
      setHeld(value)
    }, wait)
    return () => clearTimeout(t)
  }, [value, intervalMs])
  return held
}

/** Decode the SLAM map PNG into grey cells (image order) with its placement; null while decoding or absent. */
function useWallGrid(msg: GridImageMsg | null | undefined): WallGrid | null {
  const [grid, setGrid] = useState<WallGrid | null>(null)
  const png = msg?.png_b64
  const width = msg?.width
  const height = msg?.height
  const resolution = msg?.resolution
  const origin = msg?.origin
  useEffect(() => {
    if (!png || !width || !height || !resolution || !origin) {
      setGrid(null)
      return
    }
    let cancelled = false
    const img = new Image()
    img.onload = () => {
      if (cancelled) return
      const canvas = document.createElement('canvas')
      canvas.width = width
      canvas.height = height
      const ctx = canvas.getContext('2d')
      if (!ctx) return
      ctx.drawImage(img, 0, 0)
      const rgba = ctx.getImageData(0, 0, width, height).data
      const cells = new Uint8Array(width * height)
      for (let i = 0; i < cells.length; i++) cells[i] = rgba[i * 4]
      setGrid({ width, height, resolution, origin, cells })
    }
    img.onerror = () => log.warn('[map3d] failed to decode map image for lidar walls')
    img.src = `data:image/png;base64,${png}`
    return () => {
      cancelled = true
    }
  }, [png, width, height, resolution, origin])
  return grid
}

/**
 * Lidar-only structure: occupied SLAM map cells extruded to heightM, translucent and in their own flat colour without
 * vertex colours, so they are never mistaken for measured points. Columns holding a measured point are left out.
 * Recomputed when the map or the cloud changes, at most once per second.
 */
export const LidarWallsLayer = memo(function LidarWallsLayer({
  map,
  cloud,
  heightM,
  visible,
}: {
  map: GridImageMsg | null | undefined
  cloud: DecodedMap3dCloud | null
  heightM: number
  visible: boolean
}) {
  const live = useWallGrid(visible ? map : null)
  const input = useMemo(() => ({ grid: live, cloud, heightM }), [live, cloud, heightM])
  const { grid, cloud: heldCloud, heightM: height } = useThrottled(input, WALL_MIN_INTERVAL_MS)
  const boxes = useMemo(() => {
    if (!grid) return []
    const measured = heldCloud ? measuredColumnsFromCloud(heldCloud.positions, grid.resolution, grid.origin) : new Set<string>()
    return buildLidarWalls(grid, measured, height)
  }, [grid, heldCloud, height])
  const { invalidate } = useThree()
  const mesh = useRef<THREE.InstancedMesh>(null)
  useEffect(() => {
    const m = mesh.current
    if (!m) return
    const obj = new THREE.Object3D()
    boxes.forEach((b, i) => {
      const [x, y, z] = rosToThree({ x: b.center.x, y: b.center.y, z: b.height / 2 })
      obj.position.set(x, y, z)
      obj.rotation.set(0, rosYawToThreeY(b.yaw), 0)
      obj.scale.set(b.length, b.height, b.width)
      obj.updateMatrix()
      m.setMatrixAt(i, obj.matrix)
    })
    m.count = boxes.length
    m.instanceMatrix.needsUpdate = true
    invalidate()
  }, [boxes, invalidate])
  if (!visible || boxes.length === 0) return null
  return (
    <instancedMesh key={boxes.length} ref={mesh} args={[undefined, undefined, boxes.length]} frustumCulled={false}>
      <boxGeometry args={[1, 1, 1]} />
      <meshBasicMaterial color={SCENE_COLORS.lidarWall} transparent opacity={WALL_OPACITY} depthWrite={false} />
    </instancedMesh>
  )
})
