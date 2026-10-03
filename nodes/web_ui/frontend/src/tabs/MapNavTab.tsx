import { useEffect, useRef, useState } from 'react'
import type { CSSProperties, PointerEvent as ReactPointerEvent } from 'react'
import log from '../logging'
import {
  centerOn,
  fitView,
  goalFromGesture,
  MapMeta,
  mapImageTransform,
  niceScaleBar,
  panView,
  pinchView,
  Pose2D,
  screenToWorld,
  Vec2,
  View,
  worldToScreen,
  yawToQuaternion,
  zoomAboutPoint,
} from '../map/mapMath'
import { TabConfig } from '../types'

interface Props {
  tab: TabConfig
  topicData: Record<string, unknown>
  publish: (topic: string, msgType: string, data: unknown) => void
}

/** OccupancyGrid as serialized by the backend (msg_serializer.serialize_occupancy_grid). */
interface MapMsg extends MapMeta {
  png_b64: string
  frame_id: string
  stamp: number
}

interface PathMsg {
  frame_id: string
  points: [number, number][]
}

interface FramedPose extends Pose2D {
  frame_id: string
}

interface MapLayer {
  img: HTMLImageElement
  meta: MapMeta
}

interface DraftGoal {
  press: Vec2 // world
  current: Vec2 // world
  dragPx: number
}

type SaveState = { state: 'idle' | 'saving' | 'ok' | 'error'; message: string }

const ROBOT_POSE_TOPIC = '/web_ui/robot_pose'
const DEFAULT_SCALE = 50 // px per metre before a map arrives
const WHEEL_ZOOM_BASE = 1.0015 // zoom factor per wheel delta pixel

const COLORS = {
  background: '#1a1a1a',
  free: '#fefefe',
  occupied: '#000000',
  unknown: '#cdcdcd',
  robot: '#2f9bff',
  globalPath: '#2ecc40',
  localPath: '#ff851b',
  goal: '#ff4136',
  draft: '#ffdc00',
  text: '#ddd',
}

const LEGEND: [string, string][] = [
  ['Free', COLORS.free],
  ['Occupied', COLORS.occupied],
  ['Unknown', COLORS.unknown],
  ['Robot', COLORS.robot],
  ['Global path', COLORS.globalPath],
  ['Local path', COLORS.localPath],
  ['Goal', COLORS.goal],
]

const buttonStyle: CSSProperties = {
  background: '#2a2a2a',
  color: '#ddd',
  border: '1px solid #444',
  borderRadius: 4,
  padding: '4px 10px',
  fontSize: 13,
  cursor: 'pointer',
}

function inFrame(data: { frame_id?: string } | undefined, mapFrame: string): boolean {
  return Boolean(data) && (!data!.frame_id || data!.frame_id === mapFrame)
}

function drawArrow(ctx: CanvasRenderingContext2D, at: Vec2, yaw: number, length: number) {
  // Screen y is down, so the world heading flips sign.
  const tip = { x: at.x + Math.cos(yaw) * length, y: at.y - Math.sin(yaw) * length }
  const head = Math.min(10, length * 0.4)
  ctx.beginPath()
  ctx.moveTo(at.x, at.y)
  ctx.lineTo(tip.x, tip.y)
  for (const side of [-1, 1]) {
    const a = yaw + Math.PI + side * 0.45
    ctx.moveTo(tip.x, tip.y)
    ctx.lineTo(tip.x + Math.cos(a) * head, tip.y - Math.sin(a) * head)
  }
  ctx.stroke()
}

function drawPath(ctx: CanvasRenderingContext2D, view: View, path: PathMsg, color: string, width: number) {
  if (path.points.length < 2) return
  ctx.strokeStyle = color
  ctx.lineWidth = width
  ctx.setLineDash([])
  ctx.beginPath()
  path.points.forEach(([x, y], i) => {
    const s = worldToScreen(view, { x, y })
    if (i === 0) ctx.moveTo(s.x, s.y)
    else ctx.lineTo(s.x, s.y)
  })
  ctx.stroke()
}

function drawGoal(ctx: CanvasRenderingContext2D, view: View, goal: Pose2D, color: string, dashed: boolean) {
  const s = worldToScreen(view, goal)
  ctx.strokeStyle = color
  ctx.lineWidth = 2.5
  ctx.setLineDash(dashed ? [4, 3] : [])
  ctx.beginPath()
  ctx.arc(s.x, s.y, 7, 0, 2 * Math.PI)
  ctx.stroke()
  drawArrow(ctx, s, goal.yaw, 26)
  ctx.setLineDash([])
}

function drawRobot(ctx: CanvasRenderingContext2D, view: View, pose: Pose2D) {
  const s = worldToScreen(view, pose)
  const size = 12
  const pts = [
    [size, 0],
    [-size * 0.7, size * 0.6],
    [-size * 0.35, 0],
    [-size * 0.7, -size * 0.6],
  ].map(([fx, fy]) => ({
    x: s.x + fx * Math.cos(pose.yaw) - fy * Math.sin(pose.yaw),
    y: s.y - (fx * Math.sin(pose.yaw) + fy * Math.cos(pose.yaw)),
  }))
  ctx.fillStyle = COLORS.robot
  ctx.strokeStyle = '#fff'
  ctx.lineWidth = 1.5
  ctx.beginPath()
  pts.forEach((p, i) => (i === 0 ? ctx.moveTo(p.x, p.y) : ctx.lineTo(p.x, p.y)))
  ctx.closePath()
  ctx.fill()
  ctx.stroke()
}

function drawScaleBar(ctx: CanvasRenderingContext2D, view: View, height: number) {
  const bar = niceScaleBar(view.scale, 120)
  const x0 = 16
  const y0 = height - 18
  ctx.strokeStyle = COLORS.text
  ctx.fillStyle = COLORS.text
  ctx.lineWidth = 2
  ctx.beginPath()
  ctx.moveTo(x0, y0 - 5)
  ctx.lineTo(x0, y0)
  ctx.lineTo(x0 + bar.px, y0)
  ctx.lineTo(x0 + bar.px, y0 - 5)
  ctx.stroke()
  ctx.font = '12px monospace'
  ctx.textAlign = 'left'
  const label = bar.meters >= 1 ? `${bar.meters} m` : `${Math.round(bar.meters * 100)} cm`
  ctx.fillText(label, x0 + 4, y0 - 8)
}

export default function MapNavTab({ tab, topicData, publish }: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const containerRef = useRef<HTMLDivElement>(null)
  const pointersRef = useRef<Map<number, Vec2>>(new Map())
  const panningRef = useRef(false)
  const fittedRef = useRef(false)

  const [size, setSize] = useState({ w: 0, h: 0 })
  const [view, setView] = useState<View | null>(null)
  const [mapLayer, setMapLayer] = useState<MapLayer | null>(null)
  const [goalMode, setGoalMode] = useState(false)
  const [draft, setDraft] = useState<DraftGoal | null>(null)
  const [save, setSave] = useState<SaveState>({ state: 'idle', message: '' })

  const mapFrame = tab.map_frame ?? 'map'
  // Each entry keeps its identity until its own topic updates, so effects below skip unrelated traffic.
  const mapMsg = topicData[tab.map_topic ?? ''] as MapMsg | undefined
  const globalPath = topicData[tab.global_plan_topic ?? ''] as PathMsg | undefined
  const localPath = topicData[tab.local_plan_topic ?? ''] as PathMsg | undefined
  const goalMsg = topicData[tab.goal_topic ?? ''] as FramedPose | undefined
  const poseMsg = topicData[ROBOT_POSE_TOPIC] as FramedPose | undefined
  const robotPose = inFrame(poseMsg, mapFrame) ? poseMsg! : null

  // Canvas backing store follows the CSS size and device pixel ratio.
  useEffect(() => {
    const container = containerRef.current
    if (!container) return
    const ro = new ResizeObserver((entries) => {
      for (const entry of entries) {
        setSize({ w: Math.floor(entry.contentRect.width), h: Math.floor(entry.contentRect.height) })
      }
    })
    ro.observe(container)
    return () => ro.disconnect()
  }, [])

  // Decode the map PNG only when the image payload or its placement changes.
  const pngB64 = mapMsg?.png_b64
  const mapW = mapMsg?.width ?? 0
  const mapH = mapMsg?.height ?? 0
  const mapRes = mapMsg?.resolution ?? 0
  const originX = mapMsg?.origin.x ?? 0
  const originY = mapMsg?.origin.y ?? 0
  const originYaw = mapMsg?.origin.yaw ?? 0
  useEffect(() => {
    if (!pngB64) return
    let cancelled = false
    const meta: MapMeta = {
      width: mapW,
      height: mapH,
      resolution: mapRes,
      origin: { x: originX, y: originY, yaw: originYaw },
    }
    const img = new Image()
    img.onload = () => {
      if (cancelled) return
      log.debug('[map] decoded map image', meta.width, 'x', meta.height)
      setMapLayer({ img, meta })
    }
    img.onerror = () => log.warn('[map] failed to decode map image')
    img.src = `data:image/png;base64,${pngB64}`
    return () => {
      cancelled = true
    }
  }, [pngB64, mapW, mapH, mapRes, originX, originY, originYaw])

  // Initial view: fit the map once it is known, otherwise centre the world origin.
  useEffect(() => {
    if (size.w === 0 || size.h === 0) return
    if (!fittedRef.current && mapLayer) {
      fittedRef.current = true
      setView(fitView(mapLayer.meta, size.w, size.h))
    } else if (!view) {
      setView(centerOn({ scale: DEFAULT_SCALE, offsetX: 0, offsetY: 0 }, { x: 0, y: 0 }, size.w, size.h))
    }
  }, [mapLayer, size, view])

  // Wheel zoom about the cursor (native listener: React's onWheel is passive and cannot preventDefault).
  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const onWheel = (e: WheelEvent) => {
      e.preventDefault()
      const rect = canvas.getBoundingClientRect()
      const at = { x: e.clientX - rect.left, y: e.clientY - rect.top }
      const factor = Math.pow(WHEEL_ZOOM_BASE, -e.deltaY)
      setView((v) => (v ? zoomAboutPoint(v, at, factor) : v))
    }
    canvas.addEventListener('wheel', onWheel, { passive: false })
    return () => canvas.removeEventListener('wheel', onWheel)
  }, [])

  // Redraw only when map, overlays, view, gesture or size change.
  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas || size.w === 0 || size.h === 0) return
    const dpr = window.devicePixelRatio || 1
    if (canvas.width !== Math.round(size.w * dpr) || canvas.height !== Math.round(size.h * dpr)) {
      canvas.width = Math.round(size.w * dpr)
      canvas.height = Math.round(size.h * dpr)
    }
    const ctx = canvas.getContext('2d')
    if (!ctx) return
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
    ctx.fillStyle = COLORS.background
    ctx.fillRect(0, 0, size.w, size.h)
    if (!view) return

    if (mapLayer) {
      const [a, b, c, d, e, f] = mapImageTransform(mapLayer.meta, view)
      ctx.setTransform(dpr * a, dpr * b, dpr * c, dpr * d, dpr * e, dpr * f)
      ctx.imageSmoothingEnabled = false
      ctx.drawImage(mapLayer.img, 0, 0)
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
    } else {
      ctx.fillStyle = '#777'
      ctx.font = '14px monospace'
      ctx.textAlign = 'center'
      ctx.fillText(`Waiting for map on ${tab.map_topic ?? '?'}...`, size.w / 2, 24)
    }

    if (globalPath && inFrame(globalPath, mapFrame)) drawPath(ctx, view, globalPath, COLORS.globalPath, 2.5)
    if (localPath && inFrame(localPath, mapFrame)) drawPath(ctx, view, localPath, COLORS.localPath, 2)
    if (goalMsg && inFrame(goalMsg, mapFrame)) drawGoal(ctx, view, goalMsg, COLORS.goal, false)
    if (draft) {
      const dragged = goalFromGesture(draft.press, draft.current, draft.dragPx, robotPose)
      drawGoal(ctx, view, dragged, COLORS.draft, true)
    }
    if (robotPose) drawRobot(ctx, view, robotPose)
    drawScaleBar(ctx, view, size.h)
  }, [mapLayer, globalPath, localPath, goalMsg, robotPose, view, draft, size, mapFrame, tab.map_topic])

  const toCanvas = (e: ReactPointerEvent<HTMLCanvasElement>): Vec2 => {
    const rect = e.currentTarget.getBoundingClientRect()
    return { x: e.clientX - rect.left, y: e.clientY - rect.top }
  }

  const sendGoal = (goal: Pose2D) => {
    if (!tab.goal_topic) return
    log.info('[map] goal', goal.x.toFixed(2), goal.y.toFixed(2), 'yaw', goal.yaw.toFixed(2), 'in', mapFrame)
    publish(tab.goal_topic, 'geometry_msgs/PoseStamped', {
      header: { frame_id: mapFrame, stamp: { sec: 0, nanosec: 0 } }, // backend stamps with the node clock
      pose: { position: { x: goal.x, y: goal.y, z: 0 }, orientation: yawToQuaternion(goal.yaw) },
    })
  }

  const onPointerDown = (e: ReactPointerEvent<HTMLCanvasElement>) => {
    if (!view) return
    e.currentTarget.setPointerCapture(e.pointerId)
    const at = toCanvas(e)
    pointersRef.current.set(e.pointerId, at)
    if (pointersRef.current.size >= 2) {
      // Second finger: switch to pinch, abandon any goal or pan gesture.
      setDraft(null)
      panningRef.current = false
      return
    }
    if (goalMode && e.button === 0) {
      const w = screenToWorld(view, at)
      setDraft({ press: w, current: w, dragPx: 0 })
    } else {
      panningRef.current = true
    }
  }

  const onPointerMove = (e: ReactPointerEvent<HTMLCanvasElement>) => {
    const pointers = pointersRef.current
    const prev = pointers.get(e.pointerId)
    if (!prev || !view) return
    const at = toCanvas(e)
    if (pointers.size === 2) {
      const [idA, idB] = [...pointers.keys()]
      const before: [Vec2, Vec2] = [pointers.get(idA)!, pointers.get(idB)!]
      pointers.set(e.pointerId, at)
      const after: [Vec2, Vec2] = [pointers.get(idA)!, pointers.get(idB)!]
      setView((v) => (v ? pinchView(v, before, after) : v))
      return
    }
    pointers.set(e.pointerId, at)
    if (draft) {
      const pressScreen = worldToScreen(view, draft.press)
      setDraft({
        press: draft.press,
        current: screenToWorld(view, at),
        dragPx: Math.max(draft.dragPx, Math.hypot(at.x - pressScreen.x, at.y - pressScreen.y)),
      })
    } else if (panningRef.current) {
      setView((v) => (v ? panView(v, at.x - prev.x, at.y - prev.y) : v))
    }
  }

  const endPointer = (e: ReactPointerEvent<HTMLCanvasElement>, commit: boolean) => {
    pointersRef.current.delete(e.pointerId)
    if (pointersRef.current.size > 0) return
    panningRef.current = false
    if (draft && commit && view) {
      const release = screenToWorld(view, toCanvas(e))
      sendGoal(goalFromGesture(draft.press, release, draft.dragPx, robotPose))
      setGoalMode(false)
    }
    setDraft(null)
  }

  const saveMap = async () => {
    setSave({ state: 'saving', message: 'Saving map...' })
    try {
      const resp = await fetch(`/api/map/save?tab=${encodeURIComponent(tab.id)}`, { method: 'POST' })
      const body = (await resp.json()) as { ok?: boolean; message?: string }
      setSave({ state: body.ok ? 'ok' : 'error', message: body.message ?? `HTTP ${resp.status}` })
    } catch (err) {
      setSave({ state: 'error', message: `Save failed: ${String(err)}` })
    }
  }

  const saveColor = save.state === 'ok' ? '#2ecc40' : save.state === 'error' ? '#ff4136' : '#aaa'

  return (
    <div ref={containerRef} style={{ width: '100%', height: '100%', position: 'relative', overflow: 'hidden' }}>
      <canvas
        ref={canvasRef}
        style={{
          display: 'block',
          width: '100%',
          height: '100%',
          touchAction: 'none',
          cursor: goalMode ? 'crosshair' : 'grab',
        }}
        onPointerDown={onPointerDown}
        onPointerMove={onPointerMove}
        onPointerUp={(e) => endPointer(e, true)}
        onPointerCancel={(e) => endPointer(e, false)}
        onContextMenu={(e) => e.preventDefault()}
      />

      <div style={{ position: 'absolute', top: 8, left: 8, display: 'flex', gap: 6, flexWrap: 'wrap', maxWidth: '70%' }}>
        <button
          type="button"
          style={{ ...buttonStyle, ...(goalMode ? { background: '#7a5c00', borderColor: COLORS.draft } : {}) }}
          onClick={() => setGoalMode((m) => !m)}
          disabled={!tab.goal_topic}
          title="Press on the map to set the goal position; drag to set its heading"
        >
          {goalMode ? 'Click map to set goal' : 'Set goal'}
        </button>
        <button
          type="button"
          style={buttonStyle}
          disabled={!robotPose || !view}
          onClick={() => robotPose && view && setView(centerOn(view, robotPose, size.w, size.h))}
        >
          Center on robot
        </button>
        <button
          type="button"
          style={buttonStyle}
          disabled={!mapLayer}
          onClick={() => mapLayer && setView(fitView(mapLayer.meta, size.w, size.h))}
        >
          Fit map
        </button>
        <button type="button" style={buttonStyle} disabled={save.state === 'saving'} onClick={saveMap}>
          Save map
        </button>
        {save.message && <span style={{ color: saveColor, fontSize: 12, alignSelf: 'center' }}>{save.message}</span>}
      </div>

      <div
        style={{
          position: 'absolute',
          top: 8,
          right: 8,
          background: 'rgba(20,20,20,0.85)',
          border: '1px solid #333',
          borderRadius: 4,
          padding: '6px 8px',
          fontSize: 12,
          color: COLORS.text,
        }}
      >
        {LEGEND.map(([label, color]) => (
          <div key={label} style={{ display: 'flex', alignItems: 'center', gap: 6, lineHeight: '18px' }}>
            <span style={{ width: 12, height: 12, background: color, border: '1px solid #555', display: 'inline-block' }} />
            {label}
          </div>
        ))}
        <div style={{ marginTop: 4, color: '#888' }}>
          {robotPose ? `Robot ${robotPose.x.toFixed(2)}, ${robotPose.y.toFixed(2)}` : `No ${mapFrame} pose`}
        </div>
      </div>
    </div>
  )
}
