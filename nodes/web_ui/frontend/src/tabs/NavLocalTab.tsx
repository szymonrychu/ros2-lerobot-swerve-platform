import { useEffect, useRef } from 'react'
import log from '../logging'
import { TabConfig } from '../types'

interface Props {
  tab: TabConfig
  topicData: Record<string, unknown>
  publish: (topic: string, msgType: string, data: unknown) => void
}

export default function NavLocalTab({ tab, topicData, publish }: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null)

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ctx = canvas.getContext('2d')!

    const costmapData = topicData[tab.costmap_topic ?? ''] as {
      info?: { width: number; height: number; resolution: number; origin?: { position?: { x: number; y: number } } }
      data?: number[]
    } | undefined
    const scanData = topicData[tab.scan_topic ?? ''] as {
      angle_min?: number; angle_increment?: number; ranges?: number[]
    } | undefined
    const odomData = topicData[tab.odom_topic ?? ''] as {
      pose?: { pose?: { position?: { x: number; y: number }; orientation?: { z: number; w: number } } }
    } | undefined

    const W = canvas.width
    const H = canvas.height
    ctx.fillStyle = '#050505'
    ctx.fillRect(0, 0, W, H)

    if (costmapData?.info && costmapData.data) {
      const { width: gw, height: gh } = costmapData.info
      const cellPx = Math.max(1, Math.floor(Math.min(W / gw, H / gh)))
      const ox = (W - gw * cellPx) / 2
      const oy = (H - gh * cellPx) / 2
      for (let i = 0; i < costmapData.data.length; i++) {
        const val = costmapData.data[i]
        ctx.fillStyle = val > 50 ? '#444' : '#111'
        const gx = i % gw
        const gy = Math.floor(i / gw)
        ctx.fillRect(ox + gx * cellPx, oy + (gh - 1 - gy) * cellPx, cellPx, cellPx)
      }
    }

    if (scanData?.ranges) {
      ctx.fillStyle = '#0f0'
      const cx = W / 2, cy = H / 2
      const scale = costmapData?.info ? 1 / costmapData.info.resolution * Math.min(W, H) / (costmapData.info.width ?? 100) : 10
      for (let i = 0; i < scanData.ranges.length; i++) {
        const r = scanData.ranges[i]
        if (!isFinite(r) || r <= 0) continue
        const angle = (scanData.angle_min ?? 0) + i * (scanData.angle_increment ?? 0)
        ctx.fillRect(cx + Math.cos(angle) * r * scale - 1, cy - Math.sin(angle) * r * scale - 1, 2, 2)
      }
    }

    if (odomData?.pose?.pose?.position) {
      const orientation = odomData.pose.pose.orientation
      const yaw = orientation ? 2 * Math.atan2(orientation.z, orientation.w) : 0
      const cx = W / 2, cy = H / 2
      const arrowLen = 14
      ctx.strokeStyle = '#fff'
      ctx.lineWidth = 2
      ctx.beginPath()
      ctx.moveTo(cx, cy)
      ctx.lineTo(cx + Math.cos(yaw) * arrowLen, cy - Math.sin(yaw) * arrowLen)
      ctx.stroke()
    }

    log.debug('[nav] repaint: costmap', costmapData?.info?.width, 'x', costmapData?.info?.height)
  }, [topicData, tab])

  const handleClick = (evt: React.MouseEvent<HTMLCanvasElement>) => {
    if (!tab.goal_topic) return
    const canvas = canvasRef.current!
    const rect = canvas.getBoundingClientRect()
    const px = evt.clientX - rect.left
    const py = evt.clientY - rect.top
    const nx = (px / canvas.width - 0.5) * 10
    const ny = (0.5 - py / canvas.height) * 10
    publish(tab.goal_topic, 'geometry_msgs/PoseStamped', {
      header: { frame_id: 'map' },
      pose: { position: { x: nx, y: ny, z: 0.0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
    })
  }

  return (
    <canvas
      ref={canvasRef}
      width={800}
      height={600}
      style={{ width: '100%', height: '100%', cursor: 'crosshair' }}
      onClick={handleClick}
    />
  )
}
