import { useEffect, useRef } from 'react'
import uPlot from 'uplot'
import 'uplot/dist/uPlot.min.css'
import log from '../logging'
import { TabConfig } from '../types'
import { extractField } from '../utils/fieldExtract'

interface Props {
  tab: TabConfig
  topicData: Record<string, unknown>
  publish: (topic: string, msgType: string, data: unknown) => void
}

export default function SensorGraphTab({ tab, topicData }: Props) {
  const containerRef = useRef<HTMLDivElement>(null)
  const plotRef = useRef<uPlot | null>(null)
  const bufferRef = useRef<number[][]>([[]])

  const topics = tab.topics ?? []
  const seriesCount = topics.reduce((n, ts) => n + ts.fields.length, 0)
  const windowMs = (tab.window_s ?? 10) * 1000
  const maxPoints = tab.max_points ?? 500

  useEffect(() => {
    if (!containerRef.current || seriesCount === 0) return

    const series: uPlot.Series[] = [
      {},
      ...topics.flatMap((ts) =>
        ts.fields.map((f) => ({
          label: f.label,
          stroke: f.color ?? '#888',
          width: 1,
        }))
      ),
    ]

    const opts: uPlot.Options = {
      width: containerRef.current.clientWidth,
      height: containerRef.current.clientHeight - 40,
      scales: { x: { time: false }, y: {} },
      series,
      axes: [
        { stroke: '#444', ticks: { stroke: '#333' }, grid: { stroke: '#222' } },
        { stroke: '#444', ticks: { stroke: '#333' }, grid: { stroke: '#222' } },
      ],
    }

    bufferRef.current = Array.from({ length: seriesCount + 1 }, () => [])
    plotRef.current = new uPlot(opts, bufferRef.current as uPlot.AlignedData, containerRef.current)

    return () => {
      plotRef.current?.destroy()
      plotRef.current = null
    }
  }, [seriesCount, tab.id])  // eslint-disable-line react-hooks/exhaustive-deps

  useEffect(() => {
    if (!plotRef.current || seriesCount === 0) return
    const now = performance.now()
    const buf = bufferRef.current

    let seriesIdx = 1
    let hasUpdate = false
    for (const ts of topics) {
      const msgData = topicData[ts.topic] as Record<string, unknown> | undefined
      if (!msgData) { seriesIdx += ts.fields.length; continue }
      for (const f of ts.fields) {
        const val = extractField(msgData, f.path)
        if (val !== null) {
          buf[0].push(now)
          buf[seriesIdx].push(val)
          hasUpdate = true
        }
        seriesIdx++
      }
    }

    if (!hasUpdate) return

    const cutoff = now - windowMs
    const firstKeep = buf[0].findIndex((t) => t >= cutoff)
    if (firstKeep > 0) {
      for (let i = 0; i < buf.length; i++) buf[i] = buf[i].slice(firstKeep)
    }
    if (buf[0].length > maxPoints) {
      const trim = buf[0].length - maxPoints
      for (let i = 0; i < buf.length; i++) buf[i] = buf[i].slice(trim)
    }

    log.debug('[graph] append: topics updated, points:', buf[0].length)
    plotRef.current.setData(buf as uPlot.AlignedData)
  }, [topicData])  // eslint-disable-line react-hooks/exhaustive-deps

  return (
    <div
      ref={containerRef}
      style={{ width: '100%', height: '100%', background: '#050505' }}
    />
  )
}
