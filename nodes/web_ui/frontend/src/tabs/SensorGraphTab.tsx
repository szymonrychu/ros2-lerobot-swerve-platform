import { useEffect, useRef } from 'react'
import uPlot from 'uplot'
import 'uplot/dist/uPlot.min.css'
import log from '../logging'
import { TabConfig } from '../types'
import { extractField } from '../utils/fieldExtract'
import { getOrCreateBuffer, setBuffer } from '../utils/graphBuffers'

interface Props {
  tab: TabConfig
  topicData: Record<string, unknown>
  publish: (topic: string, msgType: string, data: unknown) => void
}

export default function SensorGraphTab({ tab, topicData }: Props) {
  const containerRef = useRef<HTMLDivElement>(null)
  const plotRef = useRef<uPlot | null>(null)

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

    // Restore or create persistent buffer — survives tab switches
    const buf = getOrCreateBuffer(tab.id, seriesCount)
    plotRef.current = new uPlot(opts, buf as uPlot.AlignedData, containerRef.current)

    return () => {
      plotRef.current?.destroy()
      plotRef.current = null
    }
  }, [seriesCount, tab.id])  // eslint-disable-line react-hooks/exhaustive-deps

  useEffect(() => {
    if (!plotRef.current || seriesCount === 0) return
    const now = performance.now()
    const buf = getOrCreateBuffer(tab.id, seriesCount)

    // Check whether any series has new data before touching the buffer
    let hasUpdate = false
    for (const ts of topics) {
      const msgData = topicData[ts.topic] as Record<string, unknown> | undefined
      if (!msgData) continue
      for (const f of ts.fields) {
        if (extractField(msgData, f.path) !== null) { hasUpdate = true; break }
      }
      if (hasUpdate) break
    }
    if (!hasUpdate) return

    // Push exactly one timestamp per update cycle so all series arrays stay aligned
    buf[0].push(now)
    let seriesIdx = 1
    for (const ts of topics) {
      const msgData = topicData[ts.topic] as Record<string, unknown> | undefined
      for (const f of ts.fields) {
        const val = msgData ? extractField(msgData, f.path) : null
        buf[seriesIdx].push(val ?? NaN)
        seriesIdx++
      }
    }

    const cutoff = now - windowMs
    const firstKeep = buf[0].findIndex((t) => t >= cutoff)
    if (firstKeep > 0) {
      const trimmed = buf.map((arr) => arr.slice(firstKeep))
      trimmed.forEach((arr, i) => { buf[i] = arr })
      setBuffer(tab.id, buf)
    }
    if (buf[0].length > maxPoints) {
      const trim = buf[0].length - maxPoints
      const trimmed = buf.map((arr) => arr.slice(trim))
      trimmed.forEach((arr, i) => { buf[i] = arr })
      setBuffer(tab.id, buf)
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
