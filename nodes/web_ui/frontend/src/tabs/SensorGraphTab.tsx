import { useEffect, useRef } from 'react'
import uPlot from 'uplot'
import 'uplot/dist/uPlot.min.css'
import log from '../logging'
import { TabConfig } from '../types'
import { getOrCreateBuffer } from '../utils/graphBuffers'

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

  // Create or restore uPlot instance, pre-filled with buffered data
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

    // Restore persistent buffer — already filled by App-level feedAllGraphBuffers()
    const buf = getOrCreateBuffer(tab.id, seriesCount)
    plotRef.current = new uPlot(opts, buf as uPlot.AlignedData, containerRef.current)
    log.debug('[graph] mounted tab', tab.id, '— buffer has', buf[0].length, 'points')

    return () => {
      plotRef.current?.destroy()
      plotRef.current = null
    }
  }, [seriesCount, tab.id])  // eslint-disable-line react-hooks/exhaustive-deps

  // Redraw chart when buffer is updated by App-level pump
  useEffect(() => {
    if (!plotRef.current || seriesCount === 0) return
    const buf = getOrCreateBuffer(tab.id, seriesCount)
    if (buf[0].length === 0) return
    plotRef.current.setData(buf as uPlot.AlignedData)
  }, [topicData])  // eslint-disable-line react-hooks/exhaustive-deps

  return (
    <div
      ref={containerRef}
      style={{ width: '100%', height: '100%', background: '#050505' }}
    />
  )
}
