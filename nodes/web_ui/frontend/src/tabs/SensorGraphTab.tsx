import { useEffect, useRef } from 'react'
import uPlot from 'uplot'
import 'uplot/dist/uPlot.min.css'
import Box from '@mui/material/Box'
import log from '../logging'
import { CANVAS_BG, SANS_FONT } from '../theme'
import { TabConfig } from '../types'
import { getOrCreateBuffer } from '../utils/graphBuffers'

const AXIS_STROKE = '#9da7b3'
const GRID_STROKE = 'rgba(255, 255, 255, 0.08)'
const MIN_PLOT_HEIGHT = 80

/**
 * Size the plot to its container, leaving room for the legend (which wraps, so it is measured).
 *
 * @param plot - uPlot instance
 * @param container - element the plot fills
 */
function fitPlot(plot: uPlot, container: HTMLElement): void {
  const legend = plot.root.querySelector<HTMLElement>('.u-legend')
  const legendHeight = legend?.offsetHeight ?? 0
  plot.setSize({
    width: container.clientWidth,
    height: Math.max(MIN_PLOT_HEIGHT, container.clientHeight - legendHeight - 8),
  })
}

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

    const container = containerRef.current
    const opts: uPlot.Options = {
      width: container.clientWidth,
      height: Math.max(MIN_PLOT_HEIGHT, container.clientHeight - 40),
      scales: { x: { time: false }, y: {} },
      series,
      axes: [
        { stroke: AXIS_STROKE, font: `12px ${SANS_FONT}`, ticks: { stroke: GRID_STROKE }, grid: { stroke: GRID_STROKE } },
        { stroke: AXIS_STROKE, font: `12px ${SANS_FONT}`, ticks: { stroke: GRID_STROKE }, grid: { stroke: GRID_STROKE } },
      ],
    }

    // Restore persistent buffer — already filled by App-level feedAllGraphBuffers()
    const buf = getOrCreateBuffer(tab.id, seriesCount)
    const plot = new uPlot(opts, buf as uPlot.AlignedData, container)
    plotRef.current = plot
    fitPlot(plot, container)
    log.debug('[graph] mounted tab', tab.id, '— buffer has', buf[0].length, 'points')

    // Follow the container: window resize, rotation, overlay bar wrapping.
    const ro = new ResizeObserver(() => fitPlot(plot, container))
    ro.observe(container)

    return () => {
      ro.disconnect()
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
    <Box
      ref={containerRef}
      sx={{
        width: '100%',
        height: '100%',
        overflow: 'hidden',
        bgcolor: CANVAS_BG,
        '& .u-legend': { fontFamily: SANS_FONT, fontSize: 13, color: 'text.primary' },
      }}
    />
  )
}
