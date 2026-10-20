import { useEffect, useMemo, useRef, useState } from 'react'
import type { CSSProperties, MouseEvent } from 'react'
import Box from '@mui/material/Box'
import Slider from '@mui/material/Slider'
import Stack from '@mui/material/Stack'
import Typography from '@mui/material/Typography'
import { CANVAS_BG } from '../theme'
import { WaitingMessage } from '../tabs/WaitingMessage'
import {
  LEGEND_STEPS,
  decodeRegisteredDepth,
  depthReadout,
  depthToRgba,
  latestPair,
  legendRgba,
  parseDepthPayload,
  parseRgbPayload,
  pixelAt,
  pushRecent,
  type DepthPayload,
  type RgbPayload,
  type RgbdPair,
} from './rgbd'

const LEGEND_STYLE: CSSProperties = { width: '8rem', height: '1rem', display: 'block' }
const DEFAULT_OPACITY_PCT = 50
const HOVER_HINT = 'hover the image for depth'
const LAYER_STYLE: CSSProperties = { position: 'absolute', inset: 0, width: '100%', height: '100%', objectFit: 'contain' }

interface Props {
  rgbTopic: string
  depthTopic: string
  /** Latest payloads of the two topics (raw topic values, with header stamps). */
  rgb: unknown
  depth: unknown
  /** Depth in metres at the far end of the colormap. */
  rangeM: number
}

/**
 * The latest RGB frame and depth frame that share a header stamp. A short history of each stream is kept, so a frame
 * whose partner arrives a moment later still pairs; clearing either topic drops its history (nothing stale is shown).
 *
 * @param rgb - raw value of the RGB topic
 * @param depth - raw value of the depth topic
 * @returns the pair, or null while no stamps match
 */
export function useRgbdPair(rgb: unknown, depth: unknown): RgbdPair | null {
  const rgbs = useRef<RgbPayload[]>([])
  const depths = useRef<DepthPayload[]>([])
  const parsedRgb = useMemo(() => parseRgbPayload(rgb), [rgb])
  const parsedDepth = useMemo(() => parseDepthPayload(depth), [depth])
  rgbs.current = pushRecent(rgbs.current, parsedRgb)
  depths.current = pushRecent(depths.current, parsedDepth)
  return latestPair(rgbs.current, depths.current)
}

/**
 * RGB image of /head_rgbd with the registered depth over it as a colormap (invalid depth transparent), an opacity
 * slider, a legend and a readout of the raw depth under the pointer.
 *
 * @param rgbTopic - RGB topic name shown in the waiting line
 * @param depthTopic - depth topic name shown in the waiting line
 * @param rgb - latest RGB payload
 * @param depth - latest depth payload
 * @param rangeM - depth at the far end of the colormap and legend (m)
 */
export function RgbdView({ rgbTopic, depthTopic, rgb, depth, rangeM }: Props) {
  const pair = useRgbdPair(rgb, depth)
  const pairedDepth = pair?.depth
  const grid = useMemo(() => (pairedDepth ? decodeRegisteredDepth(pairedDepth) : null), [pairedDepth])
  const [opacityPct, setOpacityPct] = useState(DEFAULT_OPACITY_PCT)
  const [hover, setHover] = useState<{ x: number; y: number } | null>(null)
  const boxRef = useRef<HTMLDivElement>(null)
  const overlayRef = useRef<HTMLCanvasElement>(null)
  const legendRef = useRef<HTMLCanvasElement>(null)

  useEffect(() => {
    const ctx = overlayRef.current?.getContext('2d')
    if (!ctx || !grid) return
    const image = ctx.createImageData(grid.width, grid.height)
    image.data.set(depthToRgba(grid.mm, rangeM))
    ctx.putImageData(image, 0, 0)
  }, [grid, rangeM])

  useEffect(() => {
    const ctx = legendRef.current?.getContext('2d')
    if (!ctx) return
    const strip = ctx.createImageData(LEGEND_STEPS, 1)
    strip.data.set(legendRgba(LEGEND_STEPS))
    ctx.putImageData(strip, 0, 0)
  }, [])

  function onMove(e: MouseEvent): void {
    const box = boxRef.current?.getBoundingClientRect()
    setHover(grid && box ? pixelAt(e.clientX, e.clientY, box, grid.width, grid.height) : null)
  }

  const readout = grid && hover ? depthReadout(grid, hover.x, hover.y) : null

  return (
    <Box sx={{ width: '100%', height: '100%', display: 'flex', flexDirection: 'column', minHeight: 0 }}>
      <Box
        ref={boxRef}
        data-testid="rgbd-view"
        onMouseMove={onMove}
        onMouseLeave={() => setHover(null)}
        sx={{ flex: '1 1 auto', minHeight: 0, position: 'relative', bgcolor: CANVAS_BG, cursor: 'crosshair' }}
      >
        {pair && grid ? (
          <>
            <img alt="head RGB-D" src={`data:image/jpeg;base64,${pair.rgb.jpeg_b64}`} style={LAYER_STYLE} />
            <canvas
              ref={overlayRef}
              data-testid="rgbd-overlay"
              width={grid.width}
              height={grid.height}
              style={{ ...LAYER_STYLE, opacity: opacityPct / 100, pointerEvents: 'none' }}
            />
          </>
        ) : (
          <WaitingMessage>{`Waiting for ${rgbTopic} and ${depthTopic} with matching stamps...`}</WaitingMessage>
        )}
      </Box>
      <Stack direction="row" spacing={2} sx={{ flex: '0 0 auto', alignItems: 'center', flexWrap: 'wrap', px: 1, py: 1 }}>
        <Stack direction="row" spacing={1} sx={{ alignItems: 'center', minWidth: '12rem', flex: '0 1 16rem' }}>
          <Typography variant="body2" color="text.secondary" noWrap>opacity</Typography>
          <Slider
            size="small"
            aria-label="overlay opacity"
            min={0}
            max={100}
            step={1}
            value={opacityPct}
            onChange={(_, v) => setOpacityPct(v as number)}
          />
          <Typography variant="body2" sx={{ minWidth: '3rem' }}>{`${opacityPct} %`}</Typography>
        </Stack>
        <Stack data-testid="rgbd-legend" direction="row" spacing={1} sx={{ alignItems: 'center' }}>
          <Typography variant="body2">0.0 m</Typography>
          <canvas ref={legendRef} data-testid="rgbd-legend-strip" width={LEGEND_STEPS} height={1} style={LEGEND_STYLE} />
          <Typography variant="body2">{`${rangeM.toFixed(1)} m`}</Typography>
        </Stack>
        <Typography data-testid="rgbd-readout" variant="body2" color={readout ? 'text.primary' : 'text.secondary'}>
          {readout ?? HOVER_HINT}
        </Typography>
      </Stack>
    </Box>
  )
}
