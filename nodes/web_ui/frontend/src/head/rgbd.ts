/**
 * Pure helpers of the Head tab RGB-D view: payload parsing, pairing of the RGB and depth messages by header stamp,
 * the depth colormap and the pixel readout. The payloads come from web_ui.bridge (roles head_rgb and head_rgbd_depth).
 */

/** Frames kept per stream so a late partner still finds its stamp. */
export const PAIR_HISTORY = 4
/** Metres of depth shown per millimetre unit. */
const MM_PER_M = 1000
const RGBA_CHANNELS = 4
const OPAQUE = 255
/** Number of colours in the legend strip. */
export const LEGEND_STEPS = 64

/** A JPEG frame of /head_rgbd/rgb/image_raw/compressed with its header stamp (seconds). */
export interface RgbPayload {
  jpeg_b64: string
  stamp: number
}

/** The registered depth of /head_rgbd/depth/image_raw at full size: little-endian uint16 mm, 0 = invalid. */
export interface DepthPayload {
  depth_raw_b64: string
  width: number
  height: number
  stamp: number
  frame_id?: string
}

/** Decoded depth: millimetres row-major, width x height. */
export interface DepthGrid {
  mm: Uint16Array
  width: number
  height: number
}

/** An RGB frame and the depth frame that has the same stamp. */
export interface RgbdPair {
  rgb: RgbPayload
  depth: DepthPayload
}

/**
 * Validate an RGB payload.
 *
 * @param data - raw topic value
 * @returns the payload, or null when absent, cleared, undecodable or without a stamp
 */
export function parseRgbPayload(data: unknown): RgbPayload | null {
  if (!data || typeof data !== 'object') return null
  const p = data as Partial<RgbPayload>
  if (typeof p.jpeg_b64 !== 'string' || !p.jpeg_b64 || typeof p.stamp !== 'number' || !Number.isFinite(p.stamp)) return null
  return { jpeg_b64: p.jpeg_b64, stamp: p.stamp }
}

/**
 * Validate a registered depth payload.
 *
 * @param data - raw topic value
 * @returns the payload, or null when absent, cleared, without a stamp or with a bad size
 */
export function parseDepthPayload(data: unknown): DepthPayload | null {
  if (!data || typeof data !== 'object') return null
  const p = data as Partial<DepthPayload>
  if (typeof p.depth_raw_b64 !== 'string' || typeof p.stamp !== 'number' || !Number.isFinite(p.stamp)) return null
  if (!Number.isInteger(p.width) || !Number.isInteger(p.height) || (p.width as number) <= 0 || (p.height as number) <= 0) return null
  return p as DepthPayload
}

/**
 * Decode the raw depth bytes.
 *
 * @param p - validated depth payload
 * @returns the depth grid at the sensor's full size, or null when the bytes are not valid base64 or not width x height uint16
 */
export function decodeRegisteredDepth(p: DepthPayload): DepthGrid | null {
  let bin: string
  try {
    bin = atob(p.depth_raw_b64)
  } catch {
    return null
  }
  if (bin.length !== p.width * p.height * 2) return null
  const mm = new Uint16Array(p.width * p.height)
  for (let i = 0; i < mm.length; i++) mm[i] = bin.charCodeAt(i * 2) | (bin.charCodeAt(i * 2 + 1) << 8)
  return { mm, width: p.width, height: p.height }
}

/**
 * Append a frame to a short history.
 *
 * @param recent - the history, oldest first
 * @param item - the newest frame; null (cleared or invalid topic) empties the history
 * @returns the history with item added (a frame of an already held stamp replaces it), capped at PAIR_HISTORY
 */
export function pushRecent<T extends { stamp: number }>(recent: T[], item: T | null): T[] {
  if (!item) return []
  const kept = recent.filter((r) => r.stamp !== item.stamp)
  return [...kept, item].slice(-PAIR_HISTORY)
}

/**
 * The newest RGB frame that has a depth frame with exactly its stamp.
 *
 * @param rgbs - recent RGB frames
 * @param depths - recent depth frames
 * @returns the pair, or null when no stamps match
 */
export function latestPair(rgbs: RgbPayload[], depths: DepthPayload[]): RgbdPair | null {
  const byStamp = new Map(depths.map((d) => [d.stamp, d]))
  const rgb = [...rgbs].sort((a, b) => b.stamp - a.stamp).find((r) => byStamp.has(r.stamp))
  return rgb ? { rgb, depth: byStamp.get(rgb.stamp) as DepthPayload } : null
}

function clamp01(t: number): number {
  return Math.min(Math.max(t, 0), 1)
}

/**
 * Turbo colormap (polynomial approximation): dark blue at 0 through green and yellow to dark red at 1.
 *
 * @param t - position in [0, 1]; values outside are clamped
 * @returns [r, g, b], each 0..255
 */
export function turbo(t: number): [number, number, number] {
  const x = clamp01(t)
  const r = 0.13572138 + x * (4.6153926 + x * (-42.66032258 + x * (132.13108234 + x * (-152.94239396 + x * 59.28637943))))
  const g = 0.09140261 + x * (2.19418839 + x * (4.84296658 + x * (-14.18503333 + x * (4.27729857 + x * 2.82956604))))
  const b = 0.1066733 + x * (12.64194608 + x * (-60.58204836 + x * (110.36276771 + x * (-89.90310912 + x * 27.34824973))))
  return [r, g, b].map((v) => Math.round(clamp01(v) * OPAQUE)) as [number, number, number]
}

/**
 * Colour the depth: near is blue, `rangeM` or farther is red; invalid depth (0) is fully transparent.
 *
 * @param mm - depth in millimetres, row-major
 * @param rangeM - depth in metres that maps to the end of the colormap
 * @returns RGBA bytes, one pixel per depth value
 */
export function depthToRgba(mm: Uint16Array, rangeM: number): Uint8ClampedArray {
  const out = new Uint8ClampedArray(mm.length * RGBA_CHANNELS)
  for (let i = 0; i < mm.length; i++) {
    if (mm[i] === 0) continue
    const [r, g, b] = turbo(mm[i] / MM_PER_M / rangeM)
    out.set([r, g, b, OPAQUE], i * RGBA_CHANNELS)
  }
  return out
}

/**
 * The legend: the colormap from near to far as one row of pixels.
 *
 * @param steps - number of pixels
 * @returns RGBA bytes, opaque
 */
export function legendRgba(steps: number = LEGEND_STEPS): Uint8ClampedArray {
  const out = new Uint8ClampedArray(steps * RGBA_CHANNELS)
  for (let i = 0; i < steps; i++) out.set([...turbo(i / (steps - 1)), OPAQUE], i * RGBA_CHANNELS)
  return out
}

/**
 * The readout text for one depth pixel.
 *
 * @param depth - decoded depth
 * @param x - pixel column
 * @param y - pixel row
 * @returns "depth X.XX m", "no depth" for an invalid pixel, or null outside the image
 */
export function depthReadout(depth: DepthGrid, x: number, y: number): string | null {
  if (x < 0 || y < 0 || x >= depth.width || y >= depth.height) return null
  const mm = depth.mm[y * depth.width + x]
  return mm === 0 ? 'no depth' : `depth ${(mm / MM_PER_M).toFixed(2)} m`
}

/**
 * The image pixel under the pointer for an image drawn with object-fit contain in a box.
 *
 * @param clientX - pointer x
 * @param clientY - pointer y
 * @param box - bounding rect of the box (left, top, width, height)
 * @param width - image width in pixels
 * @param height - image height in pixels
 * @returns the pixel, or null over the letterbox bars or outside the box
 */
export function pixelAt(
  clientX: number,
  clientY: number,
  box: { left: number; top: number; width: number; height: number },
  width: number,
  height: number,
): { x: number; y: number } | null {
  const scale = Math.min(box.width / width, box.height / height)
  if (!(scale > 0)) return null
  const dx = clientX - box.left - (box.width - width * scale) / 2
  const dy = clientY - box.top - (box.height - height * scale) / 2
  if (dx < 0 || dy < 0 || dx >= width * scale || dy >= height * scale) return null
  return { x: Math.floor(dx / scale), y: Math.floor(dy / scale) }
}
