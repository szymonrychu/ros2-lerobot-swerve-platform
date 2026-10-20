import { describe, expect, it } from 'vitest'
import {
  PAIR_HISTORY,
  decodeRegisteredDepth,
  depthReadout,
  depthToRgba,
  latestPair,
  legendRgba,
  parseDepthPayload,
  parseRgbPayload,
  pixelAt,
  pushRecent,
  turbo,
} from './rgbd'

function b64u16(values: number[]): string {
  const bytes = new Uint8Array(values.length * 2)
  const view = new DataView(bytes.buffer)
  values.forEach((v, i) => view.setUint16(i * 2, v, true))
  let s = ''
  bytes.forEach((b) => (s += String.fromCharCode(b)))
  return btoa(s)
}

const DEPTH = { depth_raw_b64: b64u16([0, 1500, 4000, 250]), width: 2, height: 2, stamp: 7.5, frame_id: 'f' }

describe('payload parsing', () => {
  it('accepts stamped RGB and depth payloads', () => {
    expect(parseRgbPayload({ jpeg_b64: 'AAA', stamp: 1 })).toEqual({ jpeg_b64: 'AAA', stamp: 1 })
    expect(parseDepthPayload(DEPTH)).not.toBeNull()
  })
  it('rejects missing stamps, empty images, cleared and malformed payloads', () => {
    expect(parseRgbPayload({ jpeg_b64: 'AAA' })).toBeNull()
    expect(parseRgbPayload({ jpeg_b64: null, stamp: 1 })).toBeNull()
    expect(parseRgbPayload(null)).toBeNull()
    expect(parseDepthPayload({ ...DEPTH, stamp: undefined })).toBeNull()
    expect(parseDepthPayload({ ...DEPTH, width: 0 })).toBeNull()
    expect(parseDepthPayload(undefined)).toBeNull()
  })
})

describe('decodeRegisteredDepth', () => {
  it('decodes little-endian uint16 mm at full size', () => {
    const d = decodeRegisteredDepth(parseDepthPayload(DEPTH)!)
    expect(d).not.toBeNull()
    expect([d!.width, d!.height]).toEqual([2, 2])
    expect(Array.from(d!.mm)).toEqual([0, 1500, 4000, 250])
  })
  it('returns null when the byte length does not match width x height', () => {
    expect(decodeRegisteredDepth(parseDepthPayload({ ...DEPTH, width: 3 })!)).toBeNull()
    expect(decodeRegisteredDepth(parseDepthPayload({ ...DEPTH, depth_raw_b64: '!!!' })!)).toBeNull()
  })
})

describe('pairing by stamp', () => {
  const rgb = (stamp: number) => ({ jpeg_b64: `J${stamp}`, stamp })
  const depth = (stamp: number) => parseDepthPayload({ ...DEPTH, stamp })!

  it('pairs only equal stamps, newest first', () => {
    expect(latestPair([rgb(1), rgb(2)], [depth(1)])!.rgb.stamp).toBe(1)
    expect(latestPair([rgb(1), rgb(2)], [depth(1), depth(2)])!.rgb.stamp).toBe(2)
    expect(latestPair([rgb(3)], [depth(2)])).toBeNull()
    expect(latestPair([], [depth(2)])).toBeNull()
  })

  it('pushRecent appends new stamps, replaces the same stamp, caps the history and clears on null', () => {
    let h = pushRecent([], rgb(1))
    h = pushRecent(h, rgb(1))
    expect(h).toHaveLength(1)
    for (let i = 2; i < 3 + PAIR_HISTORY; i++) h = pushRecent(h, rgb(i))
    expect(h).toHaveLength(PAIR_HISTORY)
    expect(h[h.length - 1].stamp).toBe(2 + PAIR_HISTORY)
    expect(pushRecent(h, null)).toEqual([])
  })
})

describe('depth colormap and readout', () => {
  it('turbo runs from blue (near) to red (far) and clamps', () => {
    const [r0, , b0] = turbo(0.1)
    const [r1, , b1] = turbo(0.95)
    expect(b0).toBeGreaterThan(r0)
    expect(r1).toBeGreaterThan(b1)
    expect(turbo(-5)).toEqual(turbo(0))
    expect(turbo(7)).toEqual(turbo(1))
  })

  it('depthToRgba makes invalid depth (0) fully transparent and valid depth opaque, colour by range', () => {
    const rgba = depthToRgba(Uint16Array.from([0, 1000, 4000, 9000]), 4.0)
    expect(rgba[3]).toBe(0)
    expect(rgba[7]).toBe(255)
    expect(rgba[11]).toBe(255)
    expect(Array.from(rgba.slice(4, 7))).not.toEqual(Array.from(rgba.slice(8, 11)))
    // beyond the range clamps to the far colour
    expect(Array.from(rgba.slice(8, 11))).toEqual(Array.from(rgba.slice(12, 15)))
  })

  it('legendRgba is an opaque near-to-far strip of the same colours', () => {
    const strip = legendRgba(8)
    expect(strip).toHaveLength(8 * 4)
    expect(strip.filter((_, i) => i % 4 === 3).every((a) => a === 255)).toBe(true)
    expect(Array.from(strip.slice(0, 3))).toEqual(turbo(0))
    expect(Array.from(strip.slice(-4, -1))).toEqual(turbo(1))
  })

  it('depthReadout reads the raw millimetres, "no depth" for 0 and nothing outside the image', () => {
    const d = decodeRegisteredDepth(parseDepthPayload(DEPTH)!)!
    expect(depthReadout(d, 1, 0)).toBe('depth 1.50 m')
    expect(depthReadout(d, 0, 1)).toBe('depth 4.00 m')
    expect(depthReadout(d, 1, 1)).toBe('depth 0.25 m')
    expect(depthReadout(d, 0, 0)).toBe('no depth')
    expect(depthReadout(d, 2, 0)).toBeNull()
    expect(depthReadout(d, 0, -1)).toBeNull()
  })
})

describe('pixelAt', () => {
  const rect = { left: 10, top: 20, width: 400, height: 400 }
  it('maps the pointer through the object-fit contain letterbox (16:9 in a square)', () => {
    // a 288x162 image fits 400 px wide, 225 px high, centred vertically (offset 87.5)
    expect(pixelAt(10 + 1, 20 + 88, rect, 288, 162)).toEqual({ x: 0, y: 0 })
    expect(pixelAt(10 + 399, 20 + 87.5 + 224, rect, 288, 162)).toEqual({ x: 287, y: 161 })
    expect(pixelAt(10 + 200, 20 + 200, rect, 288, 162)).toEqual({ x: 144, y: 81 })
  })
  it('is null in the letterbox bars and outside the box', () => {
    expect(pixelAt(10 + 200, 20 + 10, rect, 288, 162)).toBeNull()
    expect(pixelAt(5, 20 + 200, rect, 288, 162)).toBeNull()
    expect(pixelAt(10 + 200, 20 + 395, rect, 288, 162)).toBeNull()
  })
})
