/** Decode the map3d_cloud payload (int16 cm points, optional uint8 rgb) into three.js-ordered metres. */
import log from '../logging'
import type { Map3dCloudPayload } from '../types'

const CM_PER_M = 100
const POINT_BYTES = 6
const COLOR_BYTES = 3

/** A decoded cloud: positions in metres in three.js order (x, z, -y), colours 0..1 when the payload had them. */
export interface DecodedMap3dCloud {
  positions: Float32Array
  colors?: Float32Array
  count: number
  voxel: number
}

const warned = new Set<string>()

/** Log a rejection once per reason, so a stream of bad payloads does not flood the console. */
function rejectOnce(reason: string): null {
  if (!warned.has(reason)) {
    warned.add(reason)
    log.warn(`[map3d] cloud rejected: ${reason}`)
  }
  return null
}

function bytesOf(b64: string): Uint8Array {
  const bin = atob(b64)
  const out = new Uint8Array(bin.length)
  for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i)
  return out
}

/**
 * Decode a map3d_cloud payload.
 *
 * @param payload - raw topic value
 * @param expectedFrame - the tab's map3d_frame_id; a cloud in any other frame is rejected
 * @returns the decoded cloud, or null (logged once per reason) for a malformed payload, a foreign frame or byte lengths that do not match count
 */
export function decodeMap3dCloud(payload: unknown, expectedFrame?: string): DecodedMap3dCloud | null {
  if (!payload || typeof payload !== 'object') return null
  const p = payload as Partial<Map3dCloudPayload>
  if (typeof p.points_b64 !== 'string' || !Number.isInteger(p.count) || (p.count as number) < 0) {
    return rejectOnce('malformed payload')
  }
  if (p.frame_id !== expectedFrame) return rejectOnce(`frame ${String(p.frame_id)} is not ${String(expectedFrame)}`)
  const count = p.count as number
  let pts: Uint8Array
  let rgb: Uint8Array | null = null
  try {
    pts = bytesOf(p.points_b64)
    if (typeof p.colors_b64 === 'string') rgb = bytesOf(p.colors_b64)
  } catch {
    return rejectOnce('invalid base64')
  }
  if (pts.length !== count * POINT_BYTES) return rejectOnce('point bytes do not match count')
  if (rgb && rgb.length !== count * COLOR_BYTES) return rejectOnce('colour bytes do not match count')

  const cm = new DataView(pts.buffer, pts.byteOffset, pts.byteLength)
  const positions = new Float32Array(count * 3)
  for (let i = 0; i < count; i++) {
    const x = cm.getInt16(i * POINT_BYTES, true) / CM_PER_M
    const y = cm.getInt16(i * POINT_BYTES + 2, true) / CM_PER_M
    const z = cm.getInt16(i * POINT_BYTES + 4, true) / CM_PER_M
    positions[i * 3] = x
    positions[i * 3 + 1] = z
    positions[i * 3 + 2] = 0 - y
  }
  const out: DecodedMap3dCloud = { positions, count, voxel: Number(p.voxel_m) }
  if (rgb) out.colors = Float32Array.from(rgb, (v) => v / 255)
  return out
}
