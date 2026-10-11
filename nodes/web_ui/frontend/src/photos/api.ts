/** Client of the GET /api/photos routes and of the delete_photos op of POST /api/poi (photo POIs of poi_store). */
import { postPoi } from '../poi/api'

/** Photos fetched per page. */
export const PHOTO_LIMIT = 10

/** Longest side of a list thumbnail the backend renders (2x the 72 px CSS width, for high-DPI screens). */
export const THUMB_PX = 144

/** Camera pose at capture in the map frame (backend PhotoView). */
export interface PhotoView {
  x: number
  y: number
  z: number | null
  /** Heading of the camera's forward axis, rad, counter-clockwise from +x. */
  yaw: number
  /** Positive means looking down; null when the mount is unknown. */
  pitch: number | null
  hfov: number | null
  source: 'camera_mount' | 'base_heading'
}

/** One photo POI (camelCase of the backend photo_as_poi form). */
export interface Photo {
  id: string
  /** Unix seconds of capture. */
  ts: number
  camera: string
  tool: string
  name: string
  note: string
  poiIds: string[]
  /** Camera position in the map; null (never 0) when the photo is unlocated. */
  x: number | null
  y: number | null
  view: PhotoView | null
  bytes: number
  width: number
  height: number
}

/** One page of photos, newest first. */
export interface PhotoPage {
  photos: Photo[]
  /** Cursor for the next page (pass as `before`); null on the last page. */
  nextBefore: string | null
  /** Photos matching the filter, ignoring the cursor. */
  total: number
}

/** The bulk-delete selector of the backend (PhotoSelector): exactly one of ids, older_than_s, keep_newest. */
export interface PhotoDeleteSelector {
  ids?: string[]
  older_than_s?: number
  keep_newest?: number
  dry_run?: boolean
}

/** Outcome of a delete_photos request; count and bytes are removed (or, for a dry run, would be removed). */
export interface DeleteOutcome {
  ok: boolean
  message: string
  count: number
  bytes: number
}

/** Most photos one request may ask for (backend limit). */
export const PHOTO_LAYER_LIMIT = 1000

/**
 * Capture time as shown in the UI.
 *
 * @param ts - unix seconds
 * @returns ISO 8601 UTC without fractional seconds
 */
export function capturedAt(ts: number): string {
  return new Date(ts * 1000).toISOString().replace('.000Z', 'Z')
}

/**
 * URL of the photos page.
 *
 * @param poiId - only photos linked to this POI; null for all
 * @param limit - most photos to return
 * @param before - cursor from the previous page, null for the first page
 * @param since - only photos captured at or after this unix time; null for all
 * @returns the request URL
 */
export function photosUrl(poiId: string | null, limit: number, before: string | null, since: number | null = null): string {
  const poi = poiId === null ? '' : `&poi_id=${encodeURIComponent(poiId)}`
  const cursor = before === null ? '' : `&before=${encodeURIComponent(before)}`
  const from = since === null ? '' : `&since=${since}`
  return `/api/photos?limit=${limit}${poi}${cursor}${from}`
}

/**
 * URL of a photo's JPEG.
 *
 * @param id - photo id
 * @returns the image URL
 */
export function photoImageUrl(id: string): string {
  return `/api/photos/${encodeURIComponent(id)}.jpg`
}

/**
 * URL of a photo's server-side downscaled JPEG.
 *
 * @param id - photo id
 * @param maxPx - longest side in pixels
 * @returns the thumbnail URL
 */
export function photoThumbUrl(id: string, maxPx: number = THUMB_PX): string {
  return `${photoImageUrl(id)}?max_px=${maxPx}`
}

function num(v: unknown): number | null {
  return typeof v === 'number' && Number.isFinite(v) ? v : null
}

function str(v: unknown): string {
  return typeof v === 'string' ? v : ''
}

function parseView(raw: unknown): PhotoView | null {
  if (typeof raw !== 'object' || raw === null) return null
  const r = raw as Record<string, unknown>
  const x = num(r.x)
  const y = num(r.y)
  const yaw = num(r.yaw)
  if (x === null || y === null || yaw === null) return null
  return { x, y, z: num(r.z), yaw, pitch: num(r.pitch), hfov: num(r.hfov), source: r.source === 'base_heading' ? 'base_heading' : 'camera_mount' }
}

/**
 * Parse one photo POI of the backend.
 *
 * @param raw - an element of the `photos` array
 * @returns the photo, or null when it has no id
 */
export function parsePhoto(raw: unknown): Photo | null {
  if (typeof raw !== 'object' || raw === null) return null
  const r = raw as Record<string, unknown>
  if (typeof r.id !== 'string') return null
  const p = (typeof r.photo === 'object' && r.photo !== null ? r.photo : {}) as Record<string, unknown>
  return {
    id: r.id,
    ts: num(r.created_at) ?? 0,
    camera: str(p.camera),
    tool: str(p.tool),
    name: str(r.name),
    note: str(r.note),
    poiIds: Array.isArray(r.poi_ids) ? r.poi_ids.filter((v): v is string => typeof v === 'string') : [],
    x: num(r.x),
    y: num(r.y),
    view: parseView(p.view),
    bytes: num(p.bytes) ?? 0,
    width: num(p.width) ?? 0,
    height: num(p.height) ?? 0,
  }
}

/**
 * Fetch one page of photos, newest first.
 *
 * @param poiId - only photos linked to this POI; null for all
 * @param limit - most photos to return
 * @param before - cursor of the previous page, null for the first page
 * @param fetchImpl - fetch implementation (tests)
 * @param since - only photos captured at or after this unix time; null for all
 * @returns the page (no photos means there are none)
 * @throws Error with a readable reason on a network failure, an HTTP error or an unexpected body
 */
export async function fetchPhotos(
  poiId: string | null,
  limit: number,
  before: string | null,
  fetchImpl: typeof fetch = fetch,
  since: number | null = null,
): Promise<PhotoPage> {
  const resp = await fetchImpl(photosUrl(poiId, limit, before, since))
  if (!resp.ok) throw new Error(`HTTP ${resp.status}`)
  const body = (await resp.json()) as { photos?: unknown; next_before?: unknown; total_matching?: unknown }
  if (!Array.isArray(body.photos)) throw new Error('unexpected response')
  const photos = body.photos.map(parsePhoto).filter((p): p is Photo => p !== null)
  return { photos, nextBefore: typeof body.next_before === 'string' ? body.next_before : null, total: num(body.total_matching) ?? photos.length }
}

/**
 * Delete photos by selector (or preview with `dry_run`). Never throws.
 *
 * @param tabId - map_nav tab id
 * @param selector - ids, age or keep-newest selector
 * @param fetchImpl - fetch implementation (tests)
 * @returns ok, the message and the photo count and bytes removed (dry run: that would be removed)
 */
export async function deletePhotos(tabId: string, selector: PhotoDeleteSelector, fetchImpl: typeof fetch = fetch): Promise<DeleteOutcome> {
  const result = await postPoi(tabId, { op: 'delete_photos', selector: { ...selector } }, fetchImpl)
  const data = (result.poi ?? {}) as unknown as Record<string, unknown>
  return {
    ok: result.ok,
    message: result.message,
    count: num(data.removed) ?? num(data.would_remove) ?? 0,
    bytes: num(data.bytes_freed) ?? num(data.bytes) ?? 0,
  }
}
