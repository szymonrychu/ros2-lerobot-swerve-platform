/** Client of the backend POST /api/camera/zoom endpoint and the click-to-picture-coordinate maths. */

export type ZoomFactor = 2 | 4 | 8

/** Body of a successful zoom reply: the head_camera_cpp result plus the crop as base64 JPEG. */
export interface ZoomReply {
  status: string
  message?: string
  jpeg_b64: string
  [key: string]: unknown
}

/**
 * Ask the head camera for one autofocused full-resolution crop around a point of the preview.
 *
 * @param tab - head tab id (selects the topics on the backend)
 * @param u - horizontal point, 0 (left) to 1 (right) of the full 16:9 frame
 * @param v - vertical point, 0 (top) to 1 (bottom) of the full 16:9 frame
 * @param zoom - zoom factor
 * @param fetchImpl - fetch implementation (tests)
 * @returns the endpoint body
 * @throws Error carrying the HTTP status and the result message on a non-2xx reply
 */
export async function requestZoom(
  tab: string,
  u: number,
  v: number,
  zoom: ZoomFactor,
  fetchImpl: typeof fetch = fetch,
): Promise<ZoomReply> {
  const resp = await fetchImpl(`/api/camera/zoom?tab=${encodeURIComponent(tab)}`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ u, v, zoom }),
  })
  let body: Record<string, unknown> | null = null
  try {
    body = (await resp.json()) as Record<string, unknown>
  } catch {
    body = null
  }
  if (!resp.ok) {
    const message = typeof body?.message === 'string' ? body.message : 'no message'
    throw new Error(`Zoom failed (HTTP ${resp.status}): ${message}`)
  }
  return body as unknown as ZoomReply
}

/**
 * Map a click inside an object-fit: contain image to [0, 1] picture coordinates.
 *
 * @param clientX - click x in viewport pixels
 * @param clientY - click y in viewport pixels
 * @param rect - bounding box of the img element
 * @param naturalWidth - intrinsic image width
 * @param naturalHeight - intrinsic image height
 * @returns the point, or null for clicks on the letterbox bars or while the image size is unknown
 */
export function clickToNormalised(
  clientX: number,
  clientY: number,
  rect: Pick<DOMRect, 'left' | 'top' | 'width' | 'height'>,
  naturalWidth: number,
  naturalHeight: number,
): { u: number; v: number } | null {
  if (naturalWidth <= 0 || naturalHeight <= 0 || rect.width <= 0 || rect.height <= 0) return null
  const scale = Math.min(rect.width / naturalWidth, rect.height / naturalHeight)
  const w = naturalWidth * scale
  const h = naturalHeight * scale
  const u = (clientX - rect.left - (rect.width - w) / 2) / w
  const v = (clientY - rect.top - (rect.height - h) / 2) / h
  if (u < 0 || u > 1 || v < 0 || v > 1) return null
  return { u, v }
}
