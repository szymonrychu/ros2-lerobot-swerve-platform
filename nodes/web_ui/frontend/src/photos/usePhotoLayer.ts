/** Fetches the photos the map layer draws: the newest ones inside the age filter, refetched on /poi/photos changes. */
import { useEffect, useRef, useState } from 'react'
import { fetchPhotos, type Photo, PHOTO_LAYER_LIMIT } from './api'
import { ageCutoff, newestPhotos, type PhotoAge } from './geometry'

export interface PhotoLayerData {
  photos: Photo[]
  /** Photos matching the age filter on the backend (may exceed the drawn count). */
  total: number
  error: string | null
}

/**
 * Newest located photos for the layer.
 *
 * @param enabled - false skips fetching (layer toggled off)
 * @param age - age filter
 * @param revision - revision of the latched /poi/photos summary; a change refetches
 * @param fetchImpl - fetch implementation (tests)
 * @returns drawn photos (newest first, at most the layer limit), the matching total and the last error
 */
export function usePhotoLayer(enabled: boolean, age: PhotoAge, revision: number | null, fetchImpl: typeof fetch = fetch): PhotoLayerData {
  const [data, setData] = useState<PhotoLayerData>({ photos: [], total: 0, error: null })
  const fetchRef = useRef(fetchImpl)
  fetchRef.current = fetchImpl
  useEffect(() => {
    if (!enabled) return
    let cancelled = false
    const cutoff = ageCutoff(age, Date.now() / 1000)
    fetchPhotos(null, PHOTO_LAYER_LIMIT, null, fetchRef.current, cutoff === null ? null : Math.floor(cutoff)).then(
      (page) => {
        if (!cancelled) setData({ photos: newestPhotos(page.photos, PHOTO_LAYER_LIMIT), total: page.total, error: null })
      },
      (err: unknown) => {
        if (!cancelled) setData((prev) => ({ ...prev, error: err instanceof Error ? err.message : String(err) }))
      },
    )
    return () => {
      cancelled = true
    }
  }, [enabled, age, revision])
  return data
}
