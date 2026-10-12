/** Camera photo list for the map tab: paging, selection, bulk delete and age/count cleanup with a dry-run confirm. */
import { useCallback, useEffect, useRef, useState } from 'react'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import Checkbox from '@mui/material/Checkbox'
import Link from '@mui/material/Link'
import List from '@mui/material/List'
import ListItem from '@mui/material/ListItem'
import ListItemText from '@mui/material/ListItemText'
import TextField from '@mui/material/TextField'
import Typography from '@mui/material/Typography'
import { OverlayPanel } from '../layout/OverlayPanel'
import { capturedAt, deletePhotos, fetchPhotos, Photo, PHOTO_LIMIT, PhotoDeleteSelector, photoImageUrl, photoThumbUrl } from './api'

const THUMB_WIDTH = '4.5rem'
const FIELD_WIDTH = '8rem'
const LIST_MAX_HEIGHT = { xs: '24dvh', sm: '32dvh' }
const SECONDS_PER_DAY = 86400
const BYTES_PER_MB = 1_000_000

/** A previewed cleanup waiting for the person's confirmation. */
interface CleanupPreview {
  selector: PhotoDeleteSelector
  count: number
  bytes: number
}

/**
 * Format a count and byte total as the cleanup preview sentence.
 *
 * @param count - photos
 * @param bytes - total size in bytes
 * @returns e.g. "deletes 3 photos (2.5 MB)"
 */
export function previewText(count: number, bytes: number): string {
  return `deletes ${count} ${count === 1 ? 'photo' : 'photos'} (${(bytes / BYTES_PER_MB).toFixed(1)} MB)`
}

export interface PhotoListProps {
  /** map_nav tab id (selects the poi_store topics on the backend). */
  tabId: string
  /** Revision of the latched /poi/photos summary; the list refetches when it changes. null before the first message. */
  revision: number | null
  /** Show only photos linked to this POI; null for all. */
  selectedPoiId: string | null
  selectedPoiName: string | null
  /** Open a photo in the detail panel; the row shows a Details button when given. */
  onSelect?: (photo: Photo) => void
  /** fetch implementation (tests). */
  fetchImpl?: typeof fetch
}

/**
 * Photo list wired to the backend: first page on mount, on POI filter change and on /poi/photos revision change,
 * further pages with "more". A failed fetch keeps the last photos and shows the error.
 */
export function PhotoList({ tabId, revision, selectedPoiId, selectedPoiName, onSelect, fetchImpl = fetch }: PhotoListProps) {
  const [photos, setPhotos] = useState<Photo[]>([])
  const [nextBefore, setNextBefore] = useState<string | null>(null)
  const [total, setTotal] = useState(0)
  const [error, setError] = useState<string | null>(null)
  const [checked, setChecked] = useState<ReadonlySet<string>>(new Set())
  const [days, setDays] = useState('')
  const [keep, setKeep] = useState('')
  const [preview, setPreview] = useState<CleanupPreview | null>(null)
  const [busy, setBusy] = useState(false)
  const [notice, setNotice] = useState<string | null>(null)
  const [reload, setReload] = useState(0) // bumped after a delete so the list does not wait for the /poi/photos message
  const fetchRef = useRef(fetchImpl)
  fetchRef.current = fetchImpl

  useEffect(() => {
    let cancelled = false
    fetchPhotos(selectedPoiId, PHOTO_LIMIT, null, fetchRef.current).then(
      (page) => {
        if (cancelled) return
        setPhotos(page.photos)
        setNextBefore(page.nextBefore)
        setTotal(page.total)
        setError(null)
        const ids = new Set(page.photos.map((p) => p.id))
        setChecked((prev) => new Set([...prev].filter((id) => ids.has(id))))
      },
      (err: unknown) => {
        if (!cancelled) setError(err instanceof Error ? err.message : String(err))
      },
    )
    return () => {
      cancelled = true
    }
  }, [selectedPoiId, revision, reload])

  const more = useCallback(() => {
    if (nextBefore === null) return
    fetchPhotos(selectedPoiId, PHOTO_LIMIT, nextBefore, fetchRef.current).then(
      (page) => {
        setPhotos((prev) => {
          const have = new Set(prev.map((p) => p.id))
          return [...prev, ...page.photos.filter((p) => !have.has(p.id))]
        })
        setNextBefore(page.nextBefore)
        setTotal(page.total)
        setError(null)
      },
      (err: unknown) => setError(err instanceof Error ? err.message : String(err)),
    )
  }, [nextBefore, selectedPoiId])

  const toggle = (id: string) =>
    setChecked((prev) => {
      const next = new Set(prev)
      if (!next.delete(id)) next.add(id)
      return next
    })

  const send = async (selector: PhotoDeleteSelector, onDone: (count: number, bytes: number, message: string, ok: boolean) => void) => {
    setBusy(true)
    const out = await deletePhotos(tabId, selector, fetchRef.current)
    setBusy(false)
    onDone(out.count, out.bytes, out.message, out.ok)
    if (out.ok && selector.dry_run !== true) setReload((n) => n + 1)
  }

  const deleteSelected = () =>
    send({ ids: photos.filter((p) => checked.has(p.id)).map((p) => p.id) }, (count, _bytes, message, ok) => {
      setNotice(ok ? `Deleted ${count} ${count === 1 ? 'photo' : 'photos'}.` : `Delete failed: ${message}`)
      if (ok) setChecked(new Set())
    })

  const previewCleanup = (selector: PhotoDeleteSelector) => {
    setNotice(null)
    return send({ ...selector, dry_run: true }, (count, bytes, message, ok) => {
      if (ok) setPreview({ selector, count, bytes })
      else setNotice(`Preview failed: ${message}`)
    })
  }

  const confirmCleanup = () => {
    if (preview === null) return
    const { selector } = preview
    setPreview(null)
    return send(selector, (count, _bytes, message, ok) => setNotice(ok ? `Deleted ${count} ${count === 1 ? 'photo' : 'photos'}.` : `Delete failed: ${message}`))
  }

  const daysNum = Number(days)
  const keepNum = Number(keep)
  const daysValid = days.trim() !== '' && Number.isFinite(daysNum) && daysNum > 0
  const keepValid = keep.trim() !== '' && Number.isInteger(keepNum) && keepNum >= 0
  const title = selectedPoiName === null ? `Camera photos (${total})` : `Photos of ${selectedPoiName} (${total})`

  return (
    <OverlayPanel title={title} open anchor="left" docked>
      <Box data-testid="photo-list-panel">
        <Box sx={{ pb: 0.5, display: 'flex', alignItems: 'center', gap: 1 }}>
          <Button size="small" variant="outlined" color="error" disabled={busy || checked.size === 0} onClick={deleteSelected}>
            Delete selected
          </Button>
          {checked.size > 0 && <Typography variant="caption">{`${checked.size} selected`}</Typography>}
        </Box>
        <List dense disablePadding sx={{ maxHeight: LIST_MAX_HEIGHT, overflowY: 'auto' }}>
          {error !== null && (
            <Typography variant="body2" color="error" sx={{ px: 1.5, pb: 1 }} role="alert">
              {`Could not load photos: ${error}`}
            </Typography>
          )}
          {photos.length === 0 && error === null && (
            <Typography variant="body2" color="text.secondary" sx={{ px: 1.5, pb: 1 }}>
              {selectedPoiName === null ? 'No photos yet.' : `No photos linked to ${selectedPoiName} yet.`}
            </Typography>
          )}
          {photos.map((p) => (
            <ListItem key={p.id} sx={{ gap: 0.5, alignItems: 'flex-start', pl: 0.5 }} data-testid="photo-row">
              <Checkbox size="small" checked={checked.has(p.id)} onChange={() => toggle(p.id)} inputProps={{ 'aria-label': `select photo ${p.id}` }} />
              <Link href={photoImageUrl(p.id)} target="_blank" rel="noreferrer" sx={{ flexShrink: 0 }}>
                <Box
                  component="img"
                  src={photoThumbUrl(p.id)}
                  alt={`${p.camera} photo ${capturedAt(p.ts)}`}
                  loading="lazy"
                  sx={{ width: THUMB_WIDTH, height: 'auto', display: 'block', borderRadius: 0.5 }}
                />
              </Link>
              <ListItemText
                primary={`${p.camera} - ${capturedAt(p.ts)}`}
                secondary={p.name || p.note || p.tool}
                secondaryTypographyProps={{ sx: { overflowWrap: 'anywhere' } }}
              />
              {onSelect && (
                <Button size="small" onClick={() => onSelect(p)} aria-label={`details of photo ${p.id}`}>
                  Details
                </Button>
              )}
            </ListItem>
          ))}
        </List>
        {nextBefore !== null && (
          <Box sx={{ px: 1.5, py: 0.5 }}>
            <Button size="small" onClick={more}>
              more
            </Button>
          </Box>
        )}
        <Box sx={{ px: 1.5, py: 1, display: 'flex', flexWrap: 'wrap', gap: 1, alignItems: 'center' }}>
          <TextField id="photo-cleanup-days" size="small" type="number" label="Older than (days)" value={days} onChange={(e) => setDays(e.target.value)} sx={{ width: FIELD_WIDTH }} />
          <Button size="small" variant="outlined" disabled={busy || !daysValid} onClick={() => previewCleanup({ older_than_s: daysNum * SECONDS_PER_DAY })}>
            Preview older than
          </Button>
          <TextField id="photo-cleanup-keep" size="small" type="number" label="Keep newest" value={keep} onChange={(e) => setKeep(e.target.value)} sx={{ width: FIELD_WIDTH }} />
          <Button size="small" variant="outlined" disabled={busy || !keepValid} onClick={() => previewCleanup({ keep_newest: keepNum })}>
            Preview keep newest
          </Button>
        </Box>
        {preview !== null && (
          <Box sx={{ px: 1.5, pb: 1, display: 'flex', alignItems: 'center', gap: 1, flexWrap: 'wrap' }}>
            <Typography variant="body2">{previewText(preview.count, preview.bytes)}</Typography>
            {preview.count > 0 && (
              <Button size="small" variant="contained" color="error" disabled={busy} onClick={confirmCleanup}>
                Confirm delete
              </Button>
            )}
            <Button size="small" onClick={() => setPreview(null)}>
              Cancel
            </Button>
          </Box>
        )}
        {notice !== null && (
          <Typography variant="body2" color="text.secondary" sx={{ px: 1.5, pb: 1 }}>
            {notice}
          </Typography>
        )}
      </Box>
    </OverlayPanel>
  )
}
