/** "Delete marked" dialog: confirms deleting the marked POIs and photos in one action, with an extra consent for keep-out zones. */
import { useMemo, useState } from 'react'
import Alert from '@mui/material/Alert'
import Button from '@mui/material/Button'
import Checkbox from '@mui/material/Checkbox'
import Dialog from '@mui/material/Dialog'
import DialogActions from '@mui/material/DialogActions'
import DialogContent from '@mui/material/DialogContent'
import DialogTitle from '@mui/material/DialogTitle'
import FormControlLabel from '@mui/material/FormControlLabel'
import Typography from '@mui/material/Typography'
import { clearPois } from './api'
import { summariseMarked } from './selection'
import type { Photo } from '../photos/api'
import type { Poi } from './types'

export interface DeleteMarkedDialogProps {
  open?: boolean
  /** map_nav tab id. */
  tabId: string
  pois: Poi[]
  /** Photos of the map; marked ones are deleted with the POIs. */
  photos?: Photo[]
  marked: ReadonlySet<string>
  /** Called after a successful delete; the caller clears the marks. */
  onDone: () => void
  onClose: () => void
}

const NO_PHOTOS: Photo[] = []

/**
 * @param props see DeleteMarkedDialogProps.
 * @returns the dialog element.
 */
export function DeleteMarkedDialog({ open = true, tabId, pois, photos = NO_PHOTOS, marked, onDone, onClose }: DeleteMarkedDialogProps) {
  const live = useMemo(() => summariseMarked(pois, marked), [pois, marked])
  const livePhotos = useMemo(() => photos.filter((p) => marked.has(p.id)).length, [photos, marked])
  const [done, setDone] = useState<{ summary: ReturnType<typeof summariseMarked>; photos: number } | null>(null)
  // After success the parent clears the marks; keep showing what was deleted.
  const summary = done?.summary ?? live
  const photoCount = done?.photos ?? livePhotos
  const [consent, setConsent] = useState(false)
  const [busy, setBusy] = useState(false)
  const [message, setMessage] = useState<{ ok: boolean; text: string } | null>(null)

  const keepoutCount = summary.keepouts.length
  const categories = Object.entries(summary.byCategory)
    .map(([c, n]) => `${n} ${c}`)
    .join(', ')
  const canConfirm = summary.count + photoCount > 0 && !busy && !message?.ok && (keepoutCount === 0 || consent)

  const confirm = async () => {
    setBusy(true)
    setMessage(null)
    const r = await clearPois(tabId, { ids: [...marked] })
    setBusy(false)
    if (r.ok) {
      setDone({ summary, photos: photoCount })
      const pois = r.pois ?? r.removed ?? summary.count
      setMessage({ ok: true, text: r.photos === undefined ? `Removed ${pois} POIs` : `Removed ${pois} POIs and ${r.photos} photos` })
      onDone()
    } else {
      setMessage({ ok: false, text: r.message })
    }
  }

  return (
    <Dialog open={open} onClose={onClose} aria-labelledby="delete-marked-title">
      <DialogTitle id="delete-marked-title">{photoCount > 0 ? `Delete ${summary.count} POIs and ${photoCount} photos?` : `Delete ${summary.count} POIs?`}</DialogTitle>
      <DialogContent>
        <Typography data-testid="delete-marked-categories" variant="body2">
          {categories}
        </Typography>
        {keepoutCount > 0 && (
          <>
            <Alert severity="warning" sx={{ mt: 1 }}>
              Keep-out zones in this batch: {summary.keepouts.map((p) => p.name || '(unnamed)').join(', ')}. Without them the robot may drive into the
              protected area.
            </Alert>
            <FormControlLabel
              label={`I understand this removes ${keepoutCount} keep-out zone(s)`}
              control={<Checkbox checked={consent} onChange={(e) => setConsent(e.target.checked)} />}
            />
          </>
        )}
        {message && (
          <Typography role={message.ok ? 'status' : 'alert'} color={message.ok ? 'text.primary' : 'error'} variant="body2" sx={{ mt: 1 }}>
            {message.text}
          </Typography>
        )}
      </DialogContent>
      <DialogActions>
        <Button onClick={onClose}>{message?.ok ? 'Close' : 'Cancel'}</Button>
        <Button color="error" variant="outlined" disabled={!canConfirm} onClick={() => void confirm()}>
          Delete
        </Button>
      </DialogActions>
    </Dialog>
  )
}
