/** Panel of one selected photo: image, capture facts, name/note/link editor and delete (confirmed in a dialog). */
import { useState } from 'react'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import Checkbox from '@mui/material/Checkbox'
import FormControlLabel from '@mui/material/FormControlLabel'
import Link from '@mui/material/Link'
import TextField from '@mui/material/TextField'
import Typography from '@mui/material/Typography'
import { ConfirmDialog } from '../layout/ConfirmDialog'
import { OverlayPanel } from '../layout/OverlayPanel'
import { postPoi } from '../poi/api'
import { POI_NAME_MAX, POI_NOTE_MAX, type Poi } from '../poi/types'
import { capturedAt, type Photo, photoImageUrl, photoThumbUrl } from './api'

/** Longest side of the image the panel loads. */
export const PANEL_IMAGE_PX = 640
const DEG_PER_RAD = 180 / Math.PI

function degrees(rad: number | null): string {
  return rad === null ? 'unknown' : `${Math.round(rad * DEG_PER_RAD)} deg`
}

export interface PhotoPanelProps {
  /** map_nav tab id (selects the poi_store topics on the backend). */
  tabId: string
  photo: Photo
  /** Candidate POIs to link; photos in the list are ignored. */
  pois: Poi[]
  onClose: () => void
  /** Called with the photo id after poi_store confirmed the delete. */
  onDeleted: (id: string) => void
  /** fetch implementation (tests). */
  fetchImpl?: typeof fetch
}

/**
 * Photo detail and editor. Save sends one `update` with name, note and linked POI ids; Delete confirms in a dialog, then sends
 * `delete`. Remount (key by photo id) to reset the form for another photo.
 */
export function PhotoPanel({ tabId, photo, pois, onClose, onDeleted, fetchImpl = fetch }: PhotoPanelProps) {
  const [name, setName] = useState(photo.name)
  const [note, setNote] = useState(photo.note)
  const [links, setLinks] = useState<string[]>(photo.poiIds)
  const [confirming, setConfirming] = useState(false)
  const [busy, setBusy] = useState(false)
  const [message, setMessage] = useState<{ ok: boolean; text: string } | null>(null)
  const linkable = pois.filter((p) => p.kind !== 'photo')
  const view = photo.view

  const toggleLink = (id: string) => setLinks((prev) => (prev.includes(id) ? prev.filter((v) => v !== id) : [...prev, id]))

  const save = async () => {
    setBusy(true)
    const ordered = linkable.map((p) => p.id).filter((id) => links.includes(id))
    const kept = links.filter((id) => !ordered.includes(id)) // links to POIs not in the list: leave them as they are
    const r = await postPoi(tabId, { op: 'update', poi: { id: photo.id, name, note, poi_ids: [...kept, ...ordered] } }, fetchImpl)
    setBusy(false)
    setMessage({ ok: r.ok, text: r.ok ? 'Saved.' : `Save failed: ${r.message}` })
  }

  const remove = async () => {
    setBusy(true)
    const r = await postPoi(tabId, { op: 'delete', poi: { id: photo.id } }, fetchImpl)
    setBusy(false)
    setConfirming(false)
    if (r.ok) onDeleted(photo.id)
    else setMessage({ ok: false, text: `Delete failed: ${r.message}` })
  }

  return (
    <OverlayPanel title="Photo" open anchor="left" docked onClose={onClose}>
    <Box data-testid="photo-panel">
      <Link href={photoImageUrl(photo.id)} target="_blank" rel="noreferrer">
        <Box component="img" src={photoThumbUrl(photo.id, PANEL_IMAGE_PX)} alt={`${photo.camera} photo ${capturedAt(photo.ts)}`} sx={{ width: '100%', maxWidth: '100%', height: 'auto', display: 'block', borderRadius: 0.5 }} />
      </Link>
      <Typography variant="body2" sx={{ mt: 1 }}>{`Taken ${capturedAt(photo.ts)}`}</Typography>
      <Typography variant="body2">{`Camera ${photo.camera}, tool ${photo.tool}`}</Typography>
      <Typography variant="body2">
        {view === null
          ? 'Not located on the map'
          : `Yaw ${degrees(view.yaw)}, pitch ${degrees(view.pitch)}, FOV ${degrees(view.hfov)}`}
      </Typography>
      <TextField label="Name" size="small" fullWidth margin="dense" value={name} onChange={(e) => setName(e.target.value)} inputProps={{ maxLength: POI_NAME_MAX }} />
      <TextField label="Note" size="small" fullWidth margin="dense" multiline minRows={2} value={note} onChange={(e) => setNote(e.target.value)} inputProps={{ maxLength: POI_NOTE_MAX }} />
      <Typography variant="caption" color="text.secondary" sx={{ display: 'block', mt: 0.5 }}>
        Linked POIs
      </Typography>
      <Box sx={{ maxHeight: '8rem', overflowY: 'auto' }} role="group" aria-label="Linked POIs">
        {linkable.length === 0 && (
          <Typography variant="body2" color="text.secondary">
            No POIs to link.
          </Typography>
        )}
        {linkable.map((p) => (
          <FormControlLabel
            key={p.id}
            sx={{ display: 'flex', mr: 0 }}
            control={<Checkbox size="small" checked={links.includes(p.id)} onChange={() => toggleLink(p.id)} />}
            label={p.name || '(unnamed)'}
          />
        ))}
      </Box>
      <Box sx={{ display: 'flex', gap: 1, mt: 1, alignItems: 'center', flexWrap: 'wrap' }}>
        <Button variant="contained" disabled={busy} onClick={save}>
          Save
        </Button>
        <Button variant="outlined" color="error" disabled={busy} onClick={() => setConfirming(true)}>
          Delete
        </Button>
      </Box>
      <ConfirmDialog
        open={confirming}
        title="Delete photo?"
        body="Delete this photo and its image?"
        confirmLabel="Delete"
        onConfirm={() => void remove()}
        onCancel={() => setConfirming(false)}
      />
      {message !== null && (
        <Typography variant="body2" color={message.ok ? 'text.secondary' : 'error'} role={message.ok ? undefined : 'alert'} sx={{ mt: 0.5 }}>
          {message.text}
        </Typography>
      )}
    </Box>
    </OverlayPanel>
  )
}
