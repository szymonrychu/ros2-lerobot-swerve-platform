/** Named maps dialogs: the saved map list (load, delete, retry at the saved pose) and the save-as name dialog. */
import { useState } from 'react'
import Alert from '@mui/material/Alert'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import CircularProgress from '@mui/material/CircularProgress'
import Dialog from '@mui/material/Dialog'
import DialogActions from '@mui/material/DialogActions'
import DialogContent from '@mui/material/DialogContent'
import DialogTitle from '@mui/material/DialogTitle'
import List from '@mui/material/List'
import ListItem from '@mui/material/ListItem'
import ListItemText from '@mui/material/ListItemText'
import TextField from '@mui/material/TextField'
import Typography from '@mui/material/Typography'
import { ConfirmDialog } from '../layout/ConfirmDialog'
import type { ActionResult } from './mapActions'
import { MAX_NOTE_CHARS, mapNameError, postMapCommand } from './mapsApi'
import type { MapCommandBody, MapOpName } from './mapsApi'
import type { MapStatus, MapSummary } from '../types'

const IDLE: ActionResult = { state: 'idle', message: '' }

interface MapsDialogProps {
  open: boolean
  tabId: string
  /** Latest status; undefined before the first one arrives. */
  status: MapStatus | undefined
  onClose: () => void
  /** Called after a request succeeded so the page can refresh the status. */
  onChanged: () => void
}

type Pending = { op: 'load' | 'delete'; map: MapSummary } | null

/**
 * Dialog listing the saved maps with Load and Delete, a busy state and the last result.
 *
 * @param props - open flag, tab id, latest status, close and changed callbacks.
 * @returns the dialog element.
 */
export function MapsDialog({ open, tabId, status, onClose, onChanged }: MapsDialogProps) {
  const [busy, setBusy] = useState(false)
  const [result, setResult] = useState<ActionResult>(IDLE)
  const [pending, setPending] = useState<Pending>(null)

  const run = async (op: MapOpName, body: MapCommandBody) => {
    setBusy(true)
    setResult(IDLE)
    const outcome = await postMapCommand(op, tabId, body)
    setBusy(false)
    setResult(outcome)
    if (outcome.state === 'ok') onChanged()
  }

  const maps = status?.maps ?? []
  const active = maps.find((m) => m.name === status?.active_map)
  const state = status?.localisation.state
  const canRetry = active !== undefined && active.valid && (state === 'not_localised' || state === 'ambiguous')

  return (
    <>
      <Dialog open={open} onClose={onClose} fullWidth maxWidth="sm" aria-labelledby="maps-dialog-title">
        <DialogTitle id="maps-dialog-title">Maps</DialogTitle>
        <DialogContent>
          {busy && (
            <Box sx={{ display: 'flex', alignItems: 'center', gap: 1, mb: 1 }}>
              <CircularProgress size={16} />
              <Typography variant="body2">Working...</Typography>
            </Box>
          )}
          {result.state !== 'idle' && (
            <Alert severity={result.state === 'ok' ? 'success' : 'error'} sx={{ mb: 1 }}>
              {result.message}
            </Alert>
          )}
          {canRetry && active && (
            <Button
              variant="outlined"
              disabled={busy}
              sx={{ mb: 1 }}
              onClick={() => void run('load', { name: active.name, pose: active.last_pose })}
            >
              Retry at saved pose
            </Button>
          )}
          {maps.length === 0 ? (
            <Typography variant="body2" color="text.secondary">
              No saved maps
            </Typography>
          ) : (
            <List dense disablePadding>
              {maps.map((m) => (
                <ListItem
                  key={m.name}
                  disableGutters
                  secondaryAction={
                    <Box sx={{ display: 'flex', gap: 1 }}>
                      {m.valid && (
                        <Button size="small" aria-label={`Load ${m.name}`} disabled={busy} onClick={() => setPending({ op: 'load', map: m })}>
                          Load
                        </Button>
                      )}
                      <Button size="small" color="error" aria-label={`Delete ${m.name}`} disabled={busy} onClick={() => setPending({ op: 'delete', map: m })}>
                        Delete
                      </Button>
                    </Box>
                  }
                >
                  <ListItemText
                    primary={m.name}
                    secondary={
                      m.valid
                        ? `${new Date(m.saved_at * 1000).toLocaleString()}, ${m.poi_count} POIs${m.note ? `, ${m.note}` : ''}`
                        : `Invalid: ${m.error}`
                    }
                    secondaryTypographyProps={{ color: m.valid ? 'text.secondary' : 'error' }}
                    sx={{ pr: 16 }}
                  />
                </ListItem>
              ))}
            </List>
          )}
        </DialogContent>
        <DialogActions>
          <Button onClick={onClose}>Close</Button>
        </DialogActions>
      </Dialog>
      <ConfirmDialog
        open={pending?.op === 'load'}
        title={`Load ${pending?.map.name ?? ''}?`}
        body="The current map and POIs are replaced by this map and its POIs. An autosave of the current map is kept."
        confirmLabel="Load"
        onCancel={() => setPending(null)}
        onConfirm={() => {
          const name = pending?.map.name
          setPending(null)
          if (name) void run('load', { name })
        }}
      />
      <ConfirmDialog
        open={pending?.op === 'delete'}
        title={`Delete ${pending?.map.name ?? ''} and its ${pending?.map.poi_count ?? 0} POIs?`}
        confirmLabel="Delete"
        onCancel={() => setPending(null)}
        onConfirm={() => {
          const name = pending?.map.name
          setPending(null)
          if (name) void run('delete', { name })
        }}
      />
    </>
  )
}

interface SaveMapAsDialogProps {
  open: boolean
  tabId: string
  /** Names of the maps already saved; saving one of them asks to overwrite. */
  existing: readonly string[]
  onClose: () => void
  /** Called with the server's answer after a save request finished (ok or not). */
  onSaved: (result: ActionResult) => void
}

/**
 * Name dialog for "Save map as": checks the name rule, asks before overwriting, sends the save.
 *
 * @param props - open flag, tab id, existing names, close and saved callbacks.
 * @returns the dialog element.
 */
export function SaveMapAsDialog({ open, tabId, existing, onClose, onSaved }: SaveMapAsDialogProps) {
  const [name, setName] = useState('')
  const [note, setNote] = useState('')
  const [error, setError] = useState<string | null>(null)
  const [overwriteAsk, setOverwriteAsk] = useState(false)
  const [busy, setBusy] = useState(false)

  const send = async (overwrite: boolean) => {
    setBusy(true)
    const outcome = await postMapCommand('save', tabId, { name, note, overwrite })
    setBusy(false)
    if (outcome.state === 'ok') {
      setName('')
      setNote('')
    }
    onSaved(outcome)
    onClose()
  }

  const submit = () => {
    const invalid = mapNameError(name)
    setError(invalid)
    if (invalid) return
    if (existing.includes(name)) setOverwriteAsk(true)
    else void send(false)
  }

  return (
    <>
      <Dialog open={open} onClose={onClose} fullWidth maxWidth="xs" aria-labelledby="save-map-title">
        <DialogTitle id="save-map-title">Save map as</DialogTitle>
        <DialogContent>
          <TextField
            autoFocus
            fullWidth
            margin="dense"
            label="Map name"
            value={name}
            error={error !== null}
            helperText={error ?? ' '}
            onChange={(e) => setName(e.target.value)}
          />
          <TextField
            fullWidth
            margin="dense"
            label="Note (optional)"
            value={note}
            slotProps={{ htmlInput: { maxLength: MAX_NOTE_CHARS } }}
            onChange={(e) => setNote(e.target.value)}
          />
        </DialogContent>
        <DialogActions>
          <Button onClick={onClose}>Cancel</Button>
          <Button variant="outlined" disabled={busy} onClick={submit}>
            Save
          </Button>
        </DialogActions>
      </Dialog>
      <ConfirmDialog
        open={overwriteAsk}
        title={`Overwrite ${name}?`}
        body="The saved map and its POIs are replaced by the current map and POIs."
        confirmLabel="Overwrite"
        onCancel={() => setOverwriteAsk(false)}
        onConfirm={() => {
          setOverwriteAsk(false)
          void send(true)
        }}
      />
    </>
  )
}
