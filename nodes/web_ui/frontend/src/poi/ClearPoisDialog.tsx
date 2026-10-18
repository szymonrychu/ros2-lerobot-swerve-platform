/** "Clear POIs" dialog: delete all POIs or those of one creator and kind set, keeping zones unless opted in. */
import { useMemo, useState } from 'react'
import Button from '@mui/material/Button'
import Checkbox from '@mui/material/Checkbox'
import Dialog from '@mui/material/Dialog'
import DialogActions from '@mui/material/DialogActions'
import DialogContent from '@mui/material/DialogContent'
import DialogTitle from '@mui/material/DialogTitle'
import FormControlLabel from '@mui/material/FormControlLabel'
import FormGroup from '@mui/material/FormGroup'
import Link from '@mui/material/Link'
import Radio from '@mui/material/Radio'
import RadioGroup from '@mui/material/RadioGroup'
import Typography from '@mui/material/Typography'
import { clearPois } from './api'
import type { Poi, PoiCreator, PoiKind } from './types'

/** Kinds the dialog can clear (photos are managed in the photo panel). */
const CLEARABLE_KINDS: PoiKind[] = ['point', 'area', 'object']
type CreatorChoice = 'all' | PoiCreator

export interface ClearSelection {
  createdBy: PoiCreator | null
  kinds: PoiKind[]
  keepZones: boolean
}

/**
 * Number of POIs a clear with this selection removes; same rules as poi_store op_clear.
 *
 * @param pois - current POI list
 * @param sel - creator (null = all), kinds and keepZones
 * @returns the count
 */
export function previewCount(pois: Poi[], sel: ClearSelection): number {
  return pois.filter(
    (p) =>
      (sel.createdBy === null || p.created_by === sel.createdBy) &&
      sel.kinds.includes(p.kind) &&
      !(sel.keepZones && p.kind === 'area' && (p.zone === 'keepout' || p.zone === 'slow')),
  ).length
}

export interface ClearPoisDialogProps {
  open: boolean
  tabId: string
  pois: Poi[]
  onClose: () => void
  /** When given, a "Choose individual POIs" link closes the dialog and calls this. */
  onChooseIndividually?: () => void
  fetchImpl?: typeof fetch
}

/**
 * @param props see ClearPoisDialogProps.
 * @returns the dialog element.
 */
export function ClearPoisDialog({ open, tabId, pois, onClose, onChooseIndividually, fetchImpl }: ClearPoisDialogProps) {
  const [creator, setCreator] = useState<CreatorChoice>('all')
  const [kinds, setKinds] = useState<PoiKind[]>(CLEARABLE_KINDS)
  const [keepZones, setKeepZones] = useState(true)
  const [busy, setBusy] = useState(false)
  const [message, setMessage] = useState<{ ok: boolean; text: string } | null>(null)

  const createdBy = creator === 'all' ? null : creator
  const count = useMemo(() => previewCount(pois, { createdBy, kinds, keepZones }), [pois, createdBy, kinds, keepZones])

  const toggleKind = (k: PoiKind) => setKinds((cur) => CLEARABLE_KINDS.filter((x) => (x === k ? !cur.includes(x) : cur.includes(x))))

  const confirm = async () => {
    setBusy(true)
    setMessage(null)
    const r = await clearPois(tabId, { createdBy, kinds, keepZones }, fetchImpl)
    setBusy(false)
    setMessage(r.ok ? { ok: true, text: `Removed ${r.removed ?? count} POIs` } : { ok: false, text: r.message })
  }

  return (
    <Dialog open={open} onClose={onClose} aria-labelledby="clear-pois-title">
      <DialogTitle id="clear-pois-title">Clear POIs</DialogTitle>
      <DialogContent>
        <RadioGroup value={creator} onChange={(e) => setCreator(e.target.value as CreatorChoice)}>
          <FormControlLabel value="all" control={<Radio />} label="All" />
          <FormControlLabel value="user" control={<Radio />} label="Mine" />
          <FormControlLabel value="agent" control={<Radio />} label="Agent's" />
        </RadioGroup>
        <FormGroup row>
          {CLEARABLE_KINDS.map((k) => (
            <FormControlLabel key={k} label={k} control={<Checkbox checked={kinds.includes(k)} onChange={() => toggleKind(k)} />} />
          ))}
        </FormGroup>
        <FormControlLabel
          label="Keep keep-out and slow zones"
          control={<Checkbox checked={keepZones} onChange={(e) => setKeepZones(e.target.checked)} />}
        />
        <Typography data-testid="clear-pois-preview" variant="body2" sx={{ mt: 1 }}>
          {count} POIs will be removed
        </Typography>
        {message && (
          <Typography role={message.ok ? 'status' : 'alert'} color={message.ok ? 'text.primary' : 'error'} variant="body2" sx={{ mt: 1 }}>
            {message.text}
          </Typography>
        )}
        {onChooseIndividually && (
          <Link
            component="button"
            variant="body2"
            sx={{ mt: 1 }}
            onClick={() => {
              onClose()
              onChooseIndividually()
            }}
          >
            Choose individual POIs
          </Link>
        )}
      </DialogContent>
      <DialogActions>
        <Button onClick={onClose}>Close</Button>
        <Button color="error" variant="outlined" disabled={busy || count === 0} onClick={() => void confirm()}>
          Clear
        </Button>
      </DialogActions>
    </Dialog>
  )
}
