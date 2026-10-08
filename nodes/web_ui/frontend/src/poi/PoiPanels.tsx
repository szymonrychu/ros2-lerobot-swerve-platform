/** MUI panels of the POI feature: the collapsible list (click to focus) and the editor of the selected POI. */
import { useEffect, useState } from 'react'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import Chip from '@mui/material/Chip'
import Collapse from '@mui/material/Collapse'
import Dialog from '@mui/material/Dialog'
import DialogActions from '@mui/material/DialogActions'
import DialogContent from '@mui/material/DialogContent'
import DialogContentText from '@mui/material/DialogContentText'
import DialogTitle from '@mui/material/DialogTitle'
import IconButton from '@mui/material/IconButton'
import List from '@mui/material/List'
import ListItemButton from '@mui/material/ListItemButton'
import ListItemText from '@mui/material/ListItemText'
import MenuItem from '@mui/material/MenuItem'
import Paper from '@mui/material/Paper'
import Stack from '@mui/material/Stack'
import TextField from '@mui/material/TextField'
import Typography from '@mui/material/Typography'
import CloseIcon from '@mui/icons-material/Close'
import ExpandLessIcon from '@mui/icons-material/ExpandLess'
import ExpandMoreIcon from '@mui/icons-material/ExpandMore'
import { MONO_FONT } from '../theme'
import { formatUpdated, objectDetails, OBJECT_COLOR, sortPois, STATUS_COLORS } from './style'
import { Poi, POI_NAME_MAX, POI_NOTE_MAX, POI_STATUSES, PoiCommand, PoiResult, PoiStatus } from './types'

const PANEL_BG = 'rgba(22, 27, 34, 0.92)'
const KIND_TITLES = { point: 'Point', area: 'Area', object: 'Object' } as const

/** Collapsible list of all POIs; a row click selects the POI and centres the view on it. */
export function PoiListPanel({
  pois,
  selectedId,
  open,
  onToggle,
  onFocus,
}: {
  pois: Poi[]
  selectedId: string | null
  open: boolean
  onToggle: () => void
  onFocus: (poi: Poi) => void
}) {
  const now = Date.now() / 1000
  return (
    <Paper
      variant="outlined"
      sx={{ bgcolor: PANEL_BG, width: '100%' }}
      onPointerDown={(e) => e.stopPropagation()}
      data-testid="poi-list-panel"
    >
      <Box
        component="button"
        onClick={onToggle}
        aria-expanded={open}
        aria-label={open ? 'Collapse POI list' : 'Expand POI list'}
        sx={{
          all: 'unset',
          boxSizing: 'border-box',
          width: '100%',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'space-between',
          px: 1.5,
          minHeight: 40,
          cursor: 'pointer',
          color: 'text.primary',
        }}
      >
        <Typography variant="overline" sx={{ lineHeight: 2 }}>
          Points of interest ({pois.length})
        </Typography>
        {open ? <ExpandLessIcon fontSize="small" /> : <ExpandMoreIcon fontSize="small" />}
      </Box>
      <Collapse in={open}>
        <List dense disablePadding sx={{ maxHeight: { xs: '28vh', sm: '40vh' }, overflowY: 'auto' }}>
          {pois.length === 0 && (
            <Typography variant="body2" color="text.secondary" sx={{ px: 1.5, pb: 1 }}>
              No points of interest yet.
            </Typography>
          )}
          {sortPois(pois).map((p) => (
            <ListItemButton key={p.id} selected={p.id === selectedId} onClick={() => onFocus(p)} sx={{ minHeight: 48 }}>
              <ListItemText
                primary={p.name || '(unnamed)'}
                secondary={
                  p.kind === 'object'
                    ? `object - ${p.created_by} - ${objectDetails(p, now) || formatUpdated(p.updated_at, now)}`
                    : `${p.kind} - ${p.created_by} - ${formatUpdated(p.updated_at, now)}`
                }
                primaryTypographyProps={{ noWrap: true }}
                secondaryTypographyProps={{ noWrap: true }}
              />
              <Chip
                size="small"
                label={p.status}
                sx={{ ml: 1, bgcolor: p.kind === 'object' ? OBJECT_COLOR : STATUS_COLORS[p.status], color: '#0d1117', fontWeight: 700 }}
              />
            </ListItemButton>
          ))}
        </List>
      </Collapse>
    </Paper>
  )
}

/** Edit form of one POI: name, note, status (and radius for a point; objects also show their sighting details); delete asks for confirmation in a dialog. */
export function PoiEditorPanel({
  poi,
  busy,
  onClose,
  onSend,
}: {
  poi: Poi
  busy: boolean
  onClose: () => void
  onSend: (command: PoiCommand, what: string) => Promise<PoiResult>
}) {
  const [name, setName] = useState(poi.name)
  const [note, setNote] = useState(poi.note)
  const [status, setStatus] = useState<PoiStatus>(poi.status)
  const [radius, setRadius] = useState(String(poi.radius_m))
  const [confirmDelete, setConfirmDelete] = useState(false)

  const radiusValue = Number(radius)
  const radiusValid = poi.kind !== 'point' || (radius.trim() !== '' && Number.isFinite(radiusValue) && radiusValue > 0)
  const dirty =
    name !== poi.name ||
    note !== poi.note ||
    status !== poi.status ||
    (poi.kind === 'point' && radiusValid && radiusValue !== poi.radius_m)
  const valid = name.length <= POI_NAME_MAX && note.length <= POI_NOTE_MAX && radiusValid

  // Follow updates from elsewhere (the agent, another browser) unless the user has unsaved edits.
  useEffect(() => {
    if (dirty) return
    setName(poi.name)
    setNote(poi.note)
    setStatus(poi.status)
    setRadius(String(poi.radius_m))
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [poi.updated_at, poi.id])

  const save = () => {
    const changes: Partial<Poi> & { id: string } = { id: poi.id, name, note, status }
    if (poi.kind === 'point') changes.radius_m = radiusValue
    void onSend({ op: 'update', poi: changes }, 'POI saved')
  }
  const remove = () => {
    setConfirmDelete(false)
    void onSend({ op: 'delete', poi: { id: poi.id } }, 'POI deleted')
  }

  return (
    <Paper
      variant="outlined"
      sx={{ bgcolor: PANEL_BG, width: '100%', p: 1.5, maxHeight: { xs: '60vh', sm: '75vh' }, overflowY: 'auto' }}
      onPointerDown={(e) => e.stopPropagation()}
      onKeyDown={(e) => e.stopPropagation()}
      data-testid="poi-editor"
    >
      <Stack direction="row" sx={{ alignItems: 'center', justifyContent: 'space-between' }}>
        <Typography variant="overline" sx={{ lineHeight: 2 }}>
          {KIND_TITLES[poi.kind]} - created by {poi.created_by}
        </Typography>
        <IconButton aria-label="Close editor" size="small" onClick={onClose}>
          <CloseIcon fontSize="small" />
        </IconButton>
      </Stack>
      <Stack spacing={1.5} sx={{ mt: 0.5 }}>
        <TextField
          label="Name"
          size="small"
          value={name}
          onChange={(e) => setName(e.target.value)}
          error={name.length > POI_NAME_MAX}
          helperText={`${name.length}/${POI_NAME_MAX}`}
          slotProps={{ htmlInput: { maxLength: POI_NAME_MAX } }}
        />
        <TextField
          label="Note"
          size="small"
          multiline
          minRows={3}
          maxRows={8}
          value={note}
          onChange={(e) => setNote(e.target.value)}
          error={note.length > POI_NOTE_MAX}
          helperText={`${note.length}/${POI_NOTE_MAX}`}
          slotProps={{ htmlInput: { maxLength: POI_NOTE_MAX } }}
        />
        <TextField select label="Status" size="small" value={status} onChange={(e) => setStatus(e.target.value as PoiStatus)}>
          {POI_STATUSES.map((s) => (
            <MenuItem key={s} value={s}>
              {s}
            </MenuItem>
          ))}
        </TextField>
        {poi.kind === 'point' && (
          <TextField
            label="Radius (m)"
            size="small"
            type="number"
            value={radius}
            onChange={(e) => setRadius(e.target.value)}
            error={!radiusValid}
            helperText={radiusValid ? undefined : 'Must be greater than 0'}
            slotProps={{ htmlInput: { min: 0.01, step: 0.05 } }}
          />
        )}
        <Typography variant="caption" color="text.secondary" sx={{ fontFamily: MONO_FONT }}>
          {poi.kind === 'area' ? 'centre' : 'at'} {poi.x.toFixed(2)}, {poi.y.toFixed(2)} (map)
          {poi.kind === 'area' ? ` - ${poi.polygon.length} vertices` : ''}
        </Typography>
        {objectDetails(poi, Date.now() / 1000) && (
          <Typography variant="caption" color="text.secondary" data-testid="poi-object-details">
            {objectDetails(poi, Date.now() / 1000)}
          </Typography>
        )}
        <Typography variant="caption" color="text.secondary">
          Drag the selected {poi.kind === 'area' ? 'area or its vertices' : poi.kind} on the map (top view) to move it.
        </Typography>
        <Stack direction="row" spacing={1}>
          <Button variant="contained" onClick={save} disabled={busy || !dirty || !valid} sx={{ flex: 1, minHeight: 44 }}>
            Save
          </Button>
          <Button color="error" variant="outlined" onClick={() => setConfirmDelete(true)} disabled={busy} sx={{ minHeight: 44 }}>
            Delete
          </Button>
        </Stack>
      </Stack>
      <Dialog open={confirmDelete} onClose={() => setConfirmDelete(false)}>
        <DialogTitle>Delete point of interest?</DialogTitle>
        <DialogContent>
          <DialogContentText>
            &quot;{poi.name || '(unnamed)'}&quot; will be removed for everyone, including the agent.
          </DialogContentText>
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setConfirmDelete(false)}>Cancel</Button>
          <Button color="error" variant="contained" onClick={remove}>
            Delete
          </Button>
        </DialogActions>
      </Dialog>
    </Paper>
  )
}
