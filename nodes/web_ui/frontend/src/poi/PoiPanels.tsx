/** MUI panels of the POI feature: the collapsible list (click to focus) and the editor of the selected POI. */
import { useEffect, useState } from 'react'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import List from '@mui/material/List'
import ListItemButton from '@mui/material/ListItemButton'
import ListItemText from '@mui/material/ListItemText'
import MenuItem from '@mui/material/MenuItem'
import Paper from '@mui/material/Paper'
import Stack from '@mui/material/Stack'
import TextField from '@mui/material/TextField'
import Typography from '@mui/material/Typography'
import ExpandLessIcon from '@mui/icons-material/ExpandLess'
import ExpandMoreIcon from '@mui/icons-material/ExpandMore'
import IconButton from '@mui/material/IconButton'
import { StatusItem } from '../components/StatusItem'
import { ConfirmDialog } from '../layout/ConfirmDialog'
import { OverlayPanel } from '../layout/OverlayPanel'
import { MONO_FONT, TOUCH_TARGET_PX } from '../theme'
import { zonePctError, zoneUpdateCommand } from './editor'
import { formatUpdated, objectDetails, sortPois } from './style'
import { Poi, POI_NAME_MAX, POI_NOTE_MAX, POI_STATUSES, POI_ZONES, PoiCommand, PoiResult, PoiStatus, PoiZone, ZONE_PCT_MAX, ZONE_PCT_MIN } from './types'

/** Fraction of a text limit from which the character counter is shown. */
const COUNTER_FROM = 0.9

const KIND_TITLES = { point: 'Point', area: 'Area', object: 'Object', photo: 'Photo' } as const

/**
 * Character counter text, shown only within 10% of the limit.
 * @param length - current text length.
 * @param max - the limit.
 * @returns "length/max" near the limit, otherwise undefined.
 */
function counterText(length: number, max: number): string | undefined {
  return length >= max * COUNTER_FROM ? `${length}/${max}` : undefined
}

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
  if (!open) {
    return (
      <Paper variant="outlined" sx={{ width: '100%' }} onPointerDown={(e) => e.stopPropagation()} data-testid="poi-list-toggle">
        <Box
          component="button"
          onClick={onToggle}
          aria-expanded={false}
          aria-label="Expand POI list"
          sx={{
            all: 'unset',
            boxSizing: 'border-box',
            width: '100%',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'space-between',
            px: 1,
            minHeight: TOUCH_TARGET_PX,
            cursor: 'pointer',
            color: 'text.primary',
          }}
        >
          <Typography variant="overline" sx={{ lineHeight: 2 }}>
            Points of interest ({pois.length})
          </Typography>
          <ExpandMoreIcon fontSize="small" />
        </Box>
      </Paper>
    )
  }
  return (
    <OverlayPanel
      title={`Points of interest (${pois.length})`}
      open
      anchor="left"
      docked
      actions={
        <IconButton aria-label="Collapse POI list" aria-expanded onClick={onToggle}>
          <ExpandLessIcon />
        </IconButton>
      }
    >
      <Box data-testid="poi-list-panel">
        <List dense disablePadding sx={{ maxHeight: { xs: '28dvh', sm: '40dvh' }, overflowY: 'auto' }}>
          {pois.length === 0 && (
            <Typography variant="body2" color="text.secondary" sx={{ pb: 1 }}>
              No points of interest yet.
            </Typography>
          )}
          {sortPois(pois).map((p) => (
            <ListItemButton key={p.id} selected={p.id === selectedId} onClick={() => onFocus(p)} sx={{ minHeight: (t) => t.spacing(6) }}>
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
              <Box sx={{ ml: 1 }}>
                <StatusItem label="" value={p.status} marked={p.status === 'open'} />
              </Box>
            </ListItemButton>
          ))}
        </List>
      </Box>
    </OverlayPanel>
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
  const [zone, setZone] = useState<PoiZone>(poi.zone ?? 'none')
  const [pct, setPct] = useState(poi.speed_limit_pct == null ? '' : String(poi.speed_limit_pct))
  const [confirmDelete, setConfirmDelete] = useState(false)

  const radiusValue = Number(radius)
  const radiusValid = poi.kind !== 'point' || (radius.trim() !== '' && Number.isFinite(radiusValue) && radiusValue > 0)
  const isArea = poi.kind === 'area'
  const pctError = isArea ? zonePctError(zone, pct) : null
  const zoneDirty = isArea && (zone !== (poi.zone ?? 'none') || (zone === 'slow' && pct !== String(poi.speed_limit_pct ?? '')))
  const dirty =
    zoneDirty ||
    name !== poi.name ||
    note !== poi.note ||
    status !== poi.status ||
    (poi.kind === 'point' && radiusValid && radiusValue !== poi.radius_m)
  const valid = name.length <= POI_NAME_MAX && note.length <= POI_NOTE_MAX && radiusValid && !pctError

  // Follow updates from elsewhere (the agent, another browser) unless the user has unsaved edits.
  useEffect(() => {
    if (dirty) return
    setName(poi.name)
    setNote(poi.note)
    setStatus(poi.status)
    setRadius(String(poi.radius_m))
    setZone(poi.zone ?? 'none')
    setPct(poi.speed_limit_pct == null ? '' : String(poi.speed_limit_pct))
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [poi.updated_at, poi.id])

  const save = () => {
    void onSend(zoneUpdateCommand(poi, { name, note, status, radiusText: radius, zone, pctText: pct }), 'POI saved')
  }
  const remove = () => {
    setConfirmDelete(false)
    void onSend({ op: 'delete', poi: { id: poi.id } }, 'POI deleted')
  }

  return (
    <OverlayPanel title={`${KIND_TITLES[poi.kind]} - created by ${poi.created_by}`} open anchor="left" docked onClose={onClose}>
    <Box
      sx={{ maxHeight: { xs: '60dvh', sm: '75dvh' }, overflowY: 'auto' }}
      onKeyDown={(e) => e.stopPropagation()}
      data-testid="poi-editor"
    >
      <Stack spacing={1} sx={{ mt: 0.5 }}>
        <TextField
          label="Name"
          size="small"
          value={name}
          onChange={(e) => setName(e.target.value)}
          error={name.length > POI_NAME_MAX}
          helperText={counterText(name.length, POI_NAME_MAX)}
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
          helperText={counterText(note.length, POI_NOTE_MAX)}
          slotProps={{ htmlInput: { maxLength: POI_NOTE_MAX } }}
        />
        <TextField select label="Status" size="small" value={status} onChange={(e) => setStatus(e.target.value as PoiStatus)}>
          {POI_STATUSES.map((s) => (
            <MenuItem key={s} value={s}>
              {s}
            </MenuItem>
          ))}
        </TextField>
        {isArea && (
          <TextField select label="Zone" size="small" value={zone} onChange={(e) => setZone(e.target.value as PoiZone)}>
            {POI_ZONES.map((z) => (
              <MenuItem key={z} value={z}>
                {z}
              </MenuItem>
            ))}
          </TextField>
        )}
        {isArea && zone === 'slow' && (
          <TextField
            label="Speed %"
            size="small"
            type="number"
            value={pct}
            onChange={(e) => setPct(e.target.value)}
            error={!!pctError}
            helperText={pctError ?? undefined}
            slotProps={{ htmlInput: { min: ZONE_PCT_MIN, max: ZONE_PCT_MAX, step: 1 } }}
          />
        )}
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
        <Stack direction="row" spacing={1}>
          <Button variant="contained" onClick={save} disabled={busy || !dirty || !valid} sx={{ flex: 1 }}>
            Save
          </Button>
          <Button color="error" variant="outlined" onClick={() => setConfirmDelete(true)} disabled={busy}>
            Delete
          </Button>
        </Stack>
      </Stack>
      <ConfirmDialog
        open={confirmDelete}
        title="Delete point of interest?"
        body={`"${poi.name || '(unnamed)'}" will be removed for everyone, including the agent.`}
        confirmLabel="Delete"
        onConfirm={remove}
        onCancel={() => setConfirmDelete(false)}
      />
    </Box>
    </OverlayPanel>
  )
}
