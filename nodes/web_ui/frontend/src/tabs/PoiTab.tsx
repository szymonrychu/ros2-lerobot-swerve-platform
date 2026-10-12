import { useMemo, useState } from 'react'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import Chip from '@mui/material/Chip'
import List from '@mui/material/List'
import ListItem from '@mui/material/ListItem'
import Stack from '@mui/material/Stack'
import TextField from '@mui/material/TextField'
import Typography from '@mui/material/Typography'
import { filterItems, focusRequestFor, isLocated, sortItems } from '../poi/poiItems'
import type { PendingFocus, PoiItem } from '../poi/poiItems'
import { formatUpdated, KIND_COLORS, ZONE_COLORS } from '../poi/style'
import type { PoiKind } from '../poi/types'
import { TOUCH_TARGET_PX } from '../theme'

const KIND_ORDER: readonly PoiKind[] = ['point', 'area', 'object', 'photo']
const NO_POSITION_REASON = 'no map position'
const MS_PER_SECOND = 1000

interface Props {
  items: PoiItem[]
  onShowOnMap: (focus: PendingFocus) => void
  /** Unix seconds used for relative times; defaults to the current time. */
  now?: number
}

/**
 * Swatch colour of an item: its kind colour, or the zone colour of an area with a zone.
 *
 * @param item - item
 * @returns CSS colour
 */
export function swatchColor(item: PoiItem): string {
  if (item.kind === 'area') return item.zone === 'keepout' || item.zone === 'slow' ? ZONE_COLORS[item.zone] : KIND_COLORS['area-none']
  return KIND_COLORS[item.kind]
}

/**
 * Unified, filterable list of POIs and photos with a jump-to-map button per row.
 *
 * @param props - items to list, the map-focus callback and an optional "now" (unix seconds)
 * @returns the tab
 */
export default function PoiTab({ items, onShowOnMap, now }: Props) {
  const [kinds, setKinds] = useState<Set<PoiKind>>(new Set())
  const [query, setQuery] = useState('')
  const visible = useMemo(() => sortItems(filterItems(items, kinds, query)), [items, kinds, query])
  const clock = now ?? Date.now() / MS_PER_SECOND

  const toggle = (kind: PoiKind) =>
    setKinds((prev) => {
      const next = new Set(prev)
      if (!next.delete(kind)) next.add(kind)
      return next
    })

  return (
    <Box sx={{ height: '100%', overflow: 'auto', p: 2 }}>
      <Stack spacing={1.5}>
        <TextField size="small" label="Search" value={query} onChange={(e) => setQuery(e.target.value)} fullWidth />
        <Stack direction="row" spacing={1} useFlexGap sx={{ flexWrap: 'wrap' }}>
          {KIND_ORDER.map((kind) => (
            <Chip
              key={kind}
              label={kind}
              clickable
              role="button"
              color={kinds.has(kind) ? 'primary' : 'default'}
              variant={kinds.has(kind) ? 'filled' : 'outlined'}
              onClick={() => toggle(kind)}
              sx={{ minHeight: TOUCH_TARGET_PX }}
            />
          ))}
        </Stack>
        {items.length === 0 ? (
          <Typography color="text.secondary">No points of interest or photos yet.</Typography>
        ) : visible.length === 0 ? (
          <Typography color="text.secondary">No matches.</Typography>
        ) : (
          <List disablePadding>
            {visible.map((item) => (
              <ListItem key={item.key} divider sx={{ gap: 1.5, flexWrap: 'wrap', px: 0 }}>
                <Box data-testid="kind-swatch" data-color={swatchColor(item)} sx={{ width: '1rem', height: '1rem', borderRadius: '50%', bgcolor: swatchColor(item), flexShrink: 0 }} />
                <Box sx={{ flex: '1 1 12rem', minWidth: 0 }}>
                  <Typography variant="subtitle2" noWrap>{item.name}</Typography>
                  <Typography variant="caption" color="text.secondary" component="div">
                    {[item.kind, item.zone && item.zone !== 'none' ? item.zone : null, item.status, formatUpdated(item.updated, clock)]
                      .filter(Boolean)
                      .map((part, i) => <span key={i}>{i > 0 ? ' - ' : ''}{part}</span>)}
                  </Typography>
                </Box>
                <Stack alignItems="flex-end">
                  <Button
                    size="small"
                    variant="outlined"
                    disabled={!isLocated(item)}
                    onClick={() => {
                      const focus = focusRequestFor(item)
                      if (focus) onShowOnMap(focus)
                    }}
                    sx={{ minHeight: TOUCH_TARGET_PX }}
                  >
                    Show on map
                  </Button>
                  {!isLocated(item) && <Typography variant="caption" color="text.secondary">{NO_POSITION_REASON}</Typography>}
                </Stack>
              </ListItem>
            ))}
          </List>
        )}
      </Stack>
    </Box>
  )
}
