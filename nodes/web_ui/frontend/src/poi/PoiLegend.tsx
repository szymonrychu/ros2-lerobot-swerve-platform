import Box from '@mui/material/Box'
import Typography from '@mui/material/Typography'
import { KIND_COLORS, zoneStyle } from './style'
import type { Poi, PoiZone } from './types'

interface LegendEntry {
  id: string
  label: string
  color: string
}

/** Reference open area per zone, so the legend reads the zone colour from zoneStyle instead of duplicating it. */
function zoneColor(zone: Exclude<PoiZone, 'none'>): string {
  const sample = { kind: 'area', status: 'open', zone } as Poi
  return zoneStyle(sample, false)!.fill
}

const KIND_ENTRIES: LegendEntry[] = [
  { id: 'point', label: 'Point', color: KIND_COLORS.point },
  { id: 'object', label: 'Object', color: KIND_COLORS.object },
  { id: 'photo', label: 'Photo', color: KIND_COLORS.photo },
]

const ZONE_ENTRIES: LegendEntry[] = [
  { id: 'none', label: 'Area, no zone', color: KIND_COLORS['area-none'] },
  { id: 'slow', label: 'Slow zone', color: zoneColor('slow') },
  { id: 'keepout', label: 'Keepout zone', color: zoneColor('keepout') },
]

function Row({ entry }: { entry: LegendEntry }) {
  return (
    <Box sx={{ display: 'flex', alignItems: 'center', gap: 1, minHeight: '1.5rem' }}>
      <Box
        component="span"
        data-testid={`legend-swatch-${entry.id}`}
        data-color={entry.color}
        sx={{ width: '1rem', height: '1rem', bgcolor: entry.color, border: 1, borderColor: 'grey.600', flexShrink: 0, display: 'inline-block' }}
      />
      <Typography variant="body2" component="span">
        {entry.label}
      </Typography>
    </Box>
  )
}

/** Colour key of the map markers: kind fills (point, object, photo) and area zones (none, slow, keepout). */
export function PoiLegend() {
  return (
    <Box sx={{ ml: 4, mb: 0.5 }} aria-label="Marker colour legend">
      {[...KIND_ENTRIES, ...ZONE_ENTRIES].map((e) => (
        <Row key={e.id} entry={e} />
      ))}
    </Box>
  )
}
