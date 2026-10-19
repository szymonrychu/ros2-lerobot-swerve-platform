import Box from '@mui/material/Box'
import Typography from '@mui/material/Typography'
import { KIND_COLORS, KIND_SHAPE, SHAPE_CLIP, zoneStyle, type MarkerShape } from './style'
import type { Poi, PoiZone } from './types'

interface LegendEntry {
  id: string
  label: string
  color: string
  shape: MarkerShape
}

/** Reference open area per zone, so the legend reads the zone colour from zoneStyle instead of duplicating it. */
function zoneColor(zone: Exclude<PoiZone, 'none'>): string {
  const sample = { kind: 'area', status: 'open', zone } as Poi
  return zoneStyle(sample, false)!.fill
}

const KIND_ENTRIES: LegendEntry[] = [
  { id: 'point', label: 'Point', color: KIND_COLORS.point, shape: KIND_SHAPE.point },
  { id: 'object', label: 'Object', color: KIND_COLORS.object, shape: KIND_SHAPE.object },
  { id: 'marker', label: 'Marker', color: KIND_COLORS.marker, shape: KIND_SHAPE.marker },
  { id: 'photo', label: 'Photo', color: KIND_COLORS.photo, shape: KIND_SHAPE.photo },
]

const ZONE_ENTRIES: LegendEntry[] = [
  { id: 'none', label: 'Area, no zone', color: KIND_COLORS['area-none'], shape: KIND_SHAPE.area },
  { id: 'slow', label: 'Slow zone', color: zoneColor('slow'), shape: KIND_SHAPE.area },
  { id: 'keepout', label: 'Keepout zone', color: zoneColor('keepout'), shape: KIND_SHAPE.area },
]

function Row({ entry }: { entry: LegendEntry }) {
  return (
    <Box sx={{ display: 'flex', alignItems: 'center', gap: 1, minHeight: '1.5rem' }}>
      <Box
        component="span"
        data-testid={`legend-swatch-${entry.id}`}
        data-color={entry.color}
        data-shape={entry.shape}
        sx={{ width: '1rem', height: '1rem', bgcolor: entry.color, clipPath: SHAPE_CLIP[entry.shape], flexShrink: 0, display: 'inline-block' }}
      />
      <Typography variant="body2" component="span">
        {entry.label}
      </Typography>
    </Box>
  )
}

/** Key of the map markers: kind shapes and greyscale fills (point, object, marker, photo) and area zones (none, slow, keepout). */
export function PoiLegend() {
  return (
    <Box sx={{ mb: 1 }} aria-label="Marker legend">
      {[...KIND_ENTRIES, ...ZONE_ENTRIES].map((e) => (
        <Row key={e.id} entry={e} />
      ))}
    </Box>
  )
}
