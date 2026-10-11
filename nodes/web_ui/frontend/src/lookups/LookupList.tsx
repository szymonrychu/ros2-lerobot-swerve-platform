/** Thumbnails of recent camera lookups, for the map tab (filtered by the selected POI when one is selected). */
import { useEffect, useState } from 'react'
import Box from '@mui/material/Box'
import Link from '@mui/material/Link'
import List from '@mui/material/List'
import ListItem from '@mui/material/ListItem'
import ListItemText from '@mui/material/ListItemText'
import Paper from '@mui/material/Paper'
import Typography from '@mui/material/Typography'
import { fetchLookups, Lookup, LOOKUP_LIMIT, lookupImageUrl, lookupThumbUrl } from './api'

const PANEL_BG = 'rgba(22, 27, 34, 0.92)'
const THUMB_WIDTH_PX = 72
const REFRESH_MS = 15000

/**
 * Lookups of the backend, refreshed when the POI filter changes and every few seconds. A failed refresh keeps the
 * last lookups and sets the error; the next successful one clears it.
 *
 * @param poiId - only lookups linked to this POI; null for all
 * @returns the lookups (newest first) and the last fetch error, null when the last fetch worked
 */
export function useLookups(poiId: string | null): { lookups: Lookup[]; error: string | null } {
  const [lookups, setLookups] = useState<Lookup[]>([])
  const [error, setError] = useState<string | null>(null)
  useEffect(() => {
    let cancelled = false
    const load = () => {
      fetchLookups(poiId, LOOKUP_LIMIT).then(
        (list) => {
          if (cancelled) return
          setLookups(list)
          setError(null)
        },
        (err: unknown) => {
          if (!cancelled) setError(err instanceof Error ? err.message : String(err))
        },
      )
    }
    load()
    const timer = setInterval(load, REFRESH_MS)
    return () => {
      cancelled = true
      clearInterval(timer)
    }
  }, [poiId])
  return { lookups, error }
}

/** Presentational list: a thumbnail (click opens the full image), camera, time and note per lookup. */
export function LookupList({ lookups, error = null, selectedPoiName }: { lookups: Lookup[]; error?: string | null; selectedPoiName: string | null }) {
  const title = selectedPoiName === null ? `Camera lookups (${lookups.length})` : `Lookups of ${selectedPoiName} (${lookups.length})`
  return (
    <Paper variant="outlined" sx={{ bgcolor: PANEL_BG, width: '100%' }} onPointerDown={(e) => e.stopPropagation()} data-testid="lookup-list-panel">
      <Typography variant="overline" sx={{ display: 'block', px: 1.5, lineHeight: 2 }}>
        {title}
      </Typography>
      <List dense disablePadding sx={{ maxHeight: { xs: '24vh', sm: '32vh' }, overflowY: 'auto' }}>
        {error !== null && (
          <Typography variant="body2" color="error" sx={{ px: 1.5, pb: 1 }} role="alert">
            {`Could not load lookups: ${error}`}
          </Typography>
        )}
        {lookups.length === 0 && error === null && (
          <Typography variant="body2" color="text.secondary" sx={{ px: 1.5, pb: 1 }}>
            {selectedPoiName === null ? 'No lookups yet.' : `No lookups linked to ${selectedPoiName} yet.`}
          </Typography>
        )}
        {lookups.map((l) => (
          <ListItem key={l.id} sx={{ gap: 1, alignItems: 'flex-start' }} data-testid="lookup-row">
            <Link href={lookupImageUrl(l.id)} target="_blank" rel="noreferrer" sx={{ flexShrink: 0 }}>
              <Box
                component="img"
                src={lookupThumbUrl(l.id)}
                alt={`${l.camera} lookup ${l.time}`}
                loading="lazy"
                sx={{ width: THUMB_WIDTH_PX, height: 'auto', display: 'block', borderRadius: 0.5 }}
              />
            </Link>
            <ListItemText primary={`${l.camera} - ${l.time}`} secondary={l.note || l.tool} secondaryTypographyProps={{ sx: { overflowWrap: 'anywhere' } }} />
          </ListItem>
        ))}
      </List>
    </Paper>
  )
}

/** The list wired to the backend and the selected POI. */
export function LookupListPanel({ selectedPoiId, selectedPoiName }: { selectedPoiId: string | null; selectedPoiName: string | null }) {
  const { lookups, error } = useLookups(selectedPoiId)
  return <LookupList lookups={lookups} error={error} selectedPoiName={selectedPoiName} />
}
