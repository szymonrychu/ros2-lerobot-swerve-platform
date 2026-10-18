import { useCallback, useEffect, useState } from 'react'
import Alert from '@mui/material/Alert'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import Chip from '@mui/material/Chip'
import Collapse from '@mui/material/Collapse'
import Stack from '@mui/material/Stack'
import Typography from '@mui/material/Typography'
import RefreshIcon from '@mui/icons-material/Refresh'
import { fetchRetrospectives } from '../agent/agentApi'
import type { Retrospective } from '../agent/agentApi'
import { TOKENS, TOUCH_TARGET_PX } from '../theme'
import { TabConfig } from '../types'

const PAGE_SIZE = 50
const REFRESH_MS = 30000
const EMPTY_MESSAGE = 'No retrospectives yet'
const FREE_LABEL = 'free'
const OUTCOME_COLORS: Record<string, string> = {
  done: TOKENS.textPrimary,
  partial: TOKENS.warning,
  failed: TOKENS.error,
  error: TOKENS.error,
  timeout: TOKENS.warning,
  interrupted: TOKENS.textSecondary,
}

interface Props {
  tab: TabConfig
  topicData: Record<string, unknown>
  publish: (topic: string, msgType: string, data: unknown) => void
}

/**
 * Format the cost, turns and duration of a record.
 * @param r the record
 * @returns a one-line summary of the figures that are present
 */
export function statsLine(r: Retrospective): string {
  const parts: string[] = []
  if (r.cost_usd !== null) parts.push(`$${r.cost_usd.toFixed(2)}`)
  if (r.num_turns !== null) parts.push(`${r.num_turns} turns`)
  if (r.duration_s !== null) parts.push(`${Math.round(r.duration_s)} s`)
  return parts.join(' - ')
}

function Section({ title, items, warn }: { title: string; items: string[]; warn?: boolean }) {
  if (items.length === 0) return null
  return (
    <Box sx={{ mb: 1 }}>
      <Typography variant="caption" sx={{ color: warn ? TOKENS.warning : 'text.secondary', fontWeight: TOKENS.fontWeights.medium }}>
        {title}
      </Typography>
      {items.map((item, i) => (
        <Typography key={i} variant="body2" sx={warn ? { color: TOKENS.warning } : undefined}>
          - {item}
        </Typography>
      ))}
    </Box>
  )
}

function RetroRow({ r }: { r: Retrospective }) {
  const [open, setOpen] = useState(false)
  const color = OUTCOME_COLORS[r.outcome] ?? TOKENS.textSecondary
  return (
    <Box component="li" sx={{ listStyle: 'none', borderBottom: `1px solid ${TOKENS.border}` }}>
      <Box
        role="button"
        tabIndex={0}
        aria-expanded={open}
        onClick={() => setOpen(!open)}
        onKeyDown={(e) => {
          if (e.key === 'Enter' || e.key === ' ') setOpen(!open)
        }}
        sx={{ cursor: 'pointer', p: 1, minHeight: TOUCH_TARGET_PX, '&:hover': { bgcolor: TOKENS.bgRaised } }}
      >
        <Stack direction="row" spacing={1} alignItems="center" flexWrap="wrap" useFlexGap>
          <Typography variant="caption" color="text.secondary">{new Date(r.created_at * 1000).toLocaleString()}</Typography>
          <Chip size="small" variant="outlined" label={r.workflow ?? FREE_LABEL} />
          <Chip size="small" variant="outlined" label={r.outcome} sx={{ color, borderColor: color }} />
          {r.author === 'runner' && <Chip size="small" variant="outlined" label="runner" />}
          {r.skipped_mandatory.length > 0 && (
            <Chip size="small" variant="outlined" label={`${r.skipped_mandatory.length} skipped`} sx={{ color: TOKENS.warning, borderColor: TOKENS.warning }} />
          )}
        </Stack>
        <Typography variant="body2" sx={{ mt: 0.5 }}>{r.summary}</Typography>
      </Box>
      <Collapse in={open} unmountOnExit>
        <Box sx={{ px: 1, pb: 1 }}>
          <Section title="Instruction" items={[r.instruction]} />
          <Section title="Went well" items={r.went_well} />
          <Section title="Went wrong" items={r.went_wrong} />
          <Section title="Improvements" items={r.improvements} />
          <Section title="Notes for fixers" items={r.notes_for_fixers} />
          <Section title="Skipped mandatory steps" items={r.skipped_mandatory} warn />
          <Section title="Phases" items={r.phases.map((p) => `${p.name}: ${p.outcome}`)} />
          <Typography variant="caption" color="text.secondary">{statsLine(r)}</Typography>
        </Box>
      </Collapse>
    </Box>
  )
}

export default function RetrosTab(_props: Props) {
  const [records, setRecords] = useState<Retrospective[]>([])
  const [total, setTotal] = useState(0)
  const [error, setError] = useState<string | null>(null)
  const [loaded, setLoaded] = useState(false)
  const [loadingOlder, setLoadingOlder] = useState(false)

  const refresh = useCallback(async () => {
    try {
      const page = await fetchRetrospectives(PAGE_SIZE)
      setRecords((prev) => {
        const ids = new Set(page.retrospectives.map((r) => r.id))
        return [...page.retrospectives, ...prev.filter((r) => !ids.has(r.id))].sort((a, b) => b.created_at - a.created_at)
      })
      setTotal(page.total)
      setError(null)
    } catch (e) {
      setError(e instanceof Error ? e.message : 'network error')
    } finally {
      setLoaded(true)
    }
  }, [])

  useEffect(() => {
    void refresh()
    const timer = setInterval(() => {
      if (!document.hidden) void refresh()
    }, REFRESH_MS)
    return () => clearInterval(timer)
  }, [refresh])

  const loadOlder = async () => {
    const oldest = records[records.length - 1]
    if (!oldest) return
    setLoadingOlder(true)
    try {
      const page = await fetchRetrospectives(PAGE_SIZE, oldest.created_at)
      setRecords((prev) => {
        const ids = new Set(prev.map((r) => r.id))
        return [...prev, ...page.retrospectives.filter((r) => !ids.has(r.id))]
      })
      setTotal(page.total)
      setError(null)
    } catch (e) {
      setError(e instanceof Error ? e.message : 'network error')
    } finally {
      setLoadingOlder(false)
    }
  }

  return (
    <Box sx={{ height: '100%', overflow: 'auto', p: 1 }}>
      <Stack direction="row" alignItems="center" justifyContent="space-between" sx={{ mb: 1 }}>
        <Typography variant="subtitle2">Retrospectives{total > 0 ? ` (${total})` : ''}</Typography>
        <Button size="small" startIcon={<RefreshIcon />} onClick={() => void refresh()} aria-label="Refresh">
          Refresh
        </Button>
      </Stack>
      {error !== null && <Alert severity="error" sx={{ mb: 1 }}>{error}</Alert>}
      {loaded && error === null && records.length === 0 && (
        <Typography color="text.secondary">{EMPTY_MESSAGE}</Typography>
      )}
      <Box component="ul" sx={{ m: 0, p: 0 }}>
        {records.map((r) => <RetroRow key={r.id} r={r} />)}
      </Box>
      {records.length > 0 && records.length < total && (
        <Button sx={{ mt: 1 }} onClick={() => void loadOlder()} disabled={loadingOlder}>Load older</Button>
      )}
    </Box>
  )
}
