/** Collapsible list of the agent plan phases: a header dropdown ("phase n/m name") toggles the rows; closed by default, remembered per viewer. */
import { useState } from 'react'
import Box from '@mui/material/Box'
import ButtonBase from '@mui/material/ButtonBase'
import Collapse from '@mui/material/Collapse'
import LinearProgress from '@mui/material/LinearProgress'
import Stack from '@mui/material/Stack'
import Typography from '@mui/material/Typography'
import ExpandMoreIcon from '@mui/icons-material/ExpandMore'
import { PhaseProgress } from './PhaseProgress'
import { StatusItem } from '../components/StatusItem'
import { CONTROL_PX, TOKENS, TOUCH_TARGET_PX } from '../theme'
import type { PhaseRow, PhaseStatus, UsageBar, UsageLevel } from './agentModel'

export const PHASE_PANEL_STORAGE_KEY = 'web_ui.agentPhasesOpen'
/** Open phase list height cap: half the dynamic viewport so the chat stays usable on a phone. */
const PHASE_LIST_MAX_HEIGHT = '50dvh'

/** Storage subset used here, so tests can pass an in-memory stand-in for localStorage. */
export interface PhaseStorage {
  getItem(key: string): string | null
  setItem(key: string, value: string): void
}

/** The active phase as shown in the header (headerStatus().phase). */
export interface PhaseHeader {
  number: number
  total: number
  name: string
  attemptLabel: string | null
  iterationLabel: string | null
}

/**
 * Read whether the viewer left the phase list open.
 *
 * @param storage - localStorage or a stand-in; undefined when unavailable
 * @returns true only when "1" was stored; closed on missing value or any storage error
 */
export function readPhasePanelOpen(storage: PhaseStorage | undefined): boolean {
  try {
    return storage?.getItem(PHASE_PANEL_STORAGE_KEY) === '1'
  } catch {
    return false
  }
}

/**
 * Remember whether the phase list is open; a failing storage is ignored.
 *
 * @param storage - localStorage or a stand-in; undefined when unavailable
 * @param open - new state
 */
export function writePhasePanelOpen(storage: PhaseStorage | undefined, open: boolean): void {
  try {
    storage?.setItem(PHASE_PANEL_STORAGE_KEY, open ? '1' : '0')
  } catch {
    // storage blocked: the state stays per page load
  }
}

function browserStorage(): PhaseStorage | undefined {
  try {
    return window.localStorage
  } catch {
    return undefined
  }
}

/** Row status text colour: only failed and skipped carry hue, every other status stays neutral. */
const PHASE_STATUS_COLORS: Partial<Record<PhaseStatus, string>> = { failed: 'error.main', skipped: 'warning.main' }
const BAR_COLORS: Record<UsageLevel, 'primary' | 'warning' | 'error'> = { ok: 'primary', warn: 'warning', full: 'error' }

function UsageBarView({ bar }: { bar: UsageBar }) {
  return (
    <Box sx={{ minWidth: 88, flex: 1 }} data-testid={`phase-bar-${bar.key}`} data-level={bar.level}>
      <Typography variant="caption" color={bar.level === 'ok' ? 'text.secondary' : `${BAR_COLORS[bar.level]}.main`}>{bar.label}</Typography>
      <LinearProgress variant="determinate" value={bar.fraction * 100} color={BAR_COLORS[bar.level]} sx={{ height: 5, borderRadius: 1 }} />
    </Box>
  )
}

function PhaseRowView({ row }: { row: PhaseRow }) {
  return (
    <Box
      data-testid={`phase-row-${row.number}`}
      data-status={row.status}
      sx={{
        px: 1, py: 0.5, borderRadius: 1, border: 1, borderColor: row.active ? 'primary.main' : 'divider',
        bgcolor: row.active ? 'action.selected' : 'transparent',
      }}
    >
      <Stack direction="row" gap={1} alignItems="baseline" flexWrap="wrap">
        <Typography variant="caption" data-testid="phase-status" sx={{ color: PHASE_STATUS_COLORS[row.status] ?? 'text.secondary' }}>{row.status}</Typography>
        <Typography variant="body2" sx={{ fontWeight: row.active ? 500 : 400 }}>{row.number}. {row.name}</Typography>
        <PhaseProgress lead={row.raised ? 'raised' : undefined} attemptLabel={row.attemptLabel} iterationLabel={row.iterationLabel} pRetrySuccess={row.pRetrySuccess} />
        <Typography variant="caption" color="text.secondary" sx={{ overflowWrap: 'anywhere' }}>{row.goal}</Typography>
      </Stack>
      {row.status !== 'pending' && (
        <Stack direction="row" gap={2} alignItems="center" sx={{ mt: 0.5 }}>
          <Typography variant="caption" color="text.secondary">sensors {row.sensorCalls}</Typography>
          {row.bars.map((bar) => <UsageBarView key={bar.key} bar={bar} />)}
        </Stack>
      )}
      {row.summary && <Typography variant="caption" color="text.secondary" sx={{ display: 'block', overflowWrap: 'anywhere' }}>{row.summary}</Typography>}
    </Box>
  )
}

/**
 * Phase dropdown (a StatusItem that toggles the list) plus the collapsible list of all phases.
 *
 * @param rows - phase rows (phaseRows()); nothing is rendered when empty
 * @param header - the active phase for the dropdown; null after the last phase (it then shows the finished count)
 * @param storage - open/closed persistence, defaults to localStorage
 * @returns the dropdown
 */
export function PhasePanel({ rows, header, storage }: { rows: PhaseRow[]; header: PhaseHeader | null; storage?: PhaseStorage }) {
  const store = storage ?? browserStorage()
  const [open, setOpen] = useState(() => readPhasePanelOpen(store))
  if (rows.length === 0) return null
  const toggle = () => {
    const next = !open
    setOpen(next)
    writePhasePanelOpen(store, next)
  }
  const value = header !== null ? `${header.number}/${header.total} ${header.name}` : `${rows.filter((r) => r.status === 'done').length}/${rows.length} done`
  return (
    <Box sx={{ display: 'contents' }}>
      <ButtonBase onClick={toggle} aria-expanded={open} data-testid="active-phase" sx={{ gap: 0.5, minWidth: 0, borderRadius: 1, alignItems: 'center', minHeight: CONTROL_PX, '@media (pointer: coarse)': { minHeight: TOUCH_TARGET_PX } }}>
        <StatusItem label="phase" value={value} />
        <ExpandMoreIcon sx={{ fontSize: TOKENS.fontSizes.lg, alignSelf: 'center', transform: open ? 'rotate(180deg)' : 'none' }} />
      </ButtonBase>
      {header !== null && <PhaseProgress attemptLabel={header.attemptLabel} iterationLabel={header.iterationLabel} pRetrySuccess={null} />}
      <Box sx={{ flexBasis: '100%' }}>
        <Collapse in={open} unmountOnExit>
          <Box sx={{ pt: 1, maxHeight: PHASE_LIST_MAX_HEIGHT, overflowY: 'auto' }} data-testid="phase-panel">
            <Stack gap={0.5}>
              {rows.map((row) => <PhaseRowView key={row.number} row={row} />)}
            </Stack>
          </Box>
        </Collapse>
      </Box>
    </Box>
  )
}
