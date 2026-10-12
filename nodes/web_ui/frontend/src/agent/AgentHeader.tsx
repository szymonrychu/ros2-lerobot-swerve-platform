/** Agent tab header: the state, the phase dropdown and the time left, plus Stop and New session. */
import { useState } from 'react'
import Box from '@mui/material/Box'
import IconButton from '@mui/material/IconButton'
import RestartAltIcon from '@mui/icons-material/RestartAlt'
import StopIcon from '@mui/icons-material/Stop'
import { StatusItem } from '../components/StatusItem'
import { ConfirmDialog } from '../layout/ConfirmDialog'
import { headerStatus, phaseRows } from './agentModel'
import type { AgentInfo, ChatState } from './agentModel'
import { PhasePanel } from './PhasePanel'
import { TimeBudgetChip } from './TimeBudgetChip'

interface Props {
  /** Agent info from the backend (model); null before the first fetch. */
  info: AgentInfo | null
  chat: ChatState
  connected: boolean
  /** Disables New session (agent busy or battery cut-off). */
  resetBlocked: boolean
  onStop: () => void
  /** Called once the operator confirmed the reset. */
  onReset: () => void
}

/**
 * One-line agent header. Busy is shown once, as the state; model, complexity and usage sit in the state tooltip.
 *
 * @param props - agent info, chat state, connection flag, reset gate and the stop and reset handlers
 * @returns the header row
 */
export function AgentHeader({ info, chat, connected, resetBlocked, onStop, onReset }: Props) {
  const [confirmReset, setConfirmReset] = useState(false)
  const header = headerStatus(info, chat)
  const state = header.busy ? 'working' : connected ? 'idle' : 'disconnected'
  const details = [
    `model ${header.model ?? 'unknown'}`,
    ...(header.complexity !== null ? [`complexity ${header.complexity.replace('_', ' ')}`] : []),
    header.chips.map((chip) => chip.label).join(', '),
  ].join(' - ')
  return (
    <Box sx={{ px: 2, py: 1, display: 'flex', alignItems: 'center', gap: 2, flexWrap: 'wrap', borderBottom: 1, borderColor: 'divider' }}>
      <span data-testid="agent-state">
        <StatusItem label="agent" value={state} state={connected ? 'ok' : 'warn'} tooltip={details} />
      </span>
      <PhasePanel rows={phaseRows(chat.plan)} header={header.phase} />
      <TimeBudgetChip state={chat} />
      <Box sx={{ flex: 1 }} />
      <IconButton size="small" aria-label="Stop agent" color="error" disabled={!chat.busy} onClick={onStop}>
        <StopIcon />
      </IconButton>
      <IconButton size="small" aria-label="New session" disabled={resetBlocked} onClick={() => setConfirmReset(true)}>
        <RestartAltIcon />
      </IconButton>
      <ConfirmDialog
        open={confirmReset}
        title="Start a new session?"
        body="The conversation, the usage counters and the task plan are cleared."
        confirmLabel="New session"
        onCancel={() => setConfirmReset(false)}
        onConfirm={() => {
          setConfirmReset(false)
          onReset()
        }}
      />
    </Box>
  )
}
