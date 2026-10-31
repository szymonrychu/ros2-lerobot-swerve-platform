import { useEffect, useState } from 'react'
import { Alert, Box, Stack, Typography } from '@mui/material'
import BookmarkAddIcon from '@mui/icons-material/BookmarkAdd'
import HomeIcon from '@mui/icons-material/Home'
import { ToolButton, ToolGroup } from '../components/ToolButton'
import { ActionResult, confirmClick, postTabAction, RESET_CONFIRM_MS } from '../map/mapActions'
import type { ArmControl, ControlState } from './armControls'

const ALERT_SEVERITY = { idle: 'info', busy: 'info', ok: 'success', warning: 'warning', error: 'error' } as const

interface Props {
  /** Arm tab id sent as ?tab= on every request. */
  tabId: string
  /** Enabled state and reason of every arm control (armControlState). */
  controls: Record<ArmControl, ControlState>
  /** Result of the last request (state busy while one is in flight). */
  action: ActionResult
  /** Reports a new result; the host keeps the state so the other controls can read "in flight". */
  onAction: (result: ActionResult) => void
}

/**
 * Arm home and Set home of the Arm tab. Set home stores the current pose, so it needs two clicks within
 * RESET_CONFIRM_MS. Why a control is disabled is shown as visible text and as the button tooltip.
 *
 * @param tabId - arm tab id
 * @param controls - per control enabled state and reason
 * @param action - result of the last request
 * @param onAction - receives the busy state and the reply
 */
export default function ArmActions({ tabId, controls, action, onAction }: Props) {
  const [setHomeArmedAt, setSetHomeArmedAt] = useState<number | null>(null)

  useEffect(() => {
    if (setHomeArmedAt === null) return
    const timer = setTimeout(() => setSetHomeArmedAt(null), RESET_CONFIRM_MS)
    return () => clearTimeout(timer)
  }, [setHomeArmedAt])

  const run = async (path: string, busyMessage: string) => {
    onAction({ state: 'busy', message: busyMessage })
    onAction(await postTabAction(path, tabId))
  }
  const onSetHome = () => {
    const next = confirmClick(setHomeArmedAt, Date.now())
    setSetHomeArmedAt(next.armedAt)
    if (next.fire) void run('/api/arm/set_home', 'Saving arm home pose...')
  }

  const setHomeArmed = setHomeArmedAt !== null
  const reasons = [...new Set([controls.armHome, controls.setHome].map((c) => c.reason).filter(Boolean))]
  return (
    <Stack spacing={1} data-testid="arm-actions">
      <ToolGroup label="Arm home">
        <ToolButton
          label="Arm home"
          icon={<HomeIcon />}
          disabled={controls.armHome.disabled}
          tooltip={controls.armHome.reason || 'Arm home'}
          onClick={() => void run('/api/arm/home', 'Moving arm home...')}
        />
        <ToolButton
          label={setHomeArmed ? 'Confirm set home' : 'Set home'}
          icon={<BookmarkAddIcon />}
          pressed={setHomeArmed}
          disabled={controls.setHome.disabled}
          tooltip={controls.setHome.reason || (setHomeArmed ? 'Confirm set home' : 'Set home: store the current pose as home. Click twice to confirm.')}
          onClick={onSetHome}
        />
      </ToolGroup>
      {reasons.length > 0 && (
        <Box data-testid="arm-actions-reason">
          {reasons.map((reason) => (
            <Typography key={reason} variant="caption" color="text.secondary" display="block">
              {reason}
            </Typography>
          ))}
        </Box>
      )}
      {action.state !== 'idle' && (
        <Alert
          severity={ALERT_SEVERITY[action.state]}
          role={action.state === 'error' ? 'alert' : 'status'}
          onClose={action.state === 'busy' ? undefined : () => onAction({ state: 'idle', message: '' })}
        >
          {action.message}
        </Alert>
      )}
    </Stack>
  )
}
