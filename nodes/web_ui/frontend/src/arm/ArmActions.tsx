import { useEffect, useState } from 'react'
import { Alert, Box, Button, Menu, MenuItem, Stack, Typography } from '@mui/material'
import BackHandIcon from '@mui/icons-material/BackHand'
import CloseIcon from '@mui/icons-material/Close'
import BookmarkAddIcon from '@mui/icons-material/BookmarkAdd'
import HomeIcon from '@mui/icons-material/Home'
import { ToolButton, ToolGroup } from '../components/ToolButton'
import type { GraspStrategy } from '../grasp/grasp'
import { GRASP_MENU } from '../grasp/pick'
import type { GraspPanelState } from '../grasp/useGrasp'
import { ActionResult, confirmClick, postTabAction, RESET_CONFIRM_MS } from '../map/mapActions'
import type { ArmControl, ControlState } from './armControls'

const TOUCH_TARGET = '2.75rem' // 44 px minimum touch target
export const PICK_HINT = 'Click the object in the arm view'
export const STRATEGY_HINT = 'Choose a strategy in the Grasp menu, then click the object'
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
  /** Grasp state of the tab (useGrasp), or null when the tab has no grasp topic: no Grasp control then. */
  grasp: GraspPanelState | null
  /** The grasp panel is open. */
  graspOpen: boolean
  onGraspOpenChange: (open: boolean) => void
}

/**
 * Arm home, Set home and Grasp of the Arm tab. The Grasp menu picks a strategy and arms the floor pick in the arm view;
 * STOP GRASP shows only while a grasp executes. Set home stores the current pose, so it needs two clicks within
 * RESET_CONFIRM_MS. Why a control is disabled is shown as visible text and as the button tooltip.
 *
 * @param tabId - arm tab id
 * @param controls - per control enabled state and reason
 * @param action - result of the last request
 * @param onAction - receives the busy state and the reply
 * @param grasp - grasp state, or null without a grasp topic
 * @param graspOpen - the grasp panel is open
 * @param onGraspOpenChange - opens or closes the grasp panel
 */
export default function ArmActions({ tabId, controls, action, onAction, grasp, graspOpen, onGraspOpenChange }: Props) {
  const [setHomeArmedAt, setSetHomeArmedAt] = useState<number | null>(null)
  const [menuAnchor, setMenuAnchor] = useState<HTMLElement | null>(null)
  const picking = grasp?.pickMode ?? false
  const cancelPick = grasp?.cancelPick

  useEffect(() => {
    if (setHomeArmedAt === null) return
    const timer = setTimeout(() => setSetHomeArmedAt(null), RESET_CONFIRM_MS)
    return () => clearTimeout(timer)
  }, [setHomeArmedAt])

  // Esc leaves the grasp pick mode without planning.
  useEffect(() => {
    if (!picking || !cancelPick) return
    const onKey = (e: KeyboardEvent) => {
      if (e.key === 'Escape') cancelPick()
    }
    window.addEventListener('keydown', onKey)
    return () => window.removeEventListener('keydown', onKey)
  }, [picking, cancelPick])

  const chooseStrategy = (strategy: GraspStrategy) => {
    setMenuAnchor(null)
    onGraspOpenChange(true)
    grasp?.startPick(strategy)
  }

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
  const shown = grasp ? [controls.armHome, controls.setHome, controls.grasp] : [controls.armHome, controls.setHome]
  const reasons = [...new Set(shown.map((c) => c.reason).filter(Boolean))]
  const hint = picking ? PICK_HINT : graspOpen && grasp && !grasp.hasTarget ? STRATEGY_HINT : ''
  return (
    <Stack spacing={1} data-testid="arm-actions">
      <ToolGroup label="Arm actions">
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
        {grasp && (
          <ToolButton
            label="Grasp"
            icon={<BackHandIcon />}
            pressed={graspOpen || picking}
            disabled={controls.grasp.disabled}
            tooltip={controls.grasp.reason || 'Grasp'}
            expanded={menuAnchor !== null}
            onClick={(e) => setMenuAnchor(e.currentTarget)}
          />
        )}
        {picking && <ToolButton label="Cancel grasp pick" icon={<CloseIcon />} onClick={() => cancelPick?.()} />}
      </ToolGroup>
      <Menu anchorEl={menuAnchor} open={menuAnchor !== null} onClose={() => setMenuAnchor(null)}>
        {GRASP_MENU.map((m) => (
          <MenuItem key={m.strategy} disabled={grasp?.executing || grasp?.planning} onClick={() => chooseStrategy(m.strategy)} sx={{ minHeight: TOUCH_TARGET }}>
            {m.label}
          </MenuItem>
        ))}
      </Menu>
      {grasp?.executing && (
        <Button variant="contained" color="error" onClick={grasp.stop}>
          STOP GRASP
        </Button>
      )}
      {hint && (
        <Typography data-testid="arm-grasp-hint" variant="caption" color="text.secondary">
          {hint}
        </Typography>
      )}
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
