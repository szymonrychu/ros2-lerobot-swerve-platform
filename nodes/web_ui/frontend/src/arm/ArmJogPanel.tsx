import { useEffect, useState, type ReactNode } from 'react'
import { Alert, Box, Button, Stack, Tooltip, Typography } from '@mui/material'
import ArrowBackIcon from '@mui/icons-material/ArrowBack'
import ArrowDownwardIcon from '@mui/icons-material/ArrowDownward'
import ArrowForwardIcon from '@mui/icons-material/ArrowForward'
import ArrowUpwardIcon from '@mui/icons-material/ArrowUpward'
import CloseFullscreenIcon from '@mui/icons-material/CloseFullscreen'
import KeyboardArrowDownIcon from '@mui/icons-material/KeyboardArrowDown'
import KeyboardArrowUpIcon from '@mui/icons-material/KeyboardArrowUp'
import NorthIcon from '@mui/icons-material/North'
import OpenInFullIcon from '@mui/icons-material/OpenInFull'
import RotateLeftIcon from '@mui/icons-material/RotateLeft'
import RotateRightIcon from '@mui/icons-material/RotateRight'
import SouthIcon from '@mui/icons-material/South'
import UndoIcon from '@mui/icons-material/Undo'
import RedoIcon from '@mui/icons-material/Redo'
import { StatusItem } from '../components/StatusItem'
import { ToolButton, ToolGroup } from '../components/ToolButton'
import { OverlayPanel } from '../layout/OverlayPanel'
import { useLayout } from '../layout/useLayout'
import { confirmClick, RESET_CONFIRM_MS } from '../map/mapActions'
import { TabConfig } from '../types'
import { ArmAction, ArmResult, JogSpec, postArm } from './armApi'
import { activeSourceOf, HELD_SOURCE } from './armControls'

const LEASE_ACTIONS = ['take_control', 'release', 'home'] as const
type LeaseAction = (typeof LEASE_ACTIONS)[number]
const LEASE_LABELS: Record<LeaseAction, { label: string; confirm: string; hint: string }> = {
  take_control: { label: 'Take control', confirm: 'Confirm take control', hint: 'Take the arm from the current source. Click twice to confirm.' },
  release: { label: 'Release', confirm: 'Confirm release', hint: 'Hand the arm back. Click twice to confirm.' },
  home: { label: 'Home', confirm: 'Confirm home', hint: 'Move the arm to its saved home pose. Click twice to confirm.' },
}

interface JogButton {
  label: string
  icon: ReactNode
  jog: JogSpec
}

const JOG_GROUPS: { title: string; buttons: JogButton[] }[] = [
  {
    title: 'Tool',
    buttons: [
      { label: 'Up', icon: <ArrowUpwardIcon />, jog: { kind: 'cartesian', direction: 'up' } },
      { label: 'Down', icon: <ArrowDownwardIcon />, jog: { kind: 'cartesian', direction: 'down' } },
      { label: 'Forward', icon: <NorthIcon />, jog: { kind: 'cartesian', direction: 'forward' } },
      { label: 'Back', icon: <SouthIcon />, jog: { kind: 'cartesian', direction: 'back' } },
      { label: 'Left', icon: <ArrowBackIcon />, jog: { kind: 'cartesian', direction: 'left' } },
      { label: 'Right', icon: <ArrowForwardIcon />, jog: { kind: 'cartesian', direction: 'right' } },
    ],
  },
  {
    title: 'Joints',
    buttons: [
      { label: 'Pan left', icon: <UndoIcon />, jog: { kind: 'shoulder_pan', direction: 'left' } },
      { label: 'Pan right', icon: <RedoIcon />, jog: { kind: 'shoulder_pan', direction: 'right' } },
      { label: 'Wrist up', icon: <KeyboardArrowUpIcon />, jog: { kind: 'wrist_pitch', direction: 'up' } },
      { label: 'Wrist down', icon: <KeyboardArrowDownIcon />, jog: { kind: 'wrist_pitch', direction: 'down' } },
      { label: 'Roll left', icon: <RotateLeftIcon />, jog: { kind: 'wrist_roll', direction: 'left' } },
      { label: 'Roll right', icon: <RotateRightIcon />, jog: { kind: 'wrist_roll', direction: 'right' } },
    ],
  },
  {
    title: 'Gripper',
    buttons: [
      { label: 'Gripper open', icon: <OpenInFullIcon />, jog: { kind: 'gripper', direction: 'open' } },
      { label: 'Gripper close', icon: <CloseFullscreenIcon />, jog: { kind: 'gripper', direction: 'close' } },
    ],
  },
]

interface Props {
  tab: TabConfig
  topicData: Record<string, unknown>
  /** Arm torque is off: every command but Release is disabled. */
  released?: boolean
}

/**
 * Jog panel of the Arm tab: active command source, Take control / Release / Home and small-step jog buttons.
 * Jog buttons and Release are enabled only while the web UI holds the arm lease.
 *
 * @param tab - arm tab config (id, active_source_topic)
 * @param topicData - latest payload per topic
 * @param released - arm torque is off, disables everything but Release
 */
export default function ArmJogPanel({ tab, topicData, released = false }: Props) {
  const layout = useLayout()
  const [result, setResult] = useState<ArmResult | null>(null)
  const [armed, setArmed] = useState<{ action: LeaseAction; at: number } | null>(null)
  const [busy, setBusy] = useState(false)

  const source = activeSourceOf(tab, topicData)
  const held = source === HELD_SOURCE

  useEffect(() => {
    if (armed === null) return
    const timer = setTimeout(() => setArmed(null), RESET_CONFIRM_MS)
    return () => clearTimeout(timer)
  }, [armed])

  const run = async (action: ArmAction, jog?: JogSpec) => {
    setBusy(true)
    setResult(await postArm(action, tab.id, jog))
    setBusy(false)
  }

  /** Robot motion: the first click arms the action, a second click on the same one within the window fires it. */
  const onLease = (action: LeaseAction) => {
    const next = confirmClick(armed?.action === action ? armed.at : null, Date.now())
    setArmed(next.armedAt === null ? null : { action, at: next.armedAt })
    if (next.fire) void run(action)
  }

  const leaseDisabled: Record<LeaseAction, boolean> = { take_control: busy || released, release: !held || busy, home: busy || released }

  return (
    <OverlayPanel title="Arm control" open anchor="bottom" docked={layout !== 'phone'}>
      <Stack spacing={1} sx={{ p: 1, overflowY: 'auto' }}>
        <Box data-testid="arm-active-source">
          <StatusItem label="Control" value={source ?? 'unknown'} tooltip="Source currently commanding the arm" />
        </Box>
        <Stack direction="row" spacing={1} useFlexGap flexWrap="wrap">
          {LEASE_ACTIONS.map((action) => {
            const text = LEASE_LABELS[action]
            const isArmed = armed?.action === action
            return (
              <Tooltip key={action} title={text.hint}>
                <span>
                  <Button
                    variant={isArmed ? 'contained' : 'outlined'}
                    color={isArmed ? 'warning' : 'primary'}
                    disabled={leaseDisabled[action]}
                    onClick={() => onLease(action)}
                  >
                    {isArmed ? text.confirm : text.label}
                  </Button>
                </span>
              </Tooltip>
            )
          })}
        </Stack>
        {JOG_GROUPS.map((group) => (
          <Stack key={group.title} direction="row" spacing={1} alignItems="center" useFlexGap flexWrap="wrap">
            <Typography variant="caption" color="text.secondary" sx={{ minWidth: 64 }}>
              {group.title}
            </Typography>
            <ToolGroup label={group.title}>
              {group.buttons.map((b) => (
                <ToolButton key={b.label} label={b.label} icon={b.icon} disabled={!held || busy || released} onClick={() => void run('jog', b.jog)} />
              ))}
            </ToolGroup>
          </Stack>
        ))}
        {!held && <Typography variant="caption" color="text.secondary">Take control to jog the arm.</Typography>}
        {result && (
          <Alert severity={result.ok ? 'success' : 'error'} role={result.ok ? 'status' : 'alert'} onClose={() => setResult(null)}>
            {result.message}
          </Alert>
        )}
      </Stack>
    </OverlayPanel>
  )
}
