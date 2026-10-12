import { useEffect, useState } from 'react'
import { Alert, Box, Button, Chip, Stack, Typography } from '@mui/material'
import { OverlayPanel } from '../layout/OverlayPanel'
import { useLayout } from '../layout/useLayout'
import { confirmClick, RESET_CONFIRM_MS } from '../map/mapActions'
import { TOUCH_TARGET_PX } from '../theme'
import { TabConfig } from '../types'
import { ArmAction, ArmResult, JogSpec, postArm } from './armApi'

/** filter_node reports `autonomy` while the mcp_server lease taken by Take control is held. */
export const HELD_SOURCE = 'autonomy'
const DEFAULT_SOURCE_TOPIC = '/filter/active_source'
const TOUCH_SX = { minWidth: TOUCH_TARGET_PX, minHeight: TOUCH_TARGET_PX } as const

interface JogButton {
  label: string
  jog: JogSpec
}

const JOG_GROUPS: { title: string; buttons: JogButton[] }[] = [
  {
    title: 'Tool',
    buttons: [
      { label: 'Up', jog: { kind: 'cartesian', direction: 'up' } },
      { label: 'Down', jog: { kind: 'cartesian', direction: 'down' } },
      { label: 'Forward', jog: { kind: 'cartesian', direction: 'forward' } },
      { label: 'Back', jog: { kind: 'cartesian', direction: 'back' } },
      { label: 'Left', jog: { kind: 'cartesian', direction: 'left' } },
      { label: 'Right', jog: { kind: 'cartesian', direction: 'right' } },
    ],
  },
  {
    title: 'Joints',
    buttons: [
      { label: 'Pan left', jog: { kind: 'shoulder_pan', direction: 'left' } },
      { label: 'Pan right', jog: { kind: 'shoulder_pan', direction: 'right' } },
      { label: 'Wrist up', jog: { kind: 'wrist_pitch', direction: 'up' } },
      { label: 'Wrist down', jog: { kind: 'wrist_pitch', direction: 'down' } },
      { label: 'Roll left', jog: { kind: 'wrist_roll', direction: 'left' } },
      { label: 'Roll right', jog: { kind: 'wrist_roll', direction: 'right' } },
    ],
  },
  {
    title: 'Gripper',
    buttons: [
      { label: 'Gripper open', jog: { kind: 'gripper', direction: 'open' } },
      { label: 'Gripper close', jog: { kind: 'gripper', direction: 'close' } },
    ],
  },
]

interface Props {
  tab: TabConfig
  topicData: Record<string, unknown>
}

/**
 * Jog panel of the Arm tab: active command source, Take control / Release / Home and small-step jog buttons.
 * Jog buttons and Release are enabled only while the web UI holds the arm lease.
 *
 * @param tab - arm tab config (id, active_source_topic)
 * @param topicData - latest payload per topic
 */
export default function ArmJogPanel({ tab, topicData }: Props) {
  const layout = useLayout()
  const [result, setResult] = useState<ArmResult | null>(null)
  const [homeArmedAt, setHomeArmedAt] = useState<number | null>(null)
  const [busy, setBusy] = useState(false)

  const source = (topicData[tab.active_source_topic ?? DEFAULT_SOURCE_TOPIC] as { data?: string } | null | undefined)?.data
  const held = source === HELD_SOURCE

  useEffect(() => {
    if (homeArmedAt === null) return
    const timer = setTimeout(() => setHomeArmedAt(null), RESET_CONFIRM_MS)
    return () => clearTimeout(timer)
  }, [homeArmedAt])

  const run = async (action: ArmAction, jog?: JogSpec) => {
    setBusy(true)
    setResult(await postArm(action, tab.id, jog))
    setBusy(false)
  }

  const onHome = () => {
    const next = confirmClick(homeArmedAt, Date.now())
    setHomeArmedAt(next.armedAt)
    if (next.fire) void run('home')
  }

  const homeArmed = homeArmedAt !== null

  return (
    <OverlayPanel title="Arm control" open anchor="bottom" docked={layout !== 'phone'}>
      <Stack spacing={1} sx={{ p: 1, overflowY: 'auto' }}>
        <Stack direction="row" spacing={1} alignItems="center">
          <Typography variant="body2">Active source</Typography>
          <Chip data-testid="arm-active-source" size="small" color={held ? 'success' : 'default'} label={source ?? 'unknown'} />
        </Stack>
        <Stack direction="row" spacing={1} useFlexGap flexWrap="wrap">
          <Button variant="contained" disabled={busy} onClick={() => void run('take_control')}>
            Take control
          </Button>
          <Button variant="outlined" disabled={!held || busy} onClick={() => void run('release')}>
            Release
          </Button>
          <Button variant={homeArmed ? 'contained' : 'outlined'} color={homeArmed ? 'warning' : 'primary'} disabled={busy} onClick={onHome} title="Move the arm to its saved home pose. Click twice to confirm.">
            {homeArmed ? 'Confirm home' : 'Home'}
          </Button>
        </Stack>
        {JOG_GROUPS.map((group) => (
          <Box key={group.title} role="group" aria-label={group.title}>
            <Typography variant="caption" color="text.secondary">
              {group.title}
            </Typography>
            <Box sx={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(96px, 1fr))', gap: 0.5 }}>
              {group.buttons.map((b) => (
                <Button key={b.label} variant="outlined" size="small" sx={TOUCH_SX} disabled={!held || busy} onClick={() => void run('jog', b.jog)}>
                  {b.label}
                </Button>
              ))}
            </Box>
          </Box>
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
