import { useCallback, useState } from 'react'
import Alert from '@mui/material/Alert'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import Chip from '@mui/material/Chip'
import Typography from '@mui/material/Typography'
import { ConfirmDialog } from '../layout/ConfirmDialog'
import { TOUCH_TARGET_PX } from '../theme'
import { ServoAction, ServoGroup, servoAction, useServoGroups } from './servoApi'

const STEPS_PER_TURN = 4096
const DEGREES_PER_TURN = 360
const SAVE_TEXT = 'Writes the new neutral into the servo EEPROM.'
const ARM_SAVE_TEXT =
  'Arm joint offsets and the stored home become stale until re-calibrated. Teleop needs the leader arm re-zeroed in the same pose.'
const RELEASE_TEXT: Record<string, string> = {
  arm: 'Support the arm: it goes limp and will drop.',
  head: 'Support the head: it goes limp.',
  base: 'The steering servos go limp; the wheels stay braked.',
}

type TorqueLevel = 'on' | 'off' | 'partly' | 'unknown'
type Feedback = { ok: boolean; message: string } | null
type Pending = { group: ServoGroup; action: 'release' | 'save' } | null

/** Torque level of a group: unknown when there is no fresh state or no joint reading. */
export function torqueLevel(group: ServoGroup): TorqueLevel {
  const values = group.state && !group.stale ? Object.values(group.state.joints) : []
  if (values.length === 0) return 'unknown'
  if (values.every((v) => v === 1)) return 'on'
  return values.every((v) => v === 0) ? 'off' : 'partly'
}

/** Signed degrees text of a neutral shift in steps, e.g. "+8.8". */
export function shiftDegrees(deltaSteps: number): string {
  const deg = (deltaSteps * DEGREES_PER_TURN) / STEPS_PER_TURN
  return `${deg >= 0 ? '+' : ''}${deg.toFixed(1)}`
}

const CHIP: Record<TorqueLevel, { label: string; color: 'success' | 'error' | 'warning' | 'default' }> = {
  on: { label: 'Torque on', color: 'success' },
  off: { label: 'Torque off', color: 'error' },
  partly: { label: 'Partly off', color: 'warning' },
  unknown: { label: 'Unknown', color: 'default' },
}

interface CardProps {
  group: ServoGroup
  busy: boolean
  onRun: (group: ServoGroup, action: ServoAction) => void
  onConfirm: (group: ServoGroup, action: 'release' | 'save') => void
}

function ServoCard({ group, busy, onRun, onConfirm }: CardProps) {
  const level = torqueLevel(group)
  const chip = CHIP[level]
  const known = level !== 'unknown'
  const released = known && level !== 'on'
  const capture = group.state?.neutral_capture ?? null
  const hasCapture = capture !== null && Object.keys(capture).length > 0
  const btn = { minHeight: TOUCH_TARGET_PX }
  const stale = group.id === 'arm' && group.calibration_stale && group.calibration_stale.length > 0

  return (
    <Box component="section" aria-label={group.label} sx={{ border: 1, borderColor: 'divider', borderRadius: 1, p: 2, mb: 1 }}>
      <Box sx={{ display: 'flex', alignItems: 'center', gap: 1, mb: 1 }}>
        <Typography variant="subtitle1" component="h3">
          {group.label}
        </Typography>
        <Chip data-testid="torque-chip" size="small" label={chip.label} color={chip.color} />
      </Box>
      {known && group.state && (
        <Box sx={{ display: 'flex', gap: 2, flexWrap: 'wrap', mb: 1 }}>
          {Object.entries(group.state.joints).map(([name, v]) => (
            <Typography key={name} variant="body2" color="text.secondary">{`${name}: ${v === 1 ? 'on' : 'off'}`}</Typography>
          ))}
        </Box>
      )}
      {stale && (
        <Alert severity="warning" sx={{ mb: 1 }}>
          {`Calibration is stale: ${group.calibration_stale?.join(', ')}. Re-calibrate the arm.`}
        </Alert>
      )}
      {hasCapture && capture && (
        <Box sx={{ mb: 1 }}>
          <Typography variant="body2">Recorded neutral shift (degrees)</Typography>
          {Object.entries(capture).map(([name, c]) => (
            <Typography key={name} variant="body2" color="text.secondary">{`${name}: ${shiftDegrees(c.delta_steps)}`}</Typography>
          ))}
        </Box>
      )}
      <Box sx={{ display: 'flex', gap: 1, flexWrap: 'wrap' }}>
        <Button variant="outlined" color="error" sx={btn} disabled={busy} onClick={() => onConfirm(group, 'release')}>
          Torque off
        </Button>
        {group.neutral && (
          <Button variant="outlined" sx={btn} disabled={busy || !released} onClick={() => onRun(group, 'record')}>
            Record neutral
          </Button>
        )}
        {group.neutral && (
          <Button variant="contained" sx={btn} disabled={busy || !hasCapture} onClick={() => onConfirm(group, 'save')}>
            Re-enable torque + save
          </Button>
        )}
        <Button variant="outlined" sx={btn} disabled={busy || !released} onClick={() => onRun(group, 'enable')}>
          Re-enable torque
        </Button>
      </Box>
    </Box>
  )
}

/** Servo torque and neutral section of the Calibrate page: one card per servo group. */
export function ServoNeutral() {
  const { groups, notConfigured, error } = useServoGroups()
  const [busy, setBusy] = useState(false)
  const [feedback, setFeedback] = useState<Feedback>(null)
  const [pending, setPending] = useState<Pending>(null)

  const run = useCallback(async (group: ServoGroup, action: ServoAction) => {
    setPending(null)
    setBusy(true)
    const r = await servoAction(group.id, action)
    setFeedback(r.ok ? { ok: r.value.ok, message: r.value.message } : { ok: false, message: r.message })
    setBusy(false)
  }, [])

  const saveBody = pending?.group.id === 'arm' ? `${SAVE_TEXT} ${ARM_SAVE_TEXT}` : SAVE_TEXT

  return (
    <Box component="section" sx={{ mb: 2 }}>
      <Typography variant="h6" component="h2">
        Servo torque and neutral
      </Typography>
      {notConfigured && <Alert severity="info">No servo groups configured on this robot</Alert>}
      {error && <Alert severity="warning">{error}</Alert>}
      {feedback && <Alert severity={feedback.ok ? 'success' : 'error'} onClose={() => setFeedback(null)}>{feedback.message}</Alert>}
      <Box sx={{ mt: 1 }}>
        {groups.map((g) => (
          <ServoCard key={g.id} group={g} busy={busy} onRun={(grp, a) => void run(grp, a)} onConfirm={(grp, a) => setPending({ group: grp, action: a })} />
        ))}
      </Box>
      <ConfirmDialog
        open={pending !== null}
        title={pending?.action === 'save' ? `Re-enable torque + save: ${pending.group.label}?` : `Torque off: ${pending?.group.label ?? ''}?`}
        body={pending?.action === 'save' ? saveBody : pending ? RELEASE_TEXT[pending.group.id] : undefined}
        confirmLabel={pending?.action === 'save' ? 'Re-enable torque + save' : 'Torque off'}
        onConfirm={() => pending && void run(pending.group, pending.action)}
        onCancel={() => setPending(null)}
      />
    </Box>
  )
}
