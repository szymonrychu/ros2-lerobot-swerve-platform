import { useCallback, useState } from 'react'
import Alert from '@mui/material/Alert'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import Checkbox from '@mui/material/Checkbox'
import Chip from '@mui/material/Chip'
import Dialog from '@mui/material/Dialog'
import DialogActions from '@mui/material/DialogActions'
import DialogContent from '@mui/material/DialogContent'
import DialogContentText from '@mui/material/DialogContentText'
import DialogTitle from '@mui/material/DialogTitle'
import FormControlLabel from '@mui/material/FormControlLabel'
import Typography from '@mui/material/Typography'
import { ConfirmDialog } from '../layout/ConfirmDialog'
import { TOUCH_TARGET_PX } from '../theme'
import { NeutralCaptureJoint, ServoAction, ServoGroup, ServoGroupsState, servoAction, useServoGroups } from './servoApi'
import { ZeroRestorePanel, staleJointOffsets } from './ZeroRestorePanel'

const STEPS_PER_TURN = 4096
const DEGREES_PER_TURN = 360
const SAVE_TEXT = 'Writes the new neutral into the servo EEPROM.'
const ARM_SAVE_TEXT =
  'Arm joint offsets and the stored home become stale until re-calibrated. Teleop needs the leader arm re-zeroed in the same pose.'
const LARGE_SHIFT_TEXT =
  'A large shift usually means the joints were not in the neutral pose when it was recorded (for example the folded arm). Saving it moves every later pose and Park by that angle. Record again in the neutral pose unless the shift is intended.'
const LARGE_SHIFT_CONFIRM = 'The large shift is intended: save it'
export const MOVE_TO_NEUTRAL_TEXT = 'The arm moves to the neutral pose: all joints at 0. Keep the area clear.'
export const SET_HOME_TEXT =
  "Stores the arm's current pose as its home pose (as the Arm tab Set home). Nothing moves. Pose the arm first: by hand with torque off, or with Move to neutral pose. Home arm then returns to this pose."
export const SET_HOME_STALE_REASON = 'Set home is off while the arm zero is stale (joint_offsets): restore the zero first.'
/** Stale marker items the Servo home flow shows (the bridge re-zero marker, cleared by the zero restore and Set home). */
export const MARKER_ITEMS = ['joint_offsets', 'home']
export const MOVE_RELEASED_REASON = 'Move to neutral needs torque on.'
export const MOVE_STALE_REASON =
  'Move to neutral is off while the arm zero is stale (joint_offsets): restore the zero first.'
const RELEASE_TEXT: Record<string, string> = {
  arm: 'Support the arm: it goes limp and will drop.',
  head: 'Support the head: it goes limp.',
  base: 'The steering servos go limp; the wheels stay braked.',
}

type TorqueLevel = 'on' | 'off' | 'partly' | 'unknown'
type Feedback = { ok: boolean; message: string } | null
type ConfirmAction = 'release' | 'save' | 'move_to_neutral' | 'set_home'
type Pending = { group: ServoGroup; action: ConfirmAction } | null
/**
 * Parts of a group card: torque and neutral controls, the Arm zero restore panel, the Move to neutral pose button, the
 * Set home button with the stale marker state.
 */
export type ServoPart = 'torque' | 'zero' | 'move' | 'home'
export const ALL_SERVO_PARTS: ServoPart[] = ['torque', 'zero', 'move', 'home']

/**
 * Per item of MARKER_ITEMS whether the stale marker lists it (null calibration_stale: no marker, nothing stale).
 *
 * @param group - the servo group
 * @returns item -> stale
 */
export function markerState(group: ServoGroup): Record<string, boolean> {
  const stale = group.calibration_stale ?? []
  return Object.fromEntries(MARKER_ITEMS.map((item) => [item, stale.includes(item)]))
}

/** Torque level of a group: unknown when there is no fresh state or no joint reading. */
export function torqueLevel(group: ServoGroup): TorqueLevel {
  const values = group.state && !group.stale ? Object.values(group.state.joints) : []
  if (values.length === 0) return 'unknown'
  if (values.every((v) => v === 1)) return 'on'
  return values.every((v) => v === 0) ? 'off' : 'partly'
}

/** Names of the recorded joints whose shift exceeds the group's neutral_max_shift_deg. */
export function largeShiftJoints(capture: Record<string, NeutralCaptureJoint> | null): string[] {
  return Object.entries(capture ?? {})
    .filter(([, c]) => c.exceeds_max_shift)
    .map(([name]) => name)
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
  parts: ServoPart[]
  busy: boolean
  onRun: (group: ServoGroup, action: ServoAction, planRunId?: string) => void
  onConfirm: (group: ServoGroup, action: ConfirmAction) => void
}

function ServoCard({ group, parts, busy, onRun, onConfirm }: CardProps) {
  const level = torqueLevel(group)
  const chip = CHIP[level]
  const known = level !== 'unknown'
  const released = known && level !== 'on'
  const capture = group.state?.neutral_capture ?? null
  const hasCapture = capture !== null && Object.keys(capture).length > 0
  const btn = { minHeight: TOUCH_TARGET_PX }
  const stale = group.id === 'arm' && group.calibration_stale && group.calibration_stale.length > 0
  const moveReason = !known || released ? MOVE_RELEASED_REASON : staleJointOffsets(group) ? MOVE_STALE_REASON : null
  const torque = parts.includes('torque')
  const move = parts.includes('move') && group.move_to_neutral
  const home = parts.includes('home') && group.set_home
  const homeReason = staleJointOffsets(group) ? SET_HOME_STALE_REASON : null

  return (
    <Box component="section" aria-label={group.label} sx={{ border: 1, borderColor: 'divider', borderRadius: 1, p: 2, mb: 1 }}>
      <Box sx={{ display: 'flex', alignItems: 'center', gap: 1, mb: 1 }}>
        <Typography variant="subtitle1" component="h3">
          {group.label}
        </Typography>
        <Chip data-testid="torque-chip" size="small" label={chip.label} color={chip.color} />
      </Box>
      {torque && known && group.state && (
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
      {torque && hasCapture && capture && (
        <Box sx={{ mb: 1 }}>
          <Typography variant="body2">Recorded neutral shift (degrees)</Typography>
          {Object.entries(capture).map(([name, c]) => (
            <Typography key={name} variant="body2" color={c.exceeds_max_shift ? 'error' : 'text.secondary'}>
              {`${name}: ${shiftDegrees(c.delta_steps)}${c.exceeds_max_shift ? ` (above ${group.state?.neutral_max_shift_deg} deg)` : ''}`}
            </Typography>
          ))}
        </Box>
      )}
      <Box sx={{ display: 'flex', gap: 1, flexWrap: 'wrap' }}>
        {torque && (
          <Button variant="outlined" color="error" sx={btn} disabled={busy} onClick={() => onConfirm(group, 'release')}>
            Torque off
          </Button>
        )}
        {torque && group.neutral && (
          <Button variant="outlined" sx={btn} disabled={busy || !released} onClick={() => onRun(group, 'record')}>
            Record neutral
          </Button>
        )}
        {torque && group.neutral && (
          <Button variant="contained" sx={btn} disabled={busy || !hasCapture} onClick={() => onConfirm(group, 'save')}>
            Re-enable torque + save
          </Button>
        )}
        {torque && (
          <Button variant="outlined" sx={btn} disabled={busy || !released} onClick={() => onRun(group, 'enable')}>
            Re-enable torque
          </Button>
        )}
        {move && (
          <Button variant="outlined" sx={btn} disabled={busy || moveReason !== null} onClick={() => onConfirm(group, 'move_to_neutral')}>
            Move to neutral pose
          </Button>
        )}
      </Box>
      {move && moveReason && <Typography variant="body2" color="text.secondary">{moveReason}</Typography>}
      {home && (
        <Box component="section" aria-label="Stale marker" sx={{ display: 'flex', gap: 1, flexWrap: 'wrap', mb: 1 }}>
          {Object.entries(markerState(group)).map(([item, isStale]) => (
            <Chip key={item} size="small" color={isStale ? 'warning' : 'success'} label={`${item}: ${isStale ? 'stale' : 'ok'}`} />
          ))}
        </Box>
      )}
      {home && (
        <Button variant="contained" sx={btn} disabled={busy || homeReason !== null} onClick={() => onConfirm(group, 'set_home')}>
          Set home
        </Button>
      )}
      {home && homeReason && <Typography variant="body2" color="text.secondary">{homeReason}</Typography>}
      {parts.includes('zero') && group.zero_restore && (
        <ZeroRestorePanel
          group={group}
          released={released}
          known={known}
          busy={busy}
          onRun={onRun}
          onRelease={(grp) => onConfirm(grp, 'release')}
        />
      )}
    </Box>
  )
}

/**
 * Confirm of Re-enable torque + save: lists every recorded shift; a shift above the group's neutral_max_shift_deg
 * needs the explicit checkbox and then saves through save_confirmed (the bridge refuses a plain save of it).
 */
function SaveNeutralDialog(props: { group: ServoGroup | null; onSave: (group: ServoGroup, action: ServoAction) => void; onCancel: () => void }) {
  const { group } = props
  const [confirmed, setConfirmed] = useState(false)
  const capture = group?.state?.neutral_capture ?? null
  const large = largeShiftJoints(capture)
  const limit = group?.state?.neutral_max_shift_deg
  const cancel = () => {
    setConfirmed(false)
    props.onCancel()
  }
  const save = () => {
    if (!group || (large.length > 0 && !confirmed)) return
    setConfirmed(false)
    props.onSave(group, large.length > 0 ? 'save_confirmed' : 'save')
  }
  return (
    <Dialog open={group !== null} onClose={cancel} aria-labelledby="save-neutral-title">
      <DialogTitle id="save-neutral-title">{`Re-enable torque + save: ${group?.label ?? ''}?`}</DialogTitle>
      <DialogContent>
        <DialogContentText>{group?.id === 'arm' ? `${SAVE_TEXT} ${ARM_SAVE_TEXT}` : SAVE_TEXT}</DialogContentText>
        <Box sx={{ mt: 1 }}>
          {Object.entries(capture ?? {}).map(([name, c]) => (
            <Typography key={name} variant="body2" color={c.exceeds_max_shift ? 'error' : 'text.primary'}>
              {`${name}: ${shiftDegrees(c.delta_steps)} deg${c.exceeds_max_shift ? ` (above ${limit} deg)` : ''}`}
            </Typography>
          ))}
        </Box>
        {large.length > 0 && (
          <>
            <Alert severity="warning" sx={{ mt: 1 }}>
              {`${large.join(', ')} shift more than ${limit} deg. ${LARGE_SHIFT_TEXT}`}
            </Alert>
            <FormControlLabel
              control={<Checkbox checked={confirmed} onChange={(e) => setConfirmed(e.target.checked)} />}
              label={LARGE_SHIFT_CONFIRM}
            />
          </>
        )}
      </DialogContent>
      <DialogActions>
        <Button onClick={cancel}>Cancel</Button>
        <Button color="error" variant="outlined" disabled={large.length > 0 && !confirmed} onClick={save}>
          {large.length > 0 ? 'Save large shift' : 'Re-enable torque + save'}
        </Button>
      </DialogActions>
    </Dialog>
  )
}

interface ServoNeutralProps {
  /** Card parts to show; a group without the part's capability is left out when torque is not among them. */
  parts?: ServoPart[]
  /** Servo state polled by the caller; without it the section polls itself. */
  servo?: ServoGroupsState
  /** Show the section heading (off inside a flow step, which has its own title). */
  heading?: boolean
  /** Told the outcome of every action: group id, action, success. */
  onResult?: (groupId: string, action: ServoAction, ok: boolean) => void
}

/** Whether a group has anything to show for the parts. */
function groupShown(group: ServoGroup, parts: ServoPart[]): boolean {
  return (
    parts.includes('torque') ||
    (parts.includes('zero') && group.zero_restore) ||
    (parts.includes('move') && group.move_to_neutral) ||
    (parts.includes('home') && group.set_home)
  )
}

/**
 * Servo torque and neutral section of the Calibrate page: one card per servo group, limited to `parts`.
 *
 * @param props - see ServoNeutralProps
 */
export function ServoNeutral(props: ServoNeutralProps) {
  return props.servo ? <ServoNeutralView {...props} servo={props.servo} /> : <PolledServoNeutral {...props} />
}

function PolledServoNeutral(props: ServoNeutralProps) {
  return <ServoNeutralView {...props} servo={useServoGroups()} />
}

function ServoNeutralView({ parts = ALL_SERVO_PARTS, servo, heading = true, onResult }: ServoNeutralProps & { servo: ServoGroupsState }) {
  const { groups, notConfigured, error } = servo
  const [busy, setBusy] = useState(false)
  const [feedback, setFeedback] = useState<Feedback>(null)
  const [pending, setPending] = useState<Pending>(null)

  const run = useCallback(
    async (group: ServoGroup, action: ServoAction, planRunId?: string) => {
      setPending(null)
      setBusy(true)
      const r = await servoAction(group.id, action, planRunId)
      setFeedback(r.ok ? { ok: r.value.ok, message: r.value.message } : { ok: false, message: r.message })
      setBusy(false)
      onResult?.(group.id, action, r.ok && r.value.ok)
    },
    [onResult],
  )

  return (
    <Box component="section" sx={{ mb: 2 }}>
      {heading && (
        <Typography variant="h6" component="h2">
          Servo torque and neutral
        </Typography>
      )}
      {notConfigured && <Alert severity="info">No servo groups configured on this robot</Alert>}
      {error && <Alert severity="warning">{error}</Alert>}
      {feedback && <Alert severity={feedback.ok ? 'success' : 'error'} onClose={() => setFeedback(null)}>{feedback.message}</Alert>}
      <Box sx={{ mt: 1 }}>
        {groups
          .filter((g) => groupShown(g, parts))
          .map((g) => (
            <ServoCard
              key={g.id}
              group={g}
              parts={parts}
              busy={busy}
              onRun={(grp, a, id) => void run(grp, a, id)}
              onConfirm={(grp, a) => setPending({ group: grp, action: a })}
            />
          ))}
      </Box>
      <ConfirmDialog
        open={pending?.action === 'release'}
        title={`Torque off: ${pending?.group.label ?? ''}?`}
        body={pending?.action === 'release' ? RELEASE_TEXT[pending.group.id] : undefined}
        confirmLabel="Torque off"
        onConfirm={() => pending && void run(pending.group, 'release')}
        onCancel={() => setPending(null)}
      />
      <ConfirmDialog
        open={pending?.action === 'move_to_neutral'}
        title={`Move to neutral pose: ${pending?.group.label ?? ''}?`}
        body={MOVE_TO_NEUTRAL_TEXT}
        confirmLabel="Move"
        onConfirm={() => pending && void run(pending.group, 'move_to_neutral')}
        onCancel={() => setPending(null)}
      />
      <ConfirmDialog
        open={pending?.action === 'set_home'}
        title={`Set home: ${pending?.group.label ?? ''}?`}
        body={SET_HOME_TEXT}
        confirmLabel="Set home"
        onConfirm={() => pending && void run(pending.group, 'set_home')}
        onCancel={() => setPending(null)}
      />
      <SaveNeutralDialog
        group={pending?.action === 'save' ? groups.find((g) => g.id === pending.group.id) ?? pending.group : null}
        onSave={(grp, a) => void run(grp, a)}
        onCancel={() => setPending(null)}
      />
    </Box>
  )
}
