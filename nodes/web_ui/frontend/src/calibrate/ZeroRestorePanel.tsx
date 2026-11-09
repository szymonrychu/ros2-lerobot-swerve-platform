/**
 * Guided "Arm zero restore" panel of the arm servo card: torque off, dry run, apply (with the large-change confirm),
 * result, re-enable torque, then Park. The bridge (feetech follower) does the work; this panel only shows its
 * torque_state `zero_restore` and posts one action per step.
 */
import { useState } from 'react'
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
import Table from '@mui/material/Table'
import TableBody from '@mui/material/TableBody'
import TableCell from '@mui/material/TableCell'
import TableHead from '@mui/material/TableHead'
import TableRow from '@mui/material/TableRow'
import Typography from '@mui/material/Typography'
import { ConfirmDialog } from '../layout/ConfirmDialog'
import { TOUCH_TARGET_PX } from '../theme'
import type { ServoAction, ServoGroup, ZeroRestorePlan, ZeroRestoreResult, ZeroRestoreState } from './servoApi'

export const ZERO_RESTORE_INTRO =
  'Puts the arm zero back on the saved folded-rest reference by rewriting each servo offset. The arm does not move.'
export const STEP_EXPECT = {
  release: 'The arm goes limp; rest it folded on its rest, jaws empty.',
  dryRun: 'Reads every joint (keep the arm still for about a second) and shows the planned offset changes. Nothing is written.',
  apply: 'Writes the new offsets: the arm does not move and torque stays off. Each joint is verified and rolled back on a mismatch.',
  enable: 'The arm holds its pose.',
}
export const PARK_HINT = '5. Then Park the arm on the head-arm card.'
export const LARGE_CHANGE_CONFIRM = 'The arm rests folded on its rest and the large change is intended'
const APPLY_TEXT = 'Writes the planned offsets into the servo EEPROM. Torque stays off; re-enable it afterwards.'
const LARGE_CHANGE_TEXT =
  'A large change is expected only when the zero was recorded in the wrong pose. With the arm anywhere but folded on its rest it writes a wrong zero.'
const REST_REFERENCE_TEXT =
  'Stores the present joint readings as the folded-rest reference for later zero restores. Do it only with the arm resting folded on its rest and a known good zero.'
export const STALE_REFERENCE_REASON =
  'The arm zero is stale (joint_offsets): restore the zero before saving a new reference.'
export const RELEASE_FIRST_REASON = 'Torque off the arm first.'

type ZeroDialog = 'apply' | 'rest_reference' | null

/** Signed degrees with one decimal, e.g. "+79.2". */
export function signedDeg(deg: number): string {
  return `${deg >= 0 ? '+' : ''}${deg.toFixed(1)}`
}

const rad = (value: number | null) => (value === null ? '-' : value.toFixed(3))

/** Whether the arm's stale marker lists joint_offsets (the zero itself is suspect). */
export function staleJointOffsets(group: ServoGroup): boolean {
  return (group.calibration_stale ?? []).includes('joint_offsets')
}

function PlanTable({ plan }: { plan: ZeroRestorePlan }) {
  return (
    <Table size="small" aria-label="Zero restore plan">
      <TableHead>
        <TableRow>
          <TableCell>Joint</TableCell>
          <TableCell align="right">Now (rad)</TableCell>
          <TableCell align="right">Target (rad)</TableCell>
          <TableCell align="right">Change (deg)</TableCell>
          <TableCell>Guard</TableCell>
        </TableRow>
      </TableHead>
      <TableBody>
        {Object.entries(plan.joints).map(([name, j]) => (
          <TableRow key={name} aria-label={name}>
            <TableCell>{name}</TableCell>
            <TableCell align="right">{rad(j.present_rad)}</TableCell>
            <TableCell align="right">{rad(j.target_rad)}</TableCell>
            <TableCell align="right">{signedDeg(j.change_deg)}</TableCell>
            <TableCell>
              {j.exceeds_max_change ? <Chip size="small" color="warning" label="large" /> : <Chip size="small" label="ok" />}
            </TableCell>
          </TableRow>
        ))}
      </TableBody>
    </Table>
  )
}

function ResultTable({ result }: { result: ZeroRestoreResult }) {
  return (
    <Box sx={{ mt: 1 }}>
      <Typography variant="body2" color={result.success ? 'text.secondary' : 'error'}>
        {`Last apply ${result.finished_at}: ${result.message}`}
      </Typography>
      <Table size="small" aria-label="Zero restore result">
        <TableHead>
          <TableRow>
            <TableCell>Joint</TableCell>
            <TableCell>Applied</TableCell>
            <TableCell align="right">Now (rad)</TableCell>
            <TableCell align="right">Offset after</TableCell>
          </TableRow>
        </TableHead>
        <TableBody>
          {Object.entries(result.joints).map(([name, j]) => (
            <TableRow key={name} aria-label={name}>
              <TableCell>{name}</TableCell>
              <TableCell>{j.applied ? 'yes' : 'no'}</TableCell>
              <TableCell align="right">{rad(j.present_after_rad)}</TableCell>
              <TableCell align="right">{j.offset_after_steps === null ? '-' : String(j.offset_after_steps)}</TableCell>
            </TableRow>
          ))}
        </TableBody>
      </Table>
      {result.not_attempted.length > 0 && (
        <Typography variant="body2" color="text.secondary">{`Not attempted: ${result.not_attempted.join(', ')}`}</Typography>
      )}
    </Box>
  )
}

/** Apply confirm of a plan with a change above max_change_deg: the checkbox enables the confirmed apply. */
function LargeChangeDialog(props: { zero: ZeroRestoreState; open: boolean; onApply: () => void; onCancel: () => void }) {
  const [confirmed, setConfirmed] = useState(false)
  const large = Object.entries(props.zero.plan?.joints ?? {}).filter(([, j]) => j.exceeds_max_change)
  const cancel = () => {
    setConfirmed(false)
    props.onCancel()
  }
  const apply = () => {
    if (!confirmed) return
    setConfirmed(false)
    props.onApply()
  }
  return (
    <Dialog open={props.open} onClose={cancel} aria-labelledby="zero-restore-large-title">
      <DialogTitle id="zero-restore-large-title">Apply a large zero change?</DialogTitle>
      <DialogContent>
        <DialogContentText>{APPLY_TEXT}</DialogContentText>
        <Alert severity="warning" sx={{ mt: 1 }}>
          {`${large.map(([name, j]) => `${name} ${signedDeg(j.change_deg)} deg`).join(', ')} change more than ${props.zero.max_change_deg} deg. ${LARGE_CHANGE_TEXT}`}
        </Alert>
        <FormControlLabel
          control={<Checkbox checked={confirmed} onChange={(e) => setConfirmed(e.target.checked)} />}
          label={LARGE_CHANGE_CONFIRM}
        />
      </DialogContent>
      <DialogActions>
        <Button onClick={cancel}>Cancel</Button>
        <Button color="error" variant="outlined" disabled={!confirmed} onClick={apply}>
          Apply large change
        </Button>
      </DialogActions>
    </Dialog>
  )
}

interface PanelProps {
  group: ServoGroup
  /** Fresh state with every joint reading torque 0 (or partly off). */
  released: boolean
  /** Torque state is known (fresh). */
  known: boolean
  busy: boolean
  onRun: (group: ServoGroup, action: ServoAction) => void
  /** Opens the card's Torque off confirm. */
  onRelease: (group: ServoGroup) => void
}

/** The Arm zero restore panel; renders nothing without the torque_state zero_restore key. */
export function ZeroRestorePanel({ group, released, known, busy, onRun, onRelease }: PanelProps) {
  const [dialog, setDialog] = useState<ZeroDialog>(null)
  const zero = group.state?.zero_restore ?? null
  const plan = zero?.plan ?? null
  const result = zero?.last_result ?? null
  const btn = { minHeight: TOUCH_TARGET_PX }
  const staleOffsets = staleJointOffsets(group)
  const referenceReason = staleOffsets ? STALE_REFERENCE_REASON : !released ? RELEASE_FIRST_REASON : null
  const reference = zero?.reference ?? null
  const step = (button: JSX.Element, expect: string) => (
    <Box sx={{ mb: 1 }}>
      {button}
      <Typography variant="body2" color="text.secondary">{expect}</Typography>
    </Box>
  )
  const close = () => setDialog(null)
  const run = (action: ServoAction) => {
    close()
    onRun(group, action)
  }

  return (
    <Box component="section" aria-label="Arm zero restore" sx={{ borderTop: 1, borderColor: 'divider', mt: 2, pt: 1 }}>
      <Typography variant="subtitle2" component="h4">Arm zero restore</Typography>
      <Typography variant="body2" color="text.secondary" sx={{ mb: 1 }}>{ZERO_RESTORE_INTRO}</Typography>
      {zero === null ? (
        <Typography variant="body2" color="text.secondary">Waiting for the bridge zero restore state.</Typography>
      ) : (
        <>
          <Typography variant="body2">
            {reference === null
              ? 'Reference: none'
              : reference.source === 'file'
                ? `Reference: saved ${reference.saved_at ?? 'at an unknown time'}`
                : 'Reference: seed from the bridge config'}
          </Typography>
          {zero.reference_error && <Alert severity="error" sx={{ mb: 1 }}>{zero.reference_error}</Alert>}
          {step(
            <Button variant="outlined" color="error" sx={btn} disabled={busy} onClick={() => onRelease(group)}>
              1. Torque off arm
            </Button>,
            STEP_EXPECT.release,
          )}
          {step(
            <Button variant="outlined" sx={btn} disabled={busy || !released} onClick={() => onRun(group, 'zero_restore_dry_run')}>
              2. Dry run
            </Button>,
            STEP_EXPECT.dryRun,
          )}
          {plan && (
            <Box sx={{ mb: 1 }}>
              <Typography variant="body2" color="text.secondary">
                {`Plan ${plan.created_at} (reference ${plan.reference_source}, valid ${plan.max_age_s} s)`}
              </Typography>
              <PlanTable plan={plan} />
            </Box>
          )}
          {step(
            <Button variant="contained" sx={btn} disabled={busy || !released || plan === null} onClick={() => setDialog('apply')}>
              3. Apply
            </Button>,
            STEP_EXPECT.apply,
          )}
          {result && <ResultTable result={result} />}
          {step(
            <Button variant="outlined" sx={btn} disabled={busy || !known || !released} onClick={() => onRun(group, 'enable')}>
              4. Re-enable torque
            </Button>,
            STEP_EXPECT.enable,
          )}
          <Typography variant="body2" sx={{ mb: 1 }}>{PARK_HINT}</Typography>
          <Button variant="outlined" sx={btn} disabled={busy || referenceReason !== null} onClick={() => setDialog('rest_reference')}>
            Save current pose as rest reference
          </Button>
          {referenceReason && <Typography variant="body2" color="text.secondary">{referenceReason}</Typography>}
          <ConfirmDialog
            open={dialog === 'apply' && plan !== null && !plan.needs_confirm}
            title="Apply the zero restore?"
            body={APPLY_TEXT}
            confirmLabel="Apply"
            onConfirm={() => run('zero_restore_apply')}
            onCancel={close}
          />
          <LargeChangeDialog
            zero={zero}
            open={dialog === 'apply' && plan !== null && plan.needs_confirm}
            onApply={() => run('zero_restore_apply_confirmed')}
            onCancel={close}
          />
          <ConfirmDialog
            open={dialog === 'rest_reference'}
            title="Save the current pose as the rest reference?"
            body={REST_REFERENCE_TEXT}
            confirmLabel="Save reference"
            onConfirm={() => run('save_rest_reference')}
            onCancel={close}
          />
        </>
      )}
    </Box>
  )
}
