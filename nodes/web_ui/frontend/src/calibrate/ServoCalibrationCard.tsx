import { useState } from 'react'
import Alert from '@mui/material/Alert'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import Table from '@mui/material/Table'
import TableBody from '@mui/material/TableBody'
import TableCell from '@mui/material/TableCell'
import TableHead from '@mui/material/TableHead'
import TableRow from '@mui/material/TableRow'
import Typography from '@mui/material/Typography'
import { ConfirmDialog } from '../layout/ConfirmDialog'
import { TOUCH_TARGET_PX } from '../theme'
import { FlowStep, FlowStepper } from './FlowStepper'
import { RangeBars } from './RangeBars'
import { torqueLevel } from './ServoNeutral'
import type { CalibrationPlan, ServoAction, ServoActionResult, ServoGroup } from './servoApi'
import { signedDeg } from './ZeroRestorePanel'

export const RETURN_HOME_TEXT = 'Return the joints to home and let go'
export const HOME_NOT_SET_TEXT = 'Arm home not set'
export const HOME_SET_TEXT = 'Arm home set'
export const CONFIRM_TITLE = 'Store this calibration?'
const BTN = { minHeight: TOUCH_TARGET_PX }

/**
 * Lines of the confirm dialog: every joint with a large change and every joint with narrower travel than required.
 *
 * @param plan - the plan on screen
 * @returns one line per problem joint and kind
 */
export function confirmLines(plan: CalibrationPlan): string[] {
  return Object.entries(plan.joints).flatMap(([name, j]) => [
    ...(j.exceeds_max_change ? [`${name}: large change ${signedDeg(j.change_deg)} deg`] : []),
    ...(j.narrower_than_required ? [`${name}: narrower travel than required`] : []),
  ])
}

function PlanTable({ plan }: { plan: CalibrationPlan }) {
  return (
    <Table size="small" aria-label="Calibration plan">
      <TableHead>
        <TableRow>
          <TableCell>Joint</TableCell>
          <TableCell>Change (deg)</TableCell>
          <TableCell>New offset</TableCell>
          <TableCell>Limits before to after</TableCell>
          <TableCell>Problems</TableCell>
        </TableRow>
      </TableHead>
      <TableBody>
        {Object.entries(plan.joints).map(([name, j]) => (
          <TableRow key={name}>
            <TableCell>{name}</TableCell>
            <TableCell>{signedDeg(j.change_deg)}</TableCell>
            <TableCell>{j.new_offset_steps}</TableCell>
            <TableCell>{`${j.limits_before[0]}..${j.limits_before[1]} -> ${j.limits_after ? `${j.limits_after[0]}..${j.limits_after[1]}` : '-'}`}</TableCell>
            <TableCell>{j.problems.length > 0 ? j.problems.join('; ') : '-'}</TableCell>
          </TableRow>
        ))}
      </TableBody>
    </Table>
  )
}

/**
 * Servo calibration card of one group: Torque off, Record home, Record servo ranges (live bars), Store.
 *
 * @param group - the servo group (state.servo_calibration carries home, capture, plan and last result)
 * @param onAction - runs a servo action (optionally bound to a plan run_id) and resolves with its result
 */
export function ServoCalibrationCard({
  group,
  onAction,
}: {
  group: ServoGroup
  onAction: (action: ServoAction, planRunId?: string) => Promise<ServoActionResult>
}) {
  const [busy, setBusy] = useState(false)
  const [outcome, setOutcome] = useState<ServoActionResult | null>(null)
  const [error, setError] = useState<string | null>(null)
  // run_id of the plan shown when Store was pressed: the store is bound to it.
  const [confirmRunId, setConfirmRunId] = useState<string | null>(null)
  const sc = group.state?.servo_calibration ?? null
  const level = torqueLevel(group)
  const released = level === 'off'
  const plan = sc?.plan ?? null
  const capture = sc?.capture ?? null
  const running = capture?.running === true
  const lastResult = sc?.last_result ?? null
  const homeDone = sc?.home != null

  /**
   * Runs one action, tracking busy and the error line; a store action also keeps its result for the result note.
   *
   * @param action - the servo action
   * @param planRunId - plan run_id the action is bound to (store actions)
   * @param kind - 'store' keeps the result message and home_set
   */
  const run = async (action: ServoAction, planRunId?: string, kind: 'plain' | 'store' = 'plain') => {
    setBusy(true)
    setError(null)
    try {
      const result = await onAction(action, planRunId)
      if (kind === 'store') setOutcome(result)
      else if (!result.ok) setError(result.message)
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e))
    } finally {
      setBusy(false)
    }
  }
  const store = () => {
    if (plan === null) return
    if (plan.needs_confirm) setConfirmRunId(plan.run_id)
    else void run('calibration_store', plan.run_id, 'store')
  }
  const confirmed = () => {
    const id = confirmRunId
    setConfirmRunId(null)
    if (id !== null) void run('calibration_store_confirmed', id, 'store')
  }
  // The plan changed under the open confirm: the user reviews the new plan first.
  const dialogOpen = confirmRunId !== null && plan !== null && plan.needs_confirm && plan.run_id === confirmRunId
  const storeReason = running
    ? 'Stop the range capture first.'
    : plan === null
      ? 'Plan first.'
      : !plan.ok
        ? 'The plan has problems.'
        : !released
          ? 'Torque off first.'
          : null

  const steps: FlowStep[] = [
    {
      id: 'release',
      title: 'Torque off',
      see: `${group.label} goes limp.`,
      done: released,
      children: (
        <Button variant="outlined" color="error" sx={BTN} disabled={busy || level === 'unknown'} onClick={() => void run('release')}>
          Torque off
        </Button>
      ),
    },
    {
      id: 'home',
      title: 'Record home',
      see: 'The home pose is recorded.',
      done: homeDone,
      children: (
        <>
          {group.home_pose_hint && <Typography variant="body2" sx={{ mb: 1 }}>{`Home pose: ${group.home_pose_hint}`}</Typography>}
          <Button variant="outlined" sx={BTN} disabled={busy || !released || running} onClick={() => void run('calibration_record_home')}>
            Record home
          </Button>
        </>
      ),
    },
    {
      id: 'ranges',
      title: 'Record servo ranges',
      see: 'Move every joint to both ends of its travel; the bars show min and max live.',
      done: capture !== null && !running && capture.samples > 0,
      optional: true,
      children: (
        <>
          <Box sx={{ display: 'flex', gap: 1, mb: 1 }}>
            <Button variant="outlined" sx={BTN} disabled={busy || !released || !homeDone || running} onClick={() => void run('calibration_capture_start')}>
              Record servo ranges
            </Button>
            <Button variant="contained" sx={BTN} disabled={busy || !running} onClick={() => void run('calibration_capture_stop')}>
              Stop
            </Button>
          </Box>
          {capture?.paused && <Alert severity="warning" sx={{ mb: 1 }}>Capture paused: torque is on.</Alert>}
          {capture !== null && <RangeBars joints={capture.joints} />}
        </>
      ),
    },
    {
      id: 'store',
      title: 'Store calibration',
      see: 'Home and ranges are written into the servos.',
      done: lastResult?.success === true,
      children: (
        <>
          <Typography variant="body2" sx={{ mb: 1 }}>{RETURN_HOME_TEXT}</Typography>
          <Box sx={{ display: 'flex', gap: 1, mb: 1 }}>
            <Button variant="outlined" sx={BTN} disabled={busy || !released || !homeDone || running} onClick={() => void run('calibration_plan')}>
              Plan
            </Button>
            <Button variant="contained" sx={BTN} disabled={busy || storeReason !== null} onClick={store}>
              Store
            </Button>
          </Box>
          {storeReason && <Typography variant="body2" color="text.secondary">{storeReason}</Typography>}
          {plan !== null && <PlanTable plan={plan} />}
          {outcome !== null && (
            <Alert severity={outcome.ok ? 'success' : 'error'} sx={{ mt: 1 }}>
              {outcome.message}
            </Alert>
          )}
          {outcome?.home_set === false && <Alert severity="warning" sx={{ mt: 1 }}>{HOME_NOT_SET_TEXT}</Alert>}
          {outcome?.home_set === true && <Typography variant="body2" sx={{ mt: 1 }}>{HOME_SET_TEXT}</Typography>}
        </>
      ),
    },
  ]

  return (
    <Box component="section" aria-label={`${group.label} servo calibration`} sx={{ mb: 2 }}>
      <Typography variant="h6" component="h2" sx={{ mb: 1 }}>{group.label}</Typography>
      {error && <Alert severity="error" sx={{ mb: 1 }}>{error}</Alert>}
      <FlowStepper label={`${group.label} servo calibration steps`} steps={steps} />
      {released && (
        <Button variant="outlined" sx={BTN} disabled={busy} onClick={() => void run('enable')}>
          Re-enable torque
        </Button>
      )}
      <ConfirmDialog
        open={dialogOpen}
        title={CONFIRM_TITLE}
        body={plan ? confirmLines(plan).join('. ') : undefined}
        confirmLabel="Store anyway"
        onConfirm={confirmed}
        onCancel={() => setConfirmRunId(null)}
      />
    </Box>
  )
}
