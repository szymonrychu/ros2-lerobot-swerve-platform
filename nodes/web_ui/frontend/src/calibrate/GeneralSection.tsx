import { useEffect, useState } from 'react'
import Alert from '@mui/material/Alert'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import Typography from '@mui/material/Typography'
import { ConfirmDialog } from '../layout/ConfirmDialog'
import { TOUCH_TARGET_PX } from '../theme'
import { GENERAL_ACTIONS, type GeneralAction, type GeneralActionId, abortCalibration, runGeneralAction } from './generalApi'
import { getHeadArmState } from './headArmApi'
import { getFloorState } from './headFloorApi'

/** Head-arm sequence and floor sweep poll period of the General section: 0.5 Hz (the flows poll faster while they run). */
export const GENERAL_POLL_MS = 2000
/** Actions that would fight a live calibration sequence (it holds the arm and aims the head) or drop its board. */
const SEQUENCE_BLOCKED: GeneralActionId[] = ['open_gripper', 'close_gripper', 'home_arm', 'home_head']
/** Actions that would fight a floor sweep (it moves the head). */
const SWEEP_BLOCKED: GeneralActionId[] = ['home_head']
/** Sequence phases in which the arm is the sequence's or holds the board (holding: park it from the Arm to camera flow). */
const LIVE_PHASES = ['waiting_board', 'waiting_big_joints', 'holding', 'parking']
const BLOCKED_TEXT =
  'Arm and camera moves are off: a calibration sequence runs or the arm holds the board. Abort the running calibration first, or park the board from the Arm to camera flow.'
const SWEEP_TEXT = 'Home camera is off while the floor sweep moves the head.'
const ABORT_TEXT =
  'Stops the running calibration capture, sweep or sequence. The arm holds where it is (no park); park it from the flow or use Home arm afterwards.'

type Feedback = { ok: boolean; message: string; details: string[] } | null
type Pending = GeneralAction | 'abort' | null

/**
 * General section of the Calibrate page, always on top: Abort calibration plus the moving helpers (open / close
 * gripper, home arm, home camera, home steering servos), each behind a confirm.
 *
 * @param actions - the actions the robot offers (GET /api/calibration/config `general`)
 * @param rgbd - the RGB-D calibration node is configured: Abort is offered and the section follows its sequence to
 *   block arm moves
 */
export function GeneralSection({ actions, rgbd }: { actions: GeneralActionId[]; rgbd: boolean }) {
  const [pending, setPending] = useState<Pending>(null)
  const [busy, setBusy] = useState(false)
  const [feedback, setFeedback] = useState<Feedback>(null)
  const [sequenceLive, setSequenceLive] = useState(false)
  const [sweepLive, setSweepLive] = useState(false)

  useEffect(() => {
    if (!rgbd) return
    let live = true
    const poll = async () => {
      const [s, f] = await Promise.all([getHeadArmState(), getFloorState()])
      if (!live) return
      if (s.ok) setSequenceLive(s.value.sequence.running || LIVE_PHASES.includes(s.value.sequence.phase))
      if (f.ok) setSweepLive(f.value.sweep.running)
    }
    void poll()
    const id = setInterval(() => void poll(), GENERAL_POLL_MS)
    return () => {
      live = false
      clearInterval(id)
    }
  }, [rgbd])

  const run = async (action: GeneralAction) => {
    setPending(null)
    setBusy(true)
    const r = await runGeneralAction(action.id)
    setBusy(false)
    const ok = r.ok && r.value.ok
    setFeedback({ ok, message: `${action.label}: ${r.ok ? r.value.message : r.message}`, details: [] })
  }

  const abort = async () => {
    setPending(null)
    setBusy(true)
    const r = await abortCalibration()
    setBusy(false)
    const message = r.stopped.length > 0 ? `Stopped: ${r.stopped.join(', ')}. The arm holds where it is.` : 'Nothing was running.'
    setFeedback({ ok: r.errors.length === 0, message, details: r.errors })
  }

  const target = { minHeight: TOUCH_TARGET_PX }
  const offered = GENERAL_ACTIONS.filter((a) => actions.includes(a.id))
  const action = pending !== null && pending !== 'abort' ? pending : null
  return (
    <Box component="section" aria-label="General" sx={{ border: 1, borderColor: 'divider', borderRadius: 1, p: 2, mb: 2 }}>
      <Typography variant="h6" component="h2">
        General
      </Typography>
      <Box sx={{ display: 'flex', gap: 1, flexWrap: 'wrap', mt: 1 }}>
        {rgbd && (
          <Button variant="outlined" color="error" sx={target} disabled={busy} onClick={() => setPending('abort')}>
            Abort calibration
          </Button>
        )}
        {offered.map((a) => (
          <Button
            key={a.id}
            variant="outlined"
            sx={target}
            disabled={busy || (sequenceLive && SEQUENCE_BLOCKED.includes(a.id)) || (sweepLive && SWEEP_BLOCKED.includes(a.id))}
            onClick={() => setPending(a)}
          >
            {a.label}
          </Button>
        ))}
      </Box>
      {sequenceLive && (
        <Typography variant="body2" color="text.secondary" sx={{ mt: 1 }}>
          {BLOCKED_TEXT}
        </Typography>
      )}
      {sweepLive && (
        <Typography variant="body2" color="text.secondary" sx={{ mt: 1 }}>
          {SWEEP_TEXT}
        </Typography>
      )}
      {feedback && (
        <Alert severity={feedback.ok ? 'success' : 'error'} sx={{ mt: 1 }} onClose={() => setFeedback(null)}>
          {feedback.message}
          {feedback.details.map((d) => (
            <Typography key={d} variant="body2">
              {d}
            </Typography>
          ))}
        </Alert>
      )}
      <ConfirmDialog
        open={pending === 'abort'}
        title="Abort calibration?"
        body={ABORT_TEXT}
        confirmLabel="Abort"
        onCancel={() => setPending(null)}
        onConfirm={() => void abort()}
      />
      <ConfirmDialog
        open={action !== null}
        title={`${action?.label ?? ''}?`}
        body={action?.confirm}
        confirmLabel={action?.label ?? ''}
        onCancel={() => setPending(null)}
        onConfirm={() => action && void run(action)}
      />
    </Box>
  )
}
