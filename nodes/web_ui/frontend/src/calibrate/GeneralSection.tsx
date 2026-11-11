import { useCallback, useEffect, useState } from 'react'
import Alert from '@mui/material/Alert'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import Typography from '@mui/material/Typography'
import { ConfirmDialog } from '../layout/ConfirmDialog'
import { TOUCH_TARGET_PX } from '../theme'
import { BoardRecordNotice } from './BoardRecordNotice'
import { GENERAL_ACTIONS, type GeneralAction, type GeneralActionId, abortCalibration, runGeneralAction } from './generalApi'
import { type BoardRecord, type HeadArmSequence, getHeadArmState } from './headArmApi'
import { getFloorState } from './headFloorApi'
import { getGroundState } from './groundBoardApi'
import { useServoGroups } from './servoApi'
import { servoCalibrationInfoText, servoCalibrationRedoAt } from './ZeroRestorePanel'

/** Head-arm sequence, floor sweep and ground board poll period of the General section: 0.5 Hz (the flows poll faster while they run). */
export const GENERAL_POLL_MS = 2000
/** Actions that would fight a live calibration sequence (it holds the arm and aims the head) or drop its board. */
const SEQUENCE_BLOCKED: GeneralActionId[] = ['open_gripper', 'close_gripper', 'home_arm', 'home_head']
/** Actions that would fight a floor sweep, a ground board session or a ground verification (they move the head). */
const SWEEP_BLOCKED: GeneralActionId[] = ['home_head']
/** Sequence phases in which the arm is the sequence's or holds the board (holding: Home it from the Arm to camera flow). */
const LIVE_PHASES = ['waiting_board', 'waiting_big_joints', 'holding', 'homing']
const BLOCKED_TEXT =
  "Arm and camera moves are off: a calibration sequence runs or the arm holds the board. Abort the running calibration first; with the board on the gripper, home the arm with Home in the Arm to camera flow (step 7)."
const SWEEP_TEXT = 'Home camera is off while a floor sweep, a ground board session or a ground verification moves the head.'
const BOARD_HOME_HINT = 'home it with Home (board is on the gripper) in the Arm to camera flow.'
const ABORT_TEXT =
  "Stops the running calibration capture, sweep or sequence. The arm holds where it is (no Home move); use the Arm to camera flow's Home or General Home arm afterwards."

type Feedback = { ok: boolean; message: string; details: string[] } | null
type Pending = GeneralAction | 'abort' | null

/**
 * General section of the Calibrate page, always on top: Abort calibration plus the moving helpers (open / close
 * gripper, home arm, home camera, home steering servos), each behind a confirm, and the node's board record (any age)
 * with its no-motion clear. Home arm is off while the board record exists (mcp_server refuses it). After a servo calibration
 * it stays on; an info line shows the calibration time, the remedy (redo Calibrate, Arm to camera) and the zero shift.
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
  const [sequenceRunning, setSequenceRunning] = useState(false)
  // The node's board record, whatever its age: mcp_server refuses Home arm while it exists.
  const [boardRecord, setBoardRecord] = useState<BoardRecord | null>(null)
  // Head motions of the floor sweep and of the ground board (session or verification); each kept from its last read.
  const [floorLive, setFloorLive] = useState(false)
  const [groundLive, setGroundLive] = useState(false)
  const sweepLive = floorLive || groundLive
  // The arm group in the servo calibration redo state: Home arm works as usual, an info line under the buttons says
  // the joint offsets still need the Arm to camera redo and names the zero shift.
  const servoGroups = useServoGroups().groups
  const servoCalibrationAt = servoCalibrationRedoAt(servoGroups, 'arm')
  const armZeroShift = servoGroups.find((g) => g.id === 'arm')?.zero_shift_deg

  const follow = useCallback((sequence: HeadArmSequence) => {
    setSequenceLive(sequence.running || LIVE_PHASES.includes(sequence.phase))
    setSequenceRunning(sequence.running)
    setBoardRecord(sequence.board_record)
  }, [])

  useEffect(() => {
    if (!rgbd) return
    let live = true
    const poll = async () => {
      const [s, f, g] = await Promise.all([getHeadArmState(), getFloorState(), getGroundState()])
      if (!live) return
      if (s.ok) follow(s.value.sequence)
      if (f.ok) setFloorLive(f.value.sweep.running)
      if (g.ok) setGroundLive(g.value.sweep.running || g.value.verify.running)
    }
    void poll()
    const id = setInterval(() => void poll(), GENERAL_POLL_MS)
    return () => {
      live = false
      clearInterval(id)
    }
  }, [rgbd, follow])

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
            disabled={
              busy ||
              (sequenceLive && SEQUENCE_BLOCKED.includes(a.id)) ||
              (sweepLive && SWEEP_BLOCKED.includes(a.id)) ||
              (boardRecord !== null && a.id === 'home_arm')
            }
            onClick={() => setPending(a)}
          >
            {a.label}
          </Button>
        ))}
      </Box>
      {rgbd && <BoardRecordNotice record={boardRecord} running={sequenceRunning} homeHint={BOARD_HOME_HINT} onCleared={follow} />}
      {sequenceLive && (
        <Typography variant="body2" color="text.secondary" sx={{ mt: 1 }}>
          {BLOCKED_TEXT}
        </Typography>
      )}
      {servoCalibrationAt !== null && actions.includes('home_arm') && (
        <Typography variant="body2" color="text.secondary" sx={{ mt: 1 }}>
          {servoCalibrationInfoText(servoCalibrationAt, armZeroShift)}
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
