import { useCallback, useEffect, useRef, useState } from 'react'
import Alert from '@mui/material/Alert'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import LinearProgress from '@mui/material/LinearProgress'
import Typography from '@mui/material/Typography'
import { ConfirmDialog } from '../../layout/ConfirmDialog'
import { TOUCH_TARGET_PX } from '../../theme'
import { BoardNeeded } from '../BoardNeeded'
import { FlowStepper } from '../FlowStepper'
import { MoreFits } from '../MoreFits'
import { GateDecision, QualityGate, armCameraGate } from '../QualityGate'
import { ARM_POLL_MS, SequenceActivity, SkipReasons } from '../SequenceActivity'
import { staleJointOffsets } from '../ZeroRestorePanel'
import {
  ArmCameraAccept,
  ArmCameraResult,
  ArmCameraState,
  acceptArmCamera,
  computeArmCamera,
  declineArmCamera,
  getArmCameraResult,
  getArmCameraState,
} from '../armCameraApi'
import { runGeneralAction } from '../generalApi'
import type { FloorApiResult } from '../headFloorApi'
import {
  HeadArmSequence,
  HeadArmSession,
  allowBigJoints,
  boardAttached,
  getHeadArmState,
  listHeadArmSessions,
  moveToBase,
  park,
  stopSequence,
} from '../headArmApi'
import { useServoGroups } from '../servoApi'

/** Result poll period while an arm-camera fit computes: 1 Hz. */
export const RESULT_POLL_MS = 1000
/** Consecutive failed result polls before the compute step gives up and shows the error. */
const RESULT_POLL_FAILURES = 3

const BASE_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
/** Tier-2 joints named in the consent when the node has not sent its list. */
const BIG_JOINTS = ['elbow_flex', 'shoulder_lift', 'shoulder_pan']
const PHASE_IDLE = 'idle'
const PHASE_WAITING_BOARD = 'waiting_board'
const PHASE_WAITING_BIG = 'waiting_big_joints'
const PHASE_HOLDING = 'holding'
const PHASE_PARKING = 'parking'
/** Phases from the base pose on: the gripper is open at the insert width or the board is in. */
const AT_BASE_PHASES = ['waiting_board', 'capturing', 'waiting_big_joints', 'returning_to_base', 'holding', 'parking']
/** Phases after Board attached. */
const GRABBED_PHASES = ['capturing', 'waiting_big_joints', 'returning_to_base', 'holding', 'parking']
/** Phases of a capture still in progress. */
const CAPTURE_PHASES = ['moving_to_base', 'waiting_board', 'capturing', 'waiting_big_joints', 'returning_to_base']
const STALE_ZERO_TEXT = 'Arm zero is stale (joint_offsets): restore it in Servo home calibration first.'
const INTRO =
  'The arm joint offsets and arm base are fitted from the head D435: the head D435 watches the ARM board held by its handle in the jaws while the arm moves it through checked poses. Needs a camera-to-ground result (applied head chain) and a fresh arm zero. Robot not driving.'
const ACCEPT_BODY =
  'Records the arm-to-camera calibration (the arm joint offsets and arm base) as the applied one and restarts its consumer units. The same calibration as a client.yml snippet is shown below the report for review.'
const HEAD_CHAIN_CHANGED_TEXT =
  'The head chain changed since the applied arm-to-camera calibration was fitted: it no longer matches the camera. Run this flow again.'

/**
 * Text of an accept answer: the applied result and the restart state of each consumer unit.
 *
 * @param answer - the accept answer
 * @returns one line for the gate step
 */
export function acceptText(answer: ArmCameraAccept): string {
  const units = answer.units.map((u) => `${u.name} ${u.restarted ? 'restarted' : 'not restarted'} (${u.active_state})`)
  return `Applied result ${answer.arm_calibration.result}.${units.length ? ` Units: ${units.join(', ')}.` : ''}`
}

type Dialog = 'home' | 'base' | 'big' | 'park' | 'parkBoard' | null
type StepId = 'home' | 'open' | 'grab' | 'capture' | 'compute' | 'gate' | 'park'
type Errors = Partial<Record<StepId, string>>

/**
 * Flow 4, arm to camera: 1 Home arm, 2 Open gripper (base pose, insert width), 3 Grab board, 4 Capture (tiers, the
 * shoulder and elbow consent), 5 Compute, 6 the quality gate, 7 Park. Runs the head-arm sequence of
 * head_rgbd_calibration and its arm-camera fit.
 *
 * @param canHomeArm - the General Home arm action is offered (else step 1 is optional and points to the Arm tab)
 */
export function ArmCameraFlow({ canHomeArm }: { canHomeArm: boolean }) {
  const servo = useServoGroups()
  const [sequence, setSequence] = useState<HeadArmSequence | null>(null)
  const [sessions, setSessions] = useState<HeadArmSession[]>([])
  const [armState, setArmState] = useState<FloorApiResult<ArmCameraState> | null>(null)
  const [loadError, setLoadError] = useState<string | null>(null)
  const [errors, setErrors] = useState<Errors>({})
  const [dialog, setDialog] = useState<Dialog>(null)
  const [busy, setBusy] = useState(false)
  const [homed, setHomed] = useState(false)
  const [computing, setComputing] = useState<{ sid: string; rid: string } | null>(null)
  const [result, setResult] = useState<ArmCameraResult | null>(null)
  const [decision, setDecision] = useState<GateDecision>(null)
  const [decisionMessage, setDecisionMessage] = useState<string | null>(null)
  const [parkRequested, setParkRequested] = useState(false)
  // Session of a run the user closed with Start a new run: its capture no longer counts for steps 1-4.
  const [closedSid, setClosedSid] = useState<string | null>(null)
  const lastSid = useRef<string | null>(null)
  const pollFailures = useRef(0)
  const polling = useRef(false)

  const reload = useCallback(async () => {
    const [s, l] = await Promise.all([getHeadArmState(), listHeadArmSessions()])
    if (s.ok) setSequence(s.value.sequence)
    if (l.ok) setSessions(l.value)
    const failure = !s.ok ? s : !l.ok ? l : null
    setLoadError(failure ? failure.message : null)
  }, [])

  const reloadArmState = useCallback(async () => {
    setArmState(await getArmCameraState())
  }, [])

  useEffect(() => {
    void reload()
    void reloadArmState()
  }, [reload, reloadArmState])

  const running = sequence?.running === true
  useEffect(() => {
    if (!running) return
    const id = setInterval(() => void reload(), ARM_POLL_MS)
    return () => clearInterval(id)
  }, [running, reload])

  const pollResult = useCallback(async (sid: string, rid: string) => {
    if (polling.current) return // the previous poll still runs: never overlap
    polling.current = true
    const r = await getArmCameraResult(sid, rid)
    polling.current = false
    if (!r.ok) {
      pollFailures.current += 1
      if (pollFailures.current >= RESULT_POLL_FAILURES) {
        setErrors((e) => ({ ...e, compute: r.message }))
        setComputing(null)
      }
      return
    }
    pollFailures.current = 0
    if (r.value.state === 'running') return
    setComputing(null)
    if (r.value.state === 'failed') setErrors((e) => ({ ...e, compute: r.value.error ?? 'the fit failed' }))
    else setResult(r.value)
  }, [])

  useEffect(() => {
    if (!computing) return
    void pollResult(computing.sid, computing.rid)
    const id = setInterval(() => void pollResult(computing.sid, computing.rid), RESULT_POLL_MS)
    return () => clearInterval(id)
  }, [computing, pollResult])

  /** Runs one sequence call for a step and shows its sequence or its error on that step. */
  const step = async (id: StepId, call: () => Promise<FloorApiResult<HeadArmSequence>>): Promise<boolean> => {
    setDialog(null)
    setBusy(true)
    const r = await call()
    setBusy(false)
    if (r.ok) {
      setSequence(r.value)
      setErrors((e) => ({ ...e, [id]: undefined }))
    } else setErrors((e) => ({ ...e, [id]: r.message }))
    return r.ok
  }

  const home = async () => {
    setDialog(null)
    setBusy(true)
    const r = await runGeneralAction('home_arm')
    setBusy(false)
    const ok = r.ok && r.value.ok
    setHomed(ok)
    setErrors((e) => ({ ...e, home: ok ? undefined : r.ok ? r.value.message : r.message }))
  }

  const phase = sequence?.phase ?? PHASE_IDLE
  const sid = sequence?.session_id ?? null
  const captured =
    sid !== null && sid !== closedSid && (sequence?.captured ?? 0) > 0 && !running && !CAPTURE_PHASES.includes(phase)
  // The fits of More fits run on the live session, or on the newest one when the node has none (after a restart).
  const session = sid !== null ? sessions.find((s) => s.id === sid) ?? null : sessions[sessions.length - 1] ?? null
  const parked = parkRequested && phase === PHASE_IDLE && !running && !sequence?.error

  // A new capture session starts a new run: the previous result, decision and park no longer apply.
  useEffect(() => {
    if (sid === null || sid === lastSid.current) return
    const first = lastSid.current === null
    lastSid.current = sid
    if (first) return
    setComputing(null)
    setResult(null)
    setDecision(null)
    setDecisionMessage(null)
    setParkRequested(false)
    setErrors({})
  }, [sid])

  const compute = async () => {
    if (!sid) return
    setResult(null)
    setDecision(null)
    setDecisionMessage(null)
    setErrors((e) => ({ ...e, compute: undefined, gate: undefined }))
    const r = await computeArmCamera(sid)
    if (r.ok) setComputing({ sid, rid: r.value.result_id })
    else setErrors((e) => ({ ...e, compute: r.message }))
  }

  const decide = async (accept: boolean) => {
    if (!result) return
    setBusy(true)
    const r = accept ? await acceptArmCamera(result.session, result.id) : await declineArmCamera(result.session, result.id)
    setBusy(false)
    if (!r.ok) {
      setErrors((e) => ({ ...e, gate: r.message }))
      return
    }
    setErrors((e) => ({ ...e, gate: undefined }))
    setDecision(accept ? 'accepted' : 'declined')
    if ('units' in r.value) {
      setDecisionMessage(acceptText(r.value))
      void reloadArmState()
    } else {
      setResult(r.value)
      setDecisionMessage(null)
    }
  }

  const parkArm = async (boardOn: boolean) => {
    if (await step('park', () => (boardOn ? park(true, true) : park(true)))) setParkRequested(true)
  }

  const restart = () => {
    setClosedSid(sid)
    setHomed(false)
    setComputing(null)
    setResult(null)
    setDecision(null)
    setDecisionMessage(null)
    setParkRequested(false)
    setErrors({})
  }

  const armGroup = servo.groups.find((g) => g.id === 'arm')
  const zeroStale = armGroup !== undefined && staleJointOffsets(armGroup)
  const notReady = armState?.ok && !armState.value.ready ? armState.value.reason ?? 'the arm-to-camera fit is not ready' : null
  const target = { minHeight: TOUCH_TARGET_PX }
  const skipped = Object.values(sequence?.skipped ?? {}).reduce((acc, n) => acc + n, 0)
  const bigJoints = sequence?.big_joints.length ? sequence.big_joints : BIG_JOINTS
  const moveBlocked = busy || zeroStale || notReady !== null
  const snippet = typeof result?.calibration?.snippet === 'string' ? result.calibration.snippet : null

  return (
    <Box>
      <Typography variant="body2" color="text.secondary">
        {INTRO}
      </Typography>
      <BoardNeeded role="arm" />
      {zeroStale && (
        <Alert severity="warning" sx={{ mt: 1 }}>
          {STALE_ZERO_TEXT}
        </Alert>
      )}
      {notReady && (
        <Alert severity="warning" sx={{ mt: 1 }}>
          {`Not ready: ${notReady}`}
        </Alert>
      )}
      {armState && !armState.ok && (
        <Alert severity="info" sx={{ mt: 1 }}>
          {`Arm-to-camera fit state unavailable: ${armState.message}`}
        </Alert>
      )}
      {armState?.ok && armState.value.applied?.head_chain_changed && (
        <Alert severity="warning" sx={{ mt: 1 }}>
          {HEAD_CHAIN_CHANGED_TEXT}
        </Alert>
      )}
      {armState?.ok && armState.value.ready && (
        <Typography variant="body2" sx={{ mt: 1 }}>
          {`Ready: head chain from ${armState.value.head_chain_source ?? 'unknown'}${armState.value.applied ? `; arm-camera calibration ${armState.value.applied.result} is applied` : ''}.`}
        </Typography>
      )}
      {loadError && (
        <Alert severity="error" sx={{ mt: 1 }}>
          {loadError}
        </Alert>
      )}
      {sequence && <SequenceActivity sequence={sequence} withStatus={!sid} />}
      <Box sx={{ mt: 1 }}>
        <FlowStepper
          label="Arm to camera calibration steps"
          steps={[
            {
              id: 'home',
              title: 'Home arm',
              optional: !canHomeArm,
              done: homed || (phase !== PHASE_IDLE && sid !== closedSid) || captured,
              see: 'The arm folds to its stored home pose and stops; nothing on the gripper.',
              error: errors.home,
              children: (
                <Box sx={{ display: 'flex', gap: 1, flexWrap: 'wrap' }}>
                  {canHomeArm ? (
                    <Button
                      variant="contained"
                      sx={target}
                      disabled={busy || zeroStale || running || phase !== PHASE_IDLE}
                      onClick={() => setDialog('home')}
                    >
                      Home arm
                    </Button>
                  ) : (
                    <Typography variant="body2">Home the arm from the Arm tab, then continue.</Typography>
                  )}
                  {phase === PHASE_IDLE && (
                    <Button variant="outlined" sx={target} disabled={busy} onClick={() => setDialog('parkBoard')}>
                      Park (board is on the gripper)
                    </Button>
                  )}
                </Box>
              ),
            },
            {
              id: 'open',
              title: 'Open gripper',
              done: (AT_BASE_PHASES.includes(phase) && sid !== closedSid) || captured,
              see: 'The arm stands in the calibration base pose with the gripper open to the insert width. Keep the area clear.',
              error: errors.open,
              children: (
                <Button variant="contained" sx={target} disabled={moveBlocked || running} onClick={() => setDialog('base')}>
                  Open gripper
                </Button>
              ),
            },
            {
              id: 'grab',
              title: 'Grab board',
              done: (GRABBED_PHASES.includes(phase) && sid !== closedSid) || captured,
              see: 'The jaws close on the handle; the events report the grip seated, without slack. The wrist poses start.',
              error: errors.grab,
              children: (
                <>
                  <Typography variant="body2">
                    Seat the arm board handle in the gripper jaws up to the shoulder, then press Board attached.
                  </Typography>
                  <Button
                    variant="contained"
                    sx={{ ...target, mt: 1 }}
                    disabled={busy || phase !== PHASE_WAITING_BOARD}
                    onClick={() => void step('grab', boardAttached)}
                  >
                    Board attached
                  </Button>
                </>
              ),
            },
            {
              id: 'capture',
              title: 'Capture',
              done: captured,
              see: 'Tier 0 and 1 move the wrist; the elbow and shoulder tier waits for your consent. The captured count climbs, the arm returns to the base pose and holds.',
              error: errors.capture,
              status:
                sequence && sid ? (
                  <Box>
                    {running && sequence.total_poses > 0 && (
                      <LinearProgress variant="determinate" value={(100 * (sequence.arm_pose ?? 0)) / sequence.total_poses} />
                    )}
                    <Typography variant="body2">
                      {`${sequence.tier === null ? 'Tier -' : `Tier ${sequence.tier}`}, pose ${sequence.arm_pose === null ? '-' : sequence.arm_pose + 1} of ${sequence.total_poses}, ${sequence.captured} captured, ${skipped} skipped - ${sequence.status}`}
                    </Typography>
                    <SkipReasons skipped={sequence.skipped} />
                  </Box>
                ) : undefined,
              children: (
                <Box sx={{ display: 'flex', gap: 1, flexWrap: 'wrap' }}>
                  <Button variant="contained" sx={target} disabled={busy || phase !== PHASE_WAITING_BIG} onClick={() => setDialog('big')}>
                    Allow shoulder and elbow moves
                  </Button>
                  <Button
                    variant="outlined"
                    sx={target}
                    disabled={busy || phase !== PHASE_WAITING_BIG}
                    onClick={() => void step('capture', () => allowBigJoints(false))}
                  >
                    Skip them
                  </Button>
                  <Button
                    variant="outlined"
                    color="error"
                    sx={target}
                    disabled={phase === PHASE_PARKING || (!running && !CAPTURE_PHASES.includes(phase))}
                    onClick={() => void step('capture', stopSequence)}
                  >
                    Stop
                  </Button>
                </Box>
              ),
            },
            {
              id: 'compute',
              title: 'Compute',
              done: result !== null,
              see: 'A progress line while the fit runs, then the quality report opens in the next step.',
              error: errors.compute,
              status: computing ? <Typography variant="body2">Computing the arm-to-camera fit...</Typography> : undefined,
              children: (
                <>
                  <Button variant="contained" sx={target} disabled={busy || running || computing !== null || !sid} onClick={() => void compute()}>
                    Compute
                  </Button>
                </>
              ),
            },
            {
              id: 'gate',
              title: 'Check and accept',
              done: decision !== null,
              see: 'Plausible, small held-out tool error and the expected accuracy in mm; Accept applies it, Decline keeps the current calibration.',
              error: errors.gate,
              children: result?.report ? (
                <>
                  <QualityGate
                    report={armCameraGate(result.report)}
                    acceptBody={ACCEPT_BODY}
                    decision={decision}
                    busy={busy}
                    onAccept={() => void decide(true)}
                    onDecline={() => void decide(false)}
                  />
                  {decisionMessage && (
                    <Typography variant="body2" sx={{ mt: 1 }}>
                      {decisionMessage}
                    </Typography>
                  )}
                  {snippet !== null && (
                    <Box sx={{ mt: 1 }} aria-label="client.yml snippet">
                      <Typography variant="subtitle2" component="h4">
                        client.yml snippet
                      </Typography>
                      <Box component="pre" sx={{ fontFamily: 'monospace', whiteSpace: 'pre-wrap', m: 0 }}>
                        {snippet}
                      </Box>
                      <Button variant="outlined" sx={{ ...target, mt: 1 }} onClick={() => void navigator.clipboard.writeText(snippet)}>
                        Copy
                      </Button>
                    </Box>
                  )}
                </>
              ) : null,
            },
            {
              id: 'park',
              title: 'Park',
              done: parked,
              alwaysOpen: phase === PHASE_HOLDING || parkRequested,
              see: 'The arm parks with the board (its path is checked with it); remove the board afterwards.',
              error: errors.park,
              children: (
                <Box sx={{ display: 'flex', gap: 1, flexWrap: 'wrap' }}>
                  <Button variant="contained" sx={target} disabled={busy || phase !== PHASE_HOLDING} onClick={() => setDialog('park')}>
                    Park arm
                  </Button>
                  {parked && (
                    <Button variant="outlined" sx={target} onClick={restart}>
                      Start a new run
                    </Button>
                  )}
                </Box>
              ),
            },
          ]}
        />
      </Box>
      <MoreFits session={session} disabled={running} />
      <ConfirmDialog
        open={dialog === 'home'}
        title="Home the arm?"
        body="The arm moves to its stored home pose. Keep the area around the arm clear."
        confirmLabel="Home arm"
        onCancel={() => setDialog(null)}
        onConfirm={() => void home()}
      />
      <ConfirmDialog
        open={dialog === 'base'}
        title="Move the arm to the base pose and open the gripper?"
        body={`These joints move: ${BASE_JOINTS.join(', ')}. The robot must not be driving and the area around it must be clear.`}
        confirmLabel="Move"
        onCancel={() => setDialog(null)}
        onConfirm={() => void step('open', () => moveToBase(true))}
      />
      <ConfirmDialog
        open={dialog === 'big'}
        title="Allow shoulder and elbow moves?"
        body={`The next poses move ${bigJoints.join(', ')}. Allow them only with the area around the arm clear.`}
        confirmLabel="Allow"
        onCancel={() => setDialog(null)}
        onConfirm={() => void step('capture', () => allowBigJoints(true))}
      />
      <ConfirmDialog
        open={dialog === 'park'}
        title="Park the arm?"
        body="The arm moves to its park pose, with the board on the gripper or not. Park is refused, naming the rule, when the board would hit the floor, the body or the head."
        confirmLabel="Park"
        onCancel={() => setDialog(null)}
        onConfirm={() => void parkArm(false)}
      />
      <ConfirmDialog
        open={dialog === 'parkBoard'}
        title="The calibration board is on the gripper"
        body={`Use this when the arm still holds the board but this page lost the sequence (for example after a restart). The board is recorded as on the gripper, the park path is checked with it, and Park is refused, naming the rule, when the board would hit the floor, the body or the head.${
          sequence?.board_record
            ? ` The node remembers a board attached in session ${sequence.board_record.session_id ?? 'unknown'} at ${sequence.board_record.clipped_at}: confirm it is still on the gripper.`
            : ''
        }`}
        confirmLabel="Park with the board"
        onCancel={() => setDialog(null)}
        onConfirm={() => void parkArm(true)}
      />
    </Box>
  )
}
