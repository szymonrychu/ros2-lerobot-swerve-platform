import { useCallback, useEffect, useRef, useState } from 'react'
import Alert from '@mui/material/Alert'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import LinearProgress from '@mui/material/LinearProgress'
import Typography from '@mui/material/Typography'
import { ConfirmDialog } from '../../layout/ConfirmDialog'
import { TOUCH_TARGET_PX } from '../../theme'
import { BoardNeeded } from '../BoardNeeded'
import { CsiIntrinsics } from '../CsiIntrinsics'
import { FloorFitMore } from '../FloorFitMore'
import { FlowStepper } from '../FlowStepper'
import {
  BoardPreview,
  GroundAppliedView,
  GroundResultDetails,
  GroundSweepStatus,
  GroundVerifyView,
  type HeadCameraPreview,
  groundAcceptText,
  headChainApplied,
} from '../GroundBoard'
import { GateDecision, QualityGate, groundBoardGate } from '../QualityGate'
import { RgbdCalibration } from '../RgbdCalibration'
import { GENERAL_ACTIONS, runGeneralAction } from '../generalApi'
import {
  GroundResult,
  GroundState,
  GroundSweep,
  GroundVerifyState,
  acceptGround,
  computeGround,
  declineGround,
  getGroundResult,
  getGroundState,
  nextPlacement,
  startGroundSweep,
  startGroundVerify,
  stopGroundSweep,
  stopGroundVerify,
} from '../groundBoardApi'
import type { FloorApiResult } from '../headFloorApi'

/** Ground state poll period while a session or a verification runs: 1 Hz. */
export const GROUND_POLL_MS = 1000
/** Result poll period while a ground board fit computes: 1 Hz. */
export const GROUND_RESULT_POLL_MS = 1000
/** Consecutive failed result polls before the compute step gives up and shows the error. */
const RESULT_POLL_FAILURES = 3
const HOME_ACTION = GENERAL_ACTIONS.find((a) => a.id === 'home_head')

const INTRO =
  'The head chain (camera to ground) is fitted from the big ChArUco board lying flat on the floor, seen by the head D435 over a pan/tilt grid at one to three board placements. Only the head moves; the robot stands on a flat floor, not driving.'
const BOARD_TEXT =
  'Lay the board flat on the floor, close to the robot, fully visible in the head camera preview with the head homed. Nothing else about where it lies matters.'
const SWEEP_TEXT =
  'The head homes, finds the board and sweeps a grid around it. After each placement move the board to a new place (close, flat, fully visible at home) and press Next placement, or Stop to end the session.'
const ACCEPT_BODY =
  'Applies the fitted head chain (source ground board): the head camera, obstacle and 3D map nodes restart with it. Needs an accepted D435 intrinsics check. An applied arm-to-camera calibration was fitted on the previous chain: run that flow again afterwards.'
const VERIFY_TEXT =
  'After an accept, the head visits the runbook poses and checks /ground/state at each: the floor plane at height 0, level with the base, with no obstacle or drop points.'

type StepId = 'home' | 'sweep' | 'compute' | 'gate' | 'verify'
type Errors = Partial<Record<StepId, string>>

interface Props {
  /** The General Home camera action is offered (else step 3 is optional and points to the Head tab). */
  canHomeHead: boolean
  /** The head camera stream for the board check, null when the robot config has none. */
  headCamera: HeadCameraPreview | null
}

/**
 * Flow 3, camera to ground: the optional D435 intrinsics check, the front/back CSI cameras (always visible), then the
 * ground board fit: 3 Home camera, 4 Lay the board, 5 Board sweep (placements), 6 Compute, 7 the quality gate whose
 * Accept applies the head chain, 8 Verify on the ground. The depth floor fit stays reachable under More fits.
 *
 * @param props - see Props
 */
export function CameraGroundFlow({ canHomeHead, headCamera }: Props) {
  const [state, setState] = useState<GroundState | null>(null)
  const [loadError, setLoadError] = useState<string | null>(null)
  const [errors, setErrors] = useState<Errors>({})
  const [busy, setBusy] = useState(false)
  const [homeDialog, setHomeDialog] = useState(false)
  const [homed, setHomed] = useState(false)
  const [boardDown, setBoardDown] = useState(false)
  const [computing, setComputing] = useState<{ sid: string; rid: string } | null>(null)
  const [result, setResult] = useState<GroundResult | null>(null)
  const [decision, setDecision] = useState<GateDecision>(null)
  const [decisionMessage, setDecisionMessage] = useState<string | null>(null)
  const resumed = useRef(false)
  const pollFailures = useRef(0)
  const polling = useRef(false)

  const reload = useCallback(async () => {
    const r = await getGroundState()
    if (!r.ok) {
      setLoadError(r.message)
      return
    }
    setLoadError(null)
    setState(r.value)
    // A compute still running on the node (the page was reloaded): poll it to its result.
    const job = r.value.compute
    if (!resumed.current && job.running && job.session_id && job.result_id) setComputing({ sid: job.session_id, rid: job.result_id })
    resumed.current = true
  }, [])

  useEffect(() => {
    void reload()
  }, [reload])

  const sweep = state?.sweep ?? null
  const verify = state?.verify ?? null
  const sweepRunning = sweep?.running === true
  const verifyRunning = verify?.running === true
  useEffect(() => {
    if (!sweepRunning && !verifyRunning) return
    const id = setInterval(() => void reload(), GROUND_POLL_MS)
    return () => clearInterval(id)
  }, [sweepRunning, verifyRunning, reload])

  const pollResult = useCallback(async (sid: string, rid: string) => {
    if (polling.current) return // the previous poll still runs: never overlap
    polling.current = true
    const r = await getGroundResult(sid, rid)
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
    const id = setInterval(() => void pollResult(computing.sid, computing.rid), GROUND_RESULT_POLL_MS)
    return () => clearInterval(id)
  }, [computing, pollResult])

  const setSweep = (s: GroundSweep) => setState((st) => (st ? { ...st, sweep: s } : st))
  const setVerify = (v: GroundVerifyState) => setState((st) => (st ? { ...st, verify: v } : st))

  /** Runs one call for a step and shows its answer or its error on that step. */
  async function run<T>(id: StepId, call: () => Promise<FloorApiResult<T>>, onOk: (v: T) => void): Promise<void> {
    setBusy(true)
    const r = await call()
    setBusy(false)
    if (r.ok) {
      setErrors((e) => ({ ...e, [id]: undefined }))
      onOk(r.value)
    } else setErrors((e) => ({ ...e, [id]: r.message }))
  }

  const home = async () => {
    setHomeDialog(false)
    setBusy(true)
    const r = await runGeneralAction('home_head')
    setBusy(false)
    const ok = r.ok && r.value.ok
    setHomed(ok)
    setErrors((e) => ({ ...e, home: ok ? undefined : r.ok ? r.value.message : r.message }))
  }

  const start = () =>
    run('sweep', startGroundSweep, (s) => {
      setSweep(s)
      setResult(null)
      setDecision(null)
      setDecisionMessage(null)
      setErrors({})
    })

  const compute = async () => {
    const sid = sweep?.session_id
    if (!sid) return
    setResult(null)
    setDecision(null)
    setDecisionMessage(null)
    setErrors((e) => ({ ...e, gate: undefined }))
    await run('compute', () => computeGround(sid), (v) => setComputing({ sid, rid: v.result_id }))
  }

  const decide = async (accept: boolean) => {
    if (!result) return
    if (accept) {
      await run('gate', () => acceptGround(result.session, result.id), (v) => {
        setDecision('accepted')
        setDecisionMessage(groundAcceptText(v))
        void reload()
      })
    } else {
      await run('gate', () => declineGround(result.session, result.id), (v) => {
        setDecision('declined')
        setResult(v)
        setDecisionMessage(null)
      })
    }
  }

  const sid = sweep?.session_id ?? null
  const swept = sid !== null && !sweepRunning && (sweep?.views ?? 0) > 0
  const applied = headChainApplied(state?.head_chain ?? null)
  const target = { minHeight: TOUCH_TARGET_PX }
  const canNext = sweep !== null && sweep.awaiting_placement && sweep.placements_done < sweep.max_placements

  return (
    <Box>
      <Typography variant="body2" color="text.secondary">
        {INTRO}
      </Typography>
      {state && <GroundAppliedView chain={state.head_chain} accepted={state.accepted} />}
      {loadError && (
        <Alert severity="error" sx={{ mt: 1 }}>
          {`Ground board state unavailable: ${loadError}`}
        </Alert>
      )}
      <Box sx={{ mt: 1 }}>
        <FlowStepper
          label="Camera to ground calibration steps"
          steps={[
            {
              id: 'd435',
              title: 'D435 intrinsics check',
              optional: true,
              done: false,
              see: 'Every coverage cell has corners; Compute compares with the D435 factory values and Accept check records a passed check (the ground board accept needs one).',
              children: <RgbdCalibration heading={false} />,
            },
            {
              id: 'csi',
              title: 'Front/back cameras',
              optional: true,
              done: false,
              see: 'For each of the front and back cameras: views kept over the whole image, a small RMS after Compute, Apply restarts the camera node.',
              children: <CsiIntrinsics heading={false} />,
            },
            {
              id: 'home',
              title: 'Home camera',
              optional: !canHomeHead,
              done: homed || sid !== null,
              see: 'The head turns to pan 0 and tilts to its home tilt (30 deg down) and stops.',
              error: errors.home,
              children: canHomeHead ? (
                <Button variant="contained" sx={target} disabled={busy || sweepRunning || verifyRunning} onClick={() => setHomeDialog(true)}>
                  Home camera
                </Button>
              ) : (
                <Typography variant="body2">Home the camera from the Head tab, then continue.</Typography>
              ),
            },
            {
              id: 'board',
              title: 'Lay the board',
              done: boardDown || sid !== null,
              see: 'The whole board in the head camera preview, flat on the floor; the sweep refuses a placement when too few corners show at home.',
              children: (
                <>
                  <Typography variant="body2">{BOARD_TEXT}</Typography>
                  <BoardNeeded role="full" />
                  <BoardPreview preview={headCamera} />
                  <Button variant="contained" sx={{ ...target, mt: 1 }} disabled={boardDown} onClick={() => setBoardDown(true)}>
                    Board is down
                  </Button>
                </>
              ),
            },
            {
              id: 'sweep',
              title: 'Board sweep',
              done: swept,
              see: 'Per placement the head homes, locates the board and sweeps the grid; the views climb with few skipped. 2 or 3 placements give the best result.',
              error: errors.sweep ?? sweep?.error ?? null,
              status: sweep && sid ? (
                <Box>
                  {sweepRunning && !sweep.awaiting_placement && (
                    <LinearProgress variant="determinate" value={sweep.total ? (100 * sweep.pose) / sweep.total : 0} />
                  )}
                  <GroundSweepStatus sweep={sweep} />
                </Box>
              ) : undefined,
              children: (
                <>
                  <Typography variant="body2" color="text.secondary">
                    {SWEEP_TEXT}
                  </Typography>
                  {sweep?.message && (
                    <Alert severity={sweep.awaiting_placement ? 'info' : 'warning'} sx={{ mt: 1 }}>
                      {sweep.message}
                    </Alert>
                  )}
                  {sweep?.awaiting_placement && <BoardPreview preview={headCamera} />}
                  <Box sx={{ display: 'flex', gap: 1, mt: 1, flexWrap: 'wrap' }}>
                    <Button variant="contained" sx={target} disabled={busy || sweepRunning || verifyRunning} onClick={() => void start()}>
                      Start sweep
                    </Button>
                    <Button variant="outlined" sx={target} disabled={busy || !canNext} onClick={() => void run('sweep', nextPlacement, setSweep)}>
                      Next placement
                    </Button>
                    <Button
                      variant="outlined"
                      color="error"
                      sx={target}
                      disabled={!sweepRunning}
                      onClick={() => void run('sweep', stopGroundSweep, setSweep)}
                    >
                      Stop
                    </Button>
                  </Box>
                </>
              ),
            },
            {
              id: 'compute',
              title: 'Compute',
              done: result !== null,
              see: 'A progress line while the fit runs (seconds to minutes on the robot), then the quality report opens in the next step.',
              error: errors.compute,
              status: computing ? <Typography variant="body2">Computing the ground board fit...</Typography> : undefined,
              children: (
                <Button
                  variant="contained"
                  sx={target}
                  disabled={busy || sweepRunning || verifyRunning || computing !== null || !swept}
                  onClick={() => void compute()}
                >
                  Compute
                </Button>
              ),
            },
            {
              id: 'gate',
              title: 'Check and accept',
              done: decision !== null,
              see: 'Plausible, small RMS and held-out RMS, the expected floor accuracy in mm; Accept applies the head chain, Decline keeps the current one.',
              error: errors.gate,
              children: result?.report ? (
                <>
                  <QualityGate
                    report={groundBoardGate(result.report)}
                    acceptBody={ACCEPT_BODY}
                    decision={decision}
                    busy={busy || verifyRunning || sweepRunning}
                    onAccept={() => void decide(true)}
                    onDecline={() => void decide(false)}
                  />
                  <GroundResultDetails report={result.report} />
                  {decisionMessage && (
                    <Typography variant="body2" sx={{ mt: 1 }}>
                      {decisionMessage}
                    </Typography>
                  )}
                </>
              ) : null,
            },
            {
              id: 'verify',
              title: 'Verify on the ground',
              done: verify?.status === 'done' && verify.passed === true,
              alwaysOpen: applied,
              see: 'Every pose passes: plane height within 10 mm of 0, plane vs base under 0.6 deg, few obstacle points, no drop points.',
              error: errors.verify,
              children: (
                <>
                  <Typography variant="body2" color="text.secondary">
                    {VERIFY_TEXT}
                  </Typography>
                  <Box sx={{ display: 'flex', gap: 1, mt: 1, flexWrap: 'wrap' }}>
                    <Button
                      variant="contained"
                      sx={target}
                      disabled={busy || !applied || verifyRunning || sweepRunning}
                      onClick={() => void run('verify', startGroundVerify, setVerify)}
                    >
                      Start verification
                    </Button>
                    <Button
                      variant="outlined"
                      color="error"
                      sx={target}
                      disabled={!verifyRunning}
                      onClick={() => void run('verify', stopGroundVerify, setVerify)}
                    >
                      Stop verification
                    </Button>
                  </Box>
                  {verify && <GroundVerifyView verify={verify} running={verifyRunning} />}
                </>
              ),
            },
          ]}
        />
      </Box>
      <FloorFitMore disabled={sweepRunning || verifyRunning} />
      <ConfirmDialog
        open={homeDialog}
        title="Home the camera?"
        body={HOME_ACTION?.confirm}
        confirmLabel="Home camera"
        onCancel={() => setHomeDialog(false)}
        onConfirm={() => void home()}
      />
    </Box>
  )
}
