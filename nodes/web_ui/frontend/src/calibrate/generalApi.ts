/**
 * General section of the Calibrate page: the moving actions (mcp_server Trigger services behind
 * POST /api/calibration/general/{action}) and Abort calibration (the stop routes of the running calibration work).
 */
import { type ServoActionResult, type ServoApiResult } from './servoApi'
import { getCsiState, stopCsiCapture } from './csiApi'
import { getHeadArmState, stopSequence } from './headArmApi'
import { getFloorState, stopSweep } from './headFloorApi'
import { getGroundState, stopGroundSweep, stopGroundVerify } from './groundBoardApi'
import { getState, stopCapture } from './rgbdApi'

const BASE = '/api/calibration/general'
/** Sequence phases that wait for the user: the sequence is live and Abort stops it. */
const WAITING_PHASES = ['waiting_board', 'waiting_big_joints']
const HOMING_PHASE = 'homing'
/** Any CSI camera: the state carries the latest capture of every camera. */
const CSI_STATE_CAMERA = 'front'

export type GeneralActionId = 'open_gripper' | 'close_gripper' | 'home_arm' | 'home_head' | 'home_steering'

export interface GeneralAction {
  id: GeneralActionId
  label: string
  /** Body of the confirm dialog: what moves. */
  confirm: string
}

/** The moving actions of the General section, in page order. */
export const GENERAL_ACTIONS: GeneralAction[] = [
  { id: 'open_gripper', label: 'Open gripper', confirm: 'The gripper opens fully. Only the gripper moves; the arm keeps its pose.' },
  { id: 'close_gripper', label: 'Close gripper', confirm: 'The gripper closes. Keep fingers out of the jaws.' },
  { id: 'home_arm', label: 'Home arm', confirm: 'The arm moves to its stored home pose. Keep the area around the arm clear.' },
  { id: 'home_head', label: 'Home camera', confirm: 'The head turns to pan 0 and tilts to its home tilt (30 deg down).' },
  {
    id: 'home_steering',
    label: 'Home steering servos',
    confirm: 'All four wheels steer straight ahead. The robot does not drive; keep feet clear of the wheels.',
  },
]

/** What Abort stopped and what it could not read or stop. */
export interface AbortOutcome {
  stopped: string[]
  errors: string[]
}

/**
 * Run one General action; a refusal (409: torque off, a lease held, a running motion) returns the backend message.
 *
 * @param action - the action id
 * @returns the backend answer
 */
export async function runGeneralAction(action: GeneralActionId): Promise<ServoApiResult<ServoActionResult>> {
  let res: Response
  try {
    res = await fetch(`${BASE}/${action}`, { method: 'POST' })
  } catch (e) {
    return { ok: false, status: 0, message: e instanceof Error ? e.message : String(e) }
  }
  let body: unknown = null
  try {
    body = await res.json()
  } catch {
    body = null
  }
  const b = (body ?? {}) as { ok?: unknown; message?: unknown; reason?: unknown }
  const message = typeof b.message === 'string' ? b.message : typeof b.reason === 'string' ? b.reason : `HTTP ${res.status}`
  if (!res.ok) return { ok: false, status: res.status, message }
  if (typeof b.message !== 'string') return { ok: false, status: res.status, message: 'Unexpected response from the robot' }
  return { ok: true, value: { ok: b.ok === true, message } }
}

type StopResult = { ok: true } | { ok: false; message: string }
/** One stoppable kind of work: `run` reads its state and says whether it is live (and, optionally, a more precise name). */
type Probe = { live: boolean; stop: () => Promise<StopResult>; name?: string }
type Stopper = { name: string; run: () => Promise<Probe | { error: string }> }

const done = <T,>(r: { ok: true; value: T } | { ok: false; message: string }): StopResult =>
  r.ok ? { ok: true } : { ok: false, message: r.message }

const STOPPERS: Stopper[] = [
  {
    name: 'arm-camera capture',
    run: async () => {
      const s = await getHeadArmState()
      if (!s.ok) return { error: s.message }
      const seq = s.value.sequence
      // The node never interrupts a Home: a homing sequence is not stopped.
      const live = (seq.running && seq.phase !== HOMING_PHASE) || WAITING_PHASES.includes(seq.phase)
      return { live, stop: async () => done(await stopSequence()) }
    },
  },
  {
    name: 'floor sweep',
    run: async () => {
      const s = await getFloorState()
      return s.ok ? { live: s.value.sweep.running, stop: async () => done(await stopSweep()) } : { error: s.message }
    },
  },
  {
    name: 'ground board session',
    run: async () => {
      const s = await getGroundState()
      return s.ok ? { live: s.value.sweep.running, stop: async () => done(await stopGroundSweep()) } : { error: s.message }
    },
  },
  {
    name: 'ground verification',
    run: async () => {
      const s = await getGroundState()
      return s.ok ? { live: s.value.verify.running, stop: async () => done(await stopGroundVerify()) } : { error: s.message }
    },
  },
  {
    name: 'D435 capture',
    run: async () => {
      const s = await getState()
      return s.ok ? { live: s.value.capture.running, stop: async () => done(await stopCapture()) } : { error: s.message }
    },
  },
  {
    name: 'CSI capture',
    run: async () => {
      const s = await getCsiState(CSI_STATE_CAMERA)
      if (!s.ok) return { error: s.message }
      const camera = s.value.capture.camera ?? CSI_STATE_CAMERA
      return { live: s.value.capture.running, name: `${camera} camera capture`, stop: async () => done(await stopCsiCapture(camera)) }
    },
  },
]

/**
 * Abort calibration: stop every running sequence, sweep (floor and ground board), ground verification and capture. The head-arm sequence stop HOLDS the arm where it
 * is (no Home move). A state that cannot be read is reported and the others are still stopped.
 *
 * @returns what was stopped and the errors
 */
export async function abortCalibration(): Promise<AbortOutcome> {
  const outcome: AbortOutcome = { stopped: [], errors: [] }
  await Promise.all(
    STOPPERS.map(async (s) => {
      const probe = await s.run()
      if ('error' in probe) {
        outcome.errors.push(`${s.name}: ${probe.error}`)
        return
      }
      if (!probe.live) return
      const name = probe.name ?? s.name
      const r = await probe.stop()
      if (r.ok) outcome.stopped.push(name)
      else outcome.errors.push(`${name}: ${r.message}`)
    }),
  )
  return outcome
}
