/**
 * Test data of the camera to ground (ground board) flow, built from the node's contract fixtures
 * (nodes/head_rgbd_calibration/tests/fixtures/api/ground_board_*.json) and never copied: a contract change on the node
 * shows up here. The node fixtures hold one element per list, so the variants below add elements of the same shape:
 * the 95 % twin of the accuracy example, the gauge-held head mount yaw and a failed verification pose.
 * Imported by tests only.
 */
import acceptJson from '../../../../head_rgbd_calibration/tests/fixtures/api/ground_board_accept.json'
import resultJson from '../../../../head_rgbd_calibration/tests/fixtures/api/ground_board_result.json'
import implausibleJson from '../../../../head_rgbd_calibration/tests/fixtures/api/ground_board_result_implausible.json'
import runningJson from '../../../../head_rgbd_calibration/tests/fixtures/api/ground_board_result_running.json'
import sessionsJson from '../../../../head_rgbd_calibration/tests/fixtures/api/ground_board_sessions.json'
import stateJson from '../../../../head_rgbd_calibration/tests/fixtures/api/ground_board_state.json'
import sweepJson from '../../../../head_rgbd_calibration/tests/fixtures/api/ground_board_sweep_state.json'
import verifyJson from '../../../../head_rgbd_calibration/tests/fixtures/api/ground_board_verify_state.json'
import type {
  GroundAccept,
  GroundReport,
  GroundResult,
  GroundSession,
  GroundState,
  GroundSweep,
  GroundVerifyState,
} from './groundBoardApi'

/** The node's fixtures as they are. */
export const nodeAccept = acceptJson as GroundAccept
export const nodeResult = resultJson as GroundResult
export const nodeImplausible = implausibleJson as GroundResult
export const nodeRunning = runningJson as GroundResult
export const nodeSessions = sessionsJson as GroundSession[]
export const nodeState = stateJson as GroundState
export const nodeSweep = sweepJson as GroundSweep
export const nodeVerify = verifyJson as GroundVerifyState

const nodeReport = nodeResult.report as GroundReport
const example = nodeReport.accuracy_examples[0]

/** A plausible done result (no refusals): the accuracy example with its 95 % twin, the gauge-held mount yaw. */
export const doneResult: GroundResult = {
  ...nodeResult,
  report: {
    ...nodeReport,
    accuracy_examples: [example, { ...example, label: example.label.replace(', 1 sigma', ', 95 %'), value: 0.25 }],
    params: [
      ...nodeReport.params,
      { name: 'mount_yaw_deg', value: 0, sigma: null, unit: 'deg', applied_value: 0, change: 0 },
      { name: 'tilt_zero_deg', value: 1.2, sigma: 0.01, unit: 'deg', applied_value: 0.6, change: 0.6 },
    ],
  },
}

/** A done result the node refuses: not plausible, with the node's refusal. */
export const refusedResult: GroundResult = nodeImplausible

/** No session, no verification, no compute: a fresh node. */
export const idleSweep: GroundSweep = {
  ...nodeSweep,
  running: false,
  status: 'idle',
  session_id: null,
  placement: 0,
  placements_done: 0,
  awaiting_placement: false,
  pose: 0,
  total: 0,
  views: 0,
  planes: 0,
  skipped: 0,
  board: null,
  message: null,
}

/** A session capturing its first placement. */
export const sweepingSweep: GroundSweep = {
  ...nodeSweep,
  status: 'sweeping',
  placement: 0,
  placements_done: 0,
  awaiting_placement: false,
  pose: 4,
  total: 10,
  views: 11,
  planes: 8,
  message: null,
}

/** A session that ended after its placements. */
export const doneSweep: GroundSweep = {
  ...nodeSweep,
  running: false,
  status: 'done',
  placements_done: 2,
  placement: 2,
  awaiting_placement: false,
  message: null,
}

export const idleVerify: GroundVerifyState = {
  ...nodeVerify,
  running: false,
  status: 'idle',
  id: null,
  head_chain: null,
  result: null,
  pose: 0,
  total: 0,
  poses: [],
  passed: null,
}

const verifyPose = nodeVerify.poses[0]

/** A verification with a failed pose: the plane 15 mm above the floor at pan 40 / tilt 20. */
export const failedVerify: GroundVerifyState = {
  ...nodeVerify,
  passed: false,
  poses: [
    verifyPose,
    {
      ...verifyPose,
      pose: 1,
      pan_deg: 40,
      tilt_deg: 20,
      height_m: 0.015,
      checks: [{ name: 'height_m', value: 0.015, limit: 0.01, ok: false }],
      passed: false,
    },
  ],
}

/** The state of a fresh node: the URDF head chain in effect, nothing accepted. */
export const idleState: GroundState = {
  ...nodeState,
  sweep: idleSweep,
  verify: idleVerify,
  compute: { running: false, session_id: null, result_id: null, started_at: null, error: null },
  head_chain: { source: 'urdf', session: null, result: null },
  accepted: null,
}
