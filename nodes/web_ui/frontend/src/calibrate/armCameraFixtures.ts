/**
 * Test data of the arm-to-camera flow, built from the node's contract fixtures
 * (nodes/head_rgbd_calibration/tests/fixtures/api/arm_camera_*.json) and never copied: a contract change on the node
 * shows up here. The node fixtures hold one element per list (its checker types every element like the first), so the
 * variants below add elements of the same shape: the 95 % twin of the accuracy example and a gauge-held parameter.
 * Imported by tests only.
 */
import resultJson from '../../../../head_rgbd_calibration/tests/fixtures/api/arm_camera_result.json'
import runningJson from '../../../../head_rgbd_calibration/tests/fixtures/api/arm_camera_result_running.json'
import stateJson from '../../../../head_rgbd_calibration/tests/fixtures/api/arm_camera_state.json'
import acceptJson from '../../../../head_rgbd_calibration/tests/fixtures/api/arm_camera_accept.json'
import type { ArmCameraAccept, ArmCameraReport, ArmCameraResult, ArmCameraState } from './armCameraApi'

/** The node's fixtures as they are. */
export const nodeResult = resultJson as ArmCameraResult
export const nodeRunning = runningJson as ArmCameraResult
export const nodeState = stateJson as ArmCameraState
export const nodeAccept = acceptJson as ArmCameraAccept

const nodeReport = nodeResult.report as ArmCameraReport
const example = nodeReport.accuracy_examples[0]

/** A plausible done result: no refusals, the accuracy example with its 95 % twin, a gauge-held base_yaw. */
export const doneResult: ArmCameraResult = {
  ...nodeResult,
  report: {
    ...nodeReport,
    plausible: true,
    refusals: [],
    accuracy_examples: [example, { ...example, label: example.label.replace(', 1 sigma', ', 95 %'), value: 0.42 }],
    params: [
      ...nodeReport.params,
      { name: 'base_yaw', value: 0, sigma: null, held: true, unit: 'deg', applied_value: 0, change: 0 },
    ],
  },
}

/** A done result the node refuses: not plausible, with the node's refusal (the node fixture as it is). */
export const refusedResult: ArmCameraResult = { ...nodeResult, id: 'r-20261109-121000' }

/** The state with an applied calibration fitted against a head chain that has changed since. */
export const headChainChangedState: ArmCameraState = {
  ...nodeState,
  applied: { ...(nodeState.applied as NonNullable<ArmCameraState['applied']>), head_chain_changed: true },
}
