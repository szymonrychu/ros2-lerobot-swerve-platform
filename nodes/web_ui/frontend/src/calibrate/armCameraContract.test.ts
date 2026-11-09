/**
 * Pins the arm-to-camera client to the node's contract fixtures: every fixture passes the client's runtime checks and
 * every object the client checks has exactly the fixture's keys, so a key added, renamed or dropped on either side
 * fails here.
 */
import { describe, expect, it } from 'vitest'
import { ARM_CAMERA_SHAPES, badAccept, badResult, badState } from './armCameraApi'
import resultFx from '../../../../head_rgbd_calibration/tests/fixtures/api/arm_camera_result.json'
import runningFx from '../../../../head_rgbd_calibration/tests/fixtures/api/arm_camera_result_running.json'
import stateFx from '../../../../head_rgbd_calibration/tests/fixtures/api/arm_camera_state.json'
import acceptFx from '../../../../head_rgbd_calibration/tests/fixtures/api/arm_camera_accept.json'

const keys = (v: object) => Object.keys(v).sort()
const report = resultFx.report

describe('arm-camera client against the node contract fixtures', () => {
  it('accepts every node fixture', () => {
    expect(badResult(resultFx)).toBeNull()
    expect(badResult(runningFx)).toBeNull()
    expect(badState(stateFx)).toBeNull()
    expect(badAccept(acceptFx)).toBeNull()
  })

  it.each([
    ['result', resultFx],
    ['result', runningFx],
    ['report', report],
    ['score', report.score],
    ['param', report.params[0]],
    ['check', report.refusals[0]],
    ['check', report.warnings[0]],
    ['example', report.accuracy_examples[0]],
    ['comparison', report.comparison],
    ['currentScore', report.comparison.current_score],
    ['example', report.comparison.shift_examples[0]],
    ['skew', report.skew],
    ['headChain', resultFx.head_chain],
    ['state', stateFx],
    ['applied', stateFx.applied],
    ['score', stateFx.applied.score],
    ['example', stateFx.applied.accuracy_examples[0]],
    ['accept', acceptFx],
    ['acceptRecord', acceptFx.arm_calibration],
    ['score', acceptFx.arm_calibration.score],
    ['unit', acceptFx.units[0]],
  ])('the %s shape has exactly the fixture keys', (shape, fixture) => {
    expect(keys(ARM_CAMERA_SHAPES[shape])).toEqual(keys(fixture))
  })

  it('names the first bad key of a body that drifted', () => {
    expect(badResult({ ...resultFx, report: { ...report, notes: undefined } })).toBe('report.notes')
    expect(badResult({ ...resultFx, report: { ...report, refusals: ['a plain string'] } })).toBe('report.refusals')
    expect(badState({ ...stateFx, applied: { ...stateFx.applied, head_chain_changed: undefined } })).toBe(
      'applied.head_chain_changed',
    )
    expect(badAccept({ units: [] })).toBe('arm_calibration')
  })
})
