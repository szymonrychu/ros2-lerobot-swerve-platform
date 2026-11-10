/**
 * Pins the ground board client to the node's contract fixtures (nodes/head_rgbd_calibration/tests/fixtures/api/
 * ground_board_*.json): every fixture passes the client's runtime checks and every object the client checks has
 * exactly the fixture's keys, so a key added, renamed or dropped on either side fails here.
 */
import { describe, expect, it } from 'vitest'
import {
  GROUND_BOARD_SHAPES,
  badGroundAccept,
  badGroundResult,
  badGroundSessions,
  badGroundState,
  badGroundSweep,
  badGroundVerify,
} from './groundBoardApi'
import acceptFx from '../../../../head_rgbd_calibration/tests/fixtures/api/ground_board_accept.json'
import computeFx from '../../../../head_rgbd_calibration/tests/fixtures/api/ground_board_compute.json'
import resultFx from '../../../../head_rgbd_calibration/tests/fixtures/api/ground_board_result.json'
import implausibleFx from '../../../../head_rgbd_calibration/tests/fixtures/api/ground_board_result_implausible.json'
import runningFx from '../../../../head_rgbd_calibration/tests/fixtures/api/ground_board_result_running.json'
import sessionsFx from '../../../../head_rgbd_calibration/tests/fixtures/api/ground_board_sessions.json'
import stateFx from '../../../../head_rgbd_calibration/tests/fixtures/api/ground_board_state.json'
import sweepFx from '../../../../head_rgbd_calibration/tests/fixtures/api/ground_board_sweep_state.json'
import verifyFx from '../../../../head_rgbd_calibration/tests/fixtures/api/ground_board_verify_state.json'

const keys = (v: object) => Object.keys(v).sort()
const report = resultFx.report
const verifyPose = verifyFx.poses[0]
const { running: _running, ...storedVerify } = verifyFx

describe('ground board client against the node contract fixtures', () => {
  it('accepts every node fixture', () => {
    expect(badGroundState(stateFx)).toBeNull()
    expect(badGroundSweep(sweepFx)).toBeNull()
    expect(badGroundVerify(verifyFx)).toBeNull()
    expect(badGroundResult(resultFx)).toBeNull()
    expect(badGroundResult(implausibleFx)).toBeNull()
    expect(badGroundResult(runningFx)).toBeNull()
    expect(badGroundAccept(acceptFx)).toBeNull()
    expect(badGroundSessions(sessionsFx)).toBeNull()
  })

  it.each([
    ['state', stateFx],
    ['sweep', stateFx.sweep],
    ['sweep', sweepFx],
    ['board', sweepFx.board],
    ['verifyState', stateFx.verify],
    ['verifyState', verifyFx],
    ['verify', storedVerify],
    ['verifyPose', verifyPose],
    ['reached', verifyPose.reached],
    ['verifyCheck', verifyPose.checks[0]],
    ['headChain', verifyFx.head_chain],
    ['resultRef', verifyFx.result],
    ['job', stateFx.compute],
    ['headChain', stateFx.head_chain],
    ['accepted', stateFx.accepted],
    ['score', stateFx.accepted.score],
    ['example', stateFx.accepted.accuracy_examples[0]],
    ['compute', computeFx],
    ['result', resultFx],
    ['result', runningFx],
    ['result', implausibleFx],
    ['report', implausibleFx.report],
    ['headChain', resultFx.head_chain],
    ['report', report],
    ['check', implausibleFx.report.refusals[0]],
    ['check', report.warnings[0]],
    ['score', report.score],
    ['param', report.params[0]],
    ['placement', report.placements[0]],
    ['example', report.accuracy_examples[0]],
    ['comparison', report.comparison],
    ['currentScore', report.comparison.current_score],
    ['example', report.comparison.shift_examples[0]],
    ['crossCheck', report.cross_check],
    ['planeScore', report.cross_check.fitted],
    ['planeScore', report.cross_check.applied],
    ['pose', report.poses[0]],
    ['accept', acceptFx],
    ['acceptRecord', acceptFx.head_chain],
    ['unit', acceptFx.units[0]],
    ['session', sessionsFx[0]],
  ])('the %s shape has exactly the fixture keys', (shape, fixture) => {
    expect(keys(GROUND_BOARD_SHAPES[shape])).toEqual(keys(fixture))
  })

  it('has a done result fixture for each case, plausible exactly when it has no refusal', () => {
    expect([report.plausible, report.refusals]).toEqual([true, []])
    expect(implausibleFx.report.plausible).toBe(false)
    expect(implausibleFx.report.refusals.length).toBeGreaterThan(0)
  })

  it('names the first bad key of a body that drifted', () => {
    expect(badGroundResult({ ...resultFx, report: { ...report, cross_check: undefined } })).toBe('report.cross_check')
    expect(badGroundResult({ ...resultFx, kind: 'arm_camera' })).toBe('kind')
    expect(badGroundState({ ...stateFx, sweep: { ...stateFx.sweep, awaiting_placement: 'yes' } })).toBe(
      'sweep.awaiting_placement',
    )
    expect(badGroundState({ ...stateFx, accepted: { ...stateFx.accepted, verification: 'none' } })).toBe(
      'accepted.verification',
    )
    expect(badGroundVerify({ ...verifyFx, poses: [{ ...verifyPose, checks: [{ name: 'height_m' }] }] })).toBe('poses')
    expect(badGroundAccept({ units: [] })).toBe('head_chain')
  })
})
