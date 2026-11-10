import { describe, expect, it } from 'vitest'
import { DEFAULT_ARM_OFFSET } from '../grasp/grasp'
import { armScenePlacement, configuredArmCalibration, effectiveArmCalibration, toMeasuredPositions, toUrdfJointStates } from './armCalibration'
import type { AcceptedArmCalibration } from './armCalibrationApi'

const OFFSETS = { shoulder_pan: 0.03, shoulder_lift: -0.02, elbow_flex: -0.09, wrist_flex: 0.08, wrist_roll: -0.07 }
const ACCEPTED: AcceptedArmCalibration = {
  session: 'calib-20261109-120000',
  result: 'r-1',
  accepted_at: '2026-11-09T12:20:00Z',
  joint_offsets_rad: OFFSETS,
  base_in_base_link: { x: 0.061, y: -0.048, z: 0.102, yaw: 0.01 },
}

describe('configuredArmCalibration', () => {
  it('is the tab arm_offset with no yaw and no joint offsets', () => {
    expect(configuredArmCalibration([0.05, -0.04, 0.1])).toEqual({ mount: [0.05, -0.04, 0.1, 0], jointOffsets: {}, source: 'config', accepted: null })
  })

  it('uses the measured default mount without an arm_offset', () => {
    expect(configuredArmCalibration().mount).toEqual([...DEFAULT_ARM_OFFSET, 0])
  })
})

describe('effectiveArmCalibration', () => {
  it('nothing accepted draws with the configured values', () => {
    expect(effectiveArmCalibration([0.05, -0.04, 0.1], null)).toEqual(configuredArmCalibration([0.05, -0.04, 0.1]))
  })

  it('an accepted calibration replaces the mount (with its yaw) and brings its joint offsets', () => {
    const c = effectiveArmCalibration([0.05, -0.04, 0.1], ACCEPTED)
    expect(c.mount).toEqual([0.061, -0.048, 0.102, 0.01])
    expect(c.jointOffsets).toEqual(OFFSETS)
    expect(c.source).toBe('accepted')
    expect(c.accepted).toEqual({ session: 'calib-20261109-120000', result: 'r-1', accepted_at: '2026-11-09T12:20:00Z' })
  })

  it('copies the offsets (the view never shares the API object)', () => {
    expect(effectiveArmCalibration(undefined, ACCEPTED).jointOffsets).not.toBe(ACCEPTED.joint_offsets_rad)
  })
})

describe('toUrdfJointStates', () => {
  const measured = { name: ['shoulder_pan', 'elbow_flex', 'gripper'], position: [0.1, 1.0, 0.5] }

  it('adds each joint offset, the gripper has none', () => {
    const urdf = toUrdfJointStates(measured, OFFSETS)
    expect(urdf?.name).toEqual(measured.name)
    expect(urdf?.position?.[0]).toBeCloseTo(0.13)
    expect(urdf?.position?.[1]).toBeCloseTo(0.91)
    expect(urdf?.position?.[2]).toBe(0.5)
    expect(measured.position).toEqual([0.1, 1.0, 0.5])
  })

  it('returns the same object without offsets or without a message', () => {
    expect(toUrdfJointStates(measured, {})).toBe(measured)
    expect(toUrdfJointStates(undefined, OFFSETS)).toBeUndefined()
    const partial = { name: ['a'] }
    expect(toUrdfJointStates(partial, OFFSETS)).toBe(partial)
  })

  it('a position without a name passes through', () => {
    expect(toUrdfJointStates({ name: ['shoulder_pan'], position: [0.1, 0.2] }, OFFSETS)?.position?.[1]).toBe(0.2)
  })
})

describe('toMeasuredPositions', () => {
  it('subtracts each joint offset in the names order, a missing joint is 0', () => {
    const out = toMeasuredPositions(['elbow_flex', 'gripper', 'shoulder_pan'], { elbow_flex: 0.91, gripper: 0.5 }, OFFSETS)
    expect(out[0]).toBeCloseTo(1.0)
    expect(out[1]).toBe(0.5)
    expect(out[2]).toBeCloseTo(-0.03)
  })

  it('round-trips toUrdfJointStates', () => {
    const measured = { name: ['shoulder_pan', 'wrist_roll'], position: [0.2, -1.2] }
    const urdf = toUrdfJointStates(measured, OFFSETS)!
    const byName = Object.fromEntries(urdf.name!.map((n, i) => [n, urdf.position![i]]))
    const back = toMeasuredPositions(measured.name, byName, OFFSETS)
    expect(back[0]).toBeCloseTo(0.2, 12)
    expect(back[1]).toBeCloseTo(-1.2, 12)
  })
})

describe('armScenePlacement', () => {
  it('places the arm root at the mount in three coordinates, turned by the mount yaw', () => {
    expect(armScenePlacement([0.061, -0.048, 0.102, 0.01])).toEqual({ position: [0.061, 0.102, 0.048], rotation: [0, 0.01, 0] })
  })

  it('a configured mount has no turn', () => {
    expect(armScenePlacement([0.0592, -0.05, 0.1])).toEqual({ position: [0.0592, 0.1, 0.05], rotation: [0, 0, 0] })
  })
})
