import { describe, expect, it } from 'vitest'
import type { ServoGroup, ServoGroupsState } from '../calibrate/servoApi'
import { armControlState, ArmControlInput, HELD_REASON, releasedJoints } from './armControls'

function servo(group: Partial<ServoGroup> | null, joints: Record<string, number> = {}): ServoGroupsState {
  const full: ServoGroup | null = group && {
    id: 'arm',
    label: 'Arm',
    neutral: true,
    state: { namespace: 'follower', released: false, joints, stamp_ns: 1, neutral_enabled: true, neutral_capture: null, last_save: null },
    age_s: 0.1,
    stale: false,
    calibration_stale: null,
    ...group,
  }
  return { groups: full ? [full] : [], notConfigured: !full, error: null, loading: false }
}

const OK: ArmControlInput = { released: false, releasedJoints: [], busy: false, armReady: true, activeSource: 'filter', graspBusy: false }

describe('releasedJoints', () => {
  it('lists the joints whose torque reading is 0', () => {
    expect(releasedJoints(servo({}, { shoulder_pan: 1, elbow_flex: 0, gripper: 0 }), 'arm')).toEqual(['elbow_flex', 'gripper'])
  })
  it('is empty for an enabled arm, a stale state, a missing state and an unknown group', () => {
    expect(releasedJoints(servo({}, { a: 1, b: 1 }), 'arm')).toEqual([])
    expect(releasedJoints(servo({ stale: true }, { a: 0 }), 'arm')).toEqual([])
    expect(releasedJoints(servo({ state: null }), 'arm')).toEqual([])
    expect(releasedJoints(servo(null), 'arm')).toEqual([])
    expect(releasedJoints(servo({}, { a: 0 }), 'head')).toEqual([])
  })
})

describe('armControlState', () => {
  it('enables everything with an empty reason when nothing blocks', () => {
    const s = armControlState(OK)
    for (const key of ['drag', 'armHome', 'setHome', 'grasp'] as const) expect(s[key]).toEqual({ disabled: false, reason: '' })
  })

  it('released wins for every control and names the joints', () => {
    const s = armControlState({ ...OK, released: true, releasedJoints: ['elbow_flex', 'gripper'], busy: true, activeSource: 'autonomy', armReady: false })
    for (const key of ['drag', 'armHome', 'setHome', 'grasp'] as const) {
      expect(s[key]).toEqual({ disabled: true, reason: 'Arm torque is off: elbow_flex, gripper released (Calibrate page)' })
    }
  })

  it('omits the joint list when no joint is known', () => {
    expect(armControlState({ ...OK, released: true }).armHome.reason).toBe('Arm torque is off (Calibrate page)')
  })

  it('busy disables home, set home and grasp but not drag', () => {
    const s = armControlState({ ...OK, busy: true })
    expect(s.drag.disabled).toBe(false)
    for (const key of ['armHome', 'setHome', 'grasp'] as const) expect(s[key]).toEqual({ disabled: true, reason: 'Arm action in flight' })
  })

  it('a running plan or execute disables only grasp', () => {
    const s = armControlState({ ...OK, graspBusy: true })
    expect(s.grasp).toEqual({ disabled: true, reason: 'Arm action in flight' })
    expect(s.armHome.disabled).toBe(false)
    expect(s.setHome.disabled).toBe(false)
    expect(s.drag.disabled).toBe(false)
  })

  it('the lease held by the agent or Take control disables only drag', () => {
    const s = armControlState({ ...OK, activeSource: 'autonomy' })
    expect(s.drag).toEqual({ disabled: true, reason: HELD_REASON })
    expect(HELD_REASON).toBe('Arm held by the agent or Take control - Release to drag')
    expect(s.armHome.disabled).toBe(false)
  })

  it('drag waits for the arm joint states', () => {
    const s = armControlState({ ...OK, armReady: false })
    expect(s.drag).toEqual({ disabled: true, reason: 'Waiting for arm joint states' })
    expect(s.grasp.disabled).toBe(false)
  })

  it('the lease reason comes before the waiting reason', () => {
    expect(armControlState({ ...OK, activeSource: 'autonomy', armReady: false }).drag.reason).toBe(HELD_REASON)
  })
})
