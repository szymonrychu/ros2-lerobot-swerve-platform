import { describe, expect, it } from 'vitest'
import {
  ADVANCED_PARAMS,
  armPointToBaseLink,
  baseLinkPointToArm,
  buildGraspRequest,
  canExecute,
  convertFormFrame,
  DEFAULT_ARM_OFFSET,
  DEFAULT_FORM,
  describeOutcome,
  formFromPick,
  GraspForm,
  mapToBaseLink,
  objectBox,
  parseGraspResponse,
  parseGraspStream,
  planKey,
  previewObject,
  summarizePlan,
  waypointScenePoints,
} from './grasp'

const MOUNT: [number, number, number] = [0.15, -0.04, 0.15]
const FORM: GraspForm = { ...DEFAULT_FORM, x: '0.4', y: '0.05', supportZ: '0' }

function ok(form: GraspForm, action: 'plan' | 'execute' | 'release' = 'plan') {
  const r = buildGraspRequest(action, form)
  if (!r.ok) throw new Error(r.errors.join('; '))
  return r.request
}

describe('buildGraspRequest', () => {
  it('builds a plan request with the contract field names', () => {
    expect(ok(FORM)).toEqual({
      action: 'plan',
      object: { frame: 'base_link', x: 0.4, y: 0.05, support_z: 0, width_m: 0.03, depth_m: 0.03, height_m: 0.04 },
      strategy: 'auto',
    })
  })

  it('converts the yaw from degrees to radians and omits a blank yaw', () => {
    expect(ok({ ...FORM, yaw: '90' }).object?.yaw).toBeCloseTo(Math.PI / 2)
    expect(ok({ ...FORM, yaw: ' ' }).object).not.toHaveProperty('yaw')
  })

  it('sends the angled pitch only for the angled strategy', () => {
    expect(ok({ ...FORM, strategy: 'angled', pitchDeg: '30' }).approach_pitch_deg).toBe(30)
    expect(ok({ ...FORM, strategy: 'angled', pitchDeg: '' })).not.toHaveProperty('approach_pitch_deg')
    expect(ok({ ...FORM, strategy: 'scoop', pitchDeg: '30' })).not.toHaveProperty('approach_pitch_deg')
  })

  it('sends only filled advanced params, as numbers', () => {
    const req = ok({ ...FORM, params: { lift_height_m: '0.08', approach_distance_m: '', slide_speed_scale: ' ' } })
    expect(req.params).toEqual({ lift_height_m: 0.08 })
    expect(ok(FORM)).not.toHaveProperty('params')
  })

  it('sends the surface height and tilt override when filled; a half-filled tilt uses 0 for the blank axis', () => {
    expect(ok({ ...FORM, surfaceZ: '0.12' }).surface_z_m).toBe(0.12)
    expect(ok({ ...FORM, tiltRoll: '3', tiltPitch: '' }).tilt_override_deg).toEqual({ roll: 3, pitch: 0 })
    expect(ok(FORM)).not.toHaveProperty('tilt_override_deg')
  })

  it('release carries no object or strategy, only params and overrides', () => {
    const req = ok({ ...FORM, params: { release_lift_m: '0.07' }, surfaceZ: '0.1' }, 'release')
    expect(req).toEqual({ action: 'release', params: { release_lift_m: 0.07 }, surface_z_m: 0.1 })
  })

  it('release does not need a valid object', () => {
    expect(buildGraspRequest('release', DEFAULT_FORM).ok).toBe(true)
  })

  it.each([
    ['blank x', { x: '' }, 'x'],
    ['non-numeric y', { y: 'abc' }, 'y'],
    ['blank support height', { supportZ: '' }, 'support'],
    ['zero width', { width: '0' }, 'width'],
    ['negative depth', { depth: '-0.01' }, 'depth'],
    ['height above 0.5 m', { height: '0.6' }, 'height'],
    ['non-numeric yaw', { yaw: 'x' }, 'yaw'],
    ['angled pitch above 90', { strategy: 'angled', pitchDeg: '95' }, 'pitch'],
    ['surface height above 1 m', { surfaceZ: '2' }, 'surface'],
    ['tilt above 45 deg', { tiltRoll: '50' }, 'tilt'],
    ['non-numeric tilt', { tiltPitch: 'q' }, 'tilt'],
    ['non-numeric param', { params: { lift_height_m: 'x' } }, 'lift_height_m'],
    ['param below its minimum', { params: { lift_height_m: '-1' } }, 'lift_height_m'],
    ['param above its maximum', { params: { slide_speed_scale: '5' } }, 'slide_speed_scale'],
  ] as [string, Partial<GraspForm>, string][])('rejects %s', (_name, patch, word) => {
    const r = buildGraspRequest('plan', { ...FORM, ...patch })
    expect(r.ok).toBe(false)
    if (!r.ok) expect(r.errors.join(' ').toLowerCase()).toContain(word.toLowerCase())
  })

  it('rejects an invalid param for release too (it uses them)', () => {
    expect(buildGraspRequest('release', { ...FORM, params: { release_lift_m: 'x' } }).ok).toBe(false)
  })

  it('collects every error, not only the first', () => {
    const r = buildGraspRequest('plan', { ...FORM, x: '', width: '0' })
    expect(r.ok).toBe(false)
    if (!r.ok) expect(r.errors.length).toBe(2)
  })
})

describe('ADVANCED_PARAMS', () => {
  it('lists unique grasp config keys with sane bounds', () => {
    const keys = ADVANCED_PARAMS.map((p) => p.key)
    expect(new Set(keys).size).toBe(keys.length)
    for (const k of ['approach_distance_m', 'lift_height_m', 'slide_speed_scale', 'angled_pitch_deg', 'release_open_fraction']) {
      expect(keys).toContain(k)
    }
    expect(keys).not.toContain('auto_order')
    for (const p of ADVANCED_PARAMS) expect(p.min).toBeLessThan(p.max)
  })
})

describe('plan key and execute gating', () => {
  const planned = (form: GraspForm, feasible = true) => ({
    key: planKey(ok(form)),
    outcome: parseGraspResponse(
      { ok: true, request_id: 'r', action: 'plan', result: { outcome: feasible ? 'planned' : 'infeasible', reasons: [], plan: { strategy: 'scoop', feasible, waypoints: [] } } },
      200,
    ),
  })

  it('plan key ignores the action but not the inputs', () => {
    expect(planKey(ok(FORM, 'plan'))).toBe(planKey(ok(FORM, 'execute')))
    expect(planKey(ok(FORM))).not.toBe(planKey(ok({ ...FORM, x: '0.41' })))
    expect(planKey(ok(FORM))).not.toBe(planKey(ok({ ...FORM, strategy: 'scoop' })))
    expect(planKey(ok(FORM))).not.toBe(planKey(ok({ ...FORM, params: { lift_height_m: '0.1' } })))
  })

  it('execute needs a feasible plan for exactly the current inputs', () => {
    expect(canExecute(planned(FORM), FORM)).toBe(true)
    expect(canExecute(planned(FORM), { ...FORM, y: '0.06' })).toBe(false)
    expect(canExecute(planned(FORM, false), FORM)).toBe(false)
    expect(canExecute(null, FORM)).toBe(false)
    expect(canExecute(planned(FORM), { ...FORM, x: '' })).toBe(false)
  })

  it('whitespace-equivalent inputs keep the plan valid', () => {
    expect(canExecute(planned(FORM), { ...FORM, x: ' 0.40 ' })).toBe(true)
  })
})

describe('frames', () => {
  it('arm -> base_link adds the mount (yaw 0)', () => {
    expect(armPointToBaseLink({ x: 0.2, y: 0.1, z: -0.15 }, MOUNT)).toEqual({
      x: expect.closeTo(0.35), y: expect.closeTo(0.06), z: expect.closeTo(0),
    })
  })

  it('base_link -> arm subtracts the mount', () => {
    const p = baseLinkPointToArm({ x: 0.35, y: 0.06, z: 0 }, MOUNT)
    expect(p.x).toBeCloseTo(0.2)
    expect(p.y).toBeCloseTo(0.1)
    expect(p.z).toBeCloseTo(-0.15)
  })

  it('tabs without an arm_offset keep the legacy default mount', () => {
    expect(DEFAULT_ARM_OFFSET).toEqual([0.25, 0, 0])
  })

  it('map point to base_link uses the robot pose', () => {
    const p = mapToBaseLink({ x: 3, y: 2 }, { x: 2, y: 2, yaw: Math.PI / 2 })
    expect(p.x).toBeCloseTo(0)
    expect(p.y).toBeCloseTo(-1)
  })

  it('switching the frame keeps the physical object in place', () => {
    const arm = convertFormFrame({ ...FORM, x: '0.35', y: '0.06', supportZ: '0' }, 'arm', MOUNT)
    expect(arm.frame).toBe('arm')
    expect(Number(arm.x)).toBeCloseTo(0.2)
    expect(Number(arm.y)).toBeCloseTo(0.1)
    expect(Number(arm.supportZ)).toBeCloseTo(-0.15)
    const back = convertFormFrame(arm, 'base_link', MOUNT)
    expect(Number(back.x)).toBeCloseTo(0.35)
    expect(Number(back.supportZ)).toBeCloseTo(0)
  })

  it('switching the frame leaves blank or invalid fields alone', () => {
    const f = convertFormFrame({ ...FORM, x: '', supportZ: 'abc' }, 'arm', MOUNT)
    expect(f.x).toBe('')
    expect(f.supportZ).toBe('abc')
    expect(convertFormFrame(FORM, 'base_link', MOUNT)).toEqual(FORM)
  })

  it('a map pick fills x, y in base_link (converting the support height when coming from arm)', () => {
    const f = formFromPick({ ...DEFAULT_FORM, frame: 'arm', supportZ: '-0.15' }, { x: 0.5, y: -0.1 }, MOUNT)
    expect(f.frame).toBe('base_link')
    expect(f.x).toBe('0.500')
    expect(f.y).toBe('-0.100')
    expect(Number(f.supportZ)).toBeCloseTo(0)
  })
})

describe('scene transforms', () => {
  const wp = (label: string, x: number, y: number, z: number) => ({
    label, x, y, z, pitch: 0, roll: 0, gripper: null, gripper_action: 'keep', speed_scale: 0.2, linear: false,
  })

  it('waypoints become base_link scene positions through the mount', () => {
    const pts = waypointScenePoints([wp('pre_grasp', 0.2, 0.1, -0.1), wp('grasp', 0.25, 0, -0.14)], MOUNT)
    expect(pts.map((p) => p.label)).toEqual(['pre_grasp', 'grasp'])
    // ros base_link (0.35, 0.06, 0.05) -> three (x, z, -y)
    expect(pts[0].position[0]).toBeCloseTo(0.35)
    expect(pts[0].position[1]).toBeCloseTo(0.05)
    expect(pts[0].position[2]).toBeCloseTo(-0.06)
  })

  it('no waypoints, no points', () => {
    expect(waypointScenePoints([], MOUNT)).toEqual([])
  })

  it('object box in base_link: centre at support + half height, size width/height/depth', () => {
    const box = objectBox({ frame: 'base_link', x: 0.4, y: 0.1, support_z: 0, width_m: 0.03, depth_m: 0.05, height_m: 0.04, yaw: 0.5 }, MOUNT)
    expect(box.position[0]).toBeCloseTo(0.4)
    expect(box.position[1]).toBeCloseTo(0.02)
    expect(box.position[2]).toBeCloseTo(-0.1)
    expect(box.size).toEqual([0.03, 0.04, 0.05])
    expect(box.rotationY).toBeCloseTo(0.5)
  })

  it('object box in the arm frame is shifted by the mount', () => {
    const box = objectBox({ frame: 'arm', x: 0.2, y: 0.1, support_z: -0.15, width_m: 0.03, depth_m: 0.03, height_m: 0.04 }, MOUNT)
    expect(box.position[0]).toBeCloseTo(0.35)
    expect(box.position[1]).toBeCloseTo(0.02)
    expect(box.position[2]).toBeCloseTo(-0.06)
  })

  it('without a yaw the width axis lies across the approach from the arm base', () => {
    // object straight ahead of the arm (arm y = 0): approach along +x, width axis along +y (pi/2)
    const ahead = objectBox({ frame: 'arm', x: 0.2, y: 0, support_z: -0.15, width_m: 0.03, depth_m: 0.03, height_m: 0.04 }, MOUNT)
    expect(ahead.rotationY).toBeCloseTo(Math.PI / 2)
  })

  it('previewObject is null until the object fields are valid', () => {
    expect(previewObject(DEFAULT_FORM)).toBeNull()
    expect(previewObject(FORM)?.x).toBe(0.4)
  })
})

describe('response parsing', () => {
  it('parses a plan answer', () => {
    const out = parseGraspResponse(
      {
        ok: true, request_id: 'r1', action: 'plan',
        result: {
          outcome: 'planned', reasons: ['note'],
          plan: {
            strategy: 'angled', feasible: true, reasons: [], approach_pitch_deg: 45, wrist_roll_rad: -1.57, opening_m: 0.05, skim: false,
            waypoints: [{ label: 'pre_grasp', x: 0.2, y: 0, z: -0.1, pitch: 0.7, roll: -1.57, gripper: 1.2, gripper_action: 'set', speed_scale: 0.5, linear: false }],
            slow_zone: [], attempts: [{ strategy: 'scoop', feasible: false, reasons: ['unreachable'] }],
          },
        },
      },
      200,
    )
    expect(out).toMatchObject({ ok: true, accepted: false, requestId: 'r1', action: 'plan', outcome: 'planned', reasons: ['note'] })
    expect(out.plan?.strategy).toBe('angled')
    expect(out.plan?.waypoints).toHaveLength(1)
    expect(out.plan?.attempts).toHaveLength(1)
  })

  it('parses an infeasible plan with reasons', () => {
    const out = parseGraspResponse(
      { ok: true, request_id: 'r', action: 'plan', result: { outcome: 'infeasible', reasons: ['out of reach'], plan: { strategy: 'scoop', feasible: false, reasons: ['out of reach'], waypoints: [] } } },
      200,
    )
    expect(out.outcome).toBe('infeasible')
    expect(out.plan?.feasible).toBe(false)
  })

  it('parses an execute result with steps and gripper readings', () => {
    const out = parseGraspResponse(
      { ok: true, request_id: 'r', action: 'execute', result: { outcome: 'grasped', reasons: [], steps: [{ label: 'close', status: 'grasped', message: 'ok', slow_zone: null }], gripper_position_rad: 0.21, gripper_effort: 350 } },
      200,
    )
    expect(out.steps).toEqual([{ label: 'close', status: 'grasped', message: 'ok' }])
    expect(out.gripperPositionRad).toBe(0.21)
    expect(out.gripperEffort).toBe(350)
  })

  it('parses an error answer', () => {
    const out = parseGraspResponse({ ok: false, request_id: 'r', action: 'execute', error: 'battery below cut-off' }, 400)
    expect(out).toMatchObject({ ok: false, error: 'battery below cut-off', action: 'execute' })
  })

  it('parses the backend action_response error shape ({ok, message})', () => {
    expect(parseGraspResponse({ ok: false, message: 'mcp_server is not running' }, 503).error).toBe('mcp_server is not running')
  })

  it('parses the accepted answer of an execute', () => {
    const out = parseGraspResponse({ ok: true, accepted: true, request_id: 'r', action: 'execute', message: 'started' }, 202)
    expect(out).toMatchObject({ ok: true, accepted: true, requestId: 'r', action: 'execute' })
  })

  it('survives garbage', () => {
    expect(parseGraspResponse(null, 500)).toMatchObject({ ok: false, error: 'HTTP 500' })
    expect(parseGraspResponse('x', 200).ok).toBe(false)
    expect(parseGraspResponse({ ok: true, result: { outcome: 'weird' } }, 200).outcome).toBeUndefined()
  })
})

describe('stream parsing', () => {
  it('parses running and done states', () => {
    expect(parseGraspStream({ request_id: 'r', action: 'execute', state: 'running' })).toEqual({ requestId: 'r', action: 'execute', state: 'running', outcome: null })
    const done = parseGraspStream({ ok: true, request_id: 'r', action: 'execute', state: 'done', result: { outcome: 'missed', reasons: ['nothing in the jaws'], steps: [] } })
    expect(done?.state).toBe('done')
    expect(done?.outcome?.outcome).toBe('missed')
  })

  it('ignores anything else', () => {
    expect(parseGraspStream(undefined)).toBeNull()
    expect(parseGraspStream(null)).toBeNull()
    expect(parseGraspStream({ state: 'weird', request_id: 'r' })).toBeNull()
    expect(parseGraspStream({ state: 'running' })).toBeNull()
  })
})

describe('text', () => {
  it('describes outcomes with a severity', () => {
    expect(describeOutcome('grasped')).toEqual({ text: 'Grasped: holding the object', severity: 'success' })
    expect(describeOutcome('planned').severity).toBe('success')
    expect(describeOutcome('released').severity).toBe('success')
    expect(describeOutcome('missed').severity).toBe('warning')
    expect(describeOutcome('infeasible').severity).toBe('error')
    expect(describeOutcome('aborted').severity).toBe('error')
  })

  it('summarizes a plan for the confirm step', () => {
    const lines = summarizePlan({
      strategy: 'angled', feasible: true, reasons: [], approachPitchDeg: 45, wristRollRad: -1.5708, openingM: 0.05, skim: false,
      waypoints: [], slowZone: [{ label: 'grasp' }], attempts: [],
    })
    expect(lines).toContain('Strategy: angled')
    expect(lines).toContain('Approach pitch: 45 deg')
    expect(lines).toContain('Wrist roll: -90 deg')
    expect(lines).toContain('Jaw opening: 50 mm')
    expect(lines.join(' ')).toContain('slow zone')
  })
})
