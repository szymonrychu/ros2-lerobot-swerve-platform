// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import {
  acceptArmCamera,
  computeArmCamera,
  declineArmCamera,
  getArmCameraResult,
  getArmCameraState,
  pairAccuracy,
} from './armCameraApi'
import {
  doneResult as doneFx,
  nodeAccept,
  nodeResult,
  nodeRunning as runningFx,
  nodeState as stateFx,
  refusedResult as refusedFx,
} from './armCameraFixtures'

const computeFx = { result_id: 'r-20261109-120000' }

const B = '/api/calibration/rgbd/arm-camera'

function respond(status: number, body: unknown) {
  const fn = vi.fn(async (_url: string, _init?: RequestInit) => new Response(JSON.stringify(body), { status }))
  vi.stubGlobal('fetch', fn)
  return fn
}

afterEach(() => {
  vi.unstubAllGlobals()
})

describe('armCameraApi', () => {
  it('reads the state', async () => {
    const fetchMock = respond(200, stateFx)
    const r = await getArmCameraState()
    expect(fetchMock).toHaveBeenCalledWith(`${B}/state`, expect.objectContaining({ cache: 'no-store' }))
    expect(r).toEqual({ ok: true, value: stateFx })
  })

  it('rejects a state without ready', async () => {
    respond(200, { ...stateFx, ready: 'yes' })
    const r = await getArmCameraState()
    expect(r.ok).toBe(false)
    if (!r.ok) expect(r.message).toContain('ready')
  })

  it('starts a compute with POST and reads the result id (202)', async () => {
    const fetchMock = respond(202, computeFx)
    const r = await computeArmCamera('calib 1')
    expect(fetchMock).toHaveBeenCalledWith(`${B}/sessions/calib%201/compute`, expect.objectContaining({ method: 'POST' }))
    expect(r).toEqual({ ok: true, value: { result_id: 'r-20261109-120000' } })
  })

  it('reads a done result with its report', async () => {
    const fetchMock = respond(200, doneFx)
    const r = await getArmCameraResult('calib-20261109-113000', 'r-20261109-120000')
    expect(fetchMock).toHaveBeenCalledWith(
      `${B}/sessions/calib-20261109-113000/results/r-20261109-120000`,
      expect.objectContaining({ cache: 'no-store' }),
    )
    expect(r.ok && r.value.report?.score.heldout_tool_mm).toBe(nodeResult.report?.score.heldout_tool_mm)
  })

  it('reads a done result with a gauge-held parameter (sigma null) and null held-out scores', async () => {
    const report = doneFx.report!
    respond(200, { ...doneFx, report: { ...report, score: { ...report.score, heldout_rms_px: null, heldout_tool_mm: null } } })
    const r = await getArmCameraResult('s', 'r')
    expect(r.ok && r.value.report?.params.find((p) => p.name === 'base_yaw')?.sigma).toBeNull()
  })

  it('rejects a refusal that is a plain string', async () => {
    respond(200, { ...doneFx, report: { ...doneFx.report!, refusals: ['too far'] } })
    const r = await getArmCameraResult('s', 'r')
    expect(r.ok).toBe(false)
    if (!r.ok) expect(r.message).toContain('report.refusals')
  })

  it('reads a running result without a report', async () => {
    respond(200, runningFx)
    const r = await getArmCameraResult('s', 'r')
    expect(r.ok && r.value.state).toBe('running')
  })

  it('reads a refused report', async () => {
    respond(200, refusedFx)
    const r = await getArmCameraResult('s', 'r')
    expect(r.ok && r.value.report?.plausible).toBe(false)
  })

  it('rejects a done result whose report misses the score', async () => {
    respond(200, { ...doneFx, report: { ...doneFx.report!, score: undefined } })
    const r = await getArmCameraResult('s', 'r')
    expect(r.ok).toBe(false)
    if (!r.ok) expect(r.message).toContain('report.score')
  })

  it('rejects an unknown result state', async () => {
    respond(200, { ...doneFx, state: 'queued' })
    expect((await getArmCameraResult('s', 'r')).ok).toBe(false)
  })

  it('rejects a param without sigma', async () => {
    const params = [{ ...doneFx.report!.params[0], sigma: undefined }]
    respond(200, { ...doneFx, report: { ...doneFx.report!, params } })
    expect((await getArmCameraResult('s', 'r')).ok).toBe(false)
  })

  it('accepts with POST and reads the applied record and the units', async () => {
    const fetchMock = respond(200, nodeAccept)
    const r = await acceptArmCamera('s', 'r')
    expect(fetchMock).toHaveBeenCalledWith(`${B}/sessions/s/results/r/accept`, expect.objectContaining({ method: 'POST' }))
    expect(r).toEqual({ ok: true, value: nodeAccept })
  })

  it('rejects an accept answer without the units', async () => {
    respond(200, { arm_calibration: nodeAccept.arm_calibration })
    const r = await acceptArmCamera('s', 'r')
    expect(r.ok).toBe(false)
    if (!r.ok) expect(r.message).toContain('units')
  })

  it('an accept refusal (409) carries the node message', async () => {
    respond(409, { message: 'result is not plausible' })
    const r = await acceptArmCamera('s', 'r')
    expect(r).toEqual({ ok: false, status: 409, message: 'result is not plausible' })
  })

  it('declines with POST and reads the declined result', async () => {
    const declined = { ...refusedFx, declined: true, declined_at: '2026-11-09T12:20:00+00:00' }
    const fetchMock = respond(200, declined)
    const r = await declineArmCamera('s', 'r')
    expect(fetchMock).toHaveBeenCalledWith(`${B}/sessions/s/results/r/decline`, expect.objectContaining({ method: 'POST' }))
    expect(r.ok && r.value.declined).toBe(true)
  })

  it('a missing backend (404) is an error with the status', async () => {
    respond(404, { detail: 'Not Found' })
    const r = await getArmCameraState()
    expect(r).toEqual({ ok: false, status: 404, message: 'HTTP 404' })
  })
})

describe('pairAccuracy', () => {
  it('pairs the 1 sigma and 95 % labels of one example into one entry', () => {
    expect(
      pairAccuracy([
        { label: 'tool point at 30 cm reach, 1 sigma', value: 0.13, unit: 'mm' },
        { label: 'tool point at 30 cm reach, 95 %', value: 0.42, unit: 'mm' },
        { label: 'tool point at 15 cm reach, 1 sigma', value: 0.1, unit: 'mm' },
        { label: 'tool point at 15 cm reach, 95 %', value: null, unit: 'mm' },
      ]),
    ).toEqual([
      { label: 'tool point at 30 cm reach', sigma: 0.13, p95: 0.42, unit: 'mm' },
      { label: 'tool point at 15 cm reach', sigma: 0.1, p95: null, unit: 'mm' },
    ])
  })

  it('a lone 1 sigma entry keeps the 95 % empty', () => {
    expect(pairAccuracy(stateFx.applied!.accuracy_examples)).toEqual([
      { label: 'tool point at 30 cm reach', sigma: stateFx.applied!.accuracy_examples[0].value, p95: null, unit: 'mm' },
    ])
  })
})
