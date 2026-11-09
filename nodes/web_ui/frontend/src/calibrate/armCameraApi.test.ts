// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import {
  acceptArmCamera,
  computeArmCamera,
  declineArmCamera,
  getArmCameraResult,
  getArmCameraState,
} from './armCameraApi'
import stateFx from './fixtures/arm_camera_state.json'
import computeFx from './fixtures/arm_camera_compute.json'
import doneFx from './fixtures/arm_camera_result_done.json'
import refusedFx from './fixtures/arm_camera_result_refused.json'
import runningFx from './fixtures/arm_camera_result_running.json'

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
    expect(r.ok && r.value.report?.score.heldout_tool_mm).toBe(1.8)
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
    respond(200, { ...doneFx, report: { ...doneFx.report, score: undefined } })
    const r = await getArmCameraResult('s', 'r')
    expect(r.ok).toBe(false)
    if (!r.ok) expect(r.message).toContain('report.score')
  })

  it('rejects an unknown result state', async () => {
    respond(200, { ...doneFx, state: 'queued' })
    expect((await getArmCameraResult('s', 'r')).ok).toBe(false)
  })

  it('rejects a param without sigma', async () => {
    const params = [{ ...doneFx.report.params[0], sigma: undefined }]
    respond(200, { ...doneFx, report: { ...doneFx.report, params } })
    expect((await getArmCameraResult('s', 'r')).ok).toBe(false)
  })

  it('accepts with POST', async () => {
    const fetchMock = respond(200, { ok: true, message: 'applied' })
    const r = await acceptArmCamera('s', 'r')
    expect(fetchMock).toHaveBeenCalledWith(`${B}/sessions/s/results/r/accept`, expect.objectContaining({ method: 'POST' }))
    expect(r.ok).toBe(true)
  })

  it('an accept refusal (409) carries the node message', async () => {
    respond(409, { message: 'result is not plausible' })
    const r = await acceptArmCamera('s', 'r')
    expect(r).toEqual({ ok: false, status: 409, message: 'result is not plausible' })
  })

  it('declines with POST', async () => {
    const fetchMock = respond(200, {})
    const r = await declineArmCamera('s', 'r')
    expect(fetchMock).toHaveBeenCalledWith(`${B}/sessions/s/results/r/decline`, expect.objectContaining({ method: 'POST' }))
    expect(r.ok).toBe(true)
  })

  it('a missing backend (404) is an error with the status', async () => {
    respond(404, { detail: 'Not Found' })
    const r = await getArmCameraState()
    expect(r).toEqual({ ok: false, status: 404, message: 'HTTP 404' })
  })
})
