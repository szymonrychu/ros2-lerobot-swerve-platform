// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import {
  GROUND_GAUGE_PARAMS,
  acceptGround,
  computeGround,
  declineGround,
  getGroundResult,
  getGroundState,
  getGroundVerify,
  listGroundSessions,
  nextPlacement,
  startGroundSweep,
  startGroundVerify,
  stopGroundSweep,
  stopGroundVerify,
} from './groundBoardApi'
import {
  nodeAccept,
  nodeResult,
  nodeRunning,
  nodeSessions,
  nodeState,
  nodeSweep,
  nodeVerify,
} from './groundBoardFixtures'

const B = '/api/calibration/rgbd/ground-board'

function respond(status: number, body: unknown) {
  const fn = vi.fn(async (_url: string, _init?: RequestInit) => new Response(JSON.stringify(body), { status }))
  vi.stubGlobal('fetch', fn)
  return fn
}

afterEach(() => {
  vi.unstubAllGlobals()
})

describe('groundBoardApi', () => {
  it('reads the state without caching', async () => {
    const fetchMock = respond(200, nodeState)
    expect(await getGroundState()).toEqual({ ok: true, value: nodeState })
    expect(fetchMock).toHaveBeenCalledWith(`${B}/state`, expect.objectContaining({ cache: 'no-store' }))
  })

  it('names the bad key of a drifted state', async () => {
    respond(200, { ...nodeState, compute: { ...nodeState.compute, running: 'no' } })
    const r = await getGroundState()
    expect(r.ok).toBe(false)
    if (!r.ok) expect(r.message).toContain('compute.running')
  })

  it.each([
    ['start', startGroundSweep, '/sweep/start'],
    ['next placement', nextPlacement, '/sweep/next-placement'],
    ['stop', stopGroundSweep, '/sweep/stop'],
  ])('posts the sweep %s and reads the session state', async (_name, fn, path) => {
    const fetchMock = respond(200, nodeSweep)
    expect(await fn()).toEqual({ ok: true, value: nodeSweep })
    expect(fetchMock).toHaveBeenCalledWith(`${B}${path}`, expect.objectContaining({ method: 'POST' }))
  })

  it('returns the node message of a refused start (409)', async () => {
    respond(409, { ok: false, message: 'a floor sweep is running; stop it first' })
    expect(await startGroundSweep()).toEqual({ ok: false, status: 409, message: 'a floor sweep is running; stop it first' })
  })

  it('lists the sessions', async () => {
    const fetchMock = respond(200, nodeSessions)
    expect(await listGroundSessions()).toEqual({ ok: true, value: nodeSessions })
    expect(fetchMock).toHaveBeenCalledWith(`${B}/sessions`, expect.objectContaining({ cache: 'no-store' }))
  })

  it('starts a compute (202) with the escaped session id', async () => {
    const fetchMock = respond(202, { result_id: 'r-1' })
    expect(await computeGround('ground 1')).toEqual({ ok: true, value: { result_id: 'r-1' } })
    expect(fetchMock).toHaveBeenCalledWith(`${B}/sessions/ground%201/compute`, expect.objectContaining({ method: 'POST' }))
  })

  it('reads a done and a running result', async () => {
    const fetchMock = respond(200, nodeResult)
    expect(await getGroundResult(nodeResult.session, nodeResult.id)).toEqual({ ok: true, value: nodeResult })
    expect(fetchMock).toHaveBeenCalledWith(
      `${B}/sessions/${nodeResult.session}/results/${nodeResult.id}`,
      expect.objectContaining({ cache: 'no-store' }),
    )
    respond(200, nodeRunning)
    expect(await getGroundResult(nodeRunning.session, nodeRunning.id)).toEqual({ ok: true, value: nodeRunning })
  })

  it('refuses a done result without its report', async () => {
    respond(200, { ...nodeResult, report: null })
    const r = await getGroundResult('s', 'r')
    expect(r.ok).toBe(false)
    if (!r.ok) expect(r.message).toContain('report')
  })

  it('accepts and declines', async () => {
    let fetchMock = respond(200, nodeAccept)
    expect(await acceptGround('s', 'r')).toEqual({ ok: true, value: nodeAccept })
    expect(fetchMock).toHaveBeenCalledWith(`${B}/sessions/s/results/r/accept`, expect.objectContaining({ method: 'POST' }))
    const declined = { ...nodeResult, declined: true, declined_at: '2026-11-09T05:00:00+00:00' }
    fetchMock = respond(200, declined)
    expect(await declineGround('s', 'r')).toEqual({ ok: true, value: declined })
    expect(fetchMock).toHaveBeenCalledWith(`${B}/sessions/s/results/r/decline`, expect.objectContaining({ method: 'POST' }))
  })

  it('starts, stops and reads a verification', async () => {
    let fetchMock = respond(200, nodeVerify)
    expect(await startGroundVerify()).toEqual({ ok: true, value: nodeVerify })
    expect(fetchMock).toHaveBeenCalledWith(`${B}/verify/start`, expect.objectContaining({ method: 'POST' }))
    fetchMock = respond(200, nodeVerify)
    expect(await stopGroundVerify()).toEqual({ ok: true, value: nodeVerify })
    expect(fetchMock).toHaveBeenCalledWith(`${B}/verify/stop`, expect.objectContaining({ method: 'POST' }))
    fetchMock = respond(200, nodeVerify)
    expect(await getGroundVerify()).toEqual({ ok: true, value: nodeVerify })
    expect(fetchMock).toHaveBeenCalledWith(`${B}/verify/state`, expect.objectContaining({ cache: 'no-store' }))
  })

  it('names the head mount gauge of the fit', () => {
    expect(GROUND_GAUGE_PARAMS).toEqual(['mount_x_m', 'mount_y_m', 'mount_yaw_deg'])
  })
})
