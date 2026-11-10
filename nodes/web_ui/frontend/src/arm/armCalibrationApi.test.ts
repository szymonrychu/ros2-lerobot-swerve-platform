// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import { act, renderHook, waitFor } from '@testing-library/react'
import { configuredArmCalibration } from './armCalibration'
import { ARM_CALIBRATION_URL, fetchArmCalibration, parseArmCalibration, useArmCalibration } from './armCalibrationApi'

const OFFSETS = { shoulder_pan: 0.03, shoulder_lift: -0.02, elbow_flex: -0.09, wrist_flex: 0.08, wrist_roll: -0.07 }
const ACCEPTED = {
  session: 'calib-20261109-120000',
  result: 'r-1',
  accepted_at: '2026-11-09T12:20:00Z',
  joint_offsets_rad: OFFSETS,
  base_in_base_link: { x: 0.061, y: -0.048, z: 0.102, yaw: 0.01 },
}
const OFFSET: [number, number, number] = [0.0592, -0.05, 0.1]

afterEach(() => {
  vi.unstubAllGlobals()
  vi.useRealTimers()
})

function respond(status: number, body: unknown) {
  const mock = vi.fn(async (_url: string, _init?: RequestInit) => new Response(JSON.stringify(body), { status }))
  vi.stubGlobal('fetch', mock)
  return mock
}

describe('parseArmCalibration', () => {
  it('reads nothing accepted', () => {
    expect(parseArmCalibration({ accepted: null, error: null })).toEqual({ ok: true, value: { accepted: null, error: null } })
  })

  it('reads an accepted calibration and keeps only the drawn fields', () => {
    const r = parseArmCalibration({ accepted: { ...ACCEPTED, joint_offsets_rad: { ...OFFSETS, gripper: 1 }, extra: 1 }, error: null })
    expect(r).toEqual({ ok: true, value: { accepted: ACCEPTED, error: null } })
  })

  it('passes the backend error with the last good calibration', () => {
    const r = parseArmCalibration({ accepted: ACCEPTED, error: 'arm_calibration.json unreadable' })
    expect(r).toEqual({ ok: true, value: { accepted: ACCEPTED, error: 'arm_calibration.json unreadable' } })
  })

  it.each([
    [null, 'body is not an object'],
    [{ accepted: null }, 'bad error'],
    [{ accepted: 'x', error: null }, 'bad accepted'],
    [{ accepted: { ...ACCEPTED, session: 1 }, error: null }, 'bad accepted.session'],
    [{ accepted: { ...ACCEPTED, accepted_at: null }, error: null }, 'bad accepted.accepted_at'],
    [{ accepted: { ...ACCEPTED, joint_offsets_rad: [] }, error: null }, 'bad accepted.joint_offsets_rad'],
    [{ accepted: { ...ACCEPTED, joint_offsets_rad: { ...OFFSETS, wrist_roll: undefined } }, error: null }, 'bad accepted.joint_offsets_rad.wrist_roll'],
    [{ accepted: { ...ACCEPTED, joint_offsets_rad: { ...OFFSETS, elbow_flex: 'x' } }, error: null }, 'bad accepted.joint_offsets_rad.elbow_flex'],
    [{ accepted: { ...ACCEPTED, base_in_base_link: null }, error: null }, 'bad accepted.base_in_base_link'],
    [{ accepted: { ...ACCEPTED, base_in_base_link: { x: 0, y: 0, z: 0.1 } }, error: null }, 'bad accepted.base_in_base_link.yaw'],
  ])('refuses %j naming the bad key', (body, message) => {
    const r = parseArmCalibration(body)
    expect(r.ok).toBe(false)
    if (!r.ok) expect(r.message).toContain(message)
  })
})

describe('fetchArmCalibration', () => {
  it('gets the route uncached', async () => {
    const mock = respond(200, { accepted: ACCEPTED, error: null })
    expect(await fetchArmCalibration()).toEqual({ ok: true, value: { accepted: ACCEPTED, error: null } })
    expect(mock).toHaveBeenCalledWith(ARM_CALIBRATION_URL, { cache: 'no-store' })
  })

  it('reports HTTP, network and JSON errors without throwing', async () => {
    respond(500, {})
    expect(await fetchArmCalibration()).toEqual({ ok: false, message: 'arm calibration: HTTP 500' })
    vi.stubGlobal('fetch', vi.fn(async () => { throw new Error('link down') }))
    expect(await fetchArmCalibration()).toEqual({ ok: false, message: 'link down' })
    vi.stubGlobal('fetch', vi.fn(async () => new Response('not json', { status: 200 })))
    expect(await fetchArmCalibration()).toEqual({ ok: false, message: 'arm calibration: body is not JSON' })
  })
})

describe('useArmCalibration', () => {
  it('starts with the configured mount and switches to an accepted calibration', async () => {
    respond(200, { accepted: ACCEPTED, error: null })
    const { result } = renderHook(() => useArmCalibration(OFFSET, 60_000))
    expect(result.current).toEqual(configuredArmCalibration(OFFSET))
    await waitFor(() => expect(result.current.source).toBe('accepted'))
    expect(result.current.mount).toEqual([0.061, -0.048, 0.102, 0.01])
    expect(result.current.jointOffsets).toEqual(OFFSETS)
  })

  it('follows a new accept and a removed one on the next poll', async () => {
    vi.useFakeTimers({ shouldAdvanceTime: true })
    let body: unknown = { accepted: null, error: null }
    vi.stubGlobal('fetch', vi.fn(async () => new Response(JSON.stringify(body), { status: 200 })))
    const { result } = renderHook(() => useArmCalibration(OFFSET, 1000))
    await waitFor(() => expect(fetch).toHaveBeenCalledTimes(1))
    expect(result.current.source).toBe('config')
    body = { accepted: ACCEPTED, error: null }
    await act(async () => { await vi.advanceTimersByTimeAsync(1000) })
    await waitFor(() => expect(result.current.source).toBe('accepted'))
    body = { accepted: null, error: null }
    await act(async () => { await vi.advanceTimersByTimeAsync(1000) })
    await waitFor(() => expect(result.current).toEqual(configuredArmCalibration(OFFSET)))
  })

  it('keeps the last answer when a poll fails and the same object while nothing changed', async () => {
    vi.useFakeTimers({ shouldAdvanceTime: true })
    let fail = false
    vi.stubGlobal('fetch', vi.fn(async () => {
      if (fail) throw new Error('link down')
      return new Response(JSON.stringify({ accepted: ACCEPTED, error: null }), { status: 200 })
    }))
    const { result } = renderHook(() => useArmCalibration(OFFSET, 1000))
    await waitFor(() => expect(result.current.source).toBe('accepted'))
    const first = result.current
    await act(async () => { await vi.advanceTimersByTimeAsync(1000) })
    expect(result.current).toBe(first)
    fail = true
    await act(async () => { await vi.advanceTimersByTimeAsync(1000) })
    expect(result.current).toBe(first)
  })

  it('stops polling on unmount', async () => {
    vi.useFakeTimers({ shouldAdvanceTime: true })
    const mock = respond(200, { accepted: null, error: null })
    const { unmount } = renderHook(() => useArmCalibration(OFFSET, 1000))
    await waitFor(() => expect(mock).toHaveBeenCalledTimes(1))
    unmount()
    await act(async () => { await vi.advanceTimersByTimeAsync(3000) })
    expect(mock).toHaveBeenCalledTimes(1)
  })
})
