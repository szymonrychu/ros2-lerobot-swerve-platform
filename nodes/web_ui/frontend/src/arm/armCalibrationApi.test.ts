// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import { act, renderHook, waitFor } from '@testing-library/react'
import { toArmCalibration } from './armCalibration'
import { ARM_CALIBRATION_URL, fetchArmCalibration, parseArmCalibration, useArmCalibration } from './armCalibrationApi'
import { ACCEPTED, CONFIGURED, acceptedBody, configuredBody } from './armCalibrationFixtures'

const OFFSETS = ACCEPTED.joint_offsets_rad

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
  it('reads the configured values', () => {
    expect(parseArmCalibration(configuredBody)).toEqual({ ok: true, value: configuredBody })
  })

  it('reads an accepted calibration', () => {
    expect(parseArmCalibration(acceptedBody)).toEqual({ ok: true, value: acceptedBody })
  })

  it('reads no calibration (web_ui without arm_calibration)', () => {
    expect(parseArmCalibration({ calibration: null, error: null })).toEqual({ ok: true, value: { calibration: null, error: null } })
  })

  it('keeps only the drawn fields', () => {
    const r = parseArmCalibration({
      calibration: { ...ACCEPTED, accepted: { ...ACCEPTED.accepted, score: 1 }, joint_offsets_rad: { ...OFFSETS, gripper: 1 }, extra: 1 },
      error: null,
    })
    expect(r).toEqual({ ok: true, value: acceptedBody })
  })

  it('passes the backend error with the values in use', () => {
    const r = parseArmCalibration({ calibration: CONFIGURED, error: 'arm_calibration.json unreadable' })
    expect(r).toEqual({ ok: true, value: { calibration: CONFIGURED, error: 'arm_calibration.json unreadable' } })
  })

  it.each([
    [null, 'body is not an object'],
    [{ calibration: null }, 'bad error'],
    [{ calibration: 'x', error: null }, 'bad calibration'],
    [{ calibration: { ...CONFIGURED, source: 'config' }, error: null }, 'bad calibration.source'],
    [{ calibration: { ...CONFIGURED, accepted: ACCEPTED.accepted }, error: null }, 'bad calibration.accepted'],
    [{ calibration: { ...ACCEPTED, accepted: null }, error: null }, 'bad calibration.accepted'],
    [{ calibration: { ...ACCEPTED, accepted: { ...ACCEPTED.accepted, session: 1 } }, error: null }, 'bad calibration.accepted.session'],
    [{ calibration: { ...ACCEPTED, accepted: { ...ACCEPTED.accepted, accepted_at: null } }, error: null }, 'bad calibration.accepted.accepted_at'],
    [{ calibration: { ...CONFIGURED, joint_offsets_rad: [] }, error: null }, 'bad calibration.joint_offsets_rad'],
    [{ calibration: { ...CONFIGURED, joint_offsets_rad: { ...OFFSETS, wrist_roll: undefined } }, error: null }, 'bad calibration.joint_offsets_rad.wrist_roll'],
    [{ calibration: { ...CONFIGURED, joint_offsets_rad: { ...OFFSETS, elbow_flex: 'x' } }, error: null }, 'bad calibration.joint_offsets_rad.elbow_flex'],
    [{ calibration: { ...CONFIGURED, base_in_base_link: null }, error: null }, 'bad calibration.base_in_base_link'],
    [{ calibration: { ...CONFIGURED, base_in_base_link: { x: 0, y: 0, z: 0.1 } }, error: null }, 'bad calibration.base_in_base_link.yaw'],
  ])('refuses %j naming the bad key', (body, message) => {
    const r = parseArmCalibration(body)
    expect(r.ok).toBe(false)
    if (!r.ok) expect(r.message).toContain(message)
  })
})

describe('fetchArmCalibration', () => {
  it('gets the route uncached', async () => {
    const mock = respond(200, acceptedBody)
    expect(await fetchArmCalibration()).toEqual({ ok: true, value: acceptedBody })
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
  it('is null until the first answer, then the configured values', async () => {
    respond(200, configuredBody)
    const { result } = renderHook(() => useArmCalibration(60_000))
    expect(result.current).toBeNull()
    await waitFor(() => expect(result.current).toEqual(toArmCalibration(CONFIGURED)))
    expect(result.current?.source).toBe('configured')
  })

  it('stays null without a calibration (web_ui without arm_calibration)', async () => {
    const mock = respond(200, { calibration: null, error: null })
    const { result } = renderHook(() => useArmCalibration(60_000))
    await waitFor(() => expect(mock).toHaveBeenCalledTimes(1))
    expect(result.current).toBeNull()
  })

  it('follows a new accept and a removed one on the next poll', async () => {
    vi.useFakeTimers({ shouldAdvanceTime: true })
    let body: unknown = configuredBody
    vi.stubGlobal('fetch', vi.fn(async () => new Response(JSON.stringify(body), { status: 200 })))
    const { result } = renderHook(() => useArmCalibration(1000))
    await waitFor(() => expect(result.current?.source).toBe('configured'))
    body = acceptedBody
    await act(async () => { await vi.advanceTimersByTimeAsync(1000) })
    await waitFor(() => expect(result.current?.source).toBe('accepted'))
    expect(result.current?.mount).toEqual([0.061, -0.048, 0.102, 0.01])
    expect(result.current?.jointOffsets).toEqual(OFFSETS)
    body = configuredBody
    await act(async () => { await vi.advanceTimersByTimeAsync(1000) })
    await waitFor(() => expect(result.current).toEqual(toArmCalibration(CONFIGURED)))
  })

  it('keeps the last answer when a poll fails and the same object while nothing changed', async () => {
    vi.useFakeTimers({ shouldAdvanceTime: true })
    let fail = false
    vi.stubGlobal('fetch', vi.fn(async () => {
      if (fail) throw new Error('link down')
      return new Response(JSON.stringify(acceptedBody), { status: 200 })
    }))
    const { result } = renderHook(() => useArmCalibration(1000))
    await waitFor(() => expect(result.current?.source).toBe('accepted'))
    const first = result.current
    await act(async () => { await vi.advanceTimersByTimeAsync(1000) })
    expect(result.current).toBe(first)
    fail = true
    await act(async () => { await vi.advanceTimersByTimeAsync(1000) })
    expect(result.current).toBe(first)
  })

  it('stops polling on unmount', async () => {
    vi.useFakeTimers({ shouldAdvanceTime: true })
    const mock = respond(200, configuredBody)
    const { unmount } = renderHook(() => useArmCalibration(1000))
    await waitFor(() => expect(mock).toHaveBeenCalledTimes(1))
    unmount()
    await act(async () => { await vi.advanceTimersByTimeAsync(3000) })
    expect(mock).toHaveBeenCalledTimes(1)
  })
})
