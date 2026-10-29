// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import { act, cleanup, render, screen } from '@testing-library/react'
import { RgbdCalibration, RGBD_POLL_MS } from './RgbdCalibration'

vi.mock('./RgbdCapture', () => ({ RgbdCapture: (p: { sessionId: string | null }) => <div data-testid="cap">{p.sessionId}</div> }))
vi.mock('./RgbdResults', () => ({ RgbdResults: () => <div /> }))
vi.mock('./RgbdSessions', () => ({
  RgbdSessions: (p: { onOpen: (id: string) => void }) => <button onClick={() => p.onOpen('old')}>open-old</button>,
}))

afterEach(() => {
  cleanup()
  vi.unstubAllGlobals()
  vi.useRealTimers()
})

const state = {
  capture: { running: true, session_id: 'live', status: 'capturing', pose_count: 1, started_at: null, stops_at: null },
  compute: { running: false, session_id: null, result_id: null, started_at: null, error: null },
  applied: null,
}

describe('RgbdCalibration', () => {
  it('titles the section as the D435 intrinsics check with its board instructions', async () => {
    vi.stubGlobal('fetch', vi.fn(async () => new Response(JSON.stringify(state), { status: 200 })))
    render(<RgbdCalibration />)
    await act(async () => {})
    expect(screen.getByRole('heading', { name: 'D435 intrinsics check' })).toBeTruthy()
    expect(screen.queryByText('RGB-D calibration')).toBeNull()
    const help = screen.getByTestId('rgbd-help').textContent ?? ''
    expect(help).toContain('0.3-0.6 m')
    expect(help).toContain('coverage grid')
    expect(help).toContain('factory')
    expect(help).not.toMatch(/ToF|stereo|restart/i)
  })

  it('keeps an opened past session while the capture keeps polling', async () => {
    vi.useFakeTimers()
    vi.stubGlobal('fetch', vi.fn(async () => new Response(JSON.stringify(state), { status: 200 })))
    render(<RgbdCalibration />)
    await act(async () => {})
    expect(screen.getByTestId('cap').textContent).toBe('live')
    await act(async () => {
      screen.getByText('open-old').click()
    })
    expect(screen.getByTestId('cap').textContent).toBe('old')
    await act(async () => {
      await vi.advanceTimersByTimeAsync(RGBD_POLL_MS * 3)
    })
    expect(screen.getByTestId('cap').textContent).toBe('old')
  })
})
