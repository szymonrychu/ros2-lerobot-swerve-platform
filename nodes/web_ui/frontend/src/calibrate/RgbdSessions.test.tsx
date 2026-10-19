// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react'
import { RgbdSessions } from './RgbdSessions'
import type { CaptureState, ComputeState } from './rgbdApi'

const IDLE: CaptureState = { running: false, session_id: null, status: 'idle', pose_count: 0, started_at: null, stops_at: null }
const NOT_RUNNING: ComputeState = { running: false, session_id: null, result_id: null, started_at: null, error: null }

const SESSIONS = [
  { id: 's1', created: '2026-10-18T10:00:00', pose_count: 5, result_count: 1 },
  { id: 's2', created: '2026-10-19T10:00:00', pose_count: 14, result_count: 2 },
]

interface Opts {
  computeStatus?: number
  calls: string[]
  sessions?: typeof SESSIONS
}

function install(o: Opts): void {
  vi.stubGlobal(
    'fetch',
    vi.fn(async (url: string, init?: RequestInit) => {
      o.calls.push(`${init?.method ?? 'GET'} ${url}`)
      const json = (b: unknown, status = 200) => new Response(JSON.stringify(b), { status })
      if (url.endsWith('/sessions')) return json(o.sessions ?? SESSIONS)
      if (url.endsWith('/compute')) {
        return o.computeStatus === 409 ? json({ message: 'capture running' }, 409) : json({ result_id: 'r9' })
      }
      if (url.endsWith('/capture/start')) return json({ capture: { ...IDLE, running: true, session_id: 's1' }, compute: NOT_RUNNING, applied: null })
      return json({ message: 'nope' }, 404)
    }),
  )
}

afterEach(() => {
  cleanup()
  vi.unstubAllGlobals()
})

const props = (over: Partial<Parameters<typeof RgbdSessions>[0]> = {}) => ({
  capture: IDLE,
  compute: NOT_RUNNING,
  openId: null,
  onOpen: vi.fn(),
  onStarted: vi.fn(),
  ...over,
})

describe('RgbdSessions', () => {
  it('lists sessions newest first with counts and Open selects one', async () => {
    install({ calls: [] })
    const p = props()
    render(<RgbdSessions {...p} />)
    await screen.findAllByRole('button', { name: /^Open/ })
    const rows = screen.getAllByRole('row')
    expect(rows[1].textContent).toContain('s2')
    expect(rows[1].textContent).toContain('14')
    expect(rows[2].textContent).toContain('s1')
    fireEvent.click(screen.getAllByRole('button', { name: /^Open/ })[0])
    expect(p.onOpen).toHaveBeenCalledWith('s2')
  })

  it('Resume capture starts the capture on that session and calls onStarted', async () => {
    const calls: string[] = []
    install({ calls })
    const p = props()
    render(<RgbdSessions {...p} />)
    await screen.findAllByRole('button', { name: /^Open/ })
    fireEvent.click(screen.getAllByRole('button', { name: /^Resume capture/ })[1])
    await waitFor(() => expect(p.onStarted).toHaveBeenCalled())
    expect(calls).toContain('POST /api/calibration/rgbd/capture/start')
  })

  it('Compute is disabled while a capture or a compute runs', async () => {
    install({ calls: [] })
    const { rerender } = render(<RgbdSessions {...props({ capture: { ...IDLE, running: true } })} />)
    await screen.findAllByRole('button', { name: /^Open/ })
    for (const b of screen.getAllByRole('button', { name: /^Compute/ })) expect((b as HTMLButtonElement).disabled).toBe(true)
    rerender(<RgbdSessions {...props({ compute: { ...NOT_RUNNING, running: true } })} />)
    for (const b of screen.getAllByRole('button', { name: /^Compute/ })) expect((b as HTMLButtonElement).disabled).toBe(true)
  })

  it('shows the 409 message of a refused compute', async () => {
    install({ calls: [], computeStatus: 409 })
    render(<RgbdSessions {...props()} />)
    await screen.findAllByRole('button', { name: /^Open/ })
    fireEvent.click(screen.getAllByRole('button', { name: /^Compute/ })[0])
    await screen.findByText('capture running')
  })

  it('a started compute calls onStarted; the list refreshes when the compute ends and shows its error', async () => {
    const calls: string[] = []
    install({ calls })
    const p = props()
    const { rerender } = render(<RgbdSessions {...p} />)
    await screen.findAllByRole('button', { name: /^Open/ })
    fireEvent.click(screen.getAllByRole('button', { name: /^Compute/ })[0])
    await waitFor(() => expect(p.onStarted).toHaveBeenCalled())
    rerender(<RgbdSessions {...props({ compute: { ...NOT_RUNNING, running: true, session_id: 's2' } })} />)
    const before = calls.filter((c) => c.endsWith('/sessions')).length
    rerender(<RgbdSessions {...props({ compute: { ...NOT_RUNNING, error: 'too few views' } })} />)
    await waitFor(() => expect(calls.filter((c) => c.endsWith('/sessions')).length).toBeGreaterThan(before))
    await screen.findByText(/too few views/)
  })
})
