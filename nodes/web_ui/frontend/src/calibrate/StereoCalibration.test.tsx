// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { act, cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react'
import { StereoCalibration } from './StereoCalibration'

const B = '/api/calibration/stereo'
type Handler = (init?: RequestInit) => { status: number; body: unknown }

const preview = (over: Record<string, unknown> = {}) => ({
  seq: 5, left: { corners: 20 }, right: { corners: 18 }, shared_corners: 15, min_shared_corners: 12,
  capturable: true, captures: 2, min_pairs_to_solve: 3, ...over,
})
const list = (n: number) => ({
  captures: Array.from({ length: n }, (_, i) => ({ id: `c${i + 1}` })), count: n, min_pairs_to_solve: 3,
})

let routes: Record<string, Handler>
let calls: string[]

function install() {
  vi.stubGlobal('fetch', vi.fn(async (url: string, init?: RequestInit) => {
    const key = `${init?.method ?? 'GET'} ${url}`
    calls.push(key)
    const h = routes[key]
    if (!h) return new Response('{}', { status: 404 })
    const r = h(init)
    return new Response(JSON.stringify(r.body), { status: r.status })
  }))
}

beforeEach(() => {
  calls = []
  routes = {
    [`GET ${B}/preview`]: () => ({ status: 200, body: preview() }),
    [`GET ${B}/captures`]: () => ({ status: 200, body: list(2) }),
    [`GET ${B}/solve`]: () => ({ status: 200, body: { state: 'idle' } }),
  }
})
afterEach(() => {
  cleanup()
  vi.useRealTimers()
  vi.unstubAllGlobals()
})

describe('StereoCalibration', () => {
  it('shows preview counts and seq matched images', async () => {
    install()
    render(<StereoCalibration />)
    await screen.findByText('corners 20')
    expect(screen.getByText('corners 18')).toBeTruthy()
    expect(screen.getByText('shared 15 / needed 12')).toBeTruthy()
    expect((screen.getByAltText('left camera') as HTMLImageElement).src).toContain(`${B}/preview/left.jpg?seq=5`)
    expect((screen.getByAltText('right camera') as HTMLImageElement).src).toContain(`${B}/preview/right.jpg?seq=5`)
  })

  it('an error replaces the images with the server message', async () => {
    routes[`GET ${B}/preview`] = () => ({ status: 503, body: { reason: 'no fresh pair' } })
    install()
    render(<StereoCalibration />)
    await screen.findByText('no fresh pair')
    expect(screen.queryByAltText('left camera')).toBeNull()
    expect(screen.queryByAltText('right camera')).toBeNull()
  })

  it('capture 201 shows the count', async () => {
    routes[`POST ${B}/captures`] = () => ({ status: 201, body: { id: 'c3', captures: 3 } })
    install()
    render(<StereoCalibration />)
    await screen.findByText('corners 20')
    fireEvent.click(screen.getByRole('button', { name: 'Capture pair' }))
    await screen.findByText('Captured - 3 pairs')
  })

  it('capture 422 shows the reason', async () => {
    routes[`POST ${B}/captures`] = () => ({ status: 422, body: { reason: 'board not seen by left' } })
    install()
    render(<StereoCalibration />)
    await screen.findByText('corners 20')
    fireEvent.click(screen.getByRole('button', { name: 'Capture pair' }))
    await screen.findByText('board not seen by left')
  })

  it('lists captures newest first against min pairs', async () => {
    install()
    render(<StereoCalibration />)
    await screen.findByText('2 / 3 pairs')
    const imgs = screen.getAllByRole('img').map((i) => (i as HTMLImageElement).getAttribute('src'))
    const thumbs = imgs.filter((s) => s?.includes('/captures/'))
    expect(thumbs[0]).toContain('/captures/c2/left.jpg')
    expect(thumbs[thumbs.length - 1]).toContain('/captures/c1/right.jpg')
  })

  it('delete cancel sends nothing, confirm sends confirm=true and refreshes', async () => {
    routes[`DELETE ${B}/captures?confirm=true`] = () => ({ status: 200, body: { removed_captures: 2, calibration_removed: true } })
    install()
    render(<StereoCalibration />)
    await screen.findByText('2 / 3 pairs')
    fireEvent.click(screen.getByRole('button', { name: 'Delete captures' }))
    await screen.findByText('Delete all captures and the active calibration? Depth stops until you solve again.')
    fireEvent.click(screen.getByRole('button', { name: 'Cancel' }))
    await waitFor(() => expect(screen.queryByRole('dialog')).toBeNull())
    expect(calls.some((c) => c.startsWith('DELETE'))).toBe(false)
    fireEvent.click(screen.getByRole('button', { name: 'Delete captures' }))
    await screen.findByText('Delete all captures and the active calibration? Depth stops until you solve again.')
    const before = calls.filter((c) => c === `GET ${B}/captures`).length
    fireEvent.click(screen.getByRole('button', { name: 'Delete' }))
    await waitFor(() => expect(calls).toContain(`DELETE ${B}/captures?confirm=true`))
    await waitFor(() => expect(calls.filter((c) => c === `GET ${B}/captures`).length).toBeGreaterThan(before))
  })

  it('solve is disabled below min pairs', async () => {
    routes[`GET ${B}/captures`] = () => ({ status: 200, body: list(1) })
    install()
    render(<StereoCalibration />)
    await screen.findByText('1 / 3 pairs')
    expect((screen.getByRole('button', { name: 'Solve' }) as HTMLButtonElement).disabled).toBe(true)
  })

  async function solveFlow(final: Record<string, unknown>) {
    routes[`GET ${B}/captures`] = () => ({ status: 200, body: list(3) })
    routes[`POST ${B}/solve`] = () => ({ status: 202, body: { state: 'running' } })
    let n = 0
    routes[`GET ${B}/solve`] = () => ({ status: 200, body: n++ === 0 ? { state: 'running' } : final })
    install()
    vi.useFakeTimers({ shouldAdvanceTime: true })
    render(<StereoCalibration />)
    await screen.findByText('3 / 3 pairs')
    fireEvent.click(screen.getByRole('button', { name: 'Solve' }))
    await screen.findByText(/^State: running/)
    expect((screen.getByRole('button', { name: 'Capture pair' }) as HTMLButtonElement).disabled).toBe(true)
    await act(async () => { await vi.advanceTimersByTimeAsync(2100) })
    await act(async () => { await vi.advanceTimersByTimeAsync(2100) })
  }

  it('solve polls to a written result', async () => {
    await solveFlow({
      state: 'written', rms: 0.31, rms_left: 0.2, rms_right: 0.25, pairs_used: 9, pairs_skipped: 1, message: 'calibration written',
    })
    await screen.findByText(/^State: written/)
    expect(screen.getByText(/stereo 0.31/)).toBeTruthy()
    expect(screen.getByText(/left 0.2/)).toBeTruthy()
    expect(screen.getByText(/right 0.25/)).toBeTruthy()
    expect(screen.getByText(/9 used, 1 skipped/)).toBeTruthy()
    expect(screen.getByText('calibration written')).toBeTruthy()
  })

  it('solve refused shows the gate text', async () => {
    await solveFlow({ state: 'refused', rms: 0.9, rms_left: 0.8, rms_right: 0.8, pairs_used: 9, pairs_skipped: 1, message: 'rms 0.9 above 0.5' })
    await screen.findByText('rms above the gate - previous calibration kept')
  })
})
