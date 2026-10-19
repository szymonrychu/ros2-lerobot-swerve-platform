// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { act, cleanup, fireEvent, render, screen, within } from '@testing-library/react'
import { RgbdResults } from './RgbdResults'
import type { ComputeState } from './rgbdApi'
import resultFx from '../../../../head_rgbd_calibration/tests/fixtures/api/result.json'
import applyFx from '../../../../head_rgbd_calibration/tests/fixtures/api/apply.json'
import appliedFx from '../../../../head_rgbd_calibration/tests/fixtures/api/applied.json'

const NOT_RUNNING: ComputeState = { running: false, session_id: null, result_id: null, started_at: null, error: null }
const cov = { cols: 1, rows: 1, counts: [[1]] }

const PASSED = {
  id: 'r2', created: '2026-10-19T11:00:00', passed: true, failed_limits: [],
  rgb_rms_px: 0.31, tof_rms_px: 0.12, stereo_rms_px: 0.8, plane_bias_mm: 4.5, rgb_views: 14, tof_views: 13,
}
const FAILED = {
  id: 'r1', created: '2026-10-19T10:00:00', passed: false,
  failed_limits: ['limits.rgb_max_rms_px: RGB RMS 0.700 px exceeds 0.5 px'],
  rgb_rms_px: 0.7, tof_rms_px: 0.1, stereo_rms_px: 0.9, plane_bias_mm: 3, rgb_views: 14, tof_views: 14,
}

interface World {
  applied: { name: string; restarted: boolean }[]
  appliedCalls: number
  calls: string[]
}

function install(w: World): void {
  vi.stubGlobal(
    'fetch',
    vi.fn(async (url: string, init?: RequestInit) => {
      w.calls.push(`${init?.method ?? 'GET'} ${url}`)
      const json = (b: unknown, status = 200) => new Response(JSON.stringify(b), { status })
      if (url.endsWith('/sessions/s1'))
        return json({ id: 's1', created: 'x', lens_position: null, poses: [], coverage: { rgb: cov, tof: cov }, results: [FAILED, PASSED] })
      if (url.endsWith('/apply')) return json({ ...appliedFx, units: [{ name: 'head_camera_cpp', restarted: false }] })
      if (url.endsWith('/applied')) {
        w.appliedCalls += 1
        const done = w.appliedCalls >= 2
        return json({ ...appliedFx, units: w.applied.map((u) => ({ ...u, restarted: done })) })
      }
      if (url.endsWith('/results/r2')) return json({ ...PASSED, report: '# Report text' })
      return json({ message: 'nope' }, 404)
    }),
  )
}

const world = (): World => ({
  applied: [
    { name: 'head_camera_cpp', restarted: false },
    { name: 'head_rgbd_cpp', restarted: false },
    { name: 'head_state_publisher', restarted: false },
  ],
  appliedCalls: 0,
  calls: [],
})

describe('RgbdResults', () => {
  beforeEach(() => {
    vi.useFakeTimers({ shouldAdvanceTime: true })
  })
  afterEach(() => {
    cleanup()
    vi.useRealTimers()
    vi.unstubAllGlobals()
  })

  it('shows newest first with chips, the metrics table with limits, failed limits and the report', async () => {
    install(world())
    render(<RgbdResults sessionId="s1" compute={NOT_RUNNING} />)
    const chips = await screen.findAllByTestId('result-chip')
    expect(chips.map((c) => c.textContent)).toEqual(['passed', 'failed'])
    expect(screen.getAllByText('0.31').length).toBeGreaterThan(0)
    expect(screen.getAllByText(/0\.5/).length).toBeGreaterThan(0)
    expect(screen.getByText(/RGB RMS 0\.700 px exceeds/)).toBeTruthy()
    fireEvent.click(screen.getAllByText('Report')[0])
    await screen.findByText('# Report text')
  })

  it('Apply is offered for passed results only', async () => {
    install(world())
    render(<RgbdResults sessionId="s1" compute={NOT_RUNNING} />)
    await screen.findAllByTestId('result-chip')
    expect(screen.getAllByRole('button', { name: 'Apply' })).toHaveLength(1)
  })

  it('Apply goes through the dialog, then shows each unit restart state until all restarted', async () => {
    const w = world()
    install(w)
    render(<RgbdResults sessionId="s1" compute={NOT_RUNNING} />)
    await screen.findAllByTestId('result-chip')
    fireEvent.click(screen.getByRole('button', { name: 'Apply' }))
    const dialog = await screen.findByRole('dialog')
    expect(dialog.textContent).toContain('head_camera_cpp')
    expect(dialog.textContent).toContain('head_rgbd_cpp')
    expect(dialog.textContent).toContain('head_state_publisher')
    expect(w.calls.some((c) => c.endsWith('/apply'))).toBe(false)
    fireEvent.click(within(dialog).getByRole('button', { name: 'Apply' }))
    await screen.findByText(/head_camera_cpp: restarting/)
    await act(async () => { await vi.advanceTimersByTimeAsync(2100) })
    await act(async () => { await vi.advanceTimersByTimeAsync(2100) })
    await screen.findByText(/head_state_publisher: restarted/)
    const n = w.appliedCalls
    await act(async () => { await vi.advanceTimersByTimeAsync(6000) })
    expect(w.appliedCalls).toBe(n)
  })

  it('Download is an anchor to the result zip with the download attribute', async () => {
    install(world())
    render(<RgbdResults sessionId="s1" compute={NOT_RUNNING} />)
    await screen.findAllByTestId('result-chip')
    const a = screen.getAllByRole('link', { name: 'Download' })[0]
    expect(a.getAttribute('href')).toBe('/api/calibration/rgbd/sessions/s1/results/r2/download')
    expect(a.hasAttribute('download')).toBe(true)
  })
})

describe('RgbdResults with the node contract fixtures', () => {
  afterEach(() => {
    cleanup()
    vi.useRealTimers()
    vi.unstubAllGlobals()
  })

  function serve(): void {
    vi.stubGlobal(
      'fetch',
      vi.fn(async (url: string) => {
        const json = (b: unknown) => new Response(JSON.stringify(b), { status: 200 })
        if (url.endsWith('/sessions/s1'))
          return json({ id: 's1', created: 'x', lens_position: null, poses: [], coverage: { rgb: cov, tof: cov }, results: [resultFx] })
        if (url.endsWith('/apply')) return json(applyFx)
        if (url.endsWith('/applied')) return json(appliedFx)
        return json({ message: 'nope' })
      }),
    )
  }

  it('shows every limit from result.json including the plane bias limit', async () => {
    serve()
    render(<RgbdResults sessionId="s1" compute={NOT_RUNNING} />)
    await screen.findAllByTestId('result-chip')
    const row = (label: string) => screen.getByText(label).closest('tr') as HTMLElement
    expect(row('Plane bias').textContent).toContain('-3.5')
    expect(row('Plane bias').textContent).toContain(String(resultFx.limits.plane_max_bias_mm))
    expect(row('RGB RMS').textContent).toContain(String(resultFx.limits.rgb_max_rms_px))
    expect(row('RGB views').textContent).toContain(String(resultFx.limits.min_views))
  })

  it('shows no limit when result.json carries no limits object', async () => {
    const { limits: _l, ...bare } = resultFx
    void _l
    vi.stubGlobal(
      'fetch',
      vi.fn(async () =>
        new Response(JSON.stringify({ id: 's1', created: 'x', lens_position: null, poses: [], coverage: { rgb: cov, tof: cov }, results: [bare] })),
      ),
    )
    render(<RgbdResults sessionId="s1" compute={NOT_RUNNING} />)
    await screen.findAllByTestId('result-chip')
    const cells = (screen.getByText('RGB RMS').closest('tr') as HTMLElement).querySelectorAll('td')
    expect(cells[2].textContent).toBe('-')
  })

  it('Apply with apply.json shows the applied state and unit restarts without error', async () => {
    serve()
    render(<RgbdResults sessionId="s1" compute={NOT_RUNNING} />)
    await screen.findAllByTestId('result-chip')
    fireEvent.click(screen.getByRole('button', { name: 'Apply' }))
    const dialog = await screen.findByRole('dialog')
    fireEvent.click(within(dialog).getByRole('button', { name: 'Apply' }))
    await screen.findByText(/ros2-head_camera_cpp\.service: restarted/)
    expect(screen.getByText(`Applied result ${applyFx.result} of session ${applyFx.session}`)).toBeTruthy()
    expect(screen.queryByText(/Unexpected calibration response/)).toBeNull()
  })
})
