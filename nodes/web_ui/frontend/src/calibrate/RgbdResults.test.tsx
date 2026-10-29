// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { act, cleanup, fireEvent, render, screen, within } from '@testing-library/react'
import { RgbdResults } from './RgbdResults'
import type { ComputeState } from './rgbdApi'
import resultFx from '../../../../head_rgbd_calibration/tests/fixtures/api/result.json'
import applyFx from '../../../../head_rgbd_calibration/tests/fixtures/api/apply.json'

const NOT_RUNNING: ComputeState = { running: false, session_id: null, result_id: null, started_at: null, error: null }
const cov = { cols: 1, rows: 1, counts: [[1]] }

const PASSED = {
  ...resultFx, id: 'r2', created: '2026-10-19T11:00:00', passed: true, failed_limits: [],
  rms_px: 0.31, focal_diff_pct: 0.4, principal_diff_px: 1.5, plane_bias_median_mm: 4.5, plane_bias_spread_mm: 2, views: 14,
}
const FAILED = {
  ...resultFx, id: 'r1', created: '2026-10-19T10:00:00', passed: false,
  failed_limits: ['limits.intrinsics_max_rms_px: RMS 0.700 px exceeds 0.5 px'],
  rms_px: 0.7, focal_diff_pct: 0.4, principal_diff_px: 1.5, plane_bias_median_mm: 3, plane_bias_spread_mm: 2, views: 14,
}
const D435_ROWS = ['Colour RMS', 'Focal difference', 'Principal point difference', 'Plane bias', 'Plane spread', 'Colour views']

function serve(results: unknown[], calls: string[] = []): void {
  vi.stubGlobal(
    'fetch',
    vi.fn(async (url: string, init?: RequestInit) => {
      calls.push(`${init?.method ?? 'GET'} ${url}`)
      const json = (b: unknown, status = 200) => new Response(JSON.stringify(b), { status })
      if (url.endsWith('/sessions/s1')) return json({ id: 's1', created: 'x', poses: [], coverage: { rgb: cov }, results })
      if (url.endsWith('/apply')) return json(applyFx)
      if (url.endsWith('/results/r2')) return json({ ...PASSED, report: '# Report text' })
      return json({ message: 'nope' }, 404)
    }),
  )
}

/** Cells of the row labelled `label` in the card of result `rid`: [item, value, limit, check]. */
function cells(rid: string, label: string): string[] {
  const card = screen.getByTestId(`result-card-${rid}`)
  const row = within(card).getByText(label).closest('tr') as HTMLElement
  return [...row.querySelectorAll('td')].map((td) => td.textContent ?? '')
}

describe('RgbdResults', () => {
  beforeEach(() => {
    vi.useFakeTimers({ shouldAdvanceTime: true })
  })
  afterEach(() => {
    cleanup()
    vi.useRealTimers()
    vi.unstubAllGlobals()
  })

  it('shows the D435 check rows only, no ToF row', async () => {
    serve([resultFx])
    render(<RgbdResults sessionId="s1" compute={NOT_RUNNING} />)
    await screen.findAllByTestId('result-chip')
    const card = screen.getByTestId(`result-card-${resultFx.id}`)
    const labels = [...card.querySelectorAll('tbody tr')].map((tr) => tr.querySelector('td')?.textContent)
    expect(labels).toEqual(D435_ROWS)
    expect(card.textContent).not.toMatch(/ToF/)
    const lim = resultFx.limits
    expect(cells(resultFx.id, 'Colour RMS')).toEqual(['Colour RMS', '0.21', `<= ${lim.intrinsics_max_rms_px} px`, 'pass'])
    expect(cells(resultFx.id, 'Focal difference')).toEqual(['Focal difference', '0.35', `<= ${lim.intrinsics_max_focal_diff_pct} %`, 'pass'])
    expect(cells(resultFx.id, 'Principal point difference')).toEqual([
      'Principal point difference', '1.2', `<= ${lim.intrinsics_max_principal_diff_px} px`, 'pass',
    ])
    expect(cells(resultFx.id, 'Plane bias')).toEqual(['Plane bias', '-1.5', `|bias| <= ${lim.plane_max_bias_mm} mm`, 'pass'])
    expect(cells(resultFx.id, 'Plane spread')).toEqual(['Plane spread', '2.1', `<= ${lim.plane_max_spread_mm} mm`, 'pass'])
    expect(cells(resultFx.id, 'Colour views')).toEqual(['Colour views', '18', `>= ${lim.min_views}`, 'pass'])
  })

  it('shows the result rows of a passed and a failed check, newest first, with the failed limit', async () => {
    serve([FAILED, PASSED])
    render(<RgbdResults sessionId="s1" compute={NOT_RUNNING} />)
    const chips = await screen.findAllByTestId('result-chip')
    expect(chips.map((c) => c.textContent)).toEqual(['passed', 'failed'])
    for (const label of D435_ROWS) expect(cells('r2', label)[3]).toBe('pass')
    expect(cells('r1', 'Colour RMS')).toEqual(['Colour RMS', '0.7', '<= 0.5 px', 'fail'])
    for (const label of D435_ROWS.slice(1)) expect(cells('r1', label)[3]).toBe('pass')
    expect(screen.getByText(/RMS 0\.700 px exceeds/)).toBeTruthy()
  })

  it('an unmeasured plane bias fails its row, an input error shows no check per row', async () => {
    const unmeasured = {
      ...FAILED, id: 'r3', plane_bias_median_mm: null, plane_bias_spread_mm: null,
      failed_limits: ['limits.plane_max_bias_mm: board plane depth bias not measured'],
    }
    const inputError = {
      ...resultFx, id: 'r4', passed: false, error: 'calibration needs at least 12 views, got 3 views', rms_px: null,
      focal_diff_pct: null, principal_diff_px: null, plane_bias_median_mm: null, plane_bias_spread_mm: null, views: null, poses: [],
    }
    serve([unmeasured, inputError])
    render(<RgbdResults sessionId="s1" compute={NOT_RUNNING} />)
    await screen.findAllByTestId('result-chip')
    expect(cells('r3', 'Plane bias')).toEqual(['Plane bias', '-', `|bias| <= ${resultFx.limits.plane_max_bias_mm} mm`, 'fail'])
    expect(cells('r3', 'Plane spread')[3]).toBe('-')
    for (const label of D435_ROWS) {
      const [, value, , verdict] = cells('r4', label)
      expect([value, verdict]).toEqual(['-', '-'])
    }
    expect(screen.getByText('calibration needs at least 12 views, got 3 views')).toBeTruthy()
  })

  it('accept disabled on a failed check', async () => {
    serve([FAILED, PASSED])
    render(<RgbdResults sessionId="s1" compute={NOT_RUNNING} />)
    await screen.findAllByTestId('result-chip')
    const failedAccept = within(screen.getByTestId('result-card-r1')).getByRole('button', { name: 'Accept check' })
    const passedAccept = within(screen.getByTestId('result-card-r2')).getByRole('button', { name: 'Accept check' })
    expect((failedAccept as HTMLButtonElement).disabled).toBe(true)
    expect((passedAccept as HTMLButtonElement).disabled).toBe(false)
    expect(screen.queryByRole('button', { name: 'Apply' })).toBeNull()
  })

  it('Accept check goes through the dialog, records the check and waits for no restart', async () => {
    const calls: string[] = []
    serve([PASSED], calls)
    render(<RgbdResults sessionId="s1" compute={NOT_RUNNING} />)
    await screen.findAllByTestId('result-chip')
    fireEvent.click(screen.getByRole('button', { name: 'Accept check' }))
    const dialog = await screen.findByRole('dialog')
    expect(dialog.textContent).not.toMatch(/restart/i)
    expect(dialog.textContent).toContain('factory intrinsics')
    expect(calls.some((c) => c.endsWith('/apply'))).toBe(false)
    fireEvent.click(within(dialog).getByRole('button', { name: 'Accept check' }))
    await screen.findByText(`Accepted check ${applyFx.result} of session ${applyFx.session}`)
    expect(calls).toContain('POST /api/calibration/rgbd/sessions/s1/results/r2/apply')
    expect(screen.queryByText(/restart/i)).toBeNull()
    expect(screen.queryByText(/Unexpected calibration response/)).toBeNull()
    const n = calls.length
    await act(async () => { await vi.advanceTimersByTimeAsync(10000) })
    expect(calls.length).toBe(n)
    expect((screen.getByRole('button', { name: 'Accept check' }) as HTMLButtonElement).disabled).toBe(false)
  })

  it('an Accept refusal shows the server message', async () => {
    vi.stubGlobal('fetch', vi.fn(async (url: string) =>
      url.endsWith('/apply')
        ? new Response(JSON.stringify({ message: 'a compute is running; wait for it to finish' }), { status: 409 })
        : new Response(JSON.stringify({ id: 's1', created: 'x', poses: [], coverage: { rgb: cov }, results: [PASSED] })),
    ))
    render(<RgbdResults sessionId="s1" compute={NOT_RUNNING} />)
    await screen.findAllByTestId('result-chip')
    fireEvent.click(screen.getByRole('button', { name: 'Accept check' }))
    fireEvent.click(within(await screen.findByRole('dialog')).getByRole('button', { name: 'Accept check' }))
    await screen.findByText('a compute is running; wait for it to finish')
    expect(screen.queryByText(/Accepted check/)).toBeNull()
  })

  it('opens the report on demand', async () => {
    serve([FAILED, PASSED])
    render(<RgbdResults sessionId="s1" compute={NOT_RUNNING} />)
    await screen.findAllByTestId('result-chip')
    fireEvent.click(screen.getAllByText('Report')[0])
    await screen.findByText('# Report text')
  })

  it('Download is an anchor to the result zip with the download attribute', async () => {
    serve([FAILED, PASSED])
    render(<RgbdResults sessionId="s1" compute={NOT_RUNNING} />)
    await screen.findAllByTestId('result-chip')
    const a = screen.getAllByRole('link', { name: 'Download' })[0]
    expect(a.getAttribute('href')).toBe('/api/calibration/rgbd/sessions/s1/results/r2/download')
    expect(a.hasAttribute('download')).toBe(true)
  })
})
