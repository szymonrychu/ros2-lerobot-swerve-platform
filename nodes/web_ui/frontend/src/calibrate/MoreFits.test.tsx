// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { act, cleanup, fireEvent, render, screen } from '@testing-library/react'
import { MoreFits } from './MoreFits'
import resultFx from '../../../../head_rgbd_calibration/tests/fixtures/api/head_arm_result.json'
import applyFx from '../../../../head_rgbd_calibration/tests/fixtures/api/floor_apply.json'
import * as api from './headArmApi'
import type { ArmResult, HeadArmSession } from './headArmApi'

vi.mock('./headArmApi')
const m = vi.mocked(api)
const ok = <T,>(value: T) => ({ ok: true as const, value })
const SESSION: HeadArmSession = { id: 'calib-20261031-110000', created: 'x', views: 60, arm_poses: 20, results: [] }
const DEG = Math.PI / 180

const RESULT: ArmResult = {
  id: 'r-20261031-120000',
  session: SESSION.id,
  kind: 'arm',
  floor_session: 'floor-20261030-090000',
  floor: { session: 'floor-20261030-090000', result: 'r-20261030-091000' },
  created: '2026-10-31T12:00:00+01:00',
  passed: true,
  failed_limits: [],
  error: null,
  rms_px: 0.61,
  holdout_rms_px: 0.72,
  holdout_tool_mm: 1.9,
  wrist_roll_fitted: false,
  tool_fitted: true,
  tool_offset_m: [0.001, -0.002, -0.0981],
  tool_offset_current_m: [0.001, -0.0056, -0.0014],
  offsets: {
    shoulder_pan: { value: 2 * DEG, sigma: 0.1 * DEG, current: 1 * DEG },
    wrist_roll: { value: 0, sigma: null, current: 0 },
  },
  head: {
    mount_x_m: { value: 0.1565, sigma: 0.0001, urdf: 0.1555 },
    mount_y_m: { value: 0.0995, sigma: 0.0001, urdf: 0.1 },
    mount_yaw_deg: { value: 0.25, sigma: 0.01, urdf: 0.1 },
  },
  limits: [{ name: 'rms_px', value: 0.61, limit: 1.5, ok: true }],
  snippet: '    joint_offsets_rad:\n      shoulder_pan: 0.0349\n',
}

const click = async (name: string | RegExp) => {
  await act(async () => {
    fireEvent.click(screen.getByRole('button', { name }))
  })
}

/** Renders the fits of a session and opens them. */
async function open(session: HeadArmSession | null = SESSION, disabled = false): Promise<void> {
  render(<MoreFits session={session} disabled={disabled} />)
  await act(async () => {})
  await click('More fits on this session')
}

beforeEach(() => {
  vi.resetAllMocks()
})
afterEach(() => {
  cleanup()
})

describe('MoreFits: head from arm', () => {
  it('is closed until opened', async () => {
    render(<MoreFits session={SESSION} disabled={false} />)
    expect(screen.queryByRole('button', { name: 'Compute head from arm' })).toBeNull()
  })

  it('Compute shows the result table with the d435_joint rows and Apply posts', async () => {
    m.computeHeadArm.mockResolvedValue(ok(resultFx as api.HeadArmResult))
    m.applyHeadArm.mockResolvedValue(ok({ ...applyFx, head_chain: { ...applyFx.head_chain, source: 'head_arm' } } as never))
    await open()
    await click('Compute head from arm')
    expect(m.computeHeadArm).toHaveBeenCalledWith(SESSION.id)
    await screen.findByText('Passed')
    expect(screen.getByText('Mount x (mm)')).toBeTruthy()
    expect(screen.getByText('155.95')).toBeTruthy()
    expect(screen.getByText(/Held-out board error 0.48 mm/)).toBeTruthy()
    for (const label of ['D435 x (mm)', 'D435 z (mm)', 'D435 roll (deg)', 'D435 yaw (deg)']) expect(screen.getByText(label)).toBeTruthy()
    const p = resultFx.params.camera_x_m
    expect(screen.getByText('D435 x (mm)').closest('tr')?.textContent).toContain((p.value * 1000).toFixed(2))
    await click('Apply head from arm')
    expect(m.applyHeadArm).toHaveBeenCalledWith(resultFx.session, resultFx.id)
    await screen.findByText(/ros2-head_camera_cpp.service: restarted/)
    expect(screen.getByText(/source head_arm/)).toBeTruthy()
  })

  it('shows the pan axis parameters and the RMS per head offset', async () => {
    m.computeHeadArm.mockResolvedValue(ok(resultFx as api.HeadArmResult))
    await open()
    await click('Compute head from arm')
    await screen.findByText('Passed')
    const row = (label: string) => screen.getByText(label).closest('tr')?.textContent
    expect(row('Pan axis x (mm)')).toBe('Pan axis x (mm)0.00-30.10-30.100.40--')
    expect(row('Pan axis y (mm)')).toBe('Pan axis y (mm)0.00-9.90-9.900.60--')
    expect(screen.getByText('Head offset 0: 24 views, RMS 0.41 px')).toBeTruthy()
  })

  it('a result without pan axis or per-offset RMS shows neither', async () => {
    const params = Object.fromEntries(Object.entries(resultFx.params).filter(([k]) => !k.startsWith('pan_axis')))
    const { offset_rms_px: _omit, ...older } = resultFx
    m.computeHeadArm.mockResolvedValue(ok({ ...older, params } as unknown as api.HeadArmResult))
    await open()
    await click('Compute head from arm')
    await screen.findByText('Passed')
    expect(screen.queryByText('Pan axis x (mm)')).toBeNull()
    expect(screen.queryByText(/Head offset 0/)).toBeNull()
  })

  it('Apply is disabled for a failed result and the limits show', async () => {
    m.computeHeadArm.mockResolvedValue(ok({ ...resultFx, passed: false, failed_limits: ['holdout_board_mm 3 > 1'] } as api.HeadArmResult))
    await open()
    await click('Compute head from arm')
    await screen.findByText('Failed')
    expect(screen.getByText('holdout_board_mm 3 > 1')).toBeTruthy()
    expect(screen.getByRole('button', { name: 'Apply head from arm' }).hasAttribute('disabled')).toBe(true)
  })

  it('the computes are off without views, without a session or while disabled', async () => {
    await open({ ...SESSION, views: 0 })
    expect(screen.getByRole('button', { name: 'Compute head from arm' }).hasAttribute('disabled')).toBe(true)
    expect(screen.getByRole('button', { name: 'Compute arm offsets' }).hasAttribute('disabled')).toBe(true)
    cleanup()
    await open(SESSION, true)
    expect(screen.getByRole('button', { name: 'Compute head from arm' }).hasAttribute('disabled')).toBe(true)
    cleanup()
    await open(null)
    expect(screen.getByText('No capture session yet.')).toBeTruthy()
  })
})

describe('MoreFits: arm offsets', () => {
  it('compute shows the result card', async () => {
    m.computeArm.mockResolvedValue(ok(RESULT))
    await open()
    await click('Compute arm offsets')
    expect(m.computeArm).toHaveBeenCalledWith(SESSION.id)
    await screen.findByText('Passed')
    const row = (label: string) => screen.getByText(label).closest('tr')?.textContent
    expect(row('shoulder_pan')).toBe('shoulder_pan1.0002.000+1.0000.100')
    expect(row('wrist_roll')).toBe('wrist_roll0.0000.000+0.000-')
    expect(screen.getByText(/wrist_roll fixed/)).toBeTruthy()
    expect(
      screen.getByText(/Tool point current \(mm\) 1.0, -5.6, -1.4, new \(mm\) 1.0, -2.0, -98.1 \(fitted\), held-out error 1.90 mm/),
    ).toBeTruthy()
    expect(screen.getByText(/Head change: x \+1.00 mm, y -0.50 mm, yaw \+0.150 deg/)).toBeTruthy()
    expect(screen.getByText(/shoulder_pan: 0.0349/)).toBeTruthy()
    const download = screen.getByRole('link', { name: 'Download zip' })
    expect(download.getAttribute('href')).toBe(`/api/calibration/rgbd/sessions/${SESSION.id}/results/${RESULT.id}/download`)
    expect(screen.getByText(/offsets are committed by hand/)).toBeTruthy()
  })

  it('a kept tool point says so', async () => {
    m.computeArm.mockResolvedValue(ok({ ...RESULT, tool_fitted: false, tool_offset_m: [0.001, -0.0056, -0.0014], wrist_roll_fitted: true }))
    await open()
    await click('Compute arm offsets')
    await screen.findByText('Passed')
    expect(screen.getByText(/Tool point current \(mm\) 1.0, -5.6, -1.4 \(kept\)/)).toBeTruthy()
    expect(screen.queryByText(/new \(mm\)/)).toBeNull()
    expect(screen.getByText(/wrist_roll fitted/)).toBeTruthy()
  })

  it('copy writes the snippet', async () => {
    const writeText = vi.fn(async () => undefined)
    Object.defineProperty(navigator, 'clipboard', { value: { writeText }, configurable: true })
    m.computeArm.mockResolvedValue(ok(RESULT))
    await open()
    await click('Compute arm offsets')
    await screen.findByText('Passed')
    await click('Copy')
    expect(writeText).toHaveBeenCalledWith(RESULT.snippet)
  })

  it('apply disabled on a failed result', async () => {
    m.computeArm.mockResolvedValue(ok({ ...RESULT, passed: false, failed_limits: ['holdout_tool_mm 9 > 5'] }))
    await open()
    await click('Compute arm offsets')
    await screen.findByText('Failed')
    expect(screen.getByText('holdout_tool_mm 9 > 5')).toBeTruthy()
    expect(screen.getByRole('button', { name: 'Apply head part' }).hasAttribute('disabled')).toBe(true)
  })

  it('an input error shows the error and no table', async () => {
    const { rms_px: _r, offsets: _o, head: _h, snippet: _s, ...base } = RESULT
    m.computeArm.mockResolvedValue(ok({ ...base, passed: false, error: 'plate_mount is not set' }))
    await open()
    await click('Compute arm offsets')
    await screen.findByText('Failed')
    expect(screen.getByText('plate_mount is not set')).toBeTruthy()
    expect(screen.queryByRole('table')).toBeNull()
    expect(screen.queryByRole('button', { name: 'Copy' })).toBeNull()
  })

  it('Apply head part posts a passed result and shows the restart state', async () => {
    m.computeArm.mockResolvedValue(ok(RESULT))
    m.applyHeadArm.mockResolvedValue(ok({ ...applyFx, head_chain: { ...applyFx.head_chain, source: 'arm' } } as never))
    await open()
    await click('Compute arm offsets')
    await screen.findByText('Passed')
    await click('Apply head part')
    expect(m.applyHeadArm).toHaveBeenCalledWith(RESULT.session, RESULT.id)
    await screen.findByText(/source arm/)
  })

  it('shows the latest arm result of the session through getHeadArmResult', async () => {
    m.getHeadArmResult.mockResolvedValue(ok(RESULT))
    await open({ ...SESSION, results: ['r-old', RESULT.id] })
    await screen.findByText('Passed')
    expect(m.getHeadArmResult).toHaveBeenCalledWith(SESSION.id, RESULT.id)
  })

  it('a node error shows in an Alert', async () => {
    m.computeArm.mockResolvedValue({ ok: false, status: 400, message: 'apply a head calibration first' })
    await open()
    await click('Compute arm offsets')
    expect((await screen.findByRole('alert')).textContent).toContain('apply a head calibration first')
  })
})
