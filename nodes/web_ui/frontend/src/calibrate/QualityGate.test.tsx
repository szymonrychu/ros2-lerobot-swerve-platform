// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen, within } from '@testing-library/react'
import { GateReport, QualityGate, armCameraGate, floorGate } from './QualityGate'
import type { ArmCameraReport } from './armCameraApi'
import type { FloorResult } from './headFloorApi'
import doneFx from './fixtures/arm_camera_result_done.json'
import refusedFx from './fixtures/arm_camera_result_refused.json'

const REPORT: GateReport = {
  plausible: true,
  refusals: [],
  warnings: ['few views'],
  score: [{ label: 'RMS', value: '0.62 px' }],
  params: [{ name: 'tilt zero', value: 1.5, sigma: 0.02, unit: 'deg', applied: 1.2, change: 0.3, digits: 2 }],
  accuracy: [{ label: 'tool point at 30 cm reach', value: 1.9, unit: 'mm' }],
  comparison: [{ label: 'held-out tool error (mm)', current: '4.2', next: '1.8' }],
}

afterEach(cleanup)

function renderGate(report: GateReport, decision: 'accepted' | 'declined' | null = null) {
  const onAccept = vi.fn()
  const onDecline = vi.fn()
  render(<QualityGate report={report} acceptBody="Applies it." onAccept={onAccept} onDecline={onDecline} decision={decision} />)
  return { onAccept, onDecline }
}

describe('QualityGate', () => {
  it('shows the score, params with sigma and change, accuracy in plain units, comparison and warnings', () => {
    renderGate(REPORT)
    expect(screen.getByText('0.62 px')).toBeTruthy()
    const row = screen.getByRole('row', { name: /tilt zero/ })
    expect(within(row).getByText('1.50')).toBeTruthy()
    expect(within(row).getByText('0.02')).toBeTruthy()
    expect(within(row).getByText('1.20')).toBeTruthy()
    expect(within(row).getByText('+0.30')).toBeTruthy()
    expect(screen.getByText('tool point at 30 cm reach: +-1.9 mm')).toBeTruthy()
    expect(screen.getByRole('row', { name: /held-out tool error/ }).textContent).toContain('4.2')
    expect(screen.getByText('few views')).toBeTruthy()
    expect(screen.getByText('Plausible')).toBeTruthy()
  })

  it('Accept asks for a confirm, then calls onAccept', () => {
    const { onAccept } = renderGate(REPORT)
    fireEvent.click(screen.getByRole('button', { name: 'Accept' }))
    expect(onAccept).not.toHaveBeenCalled()
    const dialog = screen.getByRole('dialog')
    expect(within(dialog).getByText('Applies it.')).toBeTruthy()
    fireEvent.click(within(dialog).getByRole('button', { name: 'Accept' }))
    expect(onAccept).toHaveBeenCalledTimes(1)
  })

  it('Decline calls onDecline at once', () => {
    const { onDecline } = renderGate(REPORT)
    fireEvent.click(screen.getByRole('button', { name: 'Decline' }))
    expect(onDecline).toHaveBeenCalledTimes(1)
  })

  it('an implausible report disables Accept and names the reasons', () => {
    renderGate({ ...REPORT, plausible: false, refusals: ['held-out error 9.4 mm above 5 mm'] })
    expect((screen.getByRole('button', { name: 'Accept' }) as HTMLButtonElement).disabled).toBe(true)
    expect(screen.getByText('Not plausible')).toBeTruthy()
    expect(screen.getByText('held-out error 9.4 mm above 5 mm')).toBeTruthy()
    expect((screen.getByRole('button', { name: 'Decline' }) as HTMLButtonElement).disabled).toBe(false)
  })

  it('after a decision both buttons are off and the decision is shown', () => {
    renderGate(REPORT, 'accepted')
    expect((screen.getByRole('button', { name: 'Accept' }) as HTMLButtonElement).disabled).toBe(true)
    expect((screen.getByRole('button', { name: 'Decline' }) as HTMLButtonElement).disabled).toBe(true)
    expect(screen.getByText('Accepted')).toBeTruthy()
  })

  it('a param without an applied value shows dashes', () => {
    renderGate({ ...REPORT, params: [{ ...REPORT.params[0], applied: null, change: null }] })
    const row = screen.getByRole('row', { name: /tilt zero/ })
    expect(within(row).getAllByText('-')).toHaveLength(2)
  })
})

describe('armCameraGate', () => {
  it('maps the arm-camera report', () => {
    const gate = armCameraGate(doneFx.report as ArmCameraReport)
    expect(gate.plausible).toBe(true)
    expect(gate.warnings).toEqual(['wrist_roll kept fixed: too little roll span'])
    expect(gate.score.map((s) => s.label)).toEqual(['RMS', 'Held-out RMS', 'Held-out tool error', 'Views', 'Arm poses'])
    expect(gate.score[2].value).toBe('1.8 mm')
    expect(gate.params[0]).toMatchObject({ name: 'shoulder_pan offset', value: 1.12, sigma: 0.05, unit: 'deg', applied: 1.04, change: 0.08 })
    expect(gate.accuracy).toEqual(doneFx.report.accuracy_examples)
    expect(gate.comparison).toEqual([{ label: 'heldout_tool_mm', current: '4.2', next: '1.8' }])
  })

  it('maps the refusals', () => {
    const gate = armCameraGate(refusedFx.report as ArmCameraReport)
    expect(gate.plausible).toBe(false)
    expect(gate.refusals).toEqual(['held-out tool error 9.4 mm above the 5 mm limit'])
  })

  it('a comparison entry that is a plain value is shown as text', () => {
    const gate = armCameraGate({ ...(doneFx.report as ArmCameraReport), comparison: { note: 'first calibration' } })
    expect(gate.comparison).toEqual([{ label: 'note', current: '-', next: 'first calibration' }])
  })
})

describe('floorGate', () => {
  const p = (value: number, sigma: number, urdf: number) => ({ value, sigma, urdf })
  const RESULT: FloorResult = {
    id: 'r1',
    session: 's1',
    created: 'now',
    passed: true,
    failed_limits: [],
    params: {
      mount_z_m: p(0.5, 0.0005, 0.49),
      mount_roll_deg: p(0.1, 0.01, 0),
      mount_pitch_deg: p(0.2, 0.02, 0),
      tilt_zero_deg: p(-0.3, 0.03, 0),
    },
    height_rms_mm: 1.2,
    normal_rms_deg: 0.3,
    poses: [],
    skipped: [],
  }

  it('maps the floor fit result: limits, scores, params in mm and deg', () => {
    const gate = floorGate(RESULT)
    expect(gate.plausible).toBe(true)
    expect(gate.score.map((s) => s.value)).toEqual(['1.2 mm', '0.3 deg', '0', '0'])
    expect(gate.params[0]).toMatchObject({ name: 'Mount z', unit: 'mm', value: 500, sigma: 0.5, applied: 490 })
    expect(gate.params[0].change).toBeCloseTo(10, 9)
    expect(gate.params.map((x) => x.name)).toEqual(['Mount z', 'Mount roll', 'Mount pitch', 'Tilt zero'])
    expect(gate.accuracy[0]).toEqual({ label: 'camera height', value: 0.5, unit: 'mm' })
  })

  it('failed limits are the refusals', () => {
    const gate = floorGate({ ...RESULT, passed: false, failed_limits: ['height RMS 9 mm above 5 mm'] })
    expect(gate.plausible).toBe(false)
    expect(gate.refusals).toEqual(['height RMS 9 mm above 5 mm'])
  })
})
