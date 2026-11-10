// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen, within } from '@testing-library/react'
import { GateReport, QualityGate, armCameraGate, checkText, floorGate, groundBoardGate } from './QualityGate'
import type { ArmCameraReport } from './armCameraApi'
import type { FloorResult } from './headFloorApi'
import { doneResult, nodeResult, refusedResult } from './armCameraFixtures'
import type { GroundReport } from './groundBoardApi'
import { doneResult as groundDone, nodeResult as groundNode, refusedResult as groundRefused } from './groundBoardFixtures'

const doneReport = doneResult.report as ArmCameraReport
const nodeReport = nodeResult.report as ArmCameraReport

const REPORT: GateReport = {
  plausible: true,
  refusals: [],
  warnings: ['few views'],
  score: [{ label: 'RMS', value: '0.62 px' }],
  params: [{ name: 'tilt zero', value: 1.5, sigma: 0.02, unit: 'deg', applied: 1.2, change: 0.3, digits: 2 }],
  accuracy: [{ label: 'tool point at 30 cm reach', sigma: 1.9, p95: 5.1, unit: 'mm' }],
  comparison: [{ label: 'held-out tool error (mm)', current: '4.2', next: '1.8' }],
  notes: ['head chain fixed'],
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
    const accuracy = screen.getByRole('row', { name: /tool point at 30 cm reach/ })
    expect(within(accuracy).getByText('+-1.9 mm')).toBeTruthy()
    expect(within(accuracy).getByText('+-5.1 mm')).toBeTruthy()
    expect(screen.getByRole('columnheader', { name: '95 %' })).toBeTruthy()
    expect(screen.getByText('head chain fixed')).toBeTruthy()
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

  it('accuracy without any 95 % value has no 95 % column', () => {
    renderGate({ ...REPORT, accuracy: [{ label: 'camera height', sigma: 0.5, p95: null, unit: 'mm' }] })
    expect(screen.queryByRole('columnheader', { name: '95 %' })).toBeNull()
    expect(within(screen.getByRole('row', { name: /camera height/ })).getByText('+-0.5 mm')).toBeTruthy()
  })

  it('a gauge-held param shows held', () => {
    renderGate({ ...REPORT, params: [{ ...REPORT.params[0], sigma: null, held: true }] })
    expect(within(screen.getByRole('row', { name: /tilt zero/ })).getByText('held')).toBeTruthy()
  })

  it('a fitted param with an unknown sigma (null, not held) shows a dash, not held', () => {
    renderGate({ ...REPORT, params: [{ ...REPORT.params[0], sigma: null, held: false }] })
    const row = within(screen.getByRole('row', { name: /tilt zero/ }))
    expect(row.queryByText('held')).toBeNull()
    expect(row.getByText('-')).toBeTruthy()
  })

  it('a param without an applied value shows dashes', () => {
    renderGate({ ...REPORT, params: [{ ...REPORT.params[0], applied: null, change: null }] })
    const row = screen.getByRole('row', { name: /tilt zero/ })
    expect(within(row).getAllByText('-')).toHaveLength(2)
  })
})

describe('armCameraGate', () => {
  it('maps the arm-camera report of the node contract', () => {
    const gate = armCameraGate(doneReport)
    const score = doneReport.score
    expect(gate.plausible).toBe(true)
    expect(gate.refusals).toEqual([])
    expect(gate.warnings).toEqual([
      'fit corner RMS 3.4 px is over the advisory 3 px limit (value 3.40 px, limit 3 px)',
    ])
    expect(gate.score.map((s) => s.label)).toEqual(['RMS', 'Held-out RMS', 'Held-out tool error', 'Views', 'Arm poses'])
    expect(gate.score[2].value).toBe(`${score.heldout_tool_mm?.toFixed(1)} mm`)
    expect(gate.params[0]).toMatchObject({ name: 'shoulder_pan_offset', unit: 'deg', value: doneReport.params[0].value })
    expect(gate.params.find((p) => p.name === 'base_yaw')).toMatchObject({ sigma: null, held: true })
    expect(gate.params[0].held).toBe(false)
    expect(gate.accuracy).toEqual([{ label: 'tool point at 30 cm reach', sigma: doneReport.accuracy_examples[0].value, p95: 0.42, unit: 'mm' }])
    expect(gate.notes).toEqual(doneReport.notes)
  })

  it('compares the score of the calibration in effect with the new one and lists the tool shift', () => {
    const gate = armCameraGate(doneReport)
    const current = doneReport.comparison.current_score
    expect(gate.comparison[2]).toEqual({
      label: 'Held-out tool error',
      current: `${current.heldout_tool_mm?.toFixed(1)} mm`,
      next: `${doneReport.score.heldout_tool_mm?.toFixed(1)} mm`,
    })
    expect(gate.comparison[3]).toEqual({
      label: 'Shift of the tool point at 30 cm reach',
      current: '-',
      next: `${doneReport.comparison.shift_examples[0].value?.toFixed(1)} mm`,
    })
  })

  it('maps the refusals with value against limit', () => {
    const gate = armCameraGate(refusedResult.report as ArmCameraReport)
    expect(gate.plausible).toBe(false)
    expect(gate.refusals).toEqual([
      'elbow_flex offset change 20.0 deg is over the 15 deg plausibility bound (value 20 deg, limit 15 deg)',
    ])
  })

  it('null held-out scores read as dashes', () => {
    const gate = armCameraGate({ ...nodeReport, score: { ...nodeReport.score, heldout_rms_px: null, heldout_tool_mm: null } })
    expect(gate.score[1].value).toBe('-')
    expect(gate.score[2].value).toBe('-')
  })
})

describe('checkText', () => {
  it('shows a non-finite value or limit (null) as a dash', () => {
    expect(checkText({ name: 'n', value: null, limit: 1.5, unit: 'mm', message: 'm' })).toBe('m (value -, limit 1.50 mm)')
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
    expect(gate.accuracy[0]).toEqual({ label: 'camera height', sigma: 0.5, p95: null, unit: 'mm' })
  })

  it('failed limits are the refusals', () => {
    const gate = floorGate({ ...RESULT, passed: false, failed_limits: ['height RMS 9 mm above 5 mm'] })
    expect(gate.plausible).toBe(false)
    expect(gate.refusals).toEqual(['height RMS 9 mm above 5 mm'])
  })
})

describe('groundBoardGate', () => {
  const done = groundDone.report as GroundReport
  const node = groundNode.report as GroundReport
  const refused = groundRefused.report as GroundReport

  it('maps the ground board reports of the node contract: refusals and warnings with value against limit', () => {
    const gate = groundBoardGate(node)
    expect(gate.plausible).toBe(true)
    expect(gate.refusals).toEqual([])
    expect(gate.warnings).toEqual([checkText(node.warnings[0])])
    expect(gate.notes).toEqual(node.notes)
    const refusedGate = groundBoardGate(refused)
    expect(refusedGate.plausible).toBe(false)
    expect(refusedGate.refusals).toEqual([checkText(refused.refusals[0])])
    expect(refusedGate.refusals[0]).toContain('value 120 mm, limit 100 mm')
    expect(refusedGate.warnings).toEqual([checkText(refused.warnings[0])])
  })

  it('scores the corner RMS, the held-out RMS, the counts and the depth cross-check', () => {
    const labels = Object.fromEntries(groundBoardGate(node).score.map((s) => [s.label, s.value]))
    expect(labels).toEqual({
      RMS: '0.42 px',
      'Held-out RMS': '0.42 px',
      Views: '69',
      'Head poses': '69',
      'Board placements': '3',
      'Depth floor height RMS': '1.6 mm',
      'Depth floor normal RMS': '0.22 deg',
    })
  })

  it('pairs the accuracy examples (1 sigma and 95 %)', () => {
    expect(groundBoardGate(done).accuracy).toEqual([
      { label: 'floor height 1 m ahead of the head', sigma: node.accuracy_examples[0].value, p95: 0.25, unit: 'mm' },
    ])
  })

  it('marks the head mount gauge as held and keeps a fitted sigma', () => {
    const params = Object.fromEntries(groundBoardGate(done).params.map((p) => [p.name, p]))
    expect(params.mount_yaw_deg).toMatchObject({ held: true, sigma: null, change: 0 })
    expect(params.mount_x_m).toMatchObject({ held: true, unit: 'mm', applied: 150 })
    expect(params.tilt_zero_deg).toMatchObject({ held: false, sigma: 0.01, applied: 0.6, change: 0.6 })
  })

  it('compares the applied chain on this session with the new one, the shifts and the depth cross-check', () => {
    const rows = groundBoardGate(node).comparison
    expect(rows[0]).toEqual({ label: 'RMS', current: '5.43 px', next: '0.42 px' })
    expect(rows[1]).toEqual({ label: 'Held-out RMS', current: '8.48 px', next: '0.42 px' })
    expect(rows).toContainEqual({ label: 'Depth floor height RMS', current: '1.5 mm', next: '1.6 mm' })
    expect(rows).toContainEqual({ label: 'Depth floor normal RMS', current: '0.94 deg', next: '0.22 deg' })
    expect(rows).toContainEqual({ label: 'Shift of the floor height 1 m ahead of the head', current: '-', next: '-1.0 mm' })
  })

  it('a refused report is not plausible', () => {
    expect(groundBoardGate(groundRefused.report as GroundReport).plausible).toBe(false)
  })
})
