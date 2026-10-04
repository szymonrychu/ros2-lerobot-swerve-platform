import { describe, expect, it } from 'vitest'
import { parseActionResult } from '../map/mapActions'
import { parseWsFrame } from '../hooks/wsFrames'
import { batteryStatus, cutoffBanner, type BatteryConfig, type BatteryPayload } from './batteryStatus'

const CFG: BatteryConfig = { topic: '/battery_state', cells: 3, cutoff_cell_v: 2.8, resume_cell_v: 2.9, stale_s: 5 }

function payload(voltage: number, cutoff = false): BatteryPayload {
  return { voltage, cells: 3, cell_voltage: voltage / 3, cutoff, cutoff_v: 8.4, resume_v: 8.7, stamp: 1 }
}

const NOW = 100_000

describe('batteryStatus', () => {
  it('is grey/"--" without a payload or config', () => {
    expect(batteryStatus(undefined, CFG, NOW, NOW)).toMatchObject({ level: 'unknown', label: '--' })
    expect(batteryStatus(payload(11.4), CFG, NOW, null)).toMatchObject({ level: 'unknown', label: '--' })
    expect(batteryStatus(payload(11.4), null, NOW, NOW)).toMatchObject({ level: 'unknown', label: '--' })
    expect(batteryStatus(null, CFG, NOW, NOW)).toMatchObject({ level: 'unknown' })
  })

  it('is unknown when the last reading is older than stale_s', () => {
    expect(batteryStatus(payload(11.4), CFG, NOW + 5000, NOW).level).toBe('ok')
    expect(batteryStatus(payload(11.4), CFG, NOW + 5001, NOW)).toMatchObject({ level: 'unknown', label: '--' })
  })

  it('is green above the resume threshold and shows volts and per-cell', () => {
    const s = batteryStatus(payload(11.4), CFG, NOW, NOW)
    expect(s.level).toBe('ok')
    expect(s.color).toBe('success')
    expect(s.label).toBe('11.4 V')
    expect(s.perCellLabel).toBe('3.80 V/cell')
  })

  it('is amber between cut-off and resume', () => {
    expect(batteryStatus(payload(8.6), CFG, NOW, NOW)).toMatchObject({ level: 'low', color: 'warning' })
    expect(batteryStatus(payload(8.7), CFG, NOW, NOW).level).toBe('low')
    expect(batteryStatus(payload(8.71), CFG, NOW, NOW).level).toBe('ok')
  })

  it('is red when the backend guard reports cut-off, also while still above the cut-off voltage (hysteresis)', () => {
    expect(batteryStatus(payload(8.2, true), CFG, NOW, NOW)).toMatchObject({ level: 'cutoff', color: 'error' })
    expect(batteryStatus(payload(8.6, true), CFG, NOW, NOW).level).toBe('cutoff')
  })

  it('is red below the cut-off voltage even if the payload flag is missing', () => {
    expect(batteryStatus(payload(8.3), CFG, NOW, NOW).level).toBe('cutoff')
  })
})

describe('cutoffBanner', () => {
  it('shows the per-cell voltage in cut-off only', () => {
    const s = batteryStatus(payload(8.21, true), CFG, NOW, NOW)
    expect(cutoffBanner(s)).toBe('Battery below cut-off (2.74 V/cell) - commands are disabled')
    expect(cutoffBanner(batteryStatus(payload(11.4), CFG, NOW, NOW))).toBeNull()
    expect(cutoffBanner(batteryStatus(undefined, CFG, NOW, NOW))).toBeNull()
  })
})

describe('parseWsFrame', () => {
  it('parses topic envelopes', () => {
    expect(parseWsFrame('{"topic":"/a","data":{"v":1}}')).toEqual({ kind: 'envelope', topic: '/a', data: { v: 1 } })
  })

  it('parses error frames', () => {
    const raw = '{"type":"error","source":"battery","message":"battery below cut-off: 8.21 V"}'
    expect(parseWsFrame(raw)).toEqual({ kind: 'error', source: 'battery', message: 'battery below cut-off: 8.21 V' })
  })

  it('ignores malformed frames', () => {
    expect(parseWsFrame('not json')).toBeNull()
    expect(parseWsFrame('{}')).toBeNull()
  })
})

describe('503 battery rejection of a POST action', () => {
  it('surfaces the backend message as an error result', () => {
    const body = { ok: false, message: 'battery below cut-off: 8.21 V (2.74 V/cell < 2.80 V/cell); commands rejected' }
    expect(parseActionResult(503, body)).toEqual({ state: 'error', message: body.message })
  })
})
