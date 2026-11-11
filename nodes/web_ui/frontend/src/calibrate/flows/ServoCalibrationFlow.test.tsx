// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { act, cleanup, render, screen, waitFor, within } from '@testing-library/react'
import { ServoCalibrationFlow } from './ServoCalibrationFlow'
import type { ServoGroup } from '../servoApi'

function group(id: string, over: Partial<ServoGroup> = {}, capturing = false): ServoGroup {
  return {
    id,
    label: id[0].toUpperCase() + id.slice(1),
    neutral: true,
    state: {
      namespace: id,
      released: true,
      joints: { a: 0 },
      stamp_ns: 1,
      neutral_enabled: true,
      neutral_capture: null,
      last_save: null,
      neutral_max_shift_deg: 15,
      servo_calibration: {
        config: {} as never,
        home: null,
        capture: capturing ? { running: true, paused: false, samples: 1, joints: {} } : null,
        plan: null,
        last_result: null,
      },
    },
    age_s: 0.1,
    stale: false,
    calibration_stale: null,
    zero_restore: false,
    move_to_neutral: false,
    set_home: false,
    servo_calibration: true,
    ...over,
  }
}

let groups: ServoGroup[]
let fetchCount: number

beforeEach(() => {
  fetchCount = 0
  vi.stubGlobal(
    'fetch',
    vi.fn(async () => {
      fetchCount += 1
      return new Response(JSON.stringify({ groups }))
    }),
  )
})
afterEach(() => {
  cleanup()
  vi.useRealTimers()
  vi.unstubAllGlobals()
})

describe('ServoCalibrationFlow', () => {
  it('renders one card per calibration group', async () => {
    groups = [group('arm'), group('head'), group('base', { servo_calibration: false })]
    render(<ServoCalibrationFlow />)
    await screen.findByRole('heading', { name: 'Arm', level: 3 })
    expect(screen.getByRole('heading', { name: 'Head', level: 3 })).toBeTruthy()
    expect(screen.queryByRole('heading', { name: 'Base', level: 3 })).toBeNull()
    expect(screen.getAllByRole('button', { name: 'Record home' })).toHaveLength(2)
  })

  it('advanced section holds the neutral and zero restore panels', async () => {
    groups = [group('arm', { zero_restore: true, set_home: true, move_to_neutral: true })]
    render(<ServoCalibrationFlow />)
    // Plain DOM queries: the accordion is collapsed, so role queries would skip its content (and are slow here).
    const summary = await screen.findByText('Advanced / repair')
    const accordion = summary.closest('.MuiAccordion-root') as HTMLElement
    await waitFor(() => expect(accordion.querySelector('ol[aria-label="Servo home calibration steps"]')).not.toBeNull())
    const steps = within(accordion.querySelector('ol[aria-label="Servo home calibration steps"]') as HTMLElement)
    expect(steps.getByText('1. Record and save neutral')).toBeTruthy()
    expect(steps.getByText('2. Arm zero restore')).toBeTruthy()
    expect(steps.getByText('3. Set home')).toBeTruthy()
    expect(accordion.querySelector('ol[aria-label="Arm servo calibration steps"]')).toBeNull()
  })

  it('polls faster while a capture runs', async () => {
    groups = [group('head')]
    vi.useFakeTimers({ shouldAdvanceTime: true })
    render(<ServoCalibrationFlow />)
    await screen.findByRole('heading', { name: 'Head', level: 3 })
    await act(async () => {
      await vi.advanceTimersByTimeAsync(3000)
    })
    const slow = fetchCount
    expect(slow).toBeLessThanOrEqual(8)
    groups = [group('head', {}, true)]
    await act(async () => {
      await vi.advanceTimersByTimeAsync(1100)
    })
    const before = fetchCount
    await act(async () => {
      await vi.advanceTimersByTimeAsync(1000)
    })
    expect(fetchCount - before).toBeGreaterThanOrEqual(3)
  })
})
