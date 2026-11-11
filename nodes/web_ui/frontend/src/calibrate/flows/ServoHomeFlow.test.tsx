// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen, waitFor, within } from '@testing-library/react'
import { ServoHomeFlow, servoGate } from './ServoHomeFlow'
import type { ServoGroup } from '../servoApi'

function group(id: string, over: Partial<ServoGroup> = {}): ServoGroup {
  return {
    id,
    label: id[0].toUpperCase() + id.slice(1),
    neutral: true,
    state: {
      namespace: id,
      released: false,
      joints: { a: 1 },
      stamp_ns: 1,
      neutral_enabled: true,
      neutral_capture: null,
      last_save: null,
      neutral_max_shift_deg: 15,
    },
    age_s: 0.1,
    stale: false,
    calibration_stale: null,
    zero_restore: false,
    move_to_neutral: false,
    set_home: false,
    servo_calibration: false,
    ...over,
  }
}

let groups: ServoGroup[]
let posts: string[]

function mockFetch(): void {
  posts = []
  vi.stubGlobal(
    'fetch',
    vi.fn(async (url: string, init?: RequestInit) => {
      if (init?.method === 'POST') {
        posts.push(url)
        return new Response(JSON.stringify({ ok: true, message: 'converged' }))
      }
      return new Response(JSON.stringify({ groups }))
    }),
  )
}

afterEach(() => {
  cleanup()
  vi.unstubAllGlobals()
})

const item = (n: number) => within(screen.getByRole('list', { name: 'Servo home calibration steps' })).getAllByRole('listitem')[n]

describe('servoGate', () => {
  it('all groups on and nothing stale is plausible', () => {
    const gate = servoGate([group('arm'), group('head')])
    expect(gate.plausible).toBe(true)
    expect(gate.score).toEqual([
      { label: 'Arm', value: 'Torque on' },
      { label: 'Head', value: 'Torque on' },
    ])
  })

  it('a released, unknown or stale group is refused with the reason', () => {
    const released = group('base', { state: { ...group('base').state!, joints: { a: 0 }, released: true } })
    const gate = servoGate([released, group('head', { stale: true }), group('arm', { calibration_stale: ['joint_offsets', 'home'] })])
    expect(gate.plausible).toBe(false)
    expect(gate.refusals).toEqual([
      'Base: torque is off',
      'Head: no fresh torque state',
      'Arm: calibration stale (joint_offsets, home)',
    ])
  })

  it('no groups is refused', () => {
    expect(servoGate([]).refusals).toEqual(['No servo group state yet'])
  })
})

describe('ServoHomeFlow', () => {
  it('shows the five steps; the gate unlocks after Move to neutral pose', async () => {
    groups = [group('arm', { move_to_neutral: true, zero_restore: true, set_home: true })]
    mockFetch()
    render(<ServoHomeFlow />)
    await waitFor(() => expect(within(item(3)).getByRole('button', { name: 'Move to neutral pose' })).toBeTruthy())
    expect(within(item(0)).getByText('1. Record and save neutral')).toBeTruthy()
    expect(within(item(1)).getByText('2. Arm zero restore')).toBeTruthy()
    expect(within(item(2)).getByText('3. Set home')).toBeTruthy()
    expect(within(item(3)).getByText('4. Move to neutral pose')).toBeTruthy()
    expect(within(item(4)).getByText('5. Check and accept')).toBeTruthy()
    expect(within(item(4)).getByText('Finish step 4 first.')).toBeTruthy()
    fireEvent.click(within(item(3)).getByRole('button', { name: 'Move to neutral pose' }))
    fireEvent.click(within(screen.getByRole('dialog')).getByRole('button', { name: 'Move' }))
    await waitFor(() => expect(posts).toEqual(['/api/calibration/servos/arm/move_to_neutral']))
    await waitFor(() => expect(within(item(4)).getByText('Plausible')).toBeTruthy())
  })

  it('after a zero restore the Set home step shows home still stale and records the pose without motion', async () => {
    // The zero restore apply cleared joint_offsets from the marker; home stays stale until Set home.
    groups = [group('arm', { move_to_neutral: true, zero_restore: true, set_home: true, calibration_stale: ['home'] })]
    mockFetch()
    render(<ServoHomeFlow />)
    const marker = await within(item(2)).findByRole('region', { name: 'Stale marker' })
    expect(within(marker).getByText('joint_offsets: ok')).toBeTruthy()
    expect(within(marker).getByText('home: stale')).toBeTruthy()
    fireEvent.click(within(item(2)).getByRole('button', { name: 'Set home' }))
    const dialog = screen.getByRole('dialog')
    expect(within(dialog).getByText(/Nothing moves/)).toBeTruthy()
    expect(posts).toEqual([])
    groups = [group('arm', { move_to_neutral: true, zero_restore: true, set_home: true, calibration_stale: [] })]
    fireEvent.click(within(dialog).getByRole('button', { name: 'Set home' }))
    await waitFor(() => expect(posts).toEqual(['/api/calibration/servos/arm/set_home']))
    await waitFor(() => expect(within(item(2)).getByText('home: ok')).toBeTruthy())
  })

  it('without a group offering Set home the step says so', async () => {
    groups = [group('head')]
    mockFetch()
    render(<ServoHomeFlow />)
    expect(await within(item(2)).findByText('No servo group offers Set home.')).toBeTruthy()
  })

  it('Accept confirms and records the check; nothing is posted', async () => {
    groups = [group('head')]
    mockFetch()
    render(<ServoHomeFlow />)
    await waitFor(() => expect(within(item(4)).getByRole('button', { name: 'Accept' })).toBeTruthy())
    fireEvent.click(within(item(4)).getByRole('button', { name: 'Accept' }))
    fireEvent.click(within(screen.getByRole('dialog')).getByRole('button', { name: 'Accept' }))
    await waitFor(() => expect(screen.queryByRole('dialog')).toBeNull())
    expect(await within(item(4)).findByText('Accepted')).toBeTruthy()
    expect(posts).toEqual([])
  })

  it('Decline offers a fresh start', async () => {
    groups = [group('head')]
    mockFetch()
    render(<ServoHomeFlow />)
    await waitFor(() => expect(within(item(4)).getByRole('button', { name: 'Decline' })).toBeTruthy())
    fireEvent.click(within(item(4)).getByRole('button', { name: 'Decline' }))
    fireEvent.click(await within(item(4)).findByRole('button', { name: 'Start over' }))
    await waitFor(() => expect(within(item(4)).queryByText('Declined')).toBeNull())
  })
})
