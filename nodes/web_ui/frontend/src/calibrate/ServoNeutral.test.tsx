// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen, waitFor, within } from '@testing-library/react'
import { SET_HOME_NO_READINGS_REASON, SET_HOME_STALE_REASON, SET_HOME_TEXT, ServoNeutral, markerState } from './ServoNeutral'
import type { ServoGroup, TorqueState } from './servoApi'

function state(joints: Record<string, number>, capture: TorqueState['neutral_capture'] = null): TorqueState {
  const released = Object.values(joints).some((v) => v === 0)
  return {
    namespace: 'x',
    released,
    joints,
    stamp_ns: 1,
    neutral_enabled: true,
    neutral_capture: capture,
    last_save: null,
    neutral_max_shift_deg: 15,
  }
}

function group(id: string, over: Partial<ServoGroup> = {}): ServoGroup {
  return {
    id,
    label: id[0].toUpperCase() + id.slice(1),
    neutral: true,
    state: state({ a: 1, b: 1 }),
    age_s: 0.1,
    stale: false,
    calibration_stale: null,
    zero_restore: false,
    move_to_neutral: false,
    set_home: false,
    ...over,
  }
}

let groups: ServoGroup[]
let posts: string[]
let postReply: { status: number; body: unknown }

function mockFetch(): void {
  posts = []
  postReply = { status: 200, body: { ok: true, message: 'done' } }
  vi.stubGlobal(
    'fetch',
    vi.fn(async (url: string, init?: RequestInit) => {
      if (init?.method === 'POST') {
        posts.push(url)
        return new Response(JSON.stringify(postReply.body), { status: postReply.status })
      }
      return new Response(JSON.stringify({ groups }), { status: 200 })
    }),
  )
}

afterEach(() => {
  cleanup()
  vi.unstubAllGlobals()
})

const card = (label: string) => screen.getByRole('region', { name: label })
const inCard = async (label: string) => {
  await screen.findByRole('region', { name: label })
  return card(label)
}
const btn = (label: string, name: string) => {
  return within(card(label)).getByRole('button', { name })
}

describe('ServoNeutral', () => {
  it('shows chip states', async () => {
    groups = [
      group('arm'),
      group('base', { state: state({ a: 0, b: 0 }) }),
      group('head', { state: state({ a: 0, b: 1 }) }),
    ]
    mockFetch()
    render(<ServoNeutral />)
    await inCard('Arm')
    expect(within(card('Arm')).getByTestId('torque-chip').textContent).toBe('Torque on')
    expect(within(card('Base')).getByTestId('torque-chip').textContent).toBe('Torque off')
    expect(within(card('Head')).getByTestId('torque-chip').textContent).toBe('Partly off')
  })

  it('null or stale state is Unknown', async () => {
    groups = [group('arm', { state: null, stale: true }), group('base', { stale: true })]
    mockFetch()
    render(<ServoNeutral />)
    await inCard('Arm')
    expect(within(card('Arm')).getByTestId('torque-chip').textContent).toBe('Unknown')
    expect(within(card('Base')).getByTestId('torque-chip').textContent).toBe('Unknown')
  })

  it('shows per-joint torque', async () => {
    groups = [group('arm', { state: state({ shoulder: 1, elbow: 0 }) })]
    mockFetch()
    render(<ServoNeutral />)
    await inCard('Arm')
    expect(within(card('Arm')).getByText('shoulder: on')).toBeTruthy()
    expect(within(card('Arm')).getByText('elbow: off')).toBeTruthy()
  })

  it.each([
    ['arm', 'Support the arm: it goes limp and will drop.'],
    ['head', 'Support the head: it goes limp.'],
    ['base', 'The steering servos go limp; the wheels stay braked.'],
  ])('Torque off on %s confirms with the exact text then releases', async (id, text) => {
    groups = [group(id)]
    mockFetch()
    render(<ServoNeutral />)
    const label = id[0].toUpperCase() + id.slice(1)
    await inCard(label)
    fireEvent.click(btn(label, 'Torque off'))
    expect(await screen.findByText(text)).toBeTruthy()
    expect(posts).toEqual([])
  })

  it('confirming Torque off posts release; cancelling does not', async () => {
    groups = [group('head')]
    mockFetch()
    render(<ServoNeutral />)
    await inCard('Head')
    fireEvent.click(btn('Head', 'Torque off'))
    fireEvent.click(await screen.findByRole('button', { name: 'Cancel' }))
    await waitFor(() => expect(screen.queryByText('Support the head: it goes limp.')).toBeNull())
    await waitFor(() => expect(screen.queryByRole('dialog')).toBeNull())
    expect(posts).toEqual([])
    fireEvent.click(btn('Head', 'Torque off'))
    const dialog = await screen.findByRole('dialog')
    fireEvent.click(within(dialog).getByRole('button', { name: 'Torque off' }))
    await waitFor(() => expect(posts).toEqual(['/api/calibration/servos/head/release']))
  })

  it('Record neutral is enabled only while released and shows shifts in degrees', async () => {
    groups = [group('head')]
    mockFetch()
    render(<ServoNeutral />)
    await inCard('Head')
    expect((btn('Head', 'Record neutral') as HTMLButtonElement).disabled).toBe(true)
    cleanup()
    groups = [group('head', { state: state({ pan: 0, tilt: 0 }) })]
    mockFetch()
    render(<ServoNeutral />)
    await inCard('Head')
    expect((btn('Head', 'Record neutral') as HTMLButtonElement).disabled).toBe(false)
    fireEvent.click(btn('Head', 'Record neutral'))
    await waitFor(() => expect(posts).toEqual(['/api/calibration/servos/head/record']))
  })

  it('shows each captured joint shift in degrees', async () => {
    groups = [
      group('head', {
        state: state({ pan: 0, tilt: 0 }, { pan: { steps: 2048 + 100, delta_steps: 100, exceeds_max_shift: false }, tilt: { steps: 2048 - 51, delta_steps: -51, exceeds_max_shift: false } }),
      }),
    ]
    mockFetch()
    render(<ServoNeutral />)
    await inCard('Head')
    expect(within(card('Head')).getByText(/pan: \+8\.8/)).toBeTruthy()
    expect(within(card('Head')).getByText(/tilt: -4\.5/)).toBeTruthy()
  })

  it('Record neutral is absent for a group without neutral', async () => {
    groups = [group('base', { neutral: false })]
    mockFetch()
    render(<ServoNeutral />)
    await inCard('Base')
    expect(within(card('Base')).queryByRole('button', { name: 'Record neutral' })).toBeNull()
    expect(within(card('Base')).queryByRole('button', { name: 'Re-enable torque + save' })).toBeNull()
  })

  it('save needs a capture and confirms before posting', async () => {
    groups = [group('head', { state: state({ pan: 0 }) })]
    mockFetch()
    render(<ServoNeutral />)
    await inCard('Head')
    expect((btn('Head', 'Re-enable torque + save') as HTMLButtonElement).disabled).toBe(true)
    cleanup()
    groups = [group('head', { state: state({ pan: 0 }, { pan: { steps: 2050, delta_steps: 2, exceeds_max_shift: false } }) })]
    mockFetch()
    render(<ServoNeutral />)
    await inCard('Head')
    fireEvent.click(btn('Head', 'Re-enable torque + save'))
    expect(await screen.findByText('Writes the new neutral into the servo EEPROM.')).toBeTruthy()
    expect(screen.queryByText(/Teleop needs the leader arm re-zeroed/)).toBeNull()
    expect(posts).toEqual([])
    fireEvent.click(within(screen.getByRole('dialog')).getByRole('button', { name: 'Re-enable torque + save' }))
    await waitFor(() => expect(posts).toEqual(['/api/calibration/servos/head/save']))
  })

  it('save confirm lists the recorded shift of every joint', async () => {
    const capture = {
      pan: { steps: 2148, delta_steps: 100, exceeds_max_shift: false },
      tilt: { steps: 1997, delta_steps: -51, exceeds_max_shift: false },
    }
    groups = [group('head', { state: state({ pan: 0, tilt: 0 }, capture) })]
    mockFetch()
    render(<ServoNeutral />)
    await inCard('Head')
    fireEvent.click(btn('Head', 'Re-enable torque + save'))
    const dialog = await screen.findByRole('dialog')
    expect(within(dialog).getByText('pan: +8.8 deg')).toBeTruthy()
    expect(within(dialog).getByText('tilt: -4.5 deg')).toBeTruthy()
    expect(within(dialog).queryByRole('checkbox')).toBeNull()
  })

  it('a shift above the limit is flagged on the card and needs the explicit confirm, then posts save_confirmed', async () => {
    const capture = {
      shoulder_lift: { steps: 2048 - 933, delta_steps: -933, exceeds_max_shift: true },
      elbow_flex: { steps: 2048 + 1030, delta_steps: 1030, exceeds_max_shift: true },
      wrist_flex: { steps: 2058, delta_steps: 10, exceeds_max_shift: false },
    }
    groups = [group('arm', { state: state({ shoulder_lift: 0, elbow_flex: 0, wrist_flex: 0 }, capture) })]
    mockFetch()
    render(<ServoNeutral />)
    await inCard('Arm')
    expect(within(card('Arm')).getByText('elbow_flex: +90.5 (above 15 deg)')).toBeTruthy()
    expect(within(card('Arm')).getByText('wrist_flex: +0.9')).toBeTruthy()
    fireEvent.click(btn('Arm', 'Re-enable torque + save'))
    const dialog = await screen.findByRole('dialog')
    expect(within(dialog).getByText(/shoulder_lift, elbow_flex shift more than 15 deg/)).toBeTruthy()
    expect(within(dialog).getByText('elbow_flex: +90.5 deg (above 15 deg)')).toBeTruthy()
    const confirm = within(dialog).getByRole('button', { name: 'Save large shift' }) as HTMLButtonElement
    expect(confirm.disabled).toBe(true)
    fireEvent.click(confirm)
    expect(posts).toEqual([])
    fireEvent.click(within(dialog).getByRole('checkbox', { name: /The large shift is intended/ }))
    expect(confirm.disabled).toBe(false)
    fireEvent.click(confirm)
    await waitFor(() => expect(posts).toEqual(['/api/calibration/servos/arm/save_confirmed']))
  })

  it('cancelling the large-shift confirm posts nothing and resets the checkbox', async () => {
    const capture = { elbow_flex: { steps: 3078, delta_steps: 1030, exceeds_max_shift: true } }
    groups = [group('arm', { state: state({ elbow_flex: 0 }, capture) })]
    mockFetch()
    render(<ServoNeutral />)
    await inCard('Arm')
    fireEvent.click(btn('Arm', 'Re-enable torque + save'))
    let dialog = await screen.findByRole('dialog')
    fireEvent.click(within(dialog).getByRole('checkbox', { name: /The large shift is intended/ }))
    fireEvent.click(within(dialog).getByRole('button', { name: 'Cancel' }))
    await waitFor(() => expect(screen.queryByRole('dialog')).toBeNull())
    fireEvent.click(btn('Arm', 'Re-enable torque + save'))
    dialog = await screen.findByRole('dialog')
    expect((within(dialog).getByRole('checkbox', { name: /The large shift is intended/ }) as HTMLInputElement).checked).toBe(false)
    expect(posts).toEqual([])
  })

  it('arm save confirm carries the stale and leader warnings', async () => {
    groups = [group('arm', { state: state({ a: 0 }, { a: { steps: 2050, delta_steps: 2, exceeds_max_shift: false } }) })]
    mockFetch()
    render(<ServoNeutral />)
    await inCard('Arm')
    fireEvent.click(btn('Arm', 'Re-enable torque + save'))
    const dialog = await screen.findByRole('dialog')
    expect(within(dialog).getByText(/Writes the new neutral into the servo EEPROM\./)).toBeTruthy()
    expect(
      within(dialog).getByText(/Arm joint offsets and the stored home become stale until re-calibrated\. Teleop needs the leader arm re-zeroed in the same pose\./),
    ).toBeTruthy()
  })

  it('Re-enable torque is enabled while released and posts enable without a dialog', async () => {
    groups = [group('base', { state: state({ a: 0 }) })]
    mockFetch()
    render(<ServoNeutral />)
    await inCard('Base')
    fireEvent.click(btn('Base', 'Re-enable torque'))
    await waitFor(() => expect(posts).toEqual(['/api/calibration/servos/base/enable']))
    cleanup()
    groups = [group('base')]
    mockFetch()
    render(<ServoNeutral />)
    await inCard('Base')
    expect((btn('Base', 'Re-enable torque') as HTMLButtonElement).disabled).toBe(true)
  })

  it('arm card warns when calibration_stale is non-empty', async () => {
    groups = [group('arm', { calibration_stale: ['joint_offsets', 'home'] }), group('head')]
    mockFetch()
    render(<ServoNeutral />)
    await inCard('Arm')
    expect(within(card('Arm')).getByText(/joint_offsets, home/)).toBeTruthy()
    expect(within(card('Head')).queryByText(/stale/i)).toBeNull()
  })

  it('shows the backend message and a refusal in an Alert', async () => {
    groups = [group('base', { state: state({ a: 0 }) })]
    mockFetch()
    postReply = { status: 409, body: { ok: false, message: 'refused: moved' } }
    render(<ServoNeutral />)
    await inCard('Base')
    fireEvent.click(btn('Base', 'Re-enable torque'))
    const alert = await screen.findByRole('alert')
    expect(alert.textContent).toContain('refused: moved')
  })

  it('disables the buttons while a call is in flight', async () => {
    groups = [group('base', { state: state({ a: 0 }) })]
    mockFetch()
    let release: (r: Response) => void = () => {}
    const base = fetch as unknown as ReturnType<typeof vi.fn>
    vi.stubGlobal(
      'fetch',
      vi.fn((url: string, init?: RequestInit) =>
        init?.method === 'POST' ? new Promise<Response>((res) => (release = res)) : base(url, init),
      ),
    )
    render(<ServoNeutral />)
    await inCard('Base')
    fireEvent.click(btn('Base', 'Re-enable torque'))
    await waitFor(() => expect((btn('Base', 'Torque off') as HTMLButtonElement).disabled).toBe(true))
    expect((btn('Base', 'Re-enable torque') as HTMLButtonElement).disabled).toBe(true)
    release(new Response(JSON.stringify({ ok: true, message: 'done' })))
    await waitFor(() => expect((btn('Base', 'Re-enable torque') as HTMLButtonElement).disabled).toBe(false))
  })

  it('a network error from the poll is surfaced', async () => {
    vi.stubGlobal('fetch', vi.fn(async () => { throw new Error('link down') }))
    render(<ServoNeutral />)
    expect((await screen.findByRole('alert')).textContent).toContain('link down')
  })

  it('a 404 poll shows the not-configured notice and no error', async () => {
    vi.stubGlobal('fetch', vi.fn(async () => new Response(JSON.stringify({ detail: 'nope' }), { status: 404 })))
    render(<ServoNeutral />)
    expect((await screen.findByText('No servo groups configured on this robot'))).toBeTruthy()
    expect(screen.queryByText(/HTTP 404/)).toBeNull()
  })

  it('parts limit the card to the move to neutral button', async () => {
    groups = [group('arm', { move_to_neutral: true, zero_restore: true })]
    mockFetch()
    render(<ServoNeutral parts={['move']} />)
    await inCard('Arm')
    expect(btn('Arm', 'Move to neutral pose')).toBeTruthy()
    expect(within(card('Arm')).queryByRole('button', { name: 'Torque off' })).toBeNull()
    expect(screen.queryByRole('region', { name: 'Arm zero restore' })).toBeNull()
  })

  it('parts limit the card to the zero restore panel of the groups that have one', async () => {
    groups = [group('arm', { move_to_neutral: true, zero_restore: true }), group('head')]
    mockFetch()
    render(<ServoNeutral parts={['zero']} />)
    await screen.findByRole('region', { name: 'Arm zero restore' })
    expect(screen.queryByRole('region', { name: 'Head' })).toBeNull()
    expect(within(card('Arm')).queryByRole('button', { name: 'Move to neutral pose' })).toBeNull()
    expect(within(card('Arm')).queryByRole('button', { name: 'Re-enable torque' })).toBeNull()
  })

  it('the home part shows the stale marker items and Set home behind a no-motion confirm', async () => {
    groups = [group('arm', { set_home: true, move_to_neutral: true, calibration_stale: ['home'] })]
    mockFetch()
    render(<ServoNeutral parts={['home']} />)
    await inCard('Arm')
    const marker = within(card('Arm')).getByRole('region', { name: 'Stale marker' })
    expect(within(marker).getByText('joint_offsets: ok')).toBeTruthy()
    expect(within(marker).getByText('home: stale')).toBeTruthy()
    expect(within(card('Arm')).queryByRole('button', { name: 'Move to neutral pose' })).toBeNull()
    expect(within(card('Arm')).queryByRole('button', { name: 'Torque off' })).toBeNull()
    fireEvent.click(btn('Arm', 'Set home'))
    const dialog = screen.getByRole('dialog')
    expect(within(dialog).getByText(SET_HOME_TEXT)).toBeTruthy()
    fireEvent.click(within(dialog).getByRole('button', { name: 'Set home' }))
    await waitFor(() => expect(posts).toContain('/api/calibration/servos/arm/set_home'))
  })

  it('Set home is off with the reason while joint_offsets is stale, and absent without the service', async () => {
    groups = [group('arm', { set_home: true, calibration_stale: ['joint_offsets', 'home'] }), group('head')]
    mockFetch()
    render(<ServoNeutral parts={['home']} />)
    await inCard('Arm')
    expect((btn('Arm', 'Set home') as HTMLButtonElement).disabled).toBe(true)
    expect(within(card('Arm')).getByText(SET_HOME_STALE_REASON)).toBeTruthy()
    expect(screen.queryByRole('region', { name: 'Head' })).toBeNull()
  })

  it('Set home is off with the reason while the follower joint readings are stale', async () => {
    groups = [group('arm', { set_home: true, stale: true, calibration_stale: ['home'] })]
    mockFetch()
    render(<ServoNeutral parts={['home']} />)
    await inCard('Arm')
    expect((btn('Arm', 'Set home') as HTMLButtonElement).disabled).toBe(true)
    expect(within(card('Arm')).getByText(SET_HOME_NO_READINGS_REASON)).toBeTruthy()
    expect(SET_HOME_NO_READINGS_REASON).toContain('no fresh arm joint readings')
  })

  it('Set home is off without any follower state', async () => {
    groups = [group('arm', { set_home: true, state: null, age_s: null })]
    mockFetch()
    render(<ServoNeutral parts={['home']} />)
    await inCard('Arm')
    expect((btn('Arm', 'Set home') as HTMLButtonElement).disabled).toBe(true)
    expect(within(card('Arm')).getByText(SET_HOME_NO_READINGS_REASON)).toBeTruthy()
  })

  it('SET_HOME_TEXT says it records the current joint readings', () => {
    expect(SET_HOME_TEXT).toContain('current joint readings')
  })

  it('a Set home refused by mcp_server shows its message as an error', async () => {
    groups = [group('arm', { set_home: true, calibration_stale: ['home'] })]
    mockFetch()
    const refusal = 'set home refused: the arm zero is stale (joint_offsets); run the zero restore first (arm re-zero at x)'
    postReply = { status: 409, body: { ok: false, message: refusal } }
    render(<ServoNeutral parts={['home']} />)
    await inCard('Arm')
    fireEvent.click(btn('Arm', 'Set home'))
    fireEvent.click(within(screen.getByRole('dialog')).getByRole('button', { name: 'Set home' }))
    const shown = await screen.findByText(refusal)
    expect(shown.closest('[role="alert"]')?.className).toContain('MuiAlert-colorError')
  })

  it('markerState reads a missing marker as nothing stale', () => {
    expect(markerState(group('arm'))).toEqual({ joint_offsets: false, home: false })
    expect(markerState(group('arm', { calibration_stale: ['joint_offsets'] }))).toEqual({ joint_offsets: true, home: false })
  })

  it('reports each action outcome to onResult', async () => {
    groups = [group('base', { state: state({ a: 0 }) })]
    mockFetch()
    const onResult = vi.fn()
    render(<ServoNeutral onResult={onResult} />)
    await inCard('Base')
    fireEvent.click(btn('Base', 'Re-enable torque'))
    await waitFor(() => expect(onResult).toHaveBeenCalledWith('base', 'enable', true))
  })

  it('uses the given servo state and does not poll itself', async () => {
    mockFetch()
    render(<ServoNeutral heading={false} servo={{ groups: [group('head')], notConfigured: false, error: null, loading: false }} />)
    await inCard('Head')
    expect(fetch).not.toHaveBeenCalled()
    expect(screen.queryByText('Servo torque and neutral')).toBeNull()
  })
})
