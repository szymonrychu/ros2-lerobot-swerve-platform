// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen, waitFor, within } from '@testing-library/react'
import { ServoNeutral } from './ServoNeutral'
import type { ServoGroup, TorqueState } from './servoApi'

function state(joints: Record<string, number>, capture: TorqueState['neutral_capture'] = null): TorqueState {
  const released = Object.values(joints).some((v) => v === 0)
  return { namespace: 'x', released, joints, stamp_ns: 1, neutral_enabled: true, neutral_capture: capture, last_save: null }
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
        state: state({ pan: 0, tilt: 0 }, { pan: { steps: 2048 + 100, delta_steps: 100 }, tilt: { steps: 2048 - 51, delta_steps: -51 } }),
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
    groups = [group('head', { state: state({ pan: 0 }, { pan: { steps: 2050, delta_steps: 2 } }) })]
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

  it('arm save confirm carries the stale and leader warnings', async () => {
    groups = [group('arm', { state: state({ a: 0 }, { a: { steps: 2050, delta_steps: 2 } }) })]
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
})
