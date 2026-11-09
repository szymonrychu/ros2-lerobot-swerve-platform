// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { act, cleanup, fireEvent, render, screen, waitFor, within } from '@testing-library/react'
import { ImuFlow, imuGate } from './ImuFlow'

const B = '/api/calibration/imu'
type Handler = () => { status: number; body: unknown }
let routes: Record<string, Handler>
let calls: string[]

function install() {
  vi.stubGlobal('fetch', vi.fn(async (url: string, init?: RequestInit) => {
    const key = `${init?.method ?? 'GET'} ${url}`
    calls.push(key)
    const h = routes[key]
    if (!h) return new Response('{}', { status: 404 })
    const r = h()
    return new Response(JSON.stringify(r.body), { status: r.status })
  }))
}
const status = (over: Record<string, unknown> = {}) => ({ sys: 3, gyro: 3, accel: 2, mag: 1, restored: true, age_s: 0.3, ...over })

beforeEach(() => {
  calls = []
  routes = { [`GET ${B}`]: () => ({ status: 200, body: status() }) }
})
afterEach(() => {
  cleanup()
  vi.useRealTimers()
  vi.unstubAllGlobals()
})

async function start() {
  install()
  render(<ImuFlow />)
  expect(calls).toEqual([])
  fireEvent.click(screen.getByRole('button', { name: 'Calibrate IMU' }))
}

describe('ImuFlow', () => {
  it('does not poll until Calibrate IMU is pressed, then shows levels, restored and guidance', async () => {
    await start()
    await screen.findByText('mag 1 / 3')
    expect(screen.getByText('sys 3 / 3')).toBeTruthy()
    expect(screen.getByText('gyro 3 / 3')).toBeTruthy()
    expect(screen.getByText('accel 2 / 3')).toBeTruthy()
    expect(screen.getByText('Restored from saved offsets: yes')).toBeTruthy()
    expect(screen.getByText(/gyro: keep the robot still/i)).toBeTruthy()
    expect(screen.getByText(/accel: .*static poses.*lifting/i)).toBeTruthy()
    expect(screen.getByText(/mag: rotate .*figure-eights.*away from motors/i)).toBeTruthy()
  })

  it('polls at 1 Hz', async () => {
    vi.useFakeTimers({ shouldAdvanceTime: true })
    await start()
    await screen.findByText('mag 1 / 3')
    const n = calls.length
    await act(async () => { await vi.advanceTimersByTimeAsync(2100) })
    expect(calls.length).toBeGreaterThanOrEqual(n + 2)
  })

  it('503 shows the server message and no levels', async () => {
    routes[`GET ${B}`] = () => ({ status: 503, body: { ok: false, message: 'no IMU calibration sample yet' } })
    await start()
    await screen.findByText('no IMU calibration sample yet')
    expect(screen.queryByText(/\/ 3/)).toBeNull()
  })

  it('the gate refuses until mag is 3; Accept confirms, posts save and shows the message', async () => {
    routes[`POST ${B}/save`] = () => ({ status: 200, body: { ok: true, message: 'offsets saved' } })
    await start()
    await screen.findByText('mag 1 / 3')
    expect(screen.getByText('Finish step 2 first.')).toBeTruthy()
    routes[`GET ${B}`] = () => ({ status: 200, body: status({ mag: 3 }) })
    await screen.findByText('mag 3 / 3', {}, { timeout: 3000 })
    fireEvent.click(screen.getByRole('button', { name: 'Accept' }))
    expect(calls).not.toContain(`POST ${B}/save`)
    fireEvent.click(within(screen.getByRole('dialog')).getByRole('button', { name: 'Accept' }))
    await screen.findByText('offsets saved')
    expect(calls).toContain(`POST ${B}/save`)
    expect(screen.getByText('Accepted')).toBeTruthy()
  })

  it('a 409 refusal of the save shows the node reason and leaves the gate open', async () => {
    routes[`GET ${B}`] = () => ({ status: 200, body: status({ mag: 3 }) })
    routes[`POST ${B}/save`] = () => ({ status: 409, body: { ok: false, message: 'robot is moving' } })
    await start()
    await screen.findByText('mag 3 / 3')
    fireEvent.click(screen.getByRole('button', { name: 'Accept' }))
    fireEvent.click(within(screen.getByRole('dialog')).getByRole('button', { name: 'Accept' }))
    await screen.findByText('robot is moving')
    await waitFor(() => expect(screen.queryByRole('dialog')).toBeNull())
    expect((screen.getByRole('button', { name: 'Decline' }) as HTMLButtonElement).disabled).toBe(false)
  })

  it('Decline saves nothing', async () => {
    routes[`GET ${B}`] = () => ({ status: 200, body: status({ mag: 3 }) })
    await start()
    await screen.findByText('mag 3 / 3')
    fireEvent.click(screen.getByRole('button', { name: 'Decline' }))
    await screen.findByText('Declined')
    expect(calls).not.toContain(`POST ${B}/save`)
  })

  it('reset: cancel sends nothing, confirm posts reset', async () => {
    routes[`POST ${B}/reset`] = () => ({ status: 200, body: { ok: true, message: 'IMU reset' } })
    await start()
    await screen.findByText('mag 1 / 3')
    fireEvent.click(screen.getByRole('button', { name: 'Reset IMU calibration' }))
    await screen.findByText('The IMU forgets its calibration and starts from zero.')
    fireEvent.click(screen.getByRole('button', { name: 'Cancel' }))
    await waitFor(() => expect(screen.queryByRole('dialog')).toBeNull())
    expect(calls).not.toContain(`POST ${B}/reset`)
    fireEvent.click(screen.getByRole('button', { name: 'Reset IMU calibration' }))
    fireEvent.click(await screen.findByRole('button', { name: 'Reset' }))
    await screen.findByText('IMU reset')
    expect(calls).toContain(`POST ${B}/reset`)
  })
})

describe('imuGate', () => {
  it('mag 3 is plausible; lower levels warn', () => {
    const gate = imuGate(status({ mag: 3 }) as never)
    expect(gate.plausible).toBe(true)
    expect(gate.warnings).toEqual(['accel 2 / 3: hold a few more static poses'])
    expect(gate.score).toEqual([
      { label: 'sys', value: '3 / 3' },
      { label: 'gyro', value: '3 / 3' },
      { label: 'accel', value: '2 / 3' },
      { label: 'mag', value: '3 / 3' },
    ])
  })
  it('mag below 3 is refused', () => {
    expect(imuGate(status() as never).refusals).toEqual(['mag 1 / 3: the offsets save only with mag at 3'])
  })
  it('no status is refused', () => {
    expect(imuGate(null).refusals).toEqual(['No IMU calibration status yet'])
  })
})
