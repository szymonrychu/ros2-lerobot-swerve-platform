// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react'
import { ThemeProvider } from '@mui/material/styles'
import { CONTROL_PX, theme } from '../theme'
import { TabConfig } from '../types'
import { postArm } from './armApi'
import ArmJogPanel from './ArmJogPanel'

const SOURCE_TOPIC = '/filter/active_source'
const TAB = { id: 'arm', type: 'arm', label: 'Arm', active_source_topic: SOURCE_TOPIC } as TabConfig

function setViewport(width: number): void {
  window.matchMedia = ((query: string) => {
    const min = /min-width:\s*([\d.]+)px/.exec(query)
    const max = /max-width:\s*([\d.]+)px/.exec(query)
    const matches = !((min && width < Number(min[1])) || (max && width > Number(max[1])))
    return { matches, media: query, addEventListener: vi.fn(), removeEventListener: vi.fn(), addListener: vi.fn(), removeListener: vi.fn(), dispatchEvent: vi.fn(), onchange: null }
  }) as typeof window.matchMedia
}

function renderPanel(source: string | undefined) {
  const topicData = source === undefined ? {} : { [SOURCE_TOPIC]: { data: source } }
  return render(
    <ThemeProvider theme={theme}>
      <ArmJogPanel tab={TAB} topicData={topicData} />
    </ThemeProvider>,
  )
}

const fetchMock = vi.fn()

function reply(ok: boolean, message: string, status = 200): void {
  fetchMock.mockResolvedValue({ status, json: async () => ({ ok, message }) })
}

beforeEach(() => {
  setViewport(1400)
  fetchMock.mockReset()
  reply(true, 'done')
  vi.stubGlobal('fetch', fetchMock)
})
afterEach(() => {
  cleanup()
  vi.unstubAllGlobals()
})

const JOGS: [string, string, string][] = [
  ['Up', 'cartesian', 'up'],
  ['Down', 'cartesian', 'down'],
  ['Forward', 'cartesian', 'forward'],
  ['Back', 'cartesian', 'back'],
  ['Left', 'cartesian', 'left'],
  ['Right', 'cartesian', 'right'],
  ['Pan left', 'shoulder_pan', 'left'],
  ['Pan right', 'shoulder_pan', 'right'],
  ['Wrist up', 'wrist_pitch', 'up'],
  ['Wrist down', 'wrist_pitch', 'down'],
  ['Roll left', 'wrist_roll', 'left'],
  ['Roll right', 'wrist_roll', 'right'],
  ['Gripper open', 'gripper', 'open'],
  ['Gripper close', 'gripper', 'close'],
]

describe('postArm', () => {
  it('posts jog with the spec as body and the tab in the query', async () => {
    const f = vi.fn().mockResolvedValue({ status: 200, json: async () => ({ ok: true, message: 'jogged' }) })
    const res = await postArm('jog', 'arm', { kind: 'gripper', direction: 'open' }, f as unknown as typeof fetch)
    expect(res).toEqual({ ok: true, message: 'jogged' })
    expect(f).toHaveBeenCalledWith('/api/arm/jog?tab=arm', expect.objectContaining({ method: 'POST', body: JSON.stringify({ kind: 'gripper', direction: 'open' }) }))
  })

  it.each(['take_control', 'release', 'home'] as const)('posts %s', async (action) => {
    const f = vi.fn().mockResolvedValue({ status: 200, json: async () => ({ ok: true, message: 'x' }) })
    await postArm(action, 'a b', undefined, f as unknown as typeof fetch)
    expect(f.mock.calls[0][0]).toBe(`/api/arm/${action}?tab=a%20b`)
  })

  it('never throws: network failure and non-JSON answers become ok:false', async () => {
    const down = vi.fn().mockRejectedValue(new Error('boom'))
    expect((await postArm('home', 'arm', undefined, down as unknown as typeof fetch)).ok).toBe(false)
    const html = vi.fn().mockResolvedValue({ status: 502, json: async () => { throw new Error('not json') } })
    const res = await postArm('home', 'arm', undefined, html as unknown as typeof fetch)
    expect(res.ok).toBe(false)
    expect(res.message).toContain('502')
  })
})

describe('ArmJogPanel', () => {
  it('shows the active source', () => {
    renderPanel('leader')
    expect(screen.getByTestId('arm-active-source').textContent).toContain('leader')
  })

  it('shows an unknown source before the first message', () => {
    renderPanel(undefined)
    expect(screen.getByTestId('arm-active-source').textContent).toContain('unknown')
  })

  it.each(JOGS)('%s posts a %s %s jog when control is held', async (label, kind, direction) => {
    renderPanel('autonomy')
    fireEvent.click(screen.getByRole('button', { name: label }))
    await waitFor(() => expect(fetchMock).toHaveBeenCalled())
    const [url, init] = fetchMock.mock.calls[0]
    expect(url).toBe('/api/arm/jog?tab=arm')
    expect(JSON.parse(init.body)).toEqual({ kind, direction })
  })

  it.each(['leader', 'web_ui', 'none', undefined])('jog buttons are disabled and Release too when the source is %s', (source) => {
    renderPanel(source)
    for (const [label] of JOGS) expect((screen.getByRole('button', { name: label }) as HTMLButtonElement).disabled).toBe(true)
    expect((screen.getByRole('button', { name: 'Release' }) as HTMLButtonElement).disabled).toBe(true)
    expect((screen.getByRole('button', { name: 'Take control' }) as HTMLButtonElement).disabled).toBe(false)
  })

  it('enables jog buttons and Release when the web UI holds control', () => {
    renderPanel('autonomy')
    for (const [label] of JOGS) expect((screen.getByRole('button', { name: label }) as HTMLButtonElement).disabled).toBe(false)
    expect((screen.getByRole('button', { name: 'Release' }) as HTMLButtonElement).disabled).toBe(false)
  })

  it('Take control and Release post to their routes', async () => {
    renderPanel('autonomy')
    fireEvent.click(screen.getByRole('button', { name: 'Take control' }))
    await waitFor(() => expect(fetchMock).toHaveBeenCalledTimes(1))
    fireEvent.click(screen.getByRole('button', { name: 'Release' }))
    await waitFor(() => expect(fetchMock).toHaveBeenCalledTimes(2))
    expect(fetchMock.mock.calls.map((c) => c[0])).toEqual(['/api/arm/take_control?tab=arm', '/api/arm/release?tab=arm'])
  })

  it('Home needs two clicks', async () => {
    renderPanel('leader')
    fireEvent.click(screen.getByRole('button', { name: 'Home' }))
    expect(fetchMock).not.toHaveBeenCalled()
    fireEvent.click(screen.getByRole('button', { name: 'Confirm home' }))
    await waitFor(() => expect(fetchMock).toHaveBeenCalledTimes(1))
    expect(fetchMock.mock.calls[0][0]).toBe('/api/arm/home?tab=arm')
  })

  it('shows a refusal from the backend', async () => {
    reply(false, 'jog refused: self-collision', 400)
    renderPanel('autonomy')
    fireEvent.click(screen.getByRole('button', { name: 'Down' }))
    expect((await screen.findByRole('alert')).textContent).toContain('jog refused: self-collision')
  })

  it('shows a network error', async () => {
    fetchMock.mockRejectedValue(new Error('offline'))
    renderPanel('leader')
    fireEvent.click(screen.getByRole('button', { name: 'Take control' }))
    expect((await screen.findByRole('alert')).textContent).toContain('offline')
  })

  it('shows success messages as status, not alert', async () => {
    reply(true, 'arm control taken')
    renderPanel('leader')
    fireEvent.click(screen.getByRole('button', { name: 'Take control' }))
    expect((await screen.findByRole('status')).textContent).toContain('arm control taken')
    expect(screen.queryByRole('alert')).toBeNull()
  })

  it('every button is at least CONTROL_PX high and wide (the coarse-pointer floor is covered in theme.test)', () => {
    renderPanel('autonomy')
    for (const b of screen.getAllByRole('button')) {
      const s = getComputedStyle(b)
      expect(parseFloat(s.minHeight)).toBeGreaterThanOrEqual(CONTROL_PX)
      expect(parseFloat(s.minWidth)).toBeGreaterThanOrEqual(CONTROL_PX)
    }
  })

  it('phone: renders in an OverlayPanel bottom sheet', () => {
    setViewport(400)
    renderPanel('autonomy')
    expect(screen.getByTestId('overlay-panel').getAttribute('data-layout')).toBe('phone')
  })
})
