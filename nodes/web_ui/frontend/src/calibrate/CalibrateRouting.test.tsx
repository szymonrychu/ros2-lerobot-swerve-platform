// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react'
import { ThemeProvider } from '@mui/material/styles'
import { theme } from '../theme'
import type { AppConfig } from '../types'

let viewportWidth = 1400
let store: Map<string, string>

vi.mock('../hooks/useRosBridge', () => ({
  useRosBridge: () => ({ topicData: {}, connected: true, publish: vi.fn() }),
}))
vi.mock('../photos/usePhotoLayer', () => ({ usePhotoLayer: () => ({ photos: [] }) }))
vi.mock('../overlays/OverlayBar', () => ({ OverlayBar: () => null }))
for (const tab of ['ArmTab', 'CameraTab', 'SensorGraphTab', 'PoiTab', 'RgbdCameraTab', 'MapNavTab', 'AgentChatTab']) {
  vi.doMock(`../tabs/${tab}`, () => ({ default: () => null }))
}

const CONFIG: AppConfig = {
  http_port: 8080,
  ws_broadcast_hz: 10,
  overlays: [],
  tabs: [
    { id: 'map', label: 'Map', type: 'map_nav' },
    { id: 'pois', label: 'Places', type: 'poi' },
  ] as AppConfig['tabs'],
}

function installMatchMedia(): void {
  window.matchMedia = ((query: string) => {
    const min = /min-width:\s*([\d.]+)px/.exec(query)
    const max = /max-width:\s*([\d.]+)px/.exec(query)
    const matches = !(min && viewportWidth < Number(min[1])) && !(max && viewportWidth > Number(max[1]))
    return { matches, media: query, addEventListener: vi.fn(), removeEventListener: vi.fn(), addListener: vi.fn(), removeListener: vi.fn(), dispatchEvent: vi.fn(), onchange: null }
  }) as typeof window.matchMedia
}

async function renderApp() {
  const { default: App } = await import('../App')
  render(
    <ThemeProvider theme={theme}>
      <App />
    </ThemeProvider>,
  )
  await screen.findByRole('tablist')
}

beforeEach(() => {
  viewportWidth = 1400
  window.location.hash = ''
  installMatchMedia()
  store = new Map<string, string>()
  vi.stubGlobal('localStorage', { getItem: (k: string) => store.get(k) ?? null, setItem: (k: string, v: string) => void store.set(k, v) })
  vi.stubGlobal(
    'fetch',
    vi.fn(async (url: string) =>
      url === '/api/calibration/config'
        ? new Response(JSON.stringify({ imu: true }), { status: 200 })
        : { json: async () => CONFIG },
    ),
  )
})
afterEach(() => {
  cleanup()
  vi.unstubAllGlobals()
  window.location.hash = ''
})

describe('Calibrate routing', () => {
  it.each([400, 800, 1400])('shows the menu button at width %i', async (w) => {
    viewportWidth = w
    await renderApp()
    expect(screen.getByLabelText('Open menu')).toBeTruthy()
  })

  it('desktop drawer lists only Calibrate and Calibrate is not a tab', async () => {
    await renderApp()
    expect(screen.getAllByRole('tab').map((t) => t.textContent)).toEqual(['Map', 'Places'])
    fireEvent.click(screen.getByLabelText('Open menu'))
    await screen.findByText('Calibrate')
    expect(screen.queryAllByRole('button').filter((b) => b.textContent === 'Places' && !b.getAttribute('role'))).toHaveLength(0)
  })

  it('phone drawer lists the tabs then Calibrate', async () => {
    viewportWidth = 400
    await renderApp()
    fireEvent.click(screen.getByLabelText('Open menu'))
    const labels = await waitFor(() => {
      const found = screen.getAllByRole('button').filter((b) => ['Map', 'Places', 'Calibrate'].includes(b.textContent ?? ''))
      expect(found.length).toBe(3)
      return found.map((b) => b.textContent)
    })
    expect(labels).toEqual(['Map', 'Places', 'Calibrate'])
  })

  it('selecting Calibrate sets the hash, shows the page, stores no tab', async () => {
    await renderApp()
    fireEvent.click(screen.getByLabelText('Open menu'))
    fireEvent.click(await screen.findByText('Calibrate'))
    expect(window.location.hash).toBe('#/calibrate')
    await screen.findByText('IMU calibration')
    expect(store.size).toBe(0)
  })

  it('loading with #/calibrate opens the page directly', async () => {
    window.location.hash = '#/calibrate'
    await renderApp()
    await screen.findByText('Arm calibration - deferred')
  })

  it('back clears the hash and a tab selection leaves the page', async () => {
    window.location.hash = '#/calibrate'
    await renderApp()
    await screen.findByText('Arm calibration - deferred')
    fireEvent.click(screen.getByRole('button', { name: 'Back' }))
    await waitFor(() => expect(screen.queryByText('Arm calibration - deferred')).toBeNull())
    expect(window.location.hash).toBe('')
    window.location.hash = '#/calibrate'
    await screen.findByText('Arm calibration - deferred')
    fireEvent.click(screen.getByRole('tab', { name: 'Places' }))
    await waitFor(() => expect(screen.queryByText('Arm calibration - deferred')).toBeNull())
    expect(window.location.hash).toBe('')
  })
})
