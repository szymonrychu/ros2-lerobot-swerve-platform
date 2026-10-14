// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react'
import { ThemeProvider } from '@mui/material/styles'
import { StateBanner } from './StateBanner'
import { theme } from '../theme'
import { designViolations, SOURCES } from '../layout/designRules'
import { TILT_GUARD_STATE_KEY } from '../tilt/tiltStatus'
import type { AppConfig } from '../types'

let topicData: Record<string, unknown> = {}
let viewportWidth = 1400

vi.mock('../hooks/useRosBridge', () => ({
  useRosBridge: () => ({ topicData, connected: true, publish: vi.fn() }),
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
  topicData = {}
  viewportWidth = 1400
  installMatchMedia()
  const store = new Map<string, string>()
  vi.stubGlobal('localStorage', { getItem: (k: string) => store.get(k) ?? null, setItem: (k: string, v: string) => void store.set(k, v) })
  vi.stubGlobal('fetch', vi.fn(async () => ({ json: async () => CONFIG })))
})
afterEach(() => {
  cleanup()
  vi.unstubAllGlobals()
})

describe('StateBanner', () => {
  it('renders a standard outlined alert with the state severity and the text', () => {
    render(<StateBanner state="warn">Tilt warning</StateBanner>)
    const alert = screen.getByRole('alert')
    expect(alert.textContent).toContain('Tilt warning')
    expect(alert.className).toContain('MuiAlert-outlinedWarning')
    expect(alert.className).not.toContain('MuiAlert-filled')
  })

  it('maps the error state to the error severity', () => {
    render(<StateBanner state="error">Base drive disabled</StateBanner>)
    expect(screen.getByRole('alert').className).toContain('MuiAlert-outlinedError')
  })

  it('draws a 3 px left border in the state colour', () => {
    render(
      <ThemeProvider theme={theme}>
        <StateBanner state="error">x</StateBanner>
      </ThemeProvider>,
    )
    const css = Array.from(document.querySelectorAll('style')).map((s) => s.textContent).join('')
    expect(css).toMatch(/border-left:\s*3px solid/)
  })

  it('passes the design rules guard for App.tsx and StateBanner.tsx', () => {
    expect(designViolations('App.tsx', SOURCES['App.tsx'])).toEqual([])
    expect(designViolations('components/StateBanner.tsx', SOURCES['components/StateBanner.tsx'])).toEqual([])
  })
})

describe('App chrome', () => {
  it('shows the tilt warning once, as a banner, with no entry toast', async () => {
    topicData = { [TILT_GUARD_STATE_KEY]: { level: 'warning', roll_deg: 12.5, pitch_deg: -3 } }
    await renderApp()
    const alerts = screen.getAllByRole('alert').filter((a) => a.textContent?.includes('Tilt warning'))
    expect(alerts).toHaveLength(1)
    expect(alerts[0].className).toContain('MuiAlert-outlinedWarning')
  })

  it('shows no banner at tilt ok', async () => {
    await renderApp()
    expect(screen.queryByRole('alert')).toBeNull()
  })

  it('shows the tab bar as text labels without icons', async () => {
    await renderApp()
    const tabs = screen.getAllByRole('tab')
    expect(tabs.map((t) => t.textContent)).toEqual(['Map', 'Places'])
    for (const t of tabs) expect(t.querySelector('svg')).toBeNull()
  })

  it('keeps icons in the drawer', async () => {
    viewportWidth = 400
    await renderApp()
    fireEvent.click(screen.getByLabelText('Open menu'))
    const items = await waitFor(() => {
      const found = screen.getAllByRole('button').filter((b) => b.textContent === 'Map' || b.textContent === 'Places')
      expect(found.length).toBe(2)
      return found
    })
    for (const item of items) expect(item.querySelector('svg')).not.toBeNull()
  })
})
