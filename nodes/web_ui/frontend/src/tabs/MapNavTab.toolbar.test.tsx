// @vitest-environment jsdom
import { cleanup, fireEvent, render, screen, waitFor, within } from '@testing-library/react'
import { useEffect } from 'react'
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { ThemeProvider } from '@mui/material/styles'
import type { MapSceneProps, SceneController } from '../map3d/MapScene'
import type { PhotoLayerData } from '../photos/usePhotoLayer'
import { theme } from '../theme'
import type { TabConfig } from '../types'
import { designViolations, SOURCES } from '../layout/designRules'

let viewportWidth = 1000

/** Minimal matchMedia: evaluates min-width / max-width px conditions against viewportWidth; fine pointer. */
function matchMediaStub(query: string): MediaQueryList {
  const min = /min-width:\s*([\d.]+)px/.exec(query)
  const max = /max-width:\s*([\d.]+)px/.exec(query)
  let matches = true
  if (min && viewportWidth < Number(min[1])) matches = false
  if (max && viewportWidth > Number(max[1])) matches = false
  if (/pointer:\s*coarse/.test(query)) matches = false
  return {
    matches, media: query, onchange: null,
    addEventListener: () => undefined, removeEventListener: () => undefined,
    addListener: () => undefined, removeListener: () => undefined, dispatchEvent: () => false,
  } as MediaQueryList
}

vi.mock('../map3d/MapScene', () => {
  function FakeScene(props: MapSceneProps) {
    useEffect(() => {
      const controller: SceneController = { pick: () => null, centerOn: () => undefined, fit: () => undefined }
      props.controllerRef.current = controller
      return () => {
        props.controllerRef.current = null
      }
    }, [props.controllerRef])
    return <div data-testid="scene" />
  }
  return { default: FakeScene, MapScene: FakeScene }
})

const TAB = {
  id: 'map', type: 'map_nav', label: 'Map', goal_topic: '/goal', arm_urdf: '/arm.urdf',
  poi_list_topic: '/poi/list', poi_command_topic: '/poi/command', grasp_command_topic: '/grasp/command',
} as TabConfig
const PHOTOS: PhotoLayerData = { photos: [], total: 0, error: null }
const RUNNING = { '/web_ui/grasp_result': { request_id: 'r1', state: 'running', action: 'execute' } }

const fetchMock = vi.fn()
let mapsStatus: unknown = null
const MAPS_STATUS = {
  maps: [{ name: 'lab', saved_at: 1700000000, note: '', poi_count: 3, last_pose: { x: 1, y: 2, yaw: 0 }, valid: true, error: '' }],
  active_map: 'lab',
  load_seq: 1,
  localisation: { state: 'localised', ratio: 0.9, pose: null, message: '', updated_at: 1 },
}

beforeEach(() => {
  viewportWidth = 1000
  mapsStatus = null
  vi.stubGlobal('matchMedia', matchMediaStub)
  window.matchMedia = matchMediaStub
  const store = new Map<string, string>()
  vi.stubGlobal('localStorage', {
    getItem: (k: string) => store.get(k) ?? null,
    setItem: (k: string, v: string) => void store.set(k, v),
    removeItem: (k: string) => void store.delete(k),
    clear: () => store.clear(),
    key: () => null,
    length: 0,
  })
  fetchMock.mockReset()
  fetchMock.mockResolvedValue({ status: 200, json: async () => ({ ok: true }) })
  fetchMock.mockImplementation(async (url: string, init?: { method?: string }) =>
    init?.method !== 'POST' && url.startsWith('/api/maps?') && mapsStatus
      ? { status: 200, json: async () => mapsStatus }
      : { status: 200, json: async () => ({ ok: true }) },
  )
  vi.stubGlobal('fetch', fetchMock)
})
afterEach(() => {
  cleanup()
  vi.unstubAllGlobals()
})

async function renderTab(topicData: Record<string, unknown> = {}) {
  const { default: MapNavTab } = await import('./MapNavTab')
  render(
    <ThemeProvider theme={theme}>
      <MapNavTab tab={TAB} topicData={topicData} publish={vi.fn()} photos={PHOTOS} />
    </ThemeProvider>,
  )
  await screen.findByTestId('scene')
}

const posted = (path: string) => fetchMock.mock.calls.some(([url, init]) => url === path && init?.method === 'POST')

describe('map toolbar stops', () => {
  it('STOP is a contained error text button, weight 500, no letter spacing, and posts the same stop', async () => {
    await renderTab()
    const stop = screen.getByRole('button', { name: 'STOP' })
    expect(stop.className).toContain('MuiButton-containedError')
    const style = getComputedStyle(stop)
    expect(style.fontWeight).toBe('500')
    expect(['', 'normal', '0', '0px']).toContain(style.letterSpacing)
    expect(stop.hasAttribute('title')).toBe(false)
    fireEvent.click(stop)
    await waitFor(() => expect(posted('/api/nav/stop?tab=map')).toBe(true))
  })

  it('STOP GRASP shows only while a grasp runs, looks the same, and posts the same grasp stop', async () => {
    await renderTab()
    expect(screen.queryByRole('button', { name: 'STOP GRASP' })).toBeNull()
    cleanup()
    await renderTab(RUNNING)
    const stop = screen.getByRole('button', { name: 'STOP GRASP' })
    expect(stop.className).toContain('MuiButton-containedError')
    expect(getComputedStyle(stop).fontWeight).toBe('500')
    fireEvent.click(stop)
    await waitFor(() => expect(posted('/api/grasp/stop?tab=map')).toBe(true))
  })

  it('on a phone both stops stay in the row, not in the overflow menu', async () => {
    viewportWidth = 400
    await renderTab(RUNNING)
    expect(screen.getByRole('button', { name: 'STOP' })).toBeTruthy()
    expect(screen.getByRole('button', { name: 'STOP GRASP' })).toBeTruthy()
    fireEvent.click(screen.getByRole('button', { name: 'More map controls' }))
    const menu = screen.getByRole('menu')
    expect(within(menu).queryByText(/STOP/)).toBeNull()
  })

  it('the stops are the only filled-colour controls', async () => {
    await renderTab(RUNNING)
    const filled = screen.getAllByRole('button').filter((b) => /MuiButton-contained/.test(b.className))
    expect(filled.map((b) => b.textContent).sort()).toEqual(['STOP', 'STOP GRASP'])
  })
})

describe('map toolbar controls', () => {
  it('has no title attributes and no morphing labels', async () => {
    await renderTab()
    expect(document.querySelectorAll('[title]')).toHaveLength(0)
    expect(screen.queryByText(/Tap map|Click map|Area:/)).toBeNull()
  })

  it('goal mode is an accent toggle with a fixed label and puts its hint in the status line', async () => {
    await renderTab()
    fireEvent.click(screen.getByRole('button', { name: 'Top view' }))
    const goal = screen.getByRole('button', { name: 'Set goal' })
    expect(goal.getAttribute('aria-pressed')).toBe('false')
    fireEvent.click(goal)
    expect(screen.getByRole('button', { name: 'Set goal' }).getAttribute('aria-pressed')).toBe('true')
    expect(screen.getByRole('button', { name: 'Set goal' }).className).not.toContain('Secondary')
    expect(screen.getByTestId('map-status-line').textContent).toContain('Press on the map to set the goal')
  })

  it('POI placement shows the vertex count in the status line, not in a label', async () => {
    await renderTab()
    fireEvent.click(screen.getByRole('button', { name: 'Top view' }))
    fireEvent.click(screen.getByRole('button', { name: 'Add area' }))
    expect(screen.getByRole('button', { name: 'Add area' }).getAttribute('aria-pressed')).toBe('true')
    expect(screen.getByTestId('map-status-line').textContent).toContain('0 vertices')
  })

  it('grasp pick hint is in the status line and the floating alert is gone', async () => {
    await renderTab()
    fireEvent.click(screen.getByRole('button', { name: 'Grasp' }))
    fireEvent.click(screen.getAllByRole('menuitem')[0])
    expect(screen.getByTestId('map-status-line').textContent).toContain('Click the object on the map')
    expect(within(screen.getByTestId('map-container')).queryByRole('alert')).toBeNull()
    fireEvent.click(screen.getByRole('button', { name: 'Cancel grasp pick' }))
    expect(screen.getByTestId('map-status-line').textContent).not.toContain('Click the object on the map')
    expect(screen.getByTestId('map-status-line').textContent).toContain('Robot')
    expect(screen.getByTestId('map-status-line').textContent).toContain('Choose a strategy in the Grasp menu')
  })

  it('map reset asks in a ConfirmDialog and posts only after confirming', async () => {
    await renderTab()
    fireEvent.click(screen.getByRole('button', { name: 'Reset map' }))
    expect(posted('/api/map/reset?tab=map')).toBe(false)
    const dialog = screen.getByRole('dialog')
    expect(dialog.textContent).toContain('deletes the live POIs and photos. Saved maps are kept.')
    fireEvent.click(within(dialog).getByRole('button', { name: 'Cancel' }))
    await waitFor(() => expect(screen.queryByRole('dialog')).toBeNull())
    expect(posted('/api/map/reset?tab=map')).toBe(false)
    fireEvent.click(screen.getByRole('button', { name: 'Reset map' }))
    fireEvent.click(within(screen.getByRole('dialog')).getByRole('button', { name: 'Reset map' }))
    await waitFor(() => expect(posted('/api/map/reset?tab=map')).toBe(true))
  })

  it('set home keeps the two-click arm', async () => {
    await renderTab()
    fireEvent.click(screen.getByRole('button', { name: 'Set home' }))
    expect(posted('/api/arm/set_home?tab=map')).toBe(false)
    expect(screen.queryByRole('dialog')).toBeNull()
    fireEvent.click(screen.getByRole('button', { name: 'Confirm set home' }))
    await waitFor(() => expect(posted('/api/arm/set_home?tab=map')).toBe(true))
  })

  it('the phone overflow menu is kept', async () => {
    viewportWidth = 400
    await renderTab()
    fireEvent.click(screen.getByRole('button', { name: 'More map controls' }))
    expect(within(screen.getByRole('menu')).getByText('Save map')).toBeTruthy()
  })
})

describe('named maps toolbar', () => {
  it('shows no map status before the first status arrives', async () => {
    await renderTab()
    expect(screen.getByTestId('map-status-line').textContent).not.toMatch(/Localised|localised/)
  })

  it('shows the localisation state in the status line once polled', async () => {
    mapsStatus = MAPS_STATUS
    await renderTab()
    await waitFor(() => expect(screen.getByTestId('map-status-line').textContent).toContain('Localised on lab'))
  })

  it('Save map as asks for a name, then posts the save with the tab id', async () => {
    await renderTab()
    fireEvent.click(screen.getByRole('button', { name: 'Save map as' }))
    fireEvent.change(screen.getByLabelText('Map name'), { target: { value: 'garage' } })
    fireEvent.click(within(screen.getByRole('dialog')).getByRole('button', { name: 'Save' }))
    await waitFor(() => expect(posted('/api/maps/save?tab=map')).toBe(true))
  })

  it('Maps opens the dialog listing the saved maps', async () => {
    mapsStatus = MAPS_STATUS
    await renderTab()
    await waitFor(() => expect(screen.getByTestId('map-status-line').textContent).toContain('lab'))
    fireEvent.click(screen.getByRole('button', { name: 'Maps' }))
    expect(await screen.findByText('3 POIs', { exact: false })).toBeTruthy()
  })

  it('Save map keeps posting the old save', async () => {
    await renderTab()
    fireEvent.click(screen.getByRole('button', { name: 'Save map' }))
    await waitFor(() => expect(posted('/api/map/save?tab=map')).toBe(true))
  })

  it('the phone overflow menu holds Save map as and Maps', async () => {
    viewportWidth = 400
    await renderTab()
    fireEvent.click(screen.getByRole('button', { name: 'More map controls' }))
    const menu = within(screen.getByRole('menu'))
    expect(menu.getByText('Save map as')).toBeTruthy()
    expect(menu.getByText('Maps')).toBeTruthy()
  })
})

describe('design rules', () => {
  it('MapNavTab.tsx has no title= on buttons', () => {
    expect(designViolations('tabs/MapNavTab.tsx', SOURCES['tabs/MapNavTab.tsx']).filter((v) => v.startsWith('title'))).toEqual([])
  })
})
