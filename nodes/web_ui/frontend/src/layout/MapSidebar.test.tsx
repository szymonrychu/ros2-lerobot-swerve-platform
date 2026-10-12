// @vitest-environment jsdom
import { act, cleanup, fireEvent, render, screen, within } from '@testing-library/react'
import { useEffect } from 'react'
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { ThemeProvider } from '@mui/material/styles'
import type { MapSceneProps, SceneController } from '../map3d/MapScene'
import type { Photo } from '../photos/api'
import type { PhotoLayerData } from '../photos/usePhotoLayer'
import type { Poi } from '../poi/types'
import { PANEL_BG, theme, TOKENS } from '../theme'
import type { TabConfig } from '../types'
import { designViolations, SOURCES } from './designRules'
import { MapSidebar } from './MapSidebar'
import { OverlayPanel } from './OverlayPanel'

let viewportWidth = 800

/** Minimal matchMedia: evaluates min-width / max-width px conditions against viewportWidth; fine pointer. */
function matchMediaStub(query: string): MediaQueryList {
  const min = /min-width:\s*([\d.]+)px/.exec(query)
  const max = /max-width:\s*([\d.]+)px/.exec(query)
  let matches = true
  if (min && viewportWidth < Number(min[1])) matches = false
  if (max && viewportWidth > Number(max[1])) matches = false
  if (/pointer:\s*coarse/.test(query)) matches = false
  return {
    matches,
    media: query,
    onchange: null,
    addEventListener: () => undefined,
    removeEventListener: () => undefined,
    addListener: () => undefined,
    removeListener: () => undefined,
    dispatchEvent: () => false,
  } as MediaQueryList
}

// The real scene needs WebGL: this stand-in exposes a controller and a photo-marker click.
vi.mock('../map3d/MapScene', () => {
  function FakeScene(props: MapSceneProps) {
    useEffect(() => {
      const controller: SceneController = { pick: () => null, centerOn: () => undefined, fit: () => undefined }
      props.controllerRef.current = controller
      return () => {
        props.controllerRef.current = null
      }
    }, [props.controllerRef])
    return (
      <div data-testid="scene">
        <button type="button" onClick={() => props.onSelectPhoto?.('f1')}>
          Photo marker
        </button>
      </div>
    )
  }
  return { default: FakeScene, MapScene: FakeScene }
})

function renderWithTheme(ui: React.ReactElement) {
  return render(<ThemeProvider theme={theme}>{ui}</ThemeProvider>)
}

function poi(over: Partial<Poi>): Poi {
  return {
    id: 'p1', kind: 'point', frame: 'map', x: 1, y: 2, polygon: [], radius_m: 0.5, name: 'Door', note: '',
    status: 'open', created_by: 'user', created_at: 1, updated_at: 1, zone: 'none', ...over,
  }
}

function photo(over: Partial<Photo>): Photo {
  return {
    id: 'f1', ts: Date.now() / 1000, camera: 'head', tool: 'take_photo', name: 'Shot', note: '', poiIds: [], x: 3, y: 4,
    view: null, bytes: 1, width: 1, height: 1, ...over,
  }
}

beforeEach(() => {
  viewportWidth = 800
  vi.stubGlobal('matchMedia', matchMediaStub)
  window.matchMedia = matchMediaStub
})
afterEach(() => {
  cleanup()
  vi.unstubAllGlobals()
})

describe('MapSidebar', () => {
  it.each([
    [800, 'tablet'],
    [1400, 'desktop'],
  ])('%i px (%s): a docked right column, opaque, bordered left, scrolling internally', (width) => {
    viewportWidth = width
    renderWithTheme(
      <MapSidebar>
        <span data-testid="child">x</span>
      </MapSidebar>,
    )
    const bar = screen.getByTestId('map-sidebar')
    expect(bar.contains(screen.getByTestId('child'))).toBe(true)
    expect(bar.getAttribute('style')).toContain('clamp(280px, 30vw, 380px)')
    expect(bar.style.height).toBe('100%')
    expect(bar.style.overflowY).toBe('auto')
    expect(bar.style.position).not.toBe('absolute')
    expect(bar.style.background).toBe('rgb(22, 24, 29)') // PANEL_BG, normalised by jsdom
    expect(PANEL_BG).toBe(TOKENS.bgPanel)
    expect(bar.style.borderLeft).toBe(`${TOKENS.borderWidth}px solid rgb(42, 46, 54)`) // TOKENS.border
  })

  it('phone: renders its children unchanged, no column', () => {
    viewportWidth = 400
    const { container } = renderWithTheme(
      <MapSidebar>
        <span data-testid="child">x</span>
      </MapSidebar>,
    )
    expect(screen.queryByTestId('map-sidebar')).toBeNull()
    expect(container.firstElementChild).toBe(screen.getByTestId('child'))
  })

  it('a pointerdown inside the sidebar does not reach the map', () => {
    const map = vi.fn()
    renderWithTheme(
      <div onPointerDown={map}>
        <MapSidebar>
          <span data-testid="child">x</span>
        </MapSidebar>
      </div>,
    )
    fireEvent.pointerDown(screen.getByTestId('child'))
    expect(map).not.toHaveBeenCalled()
  })

  it.each(['left', 'right', 'bottom'] as const)('docks an OverlayPanel (anchor %s) in flow on tablet, with its header', (anchor) => {
    renderWithTheme(
      <MapSidebar>
        <OverlayPanel title="Layers" open anchor={anchor}>
          body
        </OverlayPanel>
      </MapSidebar>,
    )
    const panel = screen.getByTestId('overlay-panel')
    expect(screen.getByTestId('map-sidebar').contains(panel)).toBe(true)
    expect(panel.style.position).not.toBe('absolute')
    expect(panel.style.top).toBe('')
    expect(panel.style.bottom).toBe('')
    expect(panel.style.left).toBe('')
    expect(panel.style.right).toBe('')
    expect(panel.style.transform).toBe('')
    expect(panel.getAttribute('data-docked')).toBe('true')
    expect(within(screen.getByTestId('overlay-panel-header')).getByText('Layers')).toBeTruthy()
  })

  it('stacks several docked panels as sections in order', () => {
    viewportWidth = 1400
    renderWithTheme(
      <MapSidebar>
        <OverlayPanel title="A" open anchor="left">a</OverlayPanel>
        <OverlayPanel title="B" open anchor="right">b</OverlayPanel>
      </MapSidebar>,
    )
    const panels = screen.getAllByTestId('overlay-panel')
    expect(panels).toHaveLength(2)
    expect(panels[0].textContent).toContain('A')
    expect(panels[1].textContent).toContain('B')
    for (const p of panels) expect(p.style.position).not.toBe('absolute')
  })

  it('phone: an OverlayPanel inside the sidebar stays a bottom sheet', () => {
    viewportWidth = 400
    renderWithTheme(
      <MapSidebar>
        <OverlayPanel title="Layers" open anchor="right">
          body
        </OverlayPanel>
      </MapSidebar>,
    )
    const panel = screen.getByTestId('overlay-panel')
    expect(panel.style.position).toBe('absolute')
    expect(panel.style.bottom).toBe('0px')
    expect(panel.style.maxHeight).toBe('60dvh')
    expect(panel.getAttribute('data-docked')).toBe('false')
  })

  it('outside a sidebar a tablet OverlayPanel still floats', () => {
    renderWithTheme(
      <OverlayPanel title="T" open anchor="right">
        body
      </OverlayPanel>,
    )
    expect(screen.getByTestId('overlay-panel').style.position).toBe('absolute')
  })

  it('passes the design rules', () => {
    for (const file of ['layout/MapSidebar.tsx', 'map3d/useInvalidateOnResize.ts']) {
      expect(SOURCES[file]).toBeTruthy()
      expect(designViolations(file, SOURCES[file])).toEqual([])
    }
  })
})

describe('MapNavTab with the docked sidebar', () => {
  const TAB = {
    id: 'map', type: 'map_nav', label: 'Map', poi_list_topic: '/poi/list', poi_command_topic: '/poi/command',
    grasp_command_topic: '/grasp/command',
  } as TabConfig
  const TOPICS = { '/poi/list': { revision: 1, pois: [poi({ id: 'p1', name: 'Door' })] } }
  const PHOTOS: PhotoLayerData = { photos: [photo({ id: 'f1', name: 'Shot' })], total: 1, error: null }

  beforeEach(() => {
    const store = new Map<string, string>()
    vi.stubGlobal('localStorage', {
      getItem: (k: string) => store.get(k) ?? null,
      setItem: (k: string, v: string) => void store.set(k, v),
      removeItem: (k: string) => void store.delete(k),
      clear: () => store.clear(),
      key: () => null,
      length: 0,
    })
    vi.stubGlobal('fetch', vi.fn(() => new Promise(() => undefined)))
  })

  async function renderTab() {
    const { default: MapNavTab } = await import('../tabs/MapNavTab')
    renderWithTheme(<MapNavTab tab={TAB} topicData={TOPICS} publish={vi.fn()} photos={PHOTOS} />)
    await screen.findByTestId('scene')
  }

  /** The panel (an OverlayPanel) whose header title (not its actions) shows the given title. */
  function panelTitled(title: RegExp): HTMLElement {
    const header = screen.getAllByTestId('overlay-panel-header').find((h) => title.test(h.firstElementChild?.textContent ?? ''))
    expect(header, `panel ${title}`).toBeTruthy()
    return header!.closest('[data-testid="overlay-panel"]') as HTMLElement
  }

  function expectDocked(el: HTMLElement): void {
    const sidebar = screen.getByTestId('map-sidebar')
    const map = screen.getByTestId('map-container')
    expect(sidebar.contains(el)).toBe(true)
    expect(map.contains(el)).toBe(false)
    expect(el.style.position).not.toBe('absolute')
  }

  it('map container and sidebar sit side by side in one flex row', async () => {
    await renderTab()
    const sidebar = screen.getByTestId('map-sidebar')
    const map = screen.getByTestId('map-container')
    expect(sidebar.parentElement).toBe(map.parentElement)
    expect(map.contains(sidebar)).toBe(false)
    expect(getComputedStyle(map.parentElement!).display).toBe('flex')
    expect(getComputedStyle(map.parentElement!).flexDirection).toBe('row')
  })

  it('docks the POI list and the layers panel (open by default on tablet)', async () => {
    await renderTab()
    expectDocked(panelTitled(/^Points of interest/))
    expectDocked(panelTitled(/^Layers/))
  })

  it('docks the POI editor', async () => {
    await renderTab()
    fireEvent.click(screen.getByText('Door'))
    expectDocked(panelTitled(/created by/))
  })

  it('docks the photo panel', async () => {
    await renderTab()
    fireEvent.click(screen.getByRole('button', { name: 'Photo marker' }))
    expectDocked(panelTitled(/^Photo$/))
  })

  it('renders the photo list only when asked for, then docks it', async () => {
    await renderTab()
    expect(screen.queryByTestId('photo-list-panel')).toBeNull()
    fireEvent.click(screen.getByRole('button', { name: 'Show photo list' }))
    expectDocked(await screen.findByTestId('photo-list-panel').then((el) => el.closest('[data-testid="overlay-panel"]') as HTMLElement))
    fireEvent.click(screen.getByRole('button', { name: 'Hide photo list' }))
    expect(screen.queryByTestId('photo-list-panel')).toBeNull()
  })

  it('docks the grasp panel', async () => {
    await renderTab()
    fireEvent.click(screen.getByRole('button', { name: /Grasp/ }))
    fireEvent.click(screen.getAllByRole('menuitem')[0])
    expectDocked(panelTitled(/^Grasp$/))
  })

  it('with every panel closed there is no sidebar and the map takes the full width', async () => {
    await renderTab()
    fireEvent.click(screen.getByRole('button', { name: 'Collapse POI list' }))
    fireEvent.click(screen.getByRole('button', { name: 'Hide layers and legend' }))
    expect(screen.queryByTestId('map-sidebar')).toBeNull()
    expect(screen.queryAllByTestId('overlay-panel')).toHaveLength(0)
    expect(screen.queryByTestId('poi-list-toggle')).toBeNull()
    // The POI list comes back from the toolbar.
    fireEvent.click(screen.getByRole('button', { name: 'Show POI list' }))
    expectDocked(panelTitled(/^Points of interest/))
  })

  it('a pointerdown inside the sidebar does not reach the map', async () => {
    await renderTab()
    const map = screen.getByTestId('map-container')
    const onMap = vi.fn()
    map.addEventListener('pointerdown', onMap)
    window.addEventListener('pointerdown', onMap)
    await act(async () => {
      fireEvent.pointerDown(within(screen.getByTestId('map-sidebar')).getByText('Door'))
    })
    window.removeEventListener('pointerdown', onMap)
    expect(onMap).not.toHaveBeenCalled()
  })

  it('phone: no sidebar column, panels stay bottom sheets over the map', async () => {
    viewportWidth = 400
    await renderTab()
    expect(screen.queryByTestId('map-sidebar')).toBeNull()
    fireEvent.click(screen.getByRole('button', { name: 'Photo marker' }))
    const panel = panelTitled(/^Photo$/)
    expect(screen.getByTestId('map-container').contains(panel)).toBe(true)
    expect(panel.style.position).toBe('absolute')
    expect(panel.style.bottom).toBe('0px')
  })
})
