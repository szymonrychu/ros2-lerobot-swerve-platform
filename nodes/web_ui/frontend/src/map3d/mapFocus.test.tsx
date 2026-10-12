// @vitest-environment jsdom
import { act, cleanup, fireEvent, render, screen } from '@testing-library/react'
import ThemeProvider from '@mui/material/styles/ThemeProvider'
import { useEffect } from 'react'
import { afterEach, beforeAll, beforeEach, describe, expect, it, vi } from 'vitest'
import type { Photo } from '../photos/api'
import type { PhotoLayerData } from '../photos/usePhotoLayer'
import { itemId } from '../poi/poiItems'
import type { Poi } from '../poi/types'
import { theme, TOUCH_TARGET_PX } from '../theme'
import type { TabConfig } from '../types'
import type { MapSceneProps, SceneController } from './MapScene'
import { designViolations } from '../layout/designRules'
import { consumeFocus, resolveFocus, whenController } from './mapFocus'
import mapFocusSource from './mapFocus.ts?raw'
import mapSceneSource from './MapScene.tsx?raw'

const mocks = vi.hoisted(() => ({
  centerOn: vi.fn(),
  /** ms after mount at which the mocked scene exposes its controller; null = never. */
  controllerDelayMs: 0 as number | null,
}))

// The real scene needs WebGL: this stand-in exposes a controller (optionally late) and a photo marker to tap.
vi.mock('./MapScene', () => {
  function FakeScene(props: MapSceneProps) {
    useEffect(() => {
      const delay = mocks.controllerDelayMs
      if (delay === null) return
      const controller: SceneController = { pick: () => ({ x: 1, y: 2 }), centerOn: mocks.centerOn, fit: () => undefined }
      const timer = setTimeout(() => {
        props.controllerRef.current = controller
      }, delay)
      return () => {
        clearTimeout(timer)
        props.controllerRef.current = null
      }
    }, [props.controllerRef])
    return (
      <div data-testid="scene">
        <canvas data-testid="canvas" />
        <span data-testid="photo-count">{props.photos.length}</span>
        <button type="button" onClick={() => props.onSelectPhoto?.('f1')}>
          tap photo marker
        </button>
      </div>
    )
  }
  return { default: FakeScene, MapScene: FakeScene }
})

function poi(over: Partial<Poi>): Poi {
  return {
    id: 'p1', kind: 'point', frame: 'map', x: 1, y: 2, polygon: [], radius_m: 0.5, name: 'Door', note: '',
    status: 'open', created_by: 'user', created_at: 1, updated_at: 1, zone: 'none', ...over,
  }
}

function photo(over: Partial<Photo>): Photo {
  return {
    id: 'f1', ts: 1700000000, camera: 'head', tool: 'take_photo', name: 'Shot', note: '', poiIds: [], x: 3, y: 4,
    view: null, bytes: 1, width: 1, height: 1, ...over,
  }
}

function controllerStub(): SceneController & { centerOn: ReturnType<typeof vi.fn> } {
  return { pick: () => null, centerOn: vi.fn(), fit: () => undefined }
}

afterEach(() => {
  cleanup()
  vi.useRealTimers()
})

describe('resolveFocus', () => {
  it('finds a point POI', () => {
    const r = resolveFocus({ kind: 'poi', id: 'p1' }, [poi({})], [])
    expect(r?.anchor).toEqual({ x: 1, y: 2 })
    expect(r?.item).toMatchObject({ key: 'poi:p1', name: 'Door', kind: 'point', source: 'poi' })
  })

  it('centres an area on its centroid', () => {
    const polygon: [number, number][] = [[0, 0], [4, 0], [4, 2], [0, 2]]
    const r = resolveFocus({ kind: 'poi', id: 'a' }, [poi({ id: 'a', kind: 'area', polygon })], [])
    expect(r?.anchor.x).toBeCloseTo(2)
    expect(r?.anchor.y).toBeCloseTo(1)
  })

  it('finds a located photo', () => {
    const r = resolveFocus({ kind: 'photo', id: 'f1' }, [], [photo({})])
    expect(r?.anchor).toEqual({ x: 3, y: 4 })
    expect(r?.item).toMatchObject({ key: 'photo:f1', kind: 'photo', source: 'photo', updated: 1700000000 })
  })

  it('returns null for an unknown id or an unlocated photo', () => {
    expect(resolveFocus({ kind: 'poi', id: 'zz' }, [poi({})], [])).toBeNull()
    expect(resolveFocus({ kind: 'photo', id: 'f1' }, [], [photo({ x: null, y: null })])).toBeNull()
    expect(resolveFocus({ kind: 'photo', id: 'p1' }, [poi({})], [])).toBeNull()
  })
})

describe('itemId', () => {
  it('strips the source prefix of the key', () => {
    expect(itemId(resolveFocus({ kind: 'poi', id: 'p:1' }, [poi({ id: 'p:1' })], [])!.item)).toBe('p:1')
    expect(itemId(resolveFocus({ kind: 'photo', id: 'f1' }, [], [photo({})])!.item)).toBe('f1')
  })
})

describe('whenController / consumeFocus', () => {
  beforeEach(() => {
    vi.useFakeTimers()
  })

  it('runs at once when the controller already exists', () => {
    const c = controllerStub()
    const run = vi.fn()
    whenController(() => c, run)
    expect(run).toHaveBeenCalledWith(c)
  })

  it('centres once and reports consumption with an immediate controller', () => {
    const c = controllerStub()
    const onFocused = vi.fn()
    consumeFocus({ kind: 'poi', id: 'p1' }, [poi({})], [], () => c, onFocused)
    expect(c.centerOn).toHaveBeenCalledTimes(1)
    expect(c.centerOn).toHaveBeenCalledWith({ x: 1, y: 2 })
    expect(onFocused).toHaveBeenCalledTimes(1)
    expect(onFocused.mock.calls[0][0].item.key).toBe('poi:p1')
    vi.advanceTimersByTime(1000)
    expect(c.centerOn).toHaveBeenCalledTimes(1)
    expect(onFocused).toHaveBeenCalledTimes(1)
  })

  it('retries until a late controller exists, then centres once', () => {
    const c = controllerStub()
    let ready: SceneController | null = null
    const onFocused = vi.fn()
    consumeFocus({ kind: 'photo', id: 'f1' }, [], [photo({})], () => ready, onFocused)
    vi.advanceTimersByTime(350)
    expect(onFocused).not.toHaveBeenCalled()
    ready = c
    vi.advanceTimersByTime(100)
    expect(c.centerOn).toHaveBeenCalledTimes(1)
    expect(c.centerOn).toHaveBeenCalledWith({ x: 3, y: 4 })
    expect(onFocused).toHaveBeenCalledTimes(1)
    vi.advanceTimersByTime(1000)
    expect(c.centerOn).toHaveBeenCalledTimes(1)
  })

  it('does nothing for an unknown id', () => {
    const c = controllerStub()
    const onFocused = vi.fn()
    consumeFocus({ kind: 'poi', id: 'nope' }, [poi({})], [], () => c, onFocused)
    vi.advanceTimersByTime(1000)
    expect(c.centerOn).not.toHaveBeenCalled()
    expect(onFocused).not.toHaveBeenCalled()
  })

  it('stops retrying once cancelled', () => {
    const c = controllerStub()
    let ready: SceneController | null = null
    const onFocused = vi.fn()
    const cancel = consumeFocus({ kind: 'poi', id: 'p1' }, [poi({})], [], () => ready, onFocused)
    cancel()
    ready = c
    vi.advanceTimersByTime(1000)
    expect(onFocused).not.toHaveBeenCalled()
  })
})


describe('design rules', () => {
  it('map3d/MapScene.tsx and map3d/mapFocus.ts are clean', () => {
    expect(designViolations('map3d/MapScene.tsx', mapSceneSource)).toEqual([])
    expect(designViolations('map3d/mapFocus.ts', mapFocusSource)).toEqual([])
  })

  it('MapScene has no popup props and the popup component is gone', () => {
    expect(mapSceneSource).not.toMatch(/popup|MapPopup/i)
    expect(Object.keys(import.meta.glob('./MapPopup.tsx'))).toEqual([])
  })
})

type Viewport = 'phone' | 'desktop'

function stubMedia(viewport: Viewport, coarse: boolean): void {
  vi.stubGlobal('matchMedia', (q: string) => ({
    matches: q.includes('pointer: coarse') ? coarse : q.includes('min-width') ? viewport === 'desktop' : false,
    media: q,
    addEventListener: () => undefined,
    removeEventListener: () => undefined,
    addListener: () => undefined,
    removeListener: () => undefined,
    onchange: null,
    dispatchEvent: () => false,
  }))
}

/** Touch size a button gets: inline or plain CSS (jsdom computed style) and the pointer: coarse rules emotion injected. */
function touchSize(el: HTMLElement, prop: 'minWidth' | 'minHeight'): number {
  const values = [parseFloat(el.style[prop]), parseFloat(getComputedStyle(el)[prop])]
  for (const sheet of Array.from(document.styleSheets)) {
    for (const rule of Array.from(sheet.cssRules)) {
      if (!rule.cssText.startsWith('@media (pointer: coarse)')) continue
      for (const inner of Array.from((rule as CSSMediaRule).cssRules)) {
        const style = inner as CSSStyleRule
        if (el.matches(style.selectorText)) values.push(parseFloat(style.style.getPropertyValue(prop === 'minWidth' ? 'min-width' : 'min-height')))
      }
    }
  }
  return Math.max(0, ...values.filter((v) => !Number.isNaN(v)))
}

describe('MapNavTab show-on-map', () => {
  const TAB: TabConfig = { id: 'map', type: 'map_nav', label: 'Map', poi_list_topic: '/poi/list', poi_command_topic: '/poi/command' } as TabConfig
  const TOPICS = { '/poi/list': { revision: 1, pois: [poi({ id: 'p1', name: 'Door', note: 'front entrance' })] } }
  const PHOTOS: PhotoLayerData = { photos: [photo({ id: 'f1', name: 'Shot' })], total: 1, error: null }

  // The first render pays the cold dynamic import of MapNavTab; do it once, outside any test's 5 s budget.
  beforeAll(async () => {
    vi.stubGlobal('localStorage', { getItem: () => null, setItem: () => undefined })
    await import('../tabs/MapNavTab')
    vi.unstubAllGlobals()
  }, 60_000)

  beforeEach(() => {
    mocks.centerOn.mockClear()
    mocks.controllerDelayMs = 0
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
    stubMedia('desktop', false)
  })
  afterEach(() => {
    vi.unstubAllGlobals()
  })

  async function renderTab(focus: { kind: 'poi' | 'photo'; id: string } | null, onFocusConsumed = vi.fn(), onMapPointerDown = vi.fn()) {
    const { default: MapNavTab } = await import('../tabs/MapNavTab')
    render(
      <ThemeProvider theme={theme}>
        <div onPointerDown={onMapPointerDown}>
          <MapNavTab tab={TAB} topicData={TOPICS} publish={vi.fn()} photos={PHOTOS} pendingFocus={focus} onFocusConsumed={onFocusConsumed} />
        </div>
      </ThemeProvider>,
    )
    await screen.findByTestId('scene')
    return onFocusConsumed
  }

  it('centres the POI, opens it in the sidebar and consumes the request once the controller exists', async () => {
    const consumed = await renderTab({ kind: 'poi', id: 'p1' })
    expect(await screen.findByDisplayValue('front entrance')).toBeTruthy()
    expect(screen.getByTestId('map-sidebar').contains(screen.getByDisplayValue('front entrance'))).toBe(true)
    expect(mocks.centerOn).toHaveBeenCalledTimes(1)
    expect(mocks.centerOn).toHaveBeenCalledWith({ x: 1, y: 2 })
    expect(consumed).toHaveBeenCalledTimes(1)
    expect(screen.queryByRole('dialog')).toBeNull()
  })

  it('opens the POI as a bottom sheet on a phone', async () => {
    stubMedia('phone', false)
    await renderTab({ kind: 'poi', id: 'p1' })
    expect(await screen.findByDisplayValue('front entrance')).toBeTruthy()
    expect(screen.queryByTestId('map-sidebar')).toBeNull()
    expect(screen.getByDisplayValue('front entrance').closest('[data-testid="overlay-panel"]')?.getAttribute('data-layout')).toBe('phone')
  })

  it('waits for a late controller before centring and consuming', async () => {
    mocks.controllerDelayMs = 350
    const consumed = await renderTab({ kind: 'poi', id: 'p1' })
    expect(consumed).not.toHaveBeenCalled()
    expect(screen.queryByDisplayValue('front entrance')).toBeNull()
    await screen.findByDisplayValue('front entrance', {}, { timeout: 2000 })
    expect(mocks.centerOn).toHaveBeenCalledTimes(1)
    expect(consumed).toHaveBeenCalledTimes(1)
  })

  it('never consumes while the controller is missing', async () => {
    mocks.controllerDelayMs = null
    const consumed = await renderTab({ kind: 'poi', id: 'p1' })
    await act(() => new Promise((r) => setTimeout(r, 400)))
    expect(mocks.centerOn).not.toHaveBeenCalled()
    expect(consumed).not.toHaveBeenCalled()
    expect(screen.queryByDisplayValue('front entrance')).toBeNull()
  })

  it('leaves an unknown id unconsumed and opens nothing', async () => {
    const consumed = await renderTab({ kind: 'poi', id: 'gone' })
    await act(() => new Promise((r) => setTimeout(r, 250)))
    expect(consumed).not.toHaveBeenCalled()
    expect(screen.queryByDisplayValue('front entrance')).toBeNull()
    expect(screen.queryByRole('group', { name: 'Linked POIs' })).toBeNull()
  })

  it("opens a photo from App's photos in the photo panel", async () => {
    const consumed = await renderTab({ kind: 'photo', id: 'f1' })
    expect(await screen.findByRole('group', { name: 'Linked POIs' })).toBeTruthy()
    expect(screen.getByTestId('photo-count').textContent).toBe('1')
    expect(mocks.centerOn).toHaveBeenCalledWith({ x: 3, y: 4 })
    expect(consumed).toHaveBeenCalledTimes(1)
    expect(screen.getByTestId('map-sidebar').contains(screen.getByRole('group', { name: 'Linked POIs' }))).toBe(true)
  })

  it('a tap on a photo marker opens the photo the same way', async () => {
    await renderTab(null)
    fireEvent.click(screen.getByRole('button', { name: 'tap photo marker' }))
    expect(await screen.findByRole('group', { name: 'Linked POIs' })).toBeTruthy()
    expect(screen.getByTestId('map-sidebar').contains(screen.getByRole('group', { name: 'Linked POIs' }))).toBe(true)
  })

  it('a tap on a POI marker opens the POI in the sidebar', async () => {
    await renderTab(null)
    expect(screen.queryByDisplayValue('front entrance')).toBeNull()
    await act(() => new Promise((r) => setTimeout(r, 50))) // the mocked scene exposes its controller on a timer
    // jsdom has no PointerEvent: a MouseEvent of the same type carries the button and coordinates the handler reads.
    act(() => {
      screen.getByTestId('canvas').dispatchEvent(new MouseEvent('pointerdown', { bubbles: true, button: 0, clientX: 10, clientY: 10 }))
    })
    expect(await screen.findByDisplayValue('front entrance')).toBeTruthy()
    expect(screen.getByTestId('map-sidebar').contains(screen.getByDisplayValue('front entrance'))).toBe(true)
    expect(screen.queryByRole('dialog')).toBeNull()
  })

  it('only one item is open at a time', async () => {
    await renderTab({ kind: 'poi', id: 'p1' })
    await screen.findByDisplayValue('front entrance')
    fireEvent.click(screen.getByRole('button', { name: 'tap photo marker' }))
    expect(await screen.findByRole('group', { name: 'Linked POIs' })).toBeTruthy()
    expect(screen.queryByDisplayValue('front entrance')).toBeNull()
  })

  it('a pointerdown inside the opened panel does not reach the map', async () => {
    const mapDown = vi.fn()
    await renderTab({ kind: 'poi', id: 'p1' }, vi.fn(), mapDown)
    fireEvent.pointerDown(await screen.findByDisplayValue('front entrance'))
    expect(mapDown).not.toHaveBeenCalled()
  })

  it.each([
    ['POI editor', { kind: 'poi' as const, id: 'p1' }],
    ['photo panel', { kind: 'photo' as const, id: 'f1' }],
  ])('the %s action buttons meet TOUCH_TARGET_PX on a coarse pointer', async (_name, focus) => {
    stubMedia('desktop', true)
    await renderTab(focus)
    await screen.findByTestId('map-sidebar')
    const panel = (await screen.findByRole('button', { name: 'Save' })).closest('[data-testid="overlay-panel"]') as HTMLElement
    const buttons = Array.from(panel.querySelectorAll('button')) as HTMLElement[]
    expect(buttons.length).toBeGreaterThanOrEqual(2)
    for (const btn of buttons) {
      expect(touchSize(btn, 'minWidth'), btn.textContent ?? btn.getAttribute('aria-label') ?? '').toBeGreaterThanOrEqual(TOUCH_TARGET_PX)
      expect(touchSize(btn, 'minHeight')).toBeGreaterThanOrEqual(TOUCH_TARGET_PX)
    }
  })
})
