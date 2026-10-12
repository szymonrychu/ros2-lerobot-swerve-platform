// @vitest-environment jsdom
import { act, cleanup, fireEvent, render, screen } from '@testing-library/react'
import { useEffect } from 'react'
import type { CSSProperties, ReactNode } from 'react'
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import type { Photo } from '../photos/api'
import type { PhotoLayerData } from '../photos/usePhotoLayer'
import { itemId } from '../poi/poiItems'
import type { Poi } from '../poi/types'
import type { TabConfig } from '../types'
import type { MapSceneProps, SceneController } from './MapScene'
import { consumeFocus, resolveFocus, whenController, type FocusTarget } from './mapFocus'
import { MapPopup } from './MapPopup'

const mocks = vi.hoisted(() => ({
  invalidate: vi.fn(),
  canvas: null as HTMLCanvasElement | null,
  centerOn: vi.fn(),
  /** ms after mount at which the mocked scene exposes its controller; null = never. */
  controllerDelayMs: 0 as number | null,
}))

vi.mock('@react-three/drei', async (importOriginal) => ({
  ...(await importOriginal<typeof import('@react-three/drei')>()),
  Html: ({ children, style, position }: { children: ReactNode; style?: CSSProperties; position?: unknown }) => (
    <div data-testid="drei-html" data-position={JSON.stringify(position)} style={style}>
      {children}
    </div>
  ),
}))

vi.mock('@react-three/fiber', async (importOriginal) => {
  const state = () => ({ invalidate: mocks.invalidate, gl: { domElement: mocks.canvas } })
  return {
    ...(await importOriginal<typeof import('@react-three/fiber')>()),
    useThree: (sel?: (s: ReturnType<typeof state>) => unknown) => (sel ? sel(state()) : state()),
  }
})

// The real scene needs WebGL: this stand-in exposes a controller (optionally late) and renders the real popup.
vi.mock('./MapScene', async () => {
  const { MapPopup: Popup } = await import('./MapPopup')
  function FakeScene(props: MapSceneProps) {
    useEffect(() => {
      const delay = mocks.controllerDelayMs
      if (delay === null) return
      const controller: SceneController = { pick: () => null, centerOn: mocks.centerOn, fit: () => undefined }
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
        <span data-testid="photo-count">{props.photos.length}</span>
        {props.popup && <Popup target={props.popup} onClose={props.onPopupClose} onOpen={props.onPopupOpen} />}
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

function target(over: Partial<FocusTarget['item']> = {}): FocusTarget {
  return {
    anchor: { x: 1, y: 2 },
    item: { key: 'poi:p1', kind: 'area', name: 'Kitchen', x: 1, y: 2, status: 'done', zone: 'keepout', note: 'mind the step', updated: 1, source: 'poi', ...over },
  }
}

describe('MapPopup', () => {
  beforeEach(() => {
    mocks.invalidate.mockClear()
    mocks.canvas = document.createElement('canvas')
    document.body.appendChild(mocks.canvas)
  })
  afterEach(() => mocks.canvas?.remove())

  it('renders one Html at the anchor with pointer events on, and the POI details', () => {
    render(<MapPopup target={target()} onClose={vi.fn()} onOpen={vi.fn()} />)
    const html = screen.getAllByTestId('drei-html')
    expect(html).toHaveLength(1)
    expect(html[0].style.pointerEvents).toBe('auto')
    expect(JSON.parse(html[0].dataset.position ?? 'null')).toHaveLength(3)
    expect(screen.getByText('Kitchen')).toBeTruthy()
    expect(screen.getByText('area')).toBeTruthy()
    expect(screen.getByText('keepout')).toBeTruthy()
    expect(screen.getByText('done')).toBeTruthy()
    expect(screen.getByText('mind the step')).toBeTruthy()
    expect(screen.getByTestId('map-popup-arrow')).toBeTruthy()
  })

  it('shows a photo thumbnail and capture time', () => {
    const t: FocusTarget = {
      anchor: { x: 3, y: 4 },
      item: { key: 'photo:f1', kind: 'photo', name: 'Shot', x: 3, y: 4, note: '', updated: 1700000000, source: 'photo' },
    }
    render(<MapPopup target={t} onClose={vi.fn()} onOpen={vi.fn()} />)
    const img = screen.getByRole('img') as HTMLImageElement
    expect(img.getAttribute('src')).toBe('/api/photos/f1.jpg?max_px=144')
    expect(screen.getByText('2023-11-14T22:13:20Z')).toBeTruthy()
  })

  it('does not let a press reach the map (parent pointerdown not called)', () => {
    const parentDown = vi.fn()
    render(
      <div onPointerDown={parentDown} onDoubleClick={parentDown}>
        <MapPopup target={target()} onClose={vi.fn()} onOpen={vi.fn()} />
      </div>,
    )
    fireEvent.pointerDown(screen.getByText('Kitchen'))
    fireEvent.doubleClick(screen.getByText('Kitchen'))
    expect(parentDown).not.toHaveBeenCalled()
  })

  it('closes on its X and on Escape', () => {
    const onClose = vi.fn()
    render(<MapPopup target={target()} onClose={onClose} onOpen={vi.fn()} />)
    fireEvent.click(screen.getByRole('button', { name: 'Close popup' }))
    expect(onClose).toHaveBeenCalledTimes(1)
    fireEvent.keyDown(window, { key: 'Escape' })
    expect(onClose).toHaveBeenCalledTimes(2)
  })

  it('closes on a map tap but not on a map drag', () => {
    const onClose = vi.fn()
    render(<MapPopup target={target()} onClose={onClose} onOpen={vi.fn()} />)
    const canvas = mocks.canvas!
    // jsdom has no PointerEvent: a MouseEvent carries the coordinates and button the handlers read.
    const pointer = (el: EventTarget, type: string, x: number) =>
      el.dispatchEvent(new MouseEvent(type, { clientX: x, clientY: 10, button: 0, bubbles: true }))
    pointer(canvas, 'pointerdown', 10)
    pointer(window, 'pointerup', 80)
    expect(onClose).not.toHaveBeenCalled()
    pointer(canvas, 'pointerdown', 10)
    pointer(window, 'pointerup', 11)
    expect(onClose).toHaveBeenCalledTimes(1)
  })

  it('opens the item through its action', () => {
    const onOpen = vi.fn()
    const t = target()
    render(<MapPopup target={t} onClose={vi.fn()} onOpen={onOpen} />)
    fireEvent.click(screen.getByRole('button', { name: 'Edit' }))
    expect(onOpen).toHaveBeenCalledWith(t.item)
  })

  it('invalidates on open and on content change', () => {
    const { rerender } = render(<MapPopup target={target()} onClose={vi.fn()} onOpen={vi.fn()} />)
    expect(mocks.invalidate).toHaveBeenCalled()
    const before = mocks.invalidate.mock.calls.length
    rerender(<MapPopup target={target({ name: 'Pantry' })} onClose={vi.fn()} onOpen={vi.fn()} />)
    expect(mocks.invalidate.mock.calls.length).toBeGreaterThan(before)
  })
})

describe('MapNavTab show-on-map', () => {
  const TAB: TabConfig = { id: 'map', type: 'map_nav', label: 'Map', poi_list_topic: '/poi/list', poi_command_topic: '/poi/command' } as TabConfig
  const TOPICS = { '/poi/list': { revision: 1, pois: [poi({ id: 'p1', name: 'Door', note: 'front entrance' })] } }
  const PHOTOS: PhotoLayerData = { photos: [photo({ id: 'f1', name: 'Shot' })], total: 1, error: null }

  beforeEach(() => {
    mocks.centerOn.mockClear()
    mocks.controllerDelayMs = 0
    mocks.canvas = document.createElement('canvas')
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
    vi.stubGlobal('matchMedia', (q: string) => ({ matches: false, media: q, addEventListener: () => undefined, removeEventListener: () => undefined, addListener: () => undefined, removeListener: () => undefined, onchange: null, dispatchEvent: () => false }))
  })
  afterEach(() => {
    vi.unstubAllGlobals()
  })

  async function renderTab(focus: { kind: 'poi' | 'photo'; id: string } | null, onFocusConsumed = vi.fn()) {
    const { default: MapNavTab } = await import('../tabs/MapNavTab')
    render(
      <MapNavTab tab={TAB} topicData={TOPICS} publish={vi.fn()} photos={PHOTOS} pendingFocus={focus} onFocusConsumed={onFocusConsumed} />,
    )
    await screen.findByTestId('scene')
    return onFocusConsumed
  }

  it('centres the POI, opens its popup and consumes the request once the controller exists', async () => {
    const consumed = await renderTab({ kind: 'poi', id: 'p1' })
    await screen.findByRole('dialog', { name: 'Door' })
    expect(mocks.centerOn).toHaveBeenCalledWith({ x: 1, y: 2 })
    expect(consumed).toHaveBeenCalledTimes(1)
    expect(screen.getByText('front entrance')).toBeTruthy()
  })

  it('waits for a late controller before consuming', async () => {
    mocks.controllerDelayMs = 350
    const consumed = await renderTab({ kind: 'poi', id: 'p1' })
    expect(consumed).not.toHaveBeenCalled()
    expect(screen.queryByRole('dialog')).toBeNull()
    await screen.findByRole('dialog', { name: 'Door' }, { timeout: 2000 })
    expect(mocks.centerOn).toHaveBeenCalledTimes(1)
    expect(consumed).toHaveBeenCalledTimes(1)
  })

  it('never consumes while the controller is missing', async () => {
    mocks.controllerDelayMs = null
    const consumed = await renderTab({ kind: 'poi', id: 'p1' })
    await act(() => new Promise((r) => setTimeout(r, 400)))
    expect(mocks.centerOn).not.toHaveBeenCalled()
    expect(consumed).not.toHaveBeenCalled()
    expect(screen.queryByRole('dialog')).toBeNull()
  })

  it('leaves an unknown id unconsumed and opens nothing', async () => {
    const consumed = await renderTab({ kind: 'poi', id: 'gone' })
    await act(() => new Promise((r) => setTimeout(r, 250)))
    expect(consumed).not.toHaveBeenCalled()
    expect(screen.queryByRole('dialog')).toBeNull()
  })

  it("opens a photo from App's photos and its action opens the photo panel", async () => {
    const consumed = await renderTab({ kind: 'photo', id: 'f1' })
    await screen.findByRole('dialog', { name: 'Shot' })
    expect(screen.getByTestId('photo-count').textContent).toBe('1')
    expect(mocks.centerOn).toHaveBeenCalledWith({ x: 3, y: 4 })
    expect(consumed).toHaveBeenCalledTimes(1)
    fireEvent.click(screen.getByRole('button', { name: 'Open photo' }))
    expect(await screen.findByRole('button', { name: 'Close photo' })).toBeTruthy()
  })

  it("the popup's Edit opens the POI editor and only one popup shows", async () => {
    await renderTab({ kind: 'poi', id: 'p1' })
    await screen.findByRole('dialog', { name: 'Door' })
    expect(screen.getAllByRole('dialog')).toHaveLength(1)
    fireEvent.click(screen.getByRole('button', { name: 'Edit' }))
    expect(await screen.findByRole('button', { name: 'Close editor' })).toBeTruthy()
    expect(screen.getByDisplayValue('front entrance')).toBeTruthy()
  })
})
