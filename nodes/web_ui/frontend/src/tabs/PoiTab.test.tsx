// @vitest-environment jsdom
import { cleanup, fireEvent, render, screen, within } from '@testing-library/react'
import { afterEach, describe, expect, it, vi } from 'vitest'
import PoiTab, { swatchColor } from './PoiTab'
import { sortItems, type PoiItem } from '../poi/poiItems'
import { KIND_COLORS, markerColors, STATUS_COLORS, ZONE_COLORS } from '../poi/style'
import type { Poi } from '../poi/types'
import { TOUCH_TARGET_PX } from '../theme'

function item(over: Partial<PoiItem>): PoiItem {
  return { key: 'poi:a', kind: 'point', name: 'Door', x: 1, y: 2, note: '', updated: 1000, source: 'poi', ...over }
}

const ITEMS: PoiItem[] = [
  item({ key: 'poi:a', kind: 'point', name: 'Door', status: 'open', note: 'front entrance', updated: 1000 }),
  item({ key: 'poi:b', kind: 'area', name: 'Kitchen', zone: 'keepout', status: 'done', updated: 3000 }),
  item({ key: 'poi:c', kind: 'object', name: 'Mug', updated: 2000 }),
  item({ key: 'photo:d', kind: 'photo', name: 'Shot', source: 'photo', x: null, y: null, updated: 4000 }),
]

afterEach(cleanup)

function rows(): HTMLElement[] {
  return screen.getAllByRole('listitem')
}

describe('PoiTab', () => {
  it('lists every item in sortItems order', () => {
    render(<PoiTab items={ITEMS} onShowOnMap={vi.fn()} />)
    const expected = sortItems(ITEMS).map((i) => i.name)
    expect(rows().map((r) => expected.find((n) => within(r).queryByText(n)))).toEqual(expected)
  })

  it('toggles each kind chip as a filter', () => {
    render(<PoiTab items={ITEMS} onShowOnMap={vi.fn()} />)
    const names: Record<string, string> = { point: 'Door', area: 'Kitchen', object: 'Mug', photo: 'Shot' }
    for (const [kind, name] of Object.entries(names)) {
      fireEvent.click(screen.getByRole('button', { name: new RegExp(`^${kind}$`, 'i') }))
      expect(rows()).toHaveLength(1)
      expect(within(rows()[0]).getByText(name)).toBeTruthy()
      fireEvent.click(screen.getByRole('button', { name: new RegExp(`^${kind}$`, 'i') }))
      expect(rows()).toHaveLength(4)
    }
  })

  it('searches name and note', () => {
    render(<PoiTab items={ITEMS} onShowOnMap={vi.fn()} />)
    const box = screen.getByRole('textbox', { name: /search/i })
    fireEvent.change(box, { target: { value: 'entrance' } })
    expect(rows()).toHaveLength(1)
    fireEvent.change(box, { target: { value: 'mug' } })
    expect(rows()).toHaveLength(1)
  })

  it('shows name, kind swatch, zone, status and updated time', () => {
    render(<PoiTab items={[ITEMS[1]]} onShowOnMap={vi.fn()} now={5000} />)
    const row = rows()[0]
    expect(within(row).getByText('Kitchen')).toBeTruthy()
    expect(within(row).getByText(/keepout/i)).toBeTruthy()
    expect(within(row).getByText(/done/i)).toBeTruthy()
    expect(within(row).getByText(/33 min ago/)).toBeTruthy()
    const swatch = within(row).getByTestId('kind-swatch')
    expect(swatch.getAttribute('data-color')).toBe(STATUS_COLORS.cancelled)
    expect(within(row).getByText('area')).toBeTruthy()
  })

  it('swatch of an open keepout area is the zone colour, as on the map', () => {
    render(<PoiTab items={[{ ...ITEMS[1], status: 'open' }]} onShowOnMap={vi.fn()} />)
    expect(screen.getByTestId('kind-swatch').getAttribute('data-color')).toBe(ZONE_COLORS.keepout)
  })

  it('swatchColor is the map marker fill for every item', () => {
    for (const item of ITEMS) {
      if (item.source !== 'poi') continue
      const poi = { ...item, id: 'x', frame: 'map', polygon: [], radius_m: 1, status: item.status ?? 'open', created_by: 'user', created_at: 0, updated_at: 0 } as unknown as Poi
      expect(swatchColor(item)).toBe(markerColors(poi).fill)
    }
  })

  it('swatch uses the kind colour', () => {
    render(<PoiTab items={[ITEMS[2]]} onShowOnMap={vi.fn()} />)
    expect(screen.getByTestId('kind-swatch').getAttribute('data-color')).toBe(KIND_COLORS.object)
  })

  it('Show on map calls onShowOnMap with the focus request', () => {
    const onShow = vi.fn()
    render(<PoiTab items={[ITEMS[0]]} onShowOnMap={onShow} />)
    const btn = screen.getByRole('button', { name: /show on map/i })
    fireEvent.click(btn)
    expect(onShow).toHaveBeenCalledWith({ kind: 'poi', id: 'a' })
  })

  it('disables Show on map for an unlocated photo, with a reason', () => {
    const onShow = vi.fn()
    render(<PoiTab items={[ITEMS[3]]} onShowOnMap={onShow} />)
    const btn = screen.getByRole('button', { name: /show on map/i }) as HTMLButtonElement
    expect(btn.disabled).toBe(true)
    expect(screen.getByText(/no map position/i)).toBeTruthy()
    fireEvent.click(btn)
    expect(onShow).not.toHaveBeenCalled()
  })

  it('shows an empty state for no items', () => {
    render(<PoiTab items={[]} onShowOnMap={vi.fn()} />)
    expect(screen.queryAllByRole('listitem')).toHaveLength(0)
    expect(screen.getByText(/no points of interest/i)).toBeTruthy()
  })

  it('shows a filtered-empty state when nothing matches', () => {
    render(<PoiTab items={ITEMS} onShowOnMap={vi.fn()} />)
    fireEvent.change(screen.getByRole('textbox', { name: /search/i }), { target: { value: 'zzz' } })
    expect(screen.queryAllByRole('listitem')).toHaveLength(0)
    expect(screen.getByText(/no matches/i)).toBeTruthy()
  })

  it('uses relative sizes and touch-sized buttons', () => {
    render(<PoiTab items={[ITEMS[0]]} onShowOnMap={vi.fn()} />)
    const btn = screen.getByRole('button', { name: /show on map/i })
    expect(getComputedStyle(btn).minHeight).toBe(`${TOUCH_TARGET_PX}px`)
  })
})
