import { describe, expect, it } from 'vitest'
import { ALWAYS_VISIBLE_GROUPS, STOP_GROUPS, TOOLBAR_GROUPS, anyInRow, splitToolbar } from './toolbarGroups'
import type { LayoutClass } from '../layout/useLayout'
import { designViolations, SOURCES } from '../layout/designRules'

const ids = (gs: readonly { id: string }[]) => gs.map((g) => g.id)

describe('splitToolbar', () => {
  it('moves low-priority groups to overflow on phone', () => {
    const { visible, overflow } = splitToolbar(TOOLBAR_GROUPS, 'phone')
    expect(ids(overflow)).toEqual(expect.arrayContaining(['layers', 'poi', 'cameraView']))
    expect(ids(visible)).not.toContain('layers')
  })

  it.each<LayoutClass>(['tablet', 'desktop'])('has no overflow on %s', (layout) => {
    const { visible, overflow } = splitToolbar(TOOLBAR_GROUPS, layout)
    expect(overflow).toEqual([])
    expect(ids(visible)).toEqual(ids(TOOLBAR_GROUPS))
  })

  it('has no grasp or arm home group any more', () => {
    expect(ids(TOOLBAR_GROUPS)).toEqual(['stop', 'cameraView', 'goal', 'poi', 'layers', 'mapFile'])
    expect(ALWAYS_VISIBLE_GROUPS).toEqual(['stop', 'goal'])
  })

  it.each<LayoutClass>(['phone', 'tablet', 'desktop'])('keeps STOP and goal visible on %s', (layout) => {
    const visible = ids(splitToolbar(TOOLBAR_GROUPS, layout).visible)
    for (const id of ['stop', 'goal']) expect(visible).toContain(id)
    for (const id of ALWAYS_VISIBLE_GROUPS) expect(visible).toContain(id)
  })

  it('preserves order and loses nothing', () => {
    const { visible, overflow } = splitToolbar(TOOLBAR_GROUPS, 'phone')
    expect([...visible, ...overflow].length).toBe(TOOLBAR_GROUPS.length)
    expect(ids(visible)).toEqual(ids(TOOLBAR_GROUPS.filter((g) => g.priority === 'high')))
  })
})

describe('anyInRow', () => {
  it('is false on phone for the map view group (both children are in the menu)', () => {
    const { overflow } = splitToolbar(TOOLBAR_GROUPS, 'phone')
    expect(anyInRow(overflow, ['cameraView', 'layers'])).toBe(false)
  })

  it.each<LayoutClass>(['tablet', 'desktop'])('is true on %s', (layout) => {
    const { overflow } = splitToolbar(TOOLBAR_GROUPS, layout)
    expect(anyInRow(overflow, ['cameraView', 'layers'])).toBe(true)
  })
})

describe('stop controls', () => {
  it('STOP is the one high-priority always-visible stop group', () => {
    expect(STOP_GROUPS).toEqual(['stop'])
    expect(ALWAYS_VISIBLE_GROUPS).toContain('stop')
    expect(TOOLBAR_GROUPS.find((g) => g.id === 'stop')?.priority).toBe('high')
  })

  it.each<LayoutClass>(['phone', 'tablet', 'desktop'])('STOP is in the primary toolbar, never in overflow, on %s', (layout) => {
    const { visible, overflow } = splitToolbar(TOOLBAR_GROUPS, layout)
    expect(ids(visible)).toContain('stop')
    expect(ids(overflow)).not.toContain('stop')
  })
})

describe('design rules', () => {
  it('toolbarGroups.ts has no violations', () => {
    expect(designViolations('map/toolbarGroups.ts', SOURCES['map/toolbarGroups.ts'])).toEqual([])
  })
  it('MapNavTab.tsx has no violations', () => {
    expect(designViolations('tabs/MapNavTab.tsx', SOURCES['tabs/MapNavTab.tsx'])).toEqual([])
  })
})
