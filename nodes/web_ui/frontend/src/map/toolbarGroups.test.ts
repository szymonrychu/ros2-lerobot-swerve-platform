import { describe, expect, it } from 'vitest'
import { ALWAYS_VISIBLE_GROUPS, TOOLBAR_GROUPS, splitToolbar } from './toolbarGroups'
import type { LayoutClass } from '../layout/useLayout'

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

  it.each<LayoutClass>(['phone', 'tablet', 'desktop'])('keeps STOP and goal / arm control visible on %s', (layout) => {
    const visible = ids(splitToolbar(TOOLBAR_GROUPS, layout).visible)
    for (const id of ['stop', 'goal', 'grasp', 'armHome']) expect(visible).toContain(id)
    for (const id of ALWAYS_VISIBLE_GROUPS) expect(visible).toContain(id)
  })

  it('preserves order and loses nothing', () => {
    const { visible, overflow } = splitToolbar(TOOLBAR_GROUPS, 'phone')
    expect([...visible, ...overflow].length).toBe(TOOLBAR_GROUPS.length)
    expect(ids(visible)).toEqual(ids(TOOLBAR_GROUPS.filter((g) => g.priority === 'high')))
  })
})
