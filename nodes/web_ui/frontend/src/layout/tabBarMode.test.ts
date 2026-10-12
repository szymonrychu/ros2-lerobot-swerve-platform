import { describe, expect, it } from 'vitest'
import { tabBarMode } from './tabBarMode'

describe('tabBarMode', () => {
  it('phone uses the drawer with label-only scrollable tabs', () => {
    expect(tabBarMode('phone')).toEqual({ drawer: true, showIcons: false, showLabels: true, scrollable: true })
  })
  it('tablet uses scrollable icon+label tabs without a drawer', () => {
    expect(tabBarMode('tablet')).toEqual({ drawer: false, showIcons: true, showLabels: true, scrollable: true })
  })
  it('desktop shows full labels with icons and no scrolling', () => {
    expect(tabBarMode('desktop')).toEqual({ drawer: false, showIcons: true, showLabels: true, scrollable: false })
  })
})
