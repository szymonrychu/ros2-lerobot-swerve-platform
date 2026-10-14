import { describe, expect, it } from 'vitest'
import { tabBarMode } from './tabBarMode'

describe('tabBarMode', () => {
  it('phone: menu present, tabs in the drawer, label-only scrollable tabs', () => {
    expect(tabBarMode('phone')).toEqual({ menu: true, tabsInDrawer: true, scrollable: true })
  })
  it('tablet: menu present with Calibrate only, scrollable tab bar', () => {
    expect(tabBarMode('tablet')).toEqual({ menu: true, tabsInDrawer: false, scrollable: true })
  })
  it('desktop: menu present with Calibrate only, no scrolling', () => {
    expect(tabBarMode('desktop')).toEqual({ menu: true, tabsInDrawer: false, scrollable: false })
  })
})
