import { describe, expect, it } from 'vitest'
import { tabBarMode } from './tabBarMode'

describe('tabBarMode', () => {
  it('phone uses the drawer with label-only scrollable tabs', () => {
    expect(tabBarMode('phone')).toEqual({ drawer: true, scrollable: true })
  })
  it('tablet uses scrollable label-only tabs without a drawer', () => {
    expect(tabBarMode('tablet')).toEqual({ drawer: false, scrollable: true })
  })
  it('desktop shows label-only tabs and no scrolling', () => {
    expect(tabBarMode('desktop')).toEqual({ drawer: false, scrollable: false })
  })
})
