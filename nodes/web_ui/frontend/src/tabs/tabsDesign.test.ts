import { describe, expect, it } from 'vitest'
import { designViolations, SOURCES } from '../layout/designRules'
import { ARM_COLORS, AXIS_COLOR, GRID_COLOR, SERIES_COLORS } from '../map3d/palette'

const FILES = ['tabs/PoiTab.tsx', 'tabs/RgbdCameraTab.tsx', 'tabs/SensorGraphTab.tsx', 'components3d/RobotModel.tsx']

describe('tabs follow the design rules', () => {
  it.each(FILES)('%s has no design violations', (path) => {
    expect(SOURCES[path], path).toBeTruthy()
    expect(designViolations(path, SOURCES[path])).toEqual([])
  })

  it('RgbdCameraTab uses three dots, not the ellipsis character', () => {
    expect(SOURCES['tabs/RgbdCameraTab.tsx']).toContain('...')
    expect(SOURCES['tabs/RgbdCameraTab.tsx']).not.toContain('…')
  })

  it('SensorGraphTab takes its colours from palette.ts', () => {
    expect(SOURCES['tabs/SensorGraphTab.tsx']).toMatch(/from '\.\.\/map3d\/palette'/)
  })

  it('palette holds achromatic graph and robot colours', () => {
    const grey = (c: string) => /^#([0-9a-f]{2})\1\1$/i.test(c)
    const greyHex = (n: number) => grey(`#${n.toString(16).padStart(6, '0')}`)
    expect([AXIS_COLOR, ...SERIES_COLORS].every(grey)).toBe(true)
    expect([ARM_COLORS.ghost, ARM_COLORS.mesh].every(greyHex)).toBe(true)
    expect(GRID_COLOR).toBeTruthy()
  })
})
