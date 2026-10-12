import { describe, expect, it } from 'vitest'
import { designViolations, SOURCES } from '../layout/designRules'
import { cameraColor } from '../photos/geometry'
import { KIND_COLORS, ZONE_COLORS, STATUS_COLORS, OBJECT_COLOR, markerColors } from '../poi/style'
import { TOKENS } from '../theme'
import { ARM_COLORS, CREATOR_OUTLINE, LAYER_SWATCH, MAP_LEGEND, NAV_PATH_COLOR, PHOTO_COLOR, SCENE_COLORS, SELECTED_OUTLINE } from './palette'

const SCENE_KEY: Record<string, keyof typeof SCENE_COLORS> = {
  localCostmap: 'costmap',
  globalPlan: 'globalPath',
  localPlan: 'localPath',
  goal: 'goal',
  rtkBase: 'rtkBase',
  footprint: 'robot',
}

function achromatic(color: string): boolean {
  const m = /^#([0-9a-f]{2})([0-9a-f]{2})([0-9a-f]{2})$/i.exec(color)
  return !!m && m[1] === m[2] && m[2] === m[3]
}
const lightness = (c: string): number => parseInt(c.slice(1, 3), 16)

describe('map palette', () => {
  it('swatches equal the scene and marker colours', () => {
    for (const [layer, key] of Object.entries(SCENE_KEY)) expect(LAYER_SWATCH[layer as keyof typeof LAYER_SWATCH]).toBe(SCENE_COLORS[key])
    expect(LAYER_SWATCH.photos).toBe(PHOTO_COLOR)
    expect(KIND_COLORS.photo).toBe(PHOTO_COLOR)
    expect(LAYER_SWATCH.pois).toBe(KIND_COLORS.point)
  })

  it('zones take the error and warning tokens, selection the accent', () => {
    expect(ZONE_COLORS.keepout).toBe(TOKENS.error)
    expect(ZONE_COLORS.slow).toBe(TOKENS.warning)
    expect(SELECTED_OUTLINE).toBe(TOKENS.accent)
  })

  it('only the zones and the nav path have a hue', () => {
    const grey = [
      ...Object.entries(SCENE_COLORS).filter(([k]) => k !== 'globalPath' && k !== 'localPath').map(([, v]) => v),
      ...Object.values(KIND_COLORS),
      ...Object.values(STATUS_COLORS),
      ...Object.values(CREATOR_OUTLINE),
      ...Object.values(LAYER_SWATCH).filter((c) => c !== NAV_PATH_COLOR),
      ...MAP_LEGEND.map(([, c]) => c),
      OBJECT_COLOR,
      PHOTO_COLOR,
    ]
    for (const c of grey) expect(achromatic(c as string), c as string).toBe(true)
    expect(SCENE_COLORS.globalPath).toBe(NAV_PATH_COLOR)
    expect(achromatic(NAV_PATH_COLOR)).toBe(false)
  })

  it('POI status differs on markers by a lightness step, not hue', () => {
    const [open, done, cancelled] = [STATUS_COLORS.open, STATUS_COLORS.done, STATUS_COLORS.cancelled].map(lightness)
    expect(open).toBeGreaterThan(done)
    expect(done).toBeGreaterThan(cancelled)
    expect(markerColors({ kind: 'point', status: 'open' } as never).outline).not.toBe(markerColors({ kind: 'point', status: 'done' } as never).outline)
  })

  it('one photo colour for every camera', () => {
    expect(cameraColor('head')).toBe(PHOTO_COLOR)
    expect(cameraColor('gripper')).toBe(PHOTO_COLOR)
  })

  it('sceneLayers imports the scene colours instead of copying them', () => {
    expect(SOURCES['map3d/sceneLayers.tsx']).toMatch(/import \{ SCENE_COLORS \} from '\.\/palette'/)
  })

  it('colour files pass the design rules', () => {
    for (const p of ['map3d/palette.ts', 'map3d/sceneLayers.tsx', 'poi/style.ts', 'photos/geometry.ts']) {
      expect(designViolations(p, SOURCES[p]), p).toEqual([])
    }
  })

  it('arm scene colours come from the palette and are greyscale', () => {
    const grey = (hex: number): boolean => ((hex >> 16) & 255) === ((hex >> 8) & 255) && ((hex >> 8) & 255) === (hex & 255)
    const values = [...Object.values(ARM_COLORS.handle), ...Object.values(ARM_COLORS.endEffector), ...Object.values(ARM_COLORS.axis), ARM_COLORS.ghost, ARM_COLORS.mesh]
    for (const v of values) expect(grey(v), v.toString(16)).toBe(true)
    for (const p of ['components3d/InteractiveArm.tsx', 'components3d/RobotModel.tsx']) expect(SOURCES[p], p).toContain("from '../map3d/palette'")
    for (const p of ['components3d/InteractiveArm.tsx', 'components3d/RobotModel.tsx', 'arm/ArmView.tsx']) expect(designViolations(p, SOURCES[p]), p).toEqual([])
  })
})
