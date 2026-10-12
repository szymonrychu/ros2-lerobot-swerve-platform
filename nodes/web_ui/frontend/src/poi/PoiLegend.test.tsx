// @vitest-environment jsdom
import { cleanup, render, screen } from '@testing-library/react'
import { afterEach, describe, expect, it } from 'vitest'
import { PoiLegend } from './PoiLegend'
import { KIND_COLORS, KIND_SHAPE, zoneStyle } from './style'
import type { Poi, PoiZone } from './types'

function area(zone: PoiZone): Poi {
  return {
    id: 'a'.repeat(32), kind: 'area', frame: 'map', x: 0, y: 0, polygon: [[0, 0], [1, 0], [1, 1]], radius_m: 0,
    name: 'n', note: '', status: 'open', created_by: 'user', created_at: 0, updated_at: 0, zone,
  }
}

const swatchColor = (label: string): string | null => screen.getByTestId(`legend-swatch-${label}`).getAttribute('data-color')

const swatchShape = (label: string): string | null => screen.getByTestId(`legend-swatch-${label}`).getAttribute('data-shape')

afterEach(cleanup)

describe('PoiLegend', () => {
  it('shows a swatch per kind taken from KIND_COLORS', () => {
    render(<PoiLegend />)
    expect(swatchColor('point')).toBe(KIND_COLORS.point)
    expect(swatchColor('object')).toBe(KIND_COLORS.object)
    expect(swatchColor('photo')).toBe(KIND_COLORS.photo)
    for (const label of ['Point', 'Object', 'Photo']) expect(screen.getByText(label)).toBeTruthy()
  })

  it('shows a swatch per zone taken from zoneStyle / the neutral area colour', () => {
    render(<PoiLegend />)
    expect(swatchColor('none')).toBe(KIND_COLORS['area-none'])
    expect(swatchColor('slow')).toBe(zoneStyle(area('slow'), false)!.fill)
    expect(swatchColor('keepout')).toBe(zoneStyle(area('keepout'), false)!.fill)
    for (const label of ['Area, no zone', 'Slow zone', 'Keepout zone']) expect(screen.getByText(label)).toBeTruthy()
  })

  it('draws each kind in its KIND_SHAPE', () => {
    render(<PoiLegend />)
    expect(swatchShape('point')).toBe(KIND_SHAPE.point)
    expect(swatchShape('object')).toBe(KIND_SHAPE.object)
    expect(swatchShape('photo')).toBe(KIND_SHAPE.photo)
    for (const id of ['none', 'slow', 'keepout']) expect(swatchShape(id)).toBe(KIND_SHAPE.area)
  })
})
