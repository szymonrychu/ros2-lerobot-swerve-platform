// @vitest-environment jsdom
import { render, screen } from '@testing-library/react'
import { describe, expect, it } from 'vitest'
import { designViolations, SOURCES } from '../layout/designRules'
import { SANS_FONT, TOKENS } from '../theme'
import { MapLabel } from './MapLabel'
import { ZONE_COLORS } from './palette'

/** Computed-style view of the label element with the given text. */
const labelStyle = (text: string): CSSStyleDeclaration => screen.getByText(text).style

/** A colour as jsdom serialises it (hex becomes rgb()). */
const css = (colour: string): string => {
  const probe = document.createElement('div')
  probe.style.color = colour
  return probe.style.color
}

describe('MapLabel', () => {
  it('is one calm instrument style', () => {
    render(<MapLabel>Dock</MapLabel>)
    const s = labelStyle('Dock')
    expect(s.fontFamily).toBe(SANS_FONT)
    expect(s.fontSize).toBe('11px')
    expect(s.fontWeight).toBe('500')
    expect(s.borderRadius).toBe('2px')
    expect(s.background).toBe(css(TOKENS.bgPanel))
    expect(s.color).toBe(css(TOKENS.textPrimary))
    expect(s.border).toBe(`1px solid ${css(TOKENS.border)}`)
  })

  it('colours only the border of a zone label', () => {
    render(
      <>
        <MapLabel zone="keepout">K</MapLabel>
        <MapLabel zone="slow">S</MapLabel>
      </>,
    )
    expect(labelStyle('K').border).toBe(`1px solid ${css(ZONE_COLORS.keepout)}`)
    expect(labelStyle('S').border).toBe(`1px solid ${css(ZONE_COLORS.slow)}`)
    expect(labelStyle('K').color).toBe(css(TOKENS.textPrimary))
    expect(labelStyle('K').background).toBe(css(TOKENS.bgPanel))
  })

  it('leaves the label files clean under the design rules', () => {
    for (const path of ['map3d/MapLabel.tsx', 'map3d/sceneLayers.tsx', 'grasp/GraspPreview.tsx', 'photos/PhotoLayer.tsx']) {
      expect(SOURCES[path], path).toBeTypeOf('string')
      expect(designViolations(path, SOURCES[path]), path).toEqual([])
    }
  })

  it('has no inline label styles left in the layers', () => {
    for (const path of ['map3d/sceneLayers.tsx', 'poi/PoiLayer.tsx', 'grasp/GraspPreview.tsx']) {
      expect(SOURCES[path], path).toContain('<MapLabel')
      expect(SOURCES[path], path).not.toMatch(/font: '\d+ \d+px sans-serif'/)
    }
  })
})
