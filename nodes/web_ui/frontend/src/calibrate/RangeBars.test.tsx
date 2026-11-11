// @vitest-environment jsdom
import { afterEach, describe, expect, it } from 'vitest'
import { cleanup, render, screen } from '@testing-library/react'
import { RangeBars, barPercent } from './RangeBars'

afterEach(cleanup)

describe('RangeBars', () => {
  it('range bars show min max and present', () => {
    render(<RangeBars joints={{ elbow: { present_steps: 2048, min_steps: 1024, max_steps: 3072, range_deg: 180 } }} />)
    expect(screen.getByText(/min 1024 max 3072 present 2048 range 180 deg/)).toBeTruthy()
    expect(screen.getByTestId('span-elbow').style.left).toBe(`${(1024 / 4095) * 100}%`)
    expect(screen.getByTestId('present-elbow')).toBeTruthy()
  })

  it('clamps extremes outside 0..4095 but keeps the true values in the text', () => {
    render(<RangeBars joints={{ steer: { present_steps: 10, min_steps: -300, max_steps: 4500, range_deg: 421.9 } }} />)
    expect(barPercent(-300)).toBe(0)
    expect(barPercent(4500)).toBe(100)
    expect(screen.getByTestId('span-steer').style.width).toBe('100%')
    expect(screen.getByText(/min -300 max 4500/)).toBeTruthy()
  })

  it('draws nothing and shows "-" for null fields', () => {
    render(<RangeBars joints={{ head: { present_steps: null, min_steps: null, max_steps: null, range_deg: null } }} />)
    expect(screen.queryByTestId('span-head')).toBeNull()
    expect(screen.queryByTestId('present-head')).toBeNull()
    expect(screen.getByText('min - max - present - range -')).toBeTruthy()
  })
})
