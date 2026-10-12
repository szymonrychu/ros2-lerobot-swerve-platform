// @vitest-environment jsdom
import { afterEach, describe, expect, it } from 'vitest'
import { cleanup, render, screen } from '@testing-library/react'
import { ThemeProvider } from '@mui/material/styles'
import { MapStatusLine } from './MapStatusLine'
import { StatusItem } from '../components/StatusItem'
import { designViolations, SOURCES } from '../layout/designRules'
import { theme } from '../theme'

function wrap(node: React.ReactNode) {
  return render(<ThemeProvider theme={theme}>{node}</ThemeProvider>)
}

afterEach(cleanup)

describe('MapStatusLine', () => {
  it('renders plain text items on one line', () => {
    wrap(<MapStatusLine items={['Press on the map to set the goal', 'drag to set its heading']} />)
    const line = screen.getByTestId('map-status-line')
    expect(line.textContent).toContain('Press on the map to set the goal')
    expect(line.textContent).toContain('drag to set its heading')
    expect(getComputedStyle(line).whiteSpace).toBe('nowrap')
  })

  it('renders StatusItems', () => {
    wrap(<MapStatusLine items={[<StatusItem key="pose" label="Pose" value="1.0, 2.0" />]} />)
    expect(screen.getByText('Pose')).toBeTruthy()
    expect(screen.getByText('1.0, 2.0')).toBeTruthy()
  })

  it('is announced politely and does not take pointer events from the map', () => {
    wrap(<MapStatusLine items={['hint']} />)
    const line = screen.getByRole('status')
    expect(line.getAttribute('aria-live')).toBe('polite')
    expect(getComputedStyle(line).pointerEvents).toBe('none')
  })

  it('sits in a map corner', () => {
    wrap(<MapStatusLine items={['hint']} />)
    const style = getComputedStyle(screen.getByTestId('map-status-line'))
    expect(style.position).toBe('absolute')
    expect(style.left).not.toBe('')
    expect(style.bottom).not.toBe('')
  })

  it('renders nothing without items', () => {
    wrap(<MapStatusLine items={[]} />)
    expect(screen.queryByTestId('map-status-line')).toBeNull()
  })

  it('MapStatusLine.tsx has no design violations', () => {
    expect(designViolations('map/MapStatusLine.tsx', SOURCES['map/MapStatusLine.tsx'])).toEqual([])
  })
})
