// @vitest-environment jsdom
import { afterEach, describe, expect, it } from 'vitest'
import { cleanup, render, screen } from '@testing-library/react'
import { ThemeProvider } from '@mui/material/styles'
import { MapStatusLine, mapStatusText } from './MapStatusLine'
import type { MapStatus } from '../types'
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

const STATUS: MapStatus = {
  maps: [],
  active_map: 'lab',
  load_seq: 1,
  localisation: { state: 'localised', ratio: 0.9, pose: null, message: '', updated_at: 1 },
}
const withLoc = (state: MapStatus['localisation']['state'], message = '', active: string | null = 'lab'): MapStatus => ({
  ...STATUS,
  active_map: active,
  localisation: { ...STATUS.localisation, state, message },
})

describe('mapStatusText', () => {
  it('shows nothing before a status arrives', () => {
    expect(mapStatusText(undefined)).toBeNull()
  })

  it('says localised on the active map', () => {
    expect(mapStatusText(STATUS)).toBe('Localised on lab')
  })

  it('says not localised with the reason, also when ambiguous', () => {
    expect(mapStatusText(withLoc('not_localised', 'scan does not match'))).toBe('Not localised on lab: scan does not match')
    expect(mapStatusText(withLoc('ambiguous', 'two candidates'))).toBe('Not localised on lab: two candidates')
  })

  it('explains a discarded map', () => {
    expect(mapStatusText(withLoc('not_localised', 'map discarded', null))).toBe(
      'Map cleared after the robot was moved; load a saved map',
    )
  })

  it('shows loading and the bare active map for unknown', () => {
    expect(mapStatusText(withLoc('loading'))).toBe('Loading lab')
    expect(mapStatusText(withLoc('unknown'))).toBe('Map lab')
    expect(mapStatusText(withLoc('unknown', '', null))).toBeNull()
  })
})
