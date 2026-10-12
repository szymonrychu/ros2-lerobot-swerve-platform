// @vitest-environment jsdom
import { afterEach, describe, expect, it } from 'vitest'
import { cleanup, render, screen } from '@testing-library/react'
import { ThemeProvider } from '@mui/material/styles'
import { StatusItem } from './StatusItem'
import { BatteryChip } from './BatteryChip'
import { GpsChip } from './GpsChip'
import { OverlayBar } from '../overlays/OverlayBar'
import { TimeBudgetChip } from '../agent/TimeBudgetChip'
import { TOKENS, theme } from '../theme'
import { designViolations, SOURCES } from '../layout/designRules'
import { emptyChat } from '../agent/agentModel'

function wrap(node: React.ReactNode) {
  return render(<ThemeProvider theme={theme}>{node}</ThemeProvider>)
}

afterEach(cleanup)

/** Desktop viewport: every min-width query matches, so the bar shows its values. */
function desktopViewport(): void {
  window.matchMedia = ((query: string) => ({
    matches: true,
    media: query,
    addEventListener: () => undefined,
    removeEventListener: () => undefined,
    addListener: () => undefined,
    removeListener: () => undefined,
  })) as unknown as typeof window.matchMedia
}

describe('StatusItem', () => {
  it('renders label, value and unit', () => {
    wrap(<StatusItem label="Speed" value="1.2" unit="m/s" />)
    expect(screen.getByText('Speed')).toBeTruthy()
    expect(screen.getByText('1.2')).toBeTruthy()
    expect(screen.getByText('m/s')).toBeTruthy()
  })

  it('warn and error colour the value and add a state mark', () => {
    wrap(<StatusItem label="A" value="warnv" state="warn" />)
    wrap(<StatusItem label="B" value="errv" state="error" />)
    expect(getComputedStyle(screen.getByText('warnv')).color).toBe('rgb(226, 163, 54)')
    expect(getComputedStyle(screen.getByText('errv')).color).toBe('rgb(229, 72, 77)')
    expect(screen.getAllByTestId('status-mark')).toHaveLength(2)
    expect(TOKENS.warning).toBe('#e2a336')
  })

  it('ok and unknown have no mark and no state colour', () => {
    wrap(<StatusItem label="A" value="okv" state="ok" />)
    wrap(<StatusItem label="B" value="unkv" state="unknown" />)
    wrap(<StatusItem label="C" value="defv" />)
    expect(screen.queryAllByTestId('status-mark')).toHaveLength(0)
    expect(getComputedStyle(screen.getByText('okv')).color).toBe('rgb(214, 217, 222)')
  })

  it('shows the tooltip text as aria-label', () => {
    wrap(<StatusItem label="A" value="v" tooltip="details here" />)
    expect(screen.getByLabelText('details here')).toBeTruthy()
  })
})

describe('status components', () => {
  it('BatteryChip has no Chip and no icon, low level marks warn', () => {
    const { container } = wrap(
      <BatteryChip
        status={{
          level: 'low',
          color: 'warning',
          label: '10.5 V',
          perCellLabel: '3.5 V/cell',
          cellVoltage: 3.5,
        }}
      />,
    )
    expect(container.querySelector('.MuiChip-root')).toBeNull()
    expect(container.querySelector('svg')).toBeNull()
    expect(screen.getByText('10.5 V')).toBeTruthy()
    expect(screen.getAllByTestId('status-mark')).toHaveLength(1)
  })

  it('GpsChip quality 1 (info) is not marked', () => {
    wrap(
      <GpsChip
        status={{
          level: 'gps',
          color: 'info',
          label: 'Robot: GPS',
          shortLabel: 'GPS',
          tooltip: 't',
        }}
        role="robot"
      />,
    )
    expect(screen.queryAllByTestId('status-mark')).toHaveLength(0)
  })

  it('TimeBudgetChip renders nothing without a budget', () => {
    const { container } = wrap(<TimeBudgetChip state={emptyChat()} />)
    expect(container.innerHTML).toBe('')
  })

  it('OverlayBar shows Live or Connecting and -- for empty values', () => {
    desktopViewport()
    const overlays = [{ topic: '/t', field: 'x', label: 'X', unit: 'm' }]
    wrap(<OverlayBar overlays={overlays} topicData={{}} connected={false} />)
    expect(screen.getByText('Connecting')).toBeTruthy()
    expect(screen.getByText('--')).toBeTruthy()
    cleanup()
    wrap(<OverlayBar overlays={overlays} topicData={{}} connected />)
    expect(screen.getByText('Live')).toBeTruthy()
  })
})

describe('design rules', () => {
  it.each([
    'components/StatusItem.tsx',
    'overlays/OverlayBar.tsx',
    'components/BatteryChip.tsx',
    'components/GpsChip.tsx',
    'agent/TimeBudgetChip.tsx',
  ])('%s has no violations', (path) => {
    expect(SOURCES[path]).toBeTruthy()
    expect(designViolations(path, SOURCES[path])).toEqual([])
  })
})
