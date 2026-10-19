// @vitest-environment jsdom
import type { ReactNode } from 'react'
import { afterEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen, within } from '@testing-library/react'
import { ThemeProvider } from '@mui/material/styles'
import { LayersPanel, type LayersPanelProps } from './LayersPanel'
import { designViolations, SOURCES } from '../layout/designRules'
import { DEFAULT_LAYERS, LAYER_LABELS } from '../map3d/layers'
import { LAYER_SWATCH, MAP_LEGEND } from '../map3d/palette'
import { theme } from '../theme'

function props(over: Partial<LayersPanelProps> = {}): LayersPanelProps {
  return {
    open: true,
    onClose: vi.fn(),
    layers: { ...DEFAULT_LAYERS },
    onToggle: vi.fn(),
    gpsAvailable: true,
    rtkAvailable: true,
    costmapConfigured: true,
    tileSources: [],
    tileSource: null,
    onTileSource: vi.fn(),
    tileCaption: null,
    anchorSummary: null,
    photoAge: '7d',
    onPhotoAge: vi.fn(),
    photoStatus: { shown: 3, total: 3, error: null },
    topView: false,
    goalMode: false,
    hasArm: false,
    armReady: false,
    ...over,
  }
}

function wrap(node: ReactNode) {
  return render(<ThemeProvider theme={theme}>{node}</ThemeProvider>)
}

afterEach(cleanup)

describe('LayersPanel 3D status', () => {
  it('shows the status summary next to the 3D map row', () => {
    wrap(<LayersPanel {...props({ map3dStatus: { state: 'mapping', nodes: 120, graph_correction_m: 0.01, graph_correction_deg: 0, correction_warn_m: 0.1, correction_warn_deg: 2 } })} />)
    expect(within(screen.getByTestId('layer-row-map3dCloud')).getByText('mapping, 120 nodes')).toBeTruthy()
  })

  it('says there is no status yet when none arrived', () => {
    wrap(<LayersPanel {...props({ map3dStatus: undefined })} />)
    expect(within(screen.getByTestId('layer-row-map3dCloud')).getByText('no 3D map status yet')).toBeTruthy()
  })

  it('shows no summary on other rows', () => {
    wrap(<LayersPanel {...props({ map3dStatus: { state: 'paused' } })} />)
    expect(within(screen.getByTestId('layer-row-goal')).queryByText(/paused/)).toBeNull()
  })
})

describe('LayersPanel', () => {
  it('lists every layer as a toggle and reports toggles', () => {
    const p = props()
    wrap(<LayersPanel {...p} />)
    for (const label of Object.values(LAYER_LABELS)) expect(screen.getByLabelText(label)).toBeTruthy()
    fireEvent.click(screen.getByLabelText(LAYER_LABELS.goal))
    expect(p.onToggle).toHaveBeenCalledWith('goal')
    expect((screen.getByLabelText(LAYER_LABELS.gpsMap) as HTMLInputElement).checked).toBe(false)
  })

  it('shows the on state with the accent, not a primary fill', () => {
    wrap(<LayersPanel {...props()} />)
    const box = screen.getByLabelText(LAYER_LABELS.goal).closest('.MuiCheckbox-root') as HTMLElement
    expect(box.className).not.toMatch(/colorPrimary/)
  })

  it('puts a disabled layer reason in its tooltip, not in a caption', async () => {
    wrap(<LayersPanel {...props({ gpsAvailable: false, rtkAvailable: false, costmapConfigured: false })} />)
    expect(screen.queryByText(/Needs a GPS fix/)).toBeNull()
    expect(screen.queryByText(/Needs the GPS anchor/)).toBeNull()
    expect(screen.queryByText(/No costmap topic/)).toBeNull()
    expect((screen.getByLabelText(LAYER_LABELS.gpsMap) as HTMLInputElement).disabled).toBe(true)
    fireEvent.mouseOver(screen.getByTestId('layer-row-gpsMap'))
    expect((await screen.findByRole('tooltip')).textContent).toMatch(/Needs a GPS fix/)
  })

  it('keeps the gesture and arm help behind one info button', async () => {
    wrap(<LayersPanel {...props({ hasArm: true, armReady: false })} />)
    expect(screen.queryByText(/Orbit: drag to rotate/)).toBeNull()
    expect(screen.queryByText(/Waiting for arm servo positions/)).toBeNull()
    const header = screen.getByTestId('overlay-panel-header')
    const info = within(header).getByRole('button', { name: 'Map help' })
    fireEvent.mouseOver(info)
    const tip = (await screen.findByRole('tooltip')).textContent ?? ''
    expect(tip).toMatch(/Orbit: drag to rotate/)
    expect(tip).toMatch(/Waiting for arm servo positions/)
  })

  it('draws swatches from the palette and no robot pose line', () => {
    wrap(<LayersPanel {...props()} />)
    expect(screen.getByTestId('layer-swatch-goal').getAttribute('data-color')).toBe(LAYER_SWATCH.goal)
    for (const [label] of MAP_LEGEND) expect(screen.getByText(label)).toBeTruthy()
    expect(screen.getByLabelText('Marker legend')).toBeTruthy()
    expect(screen.queryByText(/^Robot -?\d/)).toBeNull()
    expect(screen.queryByText(/pose \(robot at origin\)/)).toBeNull()
  })

  it('renders nothing when closed', () => {
    wrap(<LayersPanel {...props({ open: false })} />)
    expect(screen.queryByTestId('overlay-panel')).toBeNull()
  })

  it('has no design violations, nor does the POI legend', () => {
    for (const path of ['map/LayersPanel.tsx', 'poi/PoiLegend.tsx']) {
      expect(SOURCES[path], path).toBeTruthy()
      expect(designViolations(path, SOURCES[path])).toEqual([])
    }
    expect(SOURCES['map/LayersPanel.tsx']).not.toMatch(/mt:\s*-0\.5|ml:\s*4\b/)
    expect(SOURCES['poi/PoiLegend.tsx']).not.toMatch(/ml:\s*4\b/)
  })
})
