// @vitest-environment jsdom
import type { ReactNode } from 'react'
import { afterEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen, within } from '@testing-library/react'
import { ThemeProvider } from '@mui/material/styles'
import { LayersPanel, type LayersPanelProps } from './LayersPanel'
import { designViolations, SOURCES } from '../layout/designRules'
import { DEFAULT_LAYERS, LAYER_LABELS } from '../map3d/layers'
import { LAYER_SWATCH, MAP_LEGEND } from '../map3d/palette'
import { GROUND_COLORS, type GroundState } from '../map3d/groundLayer'
import { theme } from '../theme'

vi.mock('../logging', () => ({ default: { warn: vi.fn(), info: vi.fn(), debug: vi.fn() } }))

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

  it('keeps the gesture help behind one info button and has no arm drag line', async () => {
    wrap(<LayersPanel {...props()} />)
    expect(screen.queryByText(/Orbit: drag to rotate/)).toBeNull()
    const header = screen.getByTestId('overlay-panel-header')
    const info = within(header).getByRole('button', { name: 'Map help' })
    fireEvent.mouseOver(info)
    const tip = (await screen.findByRole('tooltip')).textContent ?? ''
    expect(tip).toMatch(/Orbit: drag to rotate/)
    expect(tip).not.toMatch(/arm/i)
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

describe('LayersPanel head RGB-D layer', () => {
  it('is a toggle that reports clicks when the cloud topic is configured', () => {
    const p = props({ rgbdCloudConfigured: true })
    wrap(<LayersPanel {...p} />)
    const box = screen.getByLabelText('Head RGB-D') as HTMLInputElement
    expect(box.disabled).toBe(false)
    expect(box.checked).toBe(false)
    fireEvent.click(box)
    expect(p.onToggle).toHaveBeenCalledWith('headRgbd')
  })

  it('is disabled with the reason in its tooltip when the tab names no rgbd_cloud_topic', async () => {
    wrap(<LayersPanel {...props({ rgbdCloudConfigured: false, layers: { ...DEFAULT_LAYERS, headRgbd: true } })} />)
    const box = screen.getByLabelText('Head RGB-D') as HTMLInputElement
    expect(box.disabled).toBe(true)
    expect(box.checked).toBe(false)
    fireEvent.mouseOver(screen.getByTestId('layer-row-headRgbd'))
    expect((await screen.findByRole('tooltip')).textContent).toMatch(/No head RGB-D cloud topic/)
  })
})


describe('LayersPanel ground layer', () => {
  const groundState = {
    plane: { normal: [0, 0, 1], height_m: -0.031, inliers: 1, inlier_ratio: 1, source: 'fit', age_s: 0 },
    slope_deg: 6.25,
    slope_level: 'warning',
    regions: [],
    warnings: [],
  } as unknown as GroundState

  it('is a toggle, off by default, that reports clicks', () => {
    const p = props({ groundConfigured: true })
    wrap(<LayersPanel {...p} />)
    const box = screen.getByLabelText('Ground') as HTMLInputElement
    expect(box.disabled).toBe(false)
    expect(box.checked).toBe(false)
    fireEvent.click(box)
    expect(p.onToggle).toHaveBeenCalledWith('ground')
  })

  it('is disabled with the reason when the tab names no ground topics', async () => {
    wrap(<LayersPanel {...props({ groundConfigured: false, layers: { ...DEFAULT_LAYERS, ground: true } })} />)
    const box = screen.getByLabelText('Ground') as HTMLInputElement
    expect(box.disabled).toBe(true)
    expect(box.checked).toBe(false)
    fireEvent.mouseOver(screen.getByTestId('layer-row-ground'))
    expect((await screen.findByRole('tooltip')).textContent).toMatch(/No ground cloud or state topic/)
  })

  it('shows no legend while the layer is off', () => {
    wrap(<LayersPanel {...props({ groundConfigured: true })} />)
    expect(screen.queryByTestId('ground-legend')).toBeNull()
  })

  it('shows the five label colours, the edge colours, the warning fill and the read-out while on', () => {
    wrap(<LayersPanel {...props({ groundConfigured: true, layers: { ...DEFAULT_LAYERS, ground: true }, groundState })} />)
    const legend = screen.getByTestId('ground-legend')
    for (const color of [GROUND_COLORS.floor, GROUND_COLORS.obstacle, GROUND_COLORS.drop, GROUND_COLORS.mismatch, GROUND_COLORS.unlabelled, GROUND_COLORS.edgeObstacle, GROUND_COLORS.edgeWarning, GROUND_COLORS.warningFill]) {
      expect(legend.querySelector(`[data-color="${color}"]`), color).toBeTruthy()
    }
    expect(within(legend).getByText('Warning area (slow down)')).toBeTruthy()
    const readout = screen.getByTestId('ground-readout')
    expect(readout.textContent).toMatch(/6\.3 deg/)
    expect(readout.textContent).toMatch(/warning/)
    expect(readout.textContent).toMatch(/-0\.031 m/)
  })

  it('shows no read-out for an absent, stale or plane-null state', () => {
    const layers = { ...DEFAULT_LAYERS, ground: true }
    for (const gs of [undefined, { ...groundState, stale: true }, { ...groundState, plane: null }]) {
      wrap(<LayersPanel {...props({ groundConfigured: true, layers, groundState: gs as GroundState | undefined })} />)
      expect(screen.queryByTestId('ground-readout')).toBeNull()
      expect(screen.getByTestId('ground-no-estimate')).toBeTruthy()
      cleanup()
    }
  })
})
