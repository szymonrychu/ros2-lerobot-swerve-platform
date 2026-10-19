// @vitest-environment jsdom
import { afterEach, describe, expect, it } from 'vitest'
import { cleanup, render, screen } from '@testing-library/react'
import { ThemeProvider } from '@mui/material/styles'
import { Map3dStatus, map3dSummary } from './Map3dStatus'
import { theme } from '../theme'

const BASE = {
  stamp_s: 100, state: 'mapping', nodes: 120, db_size_mb: 42.5, memory_mb: 610.2,
  graph_correction_m: 0.02, graph_correction_deg: 0.5, correction_warn_m: 0.1, correction_warn_deg: 2,
  correction_ok: true, missing_inputs: [], last_update_age_s: 1.4, last_save: null, message: '',
}

function show(over: Record<string, unknown> | null | undefined) {
  const status = over === null || over === undefined ? over : { ...BASE, ...over }
  return render(
    <ThemeProvider theme={theme}>
      <Map3dStatus status={status} />
    </ThemeProvider>,
  )
}

afterEach(cleanup)

describe('Map3dStatus', () => {
  it('shows no-status text, never zeros, before a status arrived', () => {
    show(undefined)
    expect(screen.getByText('no 3D map status yet')).toBeTruthy()
    expect(screen.queryByText(/MB/)).toBeNull()
    expect(map3dSummary(undefined)).toBe('no 3D map status yet')
  })

  it('shows the mapping state with node count, sizes and age', () => {
    show({})
    expect(screen.getByTestId('map3d-state').textContent).toBe('mapping')
    expect(screen.getByText(/120 nodes/)).toBeTruthy()
    expect(screen.getByText(/42\.5 MB/)).toBeTruthy()
    expect(screen.getByText(/610 MB/)).toBeTruthy()
    expect(screen.getByText(/1\.4 s ago/)).toBeTruthy()
  })

  it.each(['paused', 'error'])('shows the %s state', (state) => {
    show({ state })
    expect(screen.getByTestId('map3d-state').textContent).toBe(state)
  })

  it('lists the missing inputs when waiting', () => {
    show({ state: 'waiting_for_inputs', nodes: null, missing_inputs: ['rgb_camera_info', 'rtabmap_info'] })
    expect(screen.getByTestId('map3d-state').textContent).toBe('waiting for inputs')
    expect(screen.getByText(/rgb_camera_info, rtabmap_info/)).toBeTruthy()
  })

  it('does not invent values for unknown (null) fields', () => {
    show({ nodes: null, db_size_mb: null, memory_mb: null, graph_correction_m: null, graph_correction_deg: null, last_update_age_s: null })
    expect(screen.queryByText(/nodes/)).toBeNull()
    expect(screen.queryByText(/MB/)).toBeNull()
    expect(screen.queryByTestId('map3d-correction')).toBeNull()
  })

  it('keeps the correction neutral within the thresholds', () => {
    show({})
    const el = screen.getByTestId('map3d-correction')
    expect(el.textContent).toMatch(/0\.02 m/)
    expect(el.textContent).toMatch(/0\.5 deg/)
    expect(el.getAttribute('data-warn')).toBe('false')
  })

  it('warns when the translation is over its threshold', () => {
    show({ graph_correction_m: 0.5 })
    const el = screen.getByTestId('map3d-correction')
    expect(el.getAttribute('data-warn')).toBe('true')
    expect(getComputedStyle(el).color).toBe(getComputedStyle(render(<span style={{ color: theme.palette.warning.main }} />).container.firstChild as Element).color)
  })

  it('warns when the yaw is over its threshold', () => {
    show({ graph_correction_deg: 5 })
    expect(screen.getByTestId('map3d-correction').getAttribute('data-warn')).toBe('true')
  })

  it('summarises on one line', () => {
    expect(map3dSummary({ ...BASE })).toBe('mapping, 120 nodes')
    expect(map3dSummary({ ...BASE, state: 'waiting_for_inputs', nodes: null })).toBe('waiting for inputs')
    expect(map3dSummary({ ...BASE, graph_correction_m: 0.5 })).toBe('mapping, 120 nodes, drifted')
  })
})
