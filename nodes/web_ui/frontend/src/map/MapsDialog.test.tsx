// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen, waitFor, within } from '@testing-library/react'
import { ThemeProvider } from '@mui/material/styles'
import { MapsDialog, SaveMapAsDialog } from './MapsDialog'
import { designViolations, SOURCES } from '../layout/designRules'
import { theme } from '../theme'
import type { MapStatus } from '../types'

const POSE = { x: 1, y: 2, yaw: 0.5 }
const STATUS: MapStatus = {
  maps: [
    { name: 'lab', saved_at: 1700000000, note: '', poi_count: 3, last_pose: POSE, valid: true, error: '' },
    { name: 'broken', saved_at: 1700000100, note: '', poi_count: 0, last_pose: POSE, valid: false, error: 'bundle checksum mismatch' },
  ],
  active_map: 'lab',
  load_seq: 2,
  localisation: { state: 'localised', ratio: 0.9, pose: POSE, message: '', updated_at: 1 },
}
const fetchMock = vi.fn()

beforeEach(() => {
  fetchMock.mockReset()
  fetchMock.mockResolvedValue({ status: 200, json: async () => ({ ok: true, message: 'done' }) })
  vi.stubGlobal('fetch', fetchMock)
})
afterEach(() => {
  cleanup()
  vi.unstubAllGlobals()
})

const wrap = (node: React.ReactNode) => render(<ThemeProvider theme={theme}>{node}</ThemeProvider>)
const lastCall = () => fetchMock.mock.calls[fetchMock.mock.calls.length - 1]
const topDialog = () => {
  const dialogs = screen.getAllByRole('dialog')
  return dialogs[dialogs.length - 1]
}
const lastBody = () => JSON.parse(lastCall()[1].body)
const lastUrl = () => lastCall()[0]

function renderDialog(status: MapStatus = STATUS, onChanged = vi.fn()) {
  wrap(<MapsDialog open tabId="map" status={status} onClose={vi.fn()} onChanged={onChanged} />)
  return onChanged
}

describe('MapsDialog list', () => {
  it('lists name, POI count and saved time, and marks an invalid map with its error and no Load button', () => {
    renderDialog()
    const lab = screen.getByText('lab').closest('li')!
    expect(lab.textContent).toContain('3 POIs')
    expect(within(lab).getByRole('button', { name: 'Load lab' })).toBeTruthy()
    const broken = screen.getByText('broken').closest('li')!
    expect(broken.textContent).toContain('bundle checksum mismatch')
    expect(within(broken).queryByRole('button', { name: /Load/ })).toBeNull()
    expect(within(broken).getByRole('button', { name: 'Delete broken' })).toBeTruthy()
  })

  it('says so when there are no saved maps', () => {
    renderDialog({ ...STATUS, maps: [] })
    expect(screen.getByText('No saved maps')).toBeTruthy()
  })
})

describe('MapsDialog load and delete', () => {
  it('load asks first, names the replacement and the autosave, then posts the name', async () => {
    const onChanged = renderDialog()
    fireEvent.click(screen.getByRole('button', { name: 'Load lab' }))
    expect(fetchMock).not.toHaveBeenCalled()
    const confirm = topDialog()
    expect(confirm.textContent).toContain('current map and POIs are replaced')
    expect(confirm.textContent).toContain('autosave')
    fireEvent.click(within(confirm).getByRole('button', { name: 'Load' }))
    await waitFor(() => expect(fetchMock).toHaveBeenCalled())
    expect(lastUrl()).toBe('/api/maps/load?tab=map')
    expect(lastBody()).toEqual({ name: 'lab' })
    await screen.findByText('done')
    expect(onChanged).toHaveBeenCalled()
  })

  it('delete names the map and its POI count and posts only after confirming', async () => {
    renderDialog()
    fireEvent.click(screen.getByRole('button', { name: 'Delete lab' }))
    const confirm = topDialog()
    expect(confirm.textContent).toContain('Delete lab and its 3 POIs?')
    expect(fetchMock).not.toHaveBeenCalled()
    fireEvent.click(within(confirm).getByRole('button', { name: 'Delete' }))
    await waitFor(() => expect(fetchMock).toHaveBeenCalled())
    expect(lastUrl()).toBe('/api/maps/delete?tab=map')
    expect(lastBody()).toEqual({ name: 'lab' })
  })

  it('a refused request shows the message and the dialog stays usable', async () => {
    fetchMock.mockResolvedValue({ status: 400, json: async () => ({ ok: false, message: 'map is busy' }) })
    renderDialog()
    fireEvent.click(screen.getByRole('button', { name: 'Delete broken' }))
    fireEvent.click(within(topDialog()).getByRole('button', { name: 'Delete' }))
    await screen.findByText('map is busy')
    await waitFor(() => expect(screen.getByRole('button', { name: 'Load lab' }).hasAttribute('disabled')).toBe(false))
  })

  it('disables every action while a request runs', async () => {
    fetchMock.mockReturnValue(new Promise(() => undefined))
    renderDialog()
    fireEvent.click(screen.getByRole('button', { name: 'Delete lab' }))
    fireEvent.click(within(topDialog()).getByRole('button', { name: 'Delete' }))
    await waitFor(() => expect(screen.getByRole('button', { name: 'Load lab' }).hasAttribute('disabled')).toBe(true))
    expect(screen.getByRole('progressbar')).toBeTruthy()
  })
})

describe('MapsDialog retry at saved pose', () => {
  it.each(['not_localised', 'ambiguous'] as const)('offers retry for the active map in %s and posts the saved pose', async (state) => {
    renderDialog({ ...STATUS, localisation: { ...STATUS.localisation, state, message: 'scores disagree' } })
    fireEvent.click(screen.getByRole('button', { name: 'Retry at saved pose' }))
    await waitFor(() => expect(fetchMock).toHaveBeenCalled())
    expect(lastUrl()).toBe('/api/maps/load?tab=map')
    expect(lastBody()).toEqual({ name: 'lab', pose: POSE })
  })

  it('does not offer retry when localised', () => {
    renderDialog()
    expect(screen.queryByRole('button', { name: 'Retry at saved pose' })).toBeNull()
  })

  it('after a discarded map the saved maps are still listed and usable', () => {
    renderDialog({ ...STATUS, active_map: null, localisation: { ...STATUS.localisation, state: 'not_localised', message: 'map discarded' } })
    expect(screen.queryByRole('button', { name: 'Retry at saved pose' })).toBeNull()
    expect(screen.getByRole('button', { name: 'Load lab' }).hasAttribute('disabled')).toBe(false)
  })
})

describe('SaveMapAsDialog', () => {
  function renderSave(existing: string[] = ['lab']) {
    const onSaved = vi.fn()
    wrap(<SaveMapAsDialog open tabId="map" existing={existing} onClose={vi.fn()} onSaved={onSaved} />)
    return onSaved
  }
  const type = (label: string, value: string) => fireEvent.change(screen.getByLabelText(label), { target: { value } })

  it('rejects an invalid name with the rule and posts nothing', () => {
    renderSave()
    type('Map name', '-bad name')
    fireEvent.click(screen.getByRole('button', { name: 'Save' }))
    expect(screen.getByText(/letters, digits/)).toBeTruthy()
    expect(fetchMock).not.toHaveBeenCalled()
  })

  it('saves a new name with its note and no overwrite', async () => {
    const onSaved = renderSave()
    type('Map name', 'garage')
    type('Note (optional)', 'second floor')
    fireEvent.click(screen.getByRole('button', { name: 'Save' }))
    await waitFor(() => expect(fetchMock).toHaveBeenCalled())
    expect(lastUrl()).toBe('/api/maps/save?tab=map')
    expect(lastBody()).toEqual({ name: 'garage', note: 'second floor', overwrite: false })
    await waitFor(() => expect(onSaved).toHaveBeenCalled())
  })

  it('an existing name asks Overwrite first and resends with overwrite true', async () => {
    renderSave()
    type('Map name', 'lab')
    fireEvent.click(screen.getByRole('button', { name: 'Save' }))
    expect(fetchMock).not.toHaveBeenCalled()
    const confirm = topDialog()
    expect(confirm.textContent).toContain('Overwrite lab?')
    fireEvent.click(within(confirm).getByRole('button', { name: 'Overwrite' }))
    await waitFor(() => expect(fetchMock).toHaveBeenCalled())
    expect(lastBody()).toEqual({ name: 'lab', note: '', overwrite: true })
  })
})

describe('design rules', () => {
  it('map files have no design violations', () => {
    for (const f of ['map/MapsDialog.tsx', 'map/mapsApi.ts', 'map/MapStatusLine.tsx']) {
      expect(designViolations(f, SOURCES[f])).toEqual([])
    }
  })
})
