// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react'
import type { Photo } from './api'
import { PhotoPanel } from './PhotoPanel'
import type { Poi } from '../poi/types'

function photo(over: Partial<Photo> = {}): Photo {
  return {
    id: 'ph1',
    ts: 1_700_000_000,
    camera: 'gripper',
    tool: 'get_camera_image',
    name: 'Desk',
    note: 'first note',
    poiIds: ['p1'],
    x: 1,
    y: 2,
    view: { x: 1, y: 2, z: 0.3, yaw: Math.PI / 2, pitch: 0.5, hfov: 1, source: 'camera_mount' },
    bytes: 10,
    width: 640,
    height: 480,
    ...over,
  }
}

function poi(id: string, kind: Poi['kind'], name: string): Poi {
  return { id, kind, frame: 'map', x: 0, y: 0, polygon: [], radius_m: 0.3, name, note: '', status: 'open', created_by: 'user', created_at: 0, updated_at: 0 }
}

const POIS = [poi('p1', 'point', 'Mug'), poi('p2', 'area', 'Kitchen'), poi('x9', 'photo', 'Stray photo')]

function makeFetch() {
  const posted: Record<string, unknown>[] = []
  const impl = vi.fn(async (_url: string, init?: RequestInit) => {
    posted.push(JSON.parse(String(init?.body)) as Record<string, unknown>)
    return { ok: true, status: 200, json: async () => ({ ok: true, message: 'ok', poi: null }) } as Response
  })
  return { impl: impl as unknown as typeof fetch, posted }
}

function ui(f: ReturnType<typeof makeFetch>, p: Photo = photo(), extra: Partial<Parameters<typeof PhotoPanel>[0]> = {}) {
  return <PhotoPanel tabId="t" photo={p} pois={POIS} onClose={() => {}} onDeleted={() => {}} fetchImpl={f.impl} {...extra} />
}

let viewportWidth = 1400

/** Minimal matchMedia: evaluates min-width / max-width px conditions against viewportWidth. */
function installMatchMedia(): void {
  window.matchMedia = ((query: string) => {
    const min = /min-width:\s*([\d.]+)px/.exec(query)
    const max = /max-width:\s*([\d.]+)px/.exec(query)
    let matches = true
    if (min && viewportWidth < Number(min[1])) matches = false
    if (max && viewportWidth > Number(max[1])) matches = false
    return { matches, media: query, addEventListener: vi.fn(), removeEventListener: vi.fn(), addListener: vi.fn(), removeListener: vi.fn(), dispatchEvent: vi.fn(), onchange: null }
  }) as typeof window.matchMedia
}

beforeEach(() => {
  viewportWidth = 1400
  installMatchMedia()
})
afterEach(cleanup)

describe.each([
  [400, 'phone'],
  [1400, 'desktop'],
])('PhotoPanel layout at %i px (%s)', (width, cls) => {
  it('renders inside OverlayPanel, closes from its header, uses relative sizes', () => {
    viewportWidth = width
    const onClose = vi.fn()
    render(ui(makeFetch(), photo(), { onClose }))
    const overlay = screen.getByTestId('overlay-panel')
    expect(overlay.getAttribute('data-layout')).toBe(cls)
    expect(overlay.contains(screen.getByTestId('photo-panel'))).toBe(true)
    expect(screen.queryByTestId('overlay-panel-handle')).toBeNull()
    fireEvent.click(screen.getByRole('button', { name: 'Close' }))
    expect(onClose).toHaveBeenCalledTimes(1)
    const img = screen.getByRole('img')
    expect(getComputedStyle(img).maxWidth).toBe('100%')
    const group = screen.getByRole('group', { name: 'Linked POIs' })
    expect(group.className).not.toMatch(/130/)
    expect(getComputedStyle(group).maxHeight).toMatch(/rem|dvh/)
  })
})

describe('PhotoPanel', () => {
  it('shows the image at max_px 640 linking to the full size, and the capture facts', () => {
    render(ui(makeFetch()))
    const img = screen.getByRole('img') as HTMLImageElement
    expect(img.getAttribute('src')).toBe('/api/photos/ph1.jpg?max_px=640')
    expect(img.closest('a')?.getAttribute('href')).toBe('/api/photos/ph1.jpg')
    expect(screen.getByText(/2023-11-14T22:13:20Z/)).toBeTruthy()
    expect(screen.getByText(/gripper/)).toBeTruthy()
    expect(screen.getByText(/get_camera_image/)).toBeTruthy()
    expect(screen.getByText(/yaw 90 deg/i)).toBeTruthy()
    expect(screen.getByText(/pitch 29 deg/i)).toBeTruthy()
    expect(screen.getByText(/FOV 57 deg/i)).toBeTruthy()
  })

  it('says unknown for a missing FOV and pitch', () => {
    const p = photo({ view: { x: 1, y: 2, z: null, yaw: 0, pitch: null, hfov: null, source: 'base_heading' } })
    render(ui(makeFetch(), p))
    expect(screen.getByText(/FOV unknown/i)).toBeTruthy()
    expect(screen.getByText(/pitch unknown/i)).toBeTruthy()
  })

  it('saves name and note with an update', async () => {
    const f = makeFetch()
    render(ui(f))
    fireEvent.change(screen.getByLabelText('Name'), { target: { value: 'Desk 2' } })
    fireEvent.change(screen.getByLabelText('Note'), { target: { value: 'changed' } })
    fireEvent.click(screen.getByRole('button', { name: 'Save' }))
    await waitFor(() => expect(f.posted).toHaveLength(1))
    expect(f.posted[0]).toEqual({ op: 'update', poi: { id: 'ph1', name: 'Desk 2', note: 'changed', poi_ids: ['p1'] } })
  })

  it('offers only non-photo POIs for links and saves the chosen ids', async () => {
    const f = makeFetch()
    render(ui(f))
    expect(screen.getByLabelText('Mug')).toBeTruthy()
    expect(screen.getByLabelText('Kitchen')).toBeTruthy()
    expect(screen.queryByLabelText('Stray photo')).toBeNull()
    fireEvent.click(screen.getByLabelText('Kitchen'))
    fireEvent.click(screen.getByRole('button', { name: 'Save' }))
    await waitFor(() => expect(f.posted).toHaveLength(1))
    expect((f.posted[0].poi as { poi_ids: string[] }).poi_ids).toEqual(['p1', 'p2'])
  })

  it('asks for confirmation before deleting, then sends delete', async () => {
    const f = makeFetch()
    const onDeleted = vi.fn()
    render(ui(f, photo(), { onDeleted }))
    fireEvent.click(screen.getByRole('button', { name: 'Delete' }))
    expect(f.posted).toHaveLength(0)
    expect(screen.getByText(/Delete this photo/)).toBeTruthy()
    fireEvent.click(screen.getByRole('button', { name: 'Confirm delete' }))
    await waitFor(() => expect(f.posted).toHaveLength(1))
    expect(f.posted[0]).toEqual({ op: 'delete', poi: { id: 'ph1' } })
    await waitFor(() => expect(onDeleted).toHaveBeenCalledWith('ph1'))
  })

  it('cancelling the confirmation sends nothing', () => {
    const f = makeFetch()
    render(ui(f))
    fireEvent.click(screen.getByRole('button', { name: 'Delete' }))
    fireEvent.click(screen.getByRole('button', { name: 'Cancel' }))
    expect(f.posted).toHaveLength(0)
    expect(screen.getByRole('button', { name: 'Delete' })).toBeTruthy()
  })
})
