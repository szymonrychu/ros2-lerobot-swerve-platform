// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react'
import { ThemeProvider } from '@mui/material/styles'
import { theme } from '../theme'
import { PhotoList } from './PhotoList'

function photo(id: string, ts: number) {
  return {
    id,
    kind: 'photo',
    x: 1,
    y: 2,
    name: '',
    note: `note-${id}`,
    status: 'open',
    created_by: 'agent',
    created_at: ts,
    updated_at: 0,
    poi_ids: [],
    photo: { file: `${id}.jpg`, bytes: 1000, width: 640, height: 480, camera: 'head', tool: 'get_camera_image', mcp_session: null, view: null },
  }
}

interface Posted {
  op: string
  selector: Record<string, unknown>
}

function makeFetch(opts: { dryRun?: unknown } = {}) {
  const posted: Posted[] = []
  const gets: string[] = []
  const impl = vi.fn(async (url: string, init?: RequestInit) => {
    if (init?.method === 'POST') {
      const body = JSON.parse(String(init.body)) as Posted
      posted.push(body)
      const poi = body.selector.dry_run === true ? (opts.dryRun ?? { would_remove: 3, bytes: 2_500_000 }) : { removed: 3, bytes_freed: 2_500_000, missing: [] }
      return { ok: true, status: 200, json: async () => ({ ok: true, message: 'ok', poi }) } as Response
    }
    gets.push(url)
    const body = url.includes('before=')
      ? { photos: [photo('c', 1)], next_before: null, total_matching: 3 }
      : { photos: [photo('a', 3), photo('b', 2)], next_before: '2:b', total_matching: 3 }
    return { ok: true, status: 200, json: async () => body } as Response
  })
  return { impl: impl as unknown as typeof fetch, posted, gets }
}

function ui(f: ReturnType<typeof makeFetch>, over: Partial<Parameters<typeof PhotoList>[0]> = {}) {
  return <PhotoList tabId="t" revision={1} selectedPoiId={null} selectedPoiName={null} fetchImpl={f.impl} {...over} />
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

afterEach(cleanup)

describe.each([
  [400, 'phone'],
  [1400, 'desktop'],
])('PhotoList on OverlayPanel at %i px (%s)', (width, cls) => {
  it('renders inside the overlay panel with rem and dvh sizes', async () => {
    viewportWidth = width
    installMatchMedia()
    render(<ThemeProvider theme={theme}>{ui(makeFetch())}</ThemeProvider>)
    await screen.findAllByTestId('photo-row')
    const panel = screen.getByTestId('overlay-panel')
    expect(panel.getAttribute('data-layout')).toBe(cls)
    expect(panel.contains(screen.getByTestId('photo-list-panel'))).toBe(true)
    expect(screen.queryByTestId('overlay-panel-handle') !== null).toBe(cls === 'phone')
    const img = document.querySelector('img') as HTMLElement
    expect(getComputedStyle(img).width).toBe('4.5rem')
  })
})

describe('PhotoList', () => {
  it('lists photos with thumbnails and filters by the selected POI', async () => {
    const f = makeFetch()
    const { rerender } = render(ui(f))
    expect(await screen.findAllByTestId('photo-row')).toHaveLength(2)
    expect(document.querySelector('img')?.getAttribute('src')).toBe('/api/photos/a.jpg?max_px=144')
    expect(f.gets[0]).toBe('/api/photos?limit=10')
    rerender(ui(f, { selectedPoiId: 'p1', selectedPoiName: 'Mug' }))
    await waitFor(() => expect(f.gets[f.gets.length - 1]).toBe('/api/photos?limit=10&poi_id=p1'))
    expect(await screen.findByText('Photos of Mug (3)')).toBeTruthy()
  })

  it("'more' pages with the cursor and appends", async () => {
    const f = makeFetch()
    render(ui(f))
    fireEvent.click(await screen.findByRole('button', { name: 'more' }))
    await waitFor(() => expect(screen.getAllByTestId('photo-row')).toHaveLength(3))
    expect(f.gets[f.gets.length - 1]).toContain('before=2%3Ab')
    expect(screen.queryByRole('button', { name: 'more' })).toBeNull()
  })

  it("'Delete selected' sends the checked ids", async () => {
    const f = makeFetch()
    render(ui(f))
    const boxes = await screen.findAllByRole('checkbox')
    expect((screen.getByRole('button', { name: 'Delete selected' }) as HTMLButtonElement).disabled).toBe(true)
    fireEvent.click(boxes[1])
    fireEvent.click(boxes[0])
    fireEvent.click(screen.getByRole('button', { name: 'Delete selected' }))
    await waitFor(() => expect(f.posted).toHaveLength(1))
    expect(f.posted[0]).toEqual({ op: 'delete_photos', selector: { ids: ['a', 'b'] } })
    await waitFor(() => expect(f.gets.length).toBeGreaterThan(1)) // refetched afterwards
  })

  it('cleanup older than N days: dry run first, real delete only after confirm', async () => {
    const f = makeFetch()
    render(ui(f))
    await screen.findAllByTestId('photo-row')
    fireEvent.change(screen.getByLabelText('Older than (days)'), { target: { value: '7' } })
    fireEvent.click(screen.getByRole('button', { name: 'Preview older than' }))
    expect(await screen.findByText('deletes 3 photos (2.5 MB)')).toBeTruthy()
    expect(f.posted).toEqual([{ op: 'delete_photos', selector: { older_than_s: 7 * 86400, dry_run: true } }])
    fireEvent.click(screen.getByRole('button', { name: 'Confirm delete' }))
    await waitFor(() => expect(f.posted).toHaveLength(2))
    expect(f.posted[1]).toEqual({ op: 'delete_photos', selector: { older_than_s: 7 * 86400 } })
    await waitFor(() => expect(screen.queryByText('deletes 3 photos (2.5 MB)')).toBeNull())
  })

  it('cleanup keep newest N: cancel sends nothing more', async () => {
    const f = makeFetch()
    render(ui(f))
    await screen.findAllByTestId('photo-row')
    fireEvent.change(screen.getByLabelText('Keep newest'), { target: { value: '20' } })
    fireEvent.click(screen.getByRole('button', { name: 'Preview keep newest' }))
    expect(await screen.findByText('deletes 3 photos (2.5 MB)')).toBeTruthy()
    expect(f.posted[0].selector).toEqual({ keep_newest: 20, dry_run: true })
    fireEvent.click(screen.getByRole('button', { name: 'Cancel' }))
    expect(screen.queryByText('deletes 3 photos (2.5 MB)')).toBeNull()
    expect(f.posted).toHaveLength(1)
  })

  it('says so when the preview deletes nothing and offers no confirm', async () => {
    const f = makeFetch({ dryRun: { would_remove: 0, bytes: 0 } })
    render(ui(f))
    await screen.findAllByTestId('photo-row')
    fireEvent.change(screen.getByLabelText('Keep newest'), { target: { value: '99' } })
    fireEvent.click(screen.getByRole('button', { name: 'Preview keep newest' }))
    expect(await screen.findByText('deletes 0 photos (0.0 MB)')).toBeTruthy()
    expect(screen.queryByRole('button', { name: 'Confirm delete' })).toBeNull()
  })

  it('refetches when the /poi/photos revision changes', async () => {
    const f = makeFetch()
    const { rerender } = render(ui(f))
    await screen.findAllByTestId('photo-row')
    const before = f.gets.length
    rerender(ui(f, { revision: 1 }))
    expect(f.gets).toHaveLength(before)
    rerender(ui(f, { revision: 2 }))
    await waitFor(() => expect(f.gets.length).toBe(before + 1))
  })

  it('shows the fetch error and keeps the last list', async () => {
    const f = makeFetch()
    const { rerender } = render(ui(f))
    await screen.findAllByTestId('photo-row')
    const bad = (async () => ({ ok: false, status: 500, json: async () => ({}) }) as Response) as unknown as typeof fetch
    rerender(<PhotoList tabId="t" revision={2} selectedPoiId={null} selectedPoiName={null} fetchImpl={bad} />)
    expect(await screen.findByRole('alert')).toBeTruthy()
    expect(screen.getByRole('alert').textContent).toContain('Could not load photos: HTTP 500')
    expect(screen.getAllByTestId('photo-row')).toHaveLength(2)
  })
})
