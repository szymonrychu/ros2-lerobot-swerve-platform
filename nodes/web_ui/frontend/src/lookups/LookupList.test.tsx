import { describe, expect, it } from 'vitest'
import { renderToStaticMarkup } from 'react-dom/server'
import { LookupList } from './LookupList'
import type { Lookup } from './api'

const LOOKUPS: Lookup[] = [
  { id: 'b', ts: 2, time: '2026-10-11T10:00:00Z', camera: 'head', tool: 'get_camera_image', note: 'red mug', poiIds: ['p1'], mcpSession: 's' },
  { id: 'a', ts: 1, time: '2026-10-11T09:00:00Z', camera: 'gripper', tool: 'get_annotated_camera_image', note: '', poiIds: [], mcpSession: null },
]

describe('LookupList', () => {
  it('renders one row per lookup with a thumbnail, camera, time and note', () => {
    const html = renderToStaticMarkup(<LookupList lookups={LOOKUPS} selectedPoiName={null} />)
    expect(html).toContain('src="/api/lookups/b.jpg?max_px=144"')
    expect(html).toContain('src="/api/lookups/a.jpg?max_px=144"')
    expect(html).toContain('href="/api/lookups/b.jpg"') // the link still opens the full image
    expect(html.indexOf('b.jpg?')).toBeLessThan(html.indexOf('a.jpg?'))
    expect(html).toContain('head')
    expect(html).toContain('red mug')
    expect(html).toContain('2026-10-11T10:00:00Z')
    expect(html).toContain('Camera lookups (2)')
  })

  it('names the selected POI in the title and in the empty message', () => {
    expect(renderToStaticMarkup(<LookupList lookups={LOOKUPS} selectedPoiName="Mug" />)).toContain('Lookups of Mug (2)')
    const empty = renderToStaticMarkup(<LookupList lookups={[]} selectedPoiName="Mug" />)
    expect(empty).toContain('No lookups linked to Mug yet.')
    expect(renderToStaticMarkup(<LookupList lookups={[]} selectedPoiName={null} />)).toContain('No lookups yet.')
  })

  it('shows the fetch error instead of "No lookups yet."', () => {
    const html = renderToStaticMarkup(<LookupList lookups={[]} error="HTTP 500" selectedPoiName={null} />)
    expect(html).toContain('Could not load lookups: HTTP 500')
    expect(html).not.toContain('No lookups yet.')
  })

  it('keeps the last lookups visible next to an error', () => {
    const html = renderToStaticMarkup(<LookupList lookups={LOOKUPS} error="offline" selectedPoiName={null} />)
    expect(html).toContain('Could not load lookups: offline')
    expect(html).toContain('red mug')
  })
})
