import { describe, expect, it, vi } from 'vitest'
import { clickToNormalised, requestZoom } from './zoomApi'

const OK_BODY = { status: 'ok', message: 'done', jpeg_b64: 'AAAA' }

function reply(status: number, body: unknown): typeof fetch {
  return vi.fn().mockResolvedValue(new Response(JSON.stringify(body), { status })) as unknown as typeof fetch
}

describe('requestZoom', () => {
  it('POSTs u, v, zoom as JSON to /api/camera/zoom with the encoded tab', async () => {
    const f = reply(200, OK_BODY)
    await requestZoom('head tab', 0.25, 0.75, 4, f)
    const [url, init] = (f as unknown as ReturnType<typeof vi.fn>).mock.calls[0]
    expect(url).toBe('/api/camera/zoom?tab=head%20tab')
    expect(init.method).toBe('POST')
    expect(JSON.parse(init.body)).toEqual({ u: 0.25, v: 0.75, zoom: 4 })
  })

  it('returns the endpoint body on success', async () => {
    expect(await requestZoom('head', 0.5, 0.5, 2, reply(200, OK_BODY))).toEqual(OK_BODY)
  })

  it.each([409, 502, 504])('rejects with status and message on %i', async (code) => {
    const f = reply(code, { ok: false, message: 'camera busy' })
    await expect(requestZoom('head', 0.5, 0.5, 8, f)).rejects.toThrow(new RegExp(`${code}.*camera busy`))
  })

  it('rejects with the status when the error body is not JSON', async () => {
    const f = vi.fn().mockResolvedValue(new Response('boom', { status: 502 })) as unknown as typeof fetch
    await expect(requestZoom('head', 0.5, 0.5, 8, f)).rejects.toThrow(/502/)
  })
})

describe('clickToNormalised', () => {
  const rect = (w: number, h: number) => ({ left: 10, top: 20, width: w, height: h }) as DOMRect

  it('maps the centre of a filled image to 0.5, 0.5', () => {
    expect(clickToNormalised(110, 70, rect(200, 100), 1600, 800)).toEqual({ u: 0.5, v: 0.5 })
  })

  it('maps the corners of a filled image to 0 and 1', () => {
    expect(clickToNormalised(10, 20, rect(200, 100), 200, 100)).toEqual({ u: 0, v: 0 })
    expect(clickToNormalised(210, 120, rect(200, 100), 200, 100)).toEqual({ u: 1, v: 1 })
  })

  it('wide container (pillarbox): image 16:9 in 400x100 spans x 111.1..288.9', () => {
    const r = rect(400, 100)
    const w = 100 * (16 / 9)
    const left = 10 + (400 - w) / 2
    expect(clickToNormalised(left - 1, 70, r, 1920, 1080)).toBeNull()
    expect(clickToNormalised(10 + 400 - (400 - w) / 2 + 1, 70, r, 1920, 1080)).toBeNull()
    const p = clickToNormalised(left + w / 4, 20 + 25, r, 1920, 1080)
    expect(p!.u).toBeCloseTo(0.25)
    expect(p!.v).toBeCloseTo(0.25)
  })

  it('tall container (letterbox): image 16:9 in 160x200 spans y 55..145 of the box', () => {
    const r = rect(160, 200)
    const h = 160 * (9 / 16)
    const top = 20 + (200 - h) / 2
    expect(clickToNormalised(50, top - 1, r, 1920, 1080)).toBeNull()
    expect(clickToNormalised(50, top + h + 1, r, 1920, 1080)).toBeNull()
    const p = clickToNormalised(10 + 120, top + h / 2, r, 1920, 1080)
    expect(p!.u).toBeCloseTo(0.75)
    expect(p!.v).toBeCloseTo(0.5)
  })

  it('returns null while the natural size is unknown', () => {
    expect(clickToNormalised(50, 50, rect(100, 100), 0, 0)).toBeNull()
  })
})
