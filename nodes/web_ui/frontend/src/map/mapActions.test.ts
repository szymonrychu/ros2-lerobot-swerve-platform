// @vitest-environment node
import { afterEach, describe, expect, it, vi } from 'vitest'
import { confirmClick, isCleared, parseActionResult, postTabAction, RESET_CONFIRM_MS } from './mapActions'

describe('confirmClick', () => {
  it('first click arms without firing', () => {
    expect(confirmClick(null, 1000)).toEqual({ fire: false, armedAt: 1000 })
  })

  it('second click within the window fires and disarms', () => {
    expect(confirmClick(1000, 1000 + RESET_CONFIRM_MS - 1)).toEqual({ fire: true, armedAt: null })
  })

  it('second click after the window re-arms instead of firing', () => {
    expect(confirmClick(1000, 1000 + RESET_CONFIRM_MS + 1)).toEqual({ fire: false, armedAt: 1000 + RESET_CONFIRM_MS + 1 })
  })

  it('honours a custom window', () => {
    expect(confirmClick(0, 50, 100).fire).toBe(true)
    expect(confirmClick(0, 150, 100).fire).toBe(false)
  })
})

describe('isCleared', () => {
  it('is true only for an explicit null payload', () => {
    expect(isCleared(null)).toBe(true)
    expect(isCleared(undefined)).toBe(false)
    expect(isCleared({ png_b64: 'x' })).toBe(false)
  })
})

describe('parseActionResult', () => {
  it('uses ok and message from the body', () => {
    expect(parseActionResult(200, { ok: true, message: 'done' })).toEqual({ state: 'ok', message: 'done' })
    expect(parseActionResult(503, { ok: false, message: 'down' })).toEqual({ state: 'error', message: 'down' })
  })

  it('falls back to the HTTP status when the body has no message', () => {
    expect(parseActionResult(500, {})).toEqual({ state: 'error', message: 'HTTP 500' })
    expect(parseActionResult(502, null)).toEqual({ state: 'error', message: 'HTTP 502' })
  })
})

describe('postTabAction', () => {
  afterEach(() => {
    vi.unstubAllGlobals()
  })

  it('posts to the route with the encoded tab id and parses the answer', async () => {
    const fetchMock = vi.fn().mockResolvedValue({ status: 200, json: async () => ({ ok: true, message: 'done' }) })
    vi.stubGlobal('fetch', fetchMock)
    expect(await postTabAction('/api/arm/home', 'a b')).toEqual({ state: 'ok', message: 'done' })
    expect(fetchMock).toHaveBeenCalledWith('/api/arm/home?tab=a%20b', { method: 'POST' })
  })

  it('turns a network failure into an error result', async () => {
    vi.stubGlobal('fetch', vi.fn().mockRejectedValue(new Error('down')))
    expect(await postTabAction('/api/arm/home', 'arm')).toEqual({ state: 'error', message: 'Request failed: Error: down' })
  })
})
