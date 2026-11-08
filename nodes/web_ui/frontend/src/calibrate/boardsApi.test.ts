// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import { boardDownloads, boardLabel, listModels } from './boardsApi'
import stateFx from '../../../../head_rgbd_calibration/tests/fixtures/api/state.json'

const ARM = stateFx.boards.arm
const FULL = stateFx.boards.full

function stub(status: number, body: unknown) {
  const f = vi.fn(async () => new Response(JSON.stringify(body), { status }))
  vi.stubGlobal('fetch', f)
  return f
}
afterEach(() => {
  vi.unstubAllGlobals()
})

describe('boardsApi', () => {
  it('offers the pdf and 3mf of the board by stem', () => {
    const files = [{ name: 'charuco-arm-6x5-28mm.3mf', size: 1, url: '/models/charuco-arm-6x5-28mm.3mf' },
                   { name: 'charuco-arm-6x5-28mm.pdf', size: 1, url: '/models/charuco-arm-6x5-28mm.pdf' },
                   { name: 'base-home.pdf', size: 1, url: '/models/base-home.pdf' }]
    expect(boardDownloads(ARM, files).map((f) => f.name)).toEqual(['charuco-arm-6x5-28mm.pdf', 'charuco-arm-6x5-28mm.3mf'])
  })
  it('offers nothing when the board files are not served', () => {
    expect(boardDownloads(FULL, [{ name: 'charuco-arm-6x5-28mm.pdf', size: 1, url: '/models/charuco-arm-6x5-28mm.pdf' }])).toEqual([])
  })
  it('labels each board with its role, name, squares and square size', () => {
    expect(boardLabel('arm', ARM)).toBe('ARM board: charuco-arm-6x5-28mm, 6x5 squares of 28 mm')
    expect(boardLabel('full', FULL)).toBe('FULL board: charuco-6x5-40mm, 6x5 squares of 40 mm')
  })
  it('listModels reads the files of GET /api/models', async () => {
    const files = [{ name: 'charuco-6x5-40mm.pdf', size: 12, url: '/models/charuco-6x5-40mm.pdf' }]
    const f = stub(200, { files })
    expect(await listModels()).toEqual({ ok: true, value: files })
    expect(f).toHaveBeenCalledWith('/api/models', { cache: 'no-store' })
  })
  it('listModels passes the API message through and refuses a malformed body', async () => {
    stub(404, { ok: false, message: 'models not configured' })
    expect(await listModels()).toEqual({ ok: false, status: 404, message: 'models not configured' })
    stub(200, { files: [{ name: 3 }] })
    expect((await listModels()).ok).toBe(false)
  })
})
