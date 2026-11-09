// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import { cleanup, render, screen } from '@testing-library/react'
import { BoardNeeded } from './BoardNeeded'
import stateFx from '../../../../head_rgbd_calibration/tests/fixtures/api/state.json'

const MODELS = {
  files: [
    { name: 'charuco-6x5-40mm.3mf', size: 1, url: '/models/charuco-6x5-40mm.3mf' },
    { name: 'charuco-6x5-40mm.pdf', size: 1, url: '/models/charuco-6x5-40mm.pdf' },
    { name: 'charuco-arm-6x5-28mm.3mf', size: 1, url: '/models/charuco-arm-6x5-28mm.3mf' },
    { name: 'charuco-arm-6x5-28mm.pdf', size: 1, url: '/models/charuco-arm-6x5-28mm.pdf' },
  ],
}

function install(state: { status: number; body: unknown }, models: { status: number; body: unknown } = { status: 200, body: MODELS }) {
  const f = vi.fn(async (url: string) => {
    const r = url === '/api/models' ? models : url === '/api/calibration/rgbd/state' ? state : { status: 404, body: { message: 'nope' } }
    return new Response(JSON.stringify(r.body), { status: r.status })
  })
  vi.stubGlobal('fetch', f)
  return f
}

afterEach(() => {
  cleanup()
  vi.unstubAllGlobals()
})

describe('BoardNeeded', () => {
  it('labels the head-arm step with the ARM board and links its files', async () => {
    install({ status: 200, body: stateFx })
    render(<BoardNeeded role="arm" />)
    expect(await screen.findByText(/ARM board: charuco-arm-6x5-28mm/)).toBeTruthy()
    expect(screen.getByRole('link', { name: /PDF/ }).getAttribute('href')).toBe('/models/charuco-arm-6x5-28mm.pdf')
    expect(screen.getByRole('link', { name: /3MF/ }).getAttribute('href')).toBe('/models/charuco-arm-6x5-28mm.3mf')
  })
  it('labels the intrinsics steps with the FULL board', async () => {
    install({ status: 200, body: stateFx })
    render(<BoardNeeded role="full" />)
    expect(await screen.findByText(/FULL board: charuco-6x5-40mm, 6x5 squares of 40 mm/)).toBeTruthy()
    expect((await screen.findByRole('link', { name: /PDF/ })).getAttribute('href')).toBe('/models/charuco-6x5-40mm.pdf')
  })
  it('shows the API message when the state fails', async () => {
    install({ status: 503, body: { ok: false, message: 'calibration node unreachable' } })
    render(<BoardNeeded role="arm" />)
    expect(await screen.findByText('calibration node unreachable')).toBeTruthy()
    expect(screen.queryByText(/ARM board:/)).toBeNull()
    expect(screen.queryByRole('link')).toBeNull()
  })
  it('keeps the label and shows the API message when the models fail', async () => {
    install({ status: 200, body: stateFx }, { status: 404, body: { ok: false, message: 'models not configured' } })
    render(<BoardNeeded role="arm" />)
    expect(await screen.findByText(/ARM board: charuco-arm-6x5-28mm/)).toBeTruthy()
    expect(await screen.findByText('models not configured')).toBeTruthy()
    expect(screen.queryByRole('link')).toBeNull()
  })
})
