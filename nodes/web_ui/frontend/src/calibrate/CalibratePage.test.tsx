// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react'
import { CalibratePage } from './CalibratePage'

function respond(status: number, body: unknown = {}): void {
  vi.stubGlobal('fetch', vi.fn(async () => new Response(JSON.stringify(body), { status })))
}

describe('CalibratePage', () => {
  beforeEach(() => {
    window.location.hash = '#/calibrate'
  })
  afterEach(() => {
    cleanup()
    vi.unstubAllGlobals()
  })

  it('fetches the calibration config', async () => {
    respond(200, { stereo: true, imu: true })
    render(<CalibratePage onBack={() => {}} />)
    await screen.findByText('Stereo calibration')
    expect(fetch).toHaveBeenCalledWith('/api/calibration/config')
  })

  it('404 shows not configured and no sections', async () => {
    respond(404)
    render(<CalibratePage onBack={() => {}} />)
    await screen.findByText('Calibration page not configured on this robot')
    expect(screen.queryByText('Stereo calibration')).toBeNull()
    expect(screen.queryByText('IMU calibration')).toBeNull()
  })

  it('network error shows the error', async () => {
    vi.stubGlobal('fetch', vi.fn(async () => { throw new Error('link down') }))
    render(<CalibratePage onBack={() => {}} />)
    await screen.findByText(/link down/)
  })

  it('renders only the enabled sections', async () => {
    respond(200, { stereo: true, imu: false })
    render(<CalibratePage onBack={() => {}} />)
    await screen.findByText('Stereo calibration')
    expect(screen.queryByText('IMU calibration')).toBeNull()
    cleanup()
    respond(200, { stereo: false, imu: true })
    render(<CalibratePage onBack={() => {}} />)
    await screen.findByText('IMU calibration')
    expect(screen.queryByText('Stereo calibration')).toBeNull()
  })

  it('always shows the deferred arm line once loaded', async () => {
    respond(200, { stereo: false, imu: false })
    render(<CalibratePage onBack={() => {}} />)
    await screen.findByText('Arm calibration - deferred')
  })

  it('back button clears the hash and calls onBack', async () => {
    respond(200, { stereo: true, imu: true })
    const onBack = vi.fn()
    render(<CalibratePage onBack={onBack} />)
    await screen.findByText('Stereo calibration')
    fireEvent.click(screen.getByRole('button', { name: 'Back' }))
    await waitFor(() => expect(onBack).toHaveBeenCalled())
    expect(window.location.hash).toBe('')
  })
})
