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
    respond(200, { imu: true, arm: 'deferred' })
    render(<CalibratePage onBack={() => {}} />)
    await screen.findByText('IMU calibration')
    expect(fetch).toHaveBeenCalledWith('/api/calibration/config')
  })

  it('404 shows not configured and no sections', async () => {
    respond(404)
    render(<CalibratePage onBack={() => {}} />)
    await screen.findByText('Calibration page not configured on this robot')
    expect(screen.queryByText('IMU calibration')).toBeNull()
  })

  it('network error shows the error', async () => {
    vi.stubGlobal('fetch', vi.fn(async () => { throw new Error('link down') }))
    render(<CalibratePage onBack={() => {}} />)
    await screen.findByText(/link down/)
  })

  it('renders the IMU section only when imu is true', async () => {
    respond(200, { imu: true, arm: 'deferred' })
    render(<CalibratePage onBack={() => {}} />)
    await screen.findByText('IMU calibration')
    cleanup()
    respond(200, { imu: false })
    render(<CalibratePage onBack={() => {}} />)
    await screen.findByText('Arm calibration - deferred')
    expect(screen.queryByText('IMU calibration')).toBeNull()
  })

  it('imu-only config renders the IMU section only', async () => {
    respond(200, { imu: true, arm: 'deferred' })
    render(<CalibratePage onBack={() => {}} />)
    await screen.findByText('IMU calibration')
  })

  it('always shows the deferred arm line once loaded', async () => {
    respond(200, { imu: false })
    render(<CalibratePage onBack={() => {}} />)
    await screen.findByText('Arm calibration - deferred')
  })

  it('renders both sections with imu and rgbd', async () => {
    vi.stubGlobal('fetch', vi.fn(async (url: string) => {
      if (url.endsWith('/state')) {
        const capture = { running: false, session_id: null, status: 'idle', pose_count: 0, started_at: null, stops_at: null }
        const compute = { running: false, session_id: null, result_id: null, started_at: null, error: null }
        return new Response(JSON.stringify({ capture, compute, applied: null }))
      }
      return new Response(JSON.stringify({ imu: true, rgbd: true }))
    }))
    render(<CalibratePage onBack={() => {}} />)
    await screen.findByText('IMU calibration')
    await screen.findByText('RGB-D calibration')
  })

  it('shows the Head floor fit step after the RGB-D section only with rgbd configured', async () => {
    vi.stubGlobal('fetch', vi.fn(async (url: string) => {
      if (url.endsWith('/floor/state')) {
        const sweep = { running: false, session_id: null, pose: 0, total: 0, observations: 0, skipped: 0, status: 'idle', error: null }
        return new Response(JSON.stringify({ sweep, applied: null }))
      }
      if (url.endsWith('/state')) {
        const capture = { running: false, session_id: null, status: 'idle', pose_count: 0, started_at: null, stops_at: null }
        const compute = { running: false, session_id: null, result_id: null, started_at: null, error: null }
        return new Response(JSON.stringify({ capture, compute, applied: null }))
      }
      return new Response(JSON.stringify({ imu: false, rgbd: true }))
    }))
    render(<CalibratePage onBack={() => {}} />)
    const rgbd = await screen.findByText('RGB-D calibration')
    const floor = await screen.findByText('Head floor fit')
    expect(rgbd.compareDocumentPosition(floor) & Node.DOCUMENT_POSITION_FOLLOWING).toBeTruthy()
    cleanup()
    respond(200, { imu: true, rgbd: false })
    render(<CalibratePage onBack={() => {}} />)
    await screen.findByText('IMU calibration')
    expect(screen.queryByText('Head floor fit')).toBeNull()
  })

  it('shows the Head from arm section after Head floor fit only with rgbd configured', async () => {
    vi.stubGlobal('fetch', vi.fn(async (url: string) => {
      if (url.endsWith('/head-arm/state')) {
        const sequence = { running: false, phase: 'idle', session_id: null, tier: 0, arm_pose: 0, total_poses: 0, captured: 0, skipped: {}, status: 'idle', error: null, big_joints: [], waiting_since: null, board_record: null }
        return new Response(JSON.stringify({ sequence, applied: null }))
      }
      if (url.endsWith('/head-arm/sessions')) return new Response('[]')
      if (url.endsWith('/floor/state')) {
        const sweep = { running: false, session_id: null, pose: 0, total: 0, observations: 0, skipped: 0, status: 'idle', error: null }
        return new Response(JSON.stringify({ sweep, applied: null }))
      }
      if (url.endsWith('/state')) {
        const capture = { running: false, session_id: null, status: 'idle', pose_count: 0, started_at: null, stops_at: null }
        const compute = { running: false, session_id: null, result_id: null, started_at: null, error: null }
        return new Response(JSON.stringify({ capture, compute, applied: null }))
      }
      return new Response(JSON.stringify({ imu: false, rgbd: true }))
    }))
    render(<CalibratePage onBack={() => {}} />)
    const floor = await screen.findByText('Head floor fit')
    const arm = await screen.findByText('Head from arm')
    expect(floor.compareDocumentPosition(arm) & Node.DOCUMENT_POSITION_FOLLOWING).toBeTruthy()
    expect(screen.getByText('Arm calibration - deferred')).toBeTruthy()
    cleanup()
    respond(200, { imu: true, rgbd: false })
    render(<CalibratePage onBack={() => {}} />)
    await screen.findByText('IMU calibration')
    expect(screen.queryByText('Head from arm')).toBeNull()
  })

  it('imu only with rgbd false', async () => {
    respond(200, { imu: true, rgbd: false })
    render(<CalibratePage onBack={() => {}} />)
    await screen.findByText('IMU calibration')
    expect(screen.queryByText('RGB-D calibration')).toBeNull()
  })

  it('back button clears the hash and calls onBack', async () => {
    respond(200, { imu: true })
    const onBack = vi.fn()
    render(<CalibratePage onBack={onBack} />)
    await screen.findByText('IMU calibration')
    fireEvent.click(screen.getByRole('button', { name: 'Back' }))
    await waitFor(() => expect(onBack).toHaveBeenCalled())
    expect(window.location.hash).toBe('')
  })

  it('renders the servo section only with servos true', async () => {
    vi.stubGlobal('fetch', vi.fn(async (url: string) => {
      if (url.startsWith('/api/calibration/servos')) return new Response(JSON.stringify({ groups: [] }))
      return new Response(JSON.stringify({ imu: false, servos: true }))
    }))
    render(<CalibratePage onBack={() => {}} />)
    await screen.findByText('Servo torque and neutral')
    cleanup()
    respond(200, { imu: false, servos: false })
    render(<CalibratePage onBack={() => {}} />)
    await screen.findByText('Arm calibration - deferred')
    expect(screen.queryByText('Servo torque and neutral')).toBeNull()
  })
})
