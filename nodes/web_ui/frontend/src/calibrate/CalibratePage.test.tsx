// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react'
import { CalibratePage } from './CalibratePage'

/** Waits until the config has loaded (the spinner is gone). */
async function loaded(): Promise<void> {
  await waitFor(() => expect(screen.queryByRole('progressbar')).toBeNull())
}

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
    await loaded()
    expect(screen.queryByText('IMU calibration')).toBeNull()
  })

  it('imu-only config renders the IMU section only', async () => {
    respond(200, { imu: true, arm: 'deferred' })
    render(<CalibratePage onBack={() => {}} />)
    await screen.findByText('IMU calibration')
  })

  it('has no deferred arm line', async () => {
    respond(200, { imu: false })
    render(<CalibratePage onBack={() => {}} />)
    await loaded()
    expect(screen.queryByText(/deferred/)).toBeNull()
    expect(screen.queryByText('Arm calibration')).toBeNull()
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
    await screen.findByText('D435 intrinsics check')
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
    const rgbd = await screen.findByText('D435 intrinsics check')
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
    const armCalibration = await screen.findByText('Arm calibration')
    expect(arm.compareDocumentPosition(armCalibration) & Node.DOCUMENT_POSITION_FOLLOWING).toBeTruthy()
    expect(screen.queryByText(/deferred/)).toBeNull()
    cleanup()
    respond(200, { imu: true, rgbd: false })
    render(<CalibratePage onBack={() => {}} />)
    await screen.findByText('IMU calibration')
    expect(screen.queryByText('Head from arm')).toBeNull()
  })

  it('shows the CSI camera intrinsics section after the RGB-D section only with rgbd configured', async () => {
    vi.stubGlobal('fetch', vi.fn(async (url: string) => {
      if (url.includes('/csi/')) {
        const capture = { running: false, camera: null, session_id: null, status: 'idle', view_count: 0, started_at: null, stops_at: null }
        return new Response(JSON.stringify({ camera: 'front', capture, session: null, applied: null }))
      }
      if (url.endsWith('/state')) {
        const capture = { running: false, session_id: null, status: 'idle', pose_count: 0, started_at: null, stops_at: null }
        const compute = { running: false, session_id: null, result_id: null, started_at: null, error: null }
        return new Response(JSON.stringify({ capture, compute, applied: null }))
      }
      if (url.endsWith('/sessions')) return new Response('[]')
      return new Response(JSON.stringify({ imu: false, rgbd: true }))
    }))
    render(<CalibratePage onBack={() => {}} />)
    const rgbd = await screen.findByText('D435 intrinsics check')
    const csi = await screen.findByText('CSI camera intrinsics')
    expect(rgbd.compareDocumentPosition(csi) & Node.DOCUMENT_POSITION_FOLLOWING).toBeTruthy()
    const floor = await screen.findByText('Head floor fit')
    expect(csi.compareDocumentPosition(floor) & Node.DOCUMENT_POSITION_FOLLOWING).toBeTruthy()
    cleanup()
    respond(200, { imu: true, rgbd: false })
    render(<CalibratePage onBack={() => {}} />)
    await screen.findByText('IMU calibration')
    expect(screen.queryByText('CSI camera intrinsics')).toBeNull()
  })

  it('imu only with rgbd false', async () => {
    respond(200, { imu: true, rgbd: false })
    render(<CalibratePage onBack={() => {}} />)
    await screen.findByText('IMU calibration')
    expect(screen.queryByText('D435 intrinsics check')).toBeNull()
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
    await loaded()
    expect(screen.queryByText('Servo torque and neutral')).toBeNull()
  })
})
