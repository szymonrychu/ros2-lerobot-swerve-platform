// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { act, cleanup, fireEvent, render, screen, waitFor, within } from '@testing-library/react'
import { CalibratePage } from './CalibratePage'

vi.mock('./flows/ServoHomeFlow', () => ({ ServoHomeFlow: () => <div>servo-home-flow</div> }))
vi.mock('./flows/ImuFlow', () => ({ ImuFlow: () => <div>imu-flow</div> }))
vi.mock('./flows/CameraGroundFlow', () => ({ CameraGroundFlow: () => <div>camera-ground-flow</div> }))
vi.mock('./flows/ArmCameraFlow', () => ({
  ArmCameraFlow: ({ canHomeArm }: { canHomeArm: boolean }) => <div>{`arm-camera-flow home=${canHomeArm}`}</div>,
}))
vi.mock('./GeneralSection', () => ({
  GeneralSection: ({ actions, rgbd }: { actions: string[]; rgbd: boolean }) => <div>{`general ${actions.join(',')} rgbd=${rgbd}`}</div>,
}))

const ALL = { imu: true, rgbd: true, servos: true, general: ['open_gripper', 'close_gripper', 'home_arm', 'home_head', 'home_steering'] }

function respond(status: number, body: unknown = {}): void {
  vi.stubGlobal('fetch', vi.fn(async () => new Response(JSON.stringify(body), { status })))
}

/** Waits until the config has loaded (the spinner is gone). */
async function loaded(): Promise<void> {
  await waitFor(() => expect(screen.queryByRole('progressbar')).toBeNull())
}

const tabs = () => within(screen.getByRole('tablist', { name: 'Calibration flows' })).getAllByRole('tab')

describe('CalibratePage', () => {
  beforeEach(() => {
    window.location.hash = '#/calibrate'
  })
  afterEach(() => {
    cleanup()
    vi.unstubAllGlobals()
    window.location.hash = ''
  })

  it('fetches the calibration config', async () => {
    respond(200, { imu: true })
    render(<CalibratePage onBack={() => {}} />)
    await screen.findByText('IMU calibration')
    expect(fetch).toHaveBeenCalledWith('/api/calibration/config')
  })

  it('404 shows not configured and no flows', async () => {
    respond(404)
    render(<CalibratePage onBack={() => {}} />)
    await screen.findByText('Calibration page not configured on this robot')
    expect(screen.queryByRole('tablist')).toBeNull()
  })

  it('network error shows the error', async () => {
    vi.stubGlobal('fetch', vi.fn(async () => { throw new Error('link down') }))
    render(<CalibratePage onBack={() => {}} />)
    await screen.findByText(/link down/)
  })

  it('shows the four flows in order, one open at a time, the first one first', async () => {
    respond(200, ALL)
    render(<CalibratePage onBack={() => {}} />)
    await loaded()
    expect(tabs().map((t) => t.textContent)).toEqual(['Servo home', 'IMU', 'Camera to ground', 'Arm to camera'])
    expect(screen.getByText('Servo home calibration')).toBeTruthy()
    expect(screen.getByText('servo-home-flow')).toBeTruthy()
    expect(screen.queryByText('imu-flow')).toBeNull()
  })

  it('a tab opens its flow and deep-links it in the hash', async () => {
    respond(200, ALL)
    render(<CalibratePage onBack={() => {}} />)
    await loaded()
    fireEvent.click(screen.getByRole('tab', { name: 'Arm to camera' }))
    expect(screen.getByText('Arm to camera calibration')).toBeTruthy()
    expect(screen.getByText('arm-camera-flow home=true')).toBeTruthy()
    expect(screen.queryByText('servo-home-flow')).toBeNull()
    expect(window.location.hash).toBe('#/calibrate/arm-camera')
  })

  it('a deep link opens its flow', async () => {
    window.location.hash = '#/calibrate/camera-ground'
    respond(200, ALL)
    render(<CalibratePage onBack={() => {}} />)
    await loaded()
    expect(screen.getByRole('tab', { name: 'Camera to ground' }).getAttribute('aria-selected')).toBe('true')
    expect(screen.getByText('camera-ground-flow')).toBeTruthy()
  })

  it('a hash change switches the flow', async () => {
    respond(200, ALL)
    render(<CalibratePage onBack={() => {}} />)
    await loaded()
    await act(async () => {
      window.location.hash = '#/calibrate/imu'
      window.dispatchEvent(new HashChangeEvent('hashchange'))
    })
    expect(screen.getByText('imu-flow')).toBeTruthy()
  })

  it('a deep link to a flow this robot does not offer opens the first offered one', async () => {
    window.location.hash = '#/calibrate/arm-camera'
    respond(200, { imu: true })
    render(<CalibratePage onBack={() => {}} />)
    await loaded()
    expect(tabs().map((t) => t.textContent)).toEqual(['IMU'])
    expect(screen.getByText('imu-flow')).toBeTruthy()
  })

  it('offers each flow only with its config', async () => {
    respond(200, { imu: false, rgbd: true })
    render(<CalibratePage onBack={() => {}} />)
    await loaded()
    expect(tabs().map((t) => t.textContent)).toEqual(['Camera to ground', 'Arm to camera'])
    cleanup()
    respond(200, { servos: true })
    render(<CalibratePage onBack={() => {}} />)
    await loaded()
    expect(tabs().map((t) => t.textContent)).toEqual(['Servo home'])
  })

  it('shows the General section on top with the offered actions', async () => {
    respond(200, ALL)
    render(<CalibratePage onBack={() => {}} />)
    const general = await screen.findByText(/^general /)
    expect(general.textContent).toBe('general open_gripper,close_gripper,home_arm,home_head,home_steering rgbd=true')
    expect(general.compareDocumentPosition(screen.getByRole('tablist')) & Node.DOCUMENT_POSITION_FOLLOWING).toBeTruthy()
  })

  it('without general actions and the RGB-D node there is no General section; Arm to camera has no home step', async () => {
    respond(200, { imu: true })
    render(<CalibratePage onBack={() => {}} />)
    await loaded()
    expect(screen.queryByText(/^general /)).toBeNull()
    cleanup()
    respond(200, { rgbd: true, general: ['home_head', 'bogus'] })
    window.location.hash = '#/calibrate/arm-camera'
    render(<CalibratePage onBack={() => {}} />)
    await loaded()
    expect(screen.getByText('general home_head rgbd=true')).toBeTruthy()
    expect(screen.getByText('arm-camera-flow home=false')).toBeTruthy()
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
})
