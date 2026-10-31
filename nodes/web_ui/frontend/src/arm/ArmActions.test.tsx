// @vitest-environment jsdom
import { act, cleanup, fireEvent, render, screen } from '@testing-library/react'
import { useState } from 'react'
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { ThemeProvider } from '@mui/material/styles'
import type { ActionResult } from '../map/mapActions'
import { theme } from '../theme'
import { GRASP_MENU } from '../grasp/pick'
import type { GraspPanelState } from '../grasp/useGrasp'
import ArmActions from './ArmActions'
import { armControlState } from './armControls'

const fetchMock = vi.fn()

/** Stateful host: ArmTab owns the action state, so the test does too. */
function fakeGrasp(over: Partial<GraspPanelState> = {}): GraspPanelState {
  return { pickMode: false, executing: false, planning: false, hasTarget: false, startPick: vi.fn(), cancelPick: vi.fn(), stop: vi.fn(), ...over } as unknown as GraspPanelState
}

interface HostProps {
  released?: boolean
  joints?: string[]
  grasp?: GraspPanelState
  onOpenChange?: (open: boolean) => void
  open?: boolean
}

function Host({ released = false, joints = [] as string[], grasp, onOpenChange = vi.fn(), open = false }: HostProps) {
  const [action, setAction] = useState<ActionResult>({ state: 'idle', message: '' })
  const controls = armControlState({
    released,
    releasedJoints: joints,
    busy: action.state === 'busy',
    armReady: true,
    activeSource: undefined,
    graspBusy: grasp ? grasp.executing || grasp.planning : false,
  })
  return (
    <ThemeProvider theme={theme}>
      <ArmActions tabId="arm" controls={controls} action={action} onAction={setAction} grasp={grasp ?? null} graspOpen={open} onGraspOpenChange={onOpenChange} />
    </ThemeProvider>
  )
}

const button = (name: string) => screen.getByRole('button', { name }) as HTMLButtonElement
const posts = () => fetchMock.mock.calls.map(([url, init]) => `${init?.method} ${url}`)

beforeEach(() => {
  fetchMock.mockReset()
  fetchMock.mockResolvedValue({ status: 200, json: async () => ({ ok: true, message: 'Arm homed' }) })
  vi.stubGlobal('fetch', fetchMock)
  window.matchMedia = (() => ({ matches: false, addEventListener: vi.fn(), removeEventListener: vi.fn() })) as unknown as typeof window.matchMedia
})
afterEach(() => {
  cleanup()
  vi.useRealTimers()
  vi.unstubAllGlobals()
})

describe('ArmActions', () => {
  it('renders Arm home and Set home, enabled, with no reason', () => {
    render(<Host />)
    expect(button('Arm home').disabled).toBe(false)
    expect(button('Set home').disabled).toBe(false)
    expect(screen.queryByTestId('arm-actions-reason')).toBeNull()
  })

  it('Arm home posts on one click and shows the reply as an alert', async () => {
    render(<Host />)
    await act(async () => fireEvent.click(button('Arm home')))
    expect(posts()).toEqual(['POST /api/arm/home?tab=arm'])
    expect((await screen.findByRole('status')).textContent).toContain('Arm homed')
  })

  it('Set home needs two clicks inside the confirm window', async () => {
    render(<Host />)
    fireEvent.click(button('Set home'))
    expect(fetchMock).not.toHaveBeenCalled()
    await act(async () => fireEvent.click(button('Confirm set home')))
    expect(posts()).toEqual(['POST /api/arm/set_home?tab=arm'])
  })

  it('Set home disarms after the confirm window', () => {
    vi.useFakeTimers()
    render(<Host />)
    fireEvent.click(button('Set home'))
    act(() => vi.advanceTimersByTime(5000))
    expect(button('Set home')).toBeTruthy()
    fireEvent.click(button('Set home'))
    expect(fetchMock).not.toHaveBeenCalled()
  })

  it('a failed reply is an error alert', async () => {
    fetchMock.mockResolvedValue({ status: 409, json: async () => ({ ok: false, message: 'Arm servo torque is off (Calibrate page)' }) })
    render(<Host />)
    await act(async () => fireEvent.click(button('Arm home')))
    expect((await screen.findByRole('alert')).textContent).toContain('torque is off')
  })

  it('both are disabled with the in-flight reason while a call is running', async () => {
    let resolve: (v: unknown) => void = () => undefined
    fetchMock.mockReturnValue(new Promise((r) => (resolve = r)))
    render(<Host />)
    fireEvent.click(button('Arm home'))
    expect(button('Arm home').disabled).toBe(true)
    expect(button('Set home').disabled).toBe(true)
    expect(screen.getByTestId('arm-actions-reason').textContent).toBe('Arm action in flight')
    await act(async () => resolve({ status: 200, json: async () => ({ ok: true, message: 'done' }) }))
    expect(button('Arm home').disabled).toBe(false)
  })

  it('released disables both and shows the joint names as visible text and as the tooltip', async () => {
    render(<Host released joints={['elbow_flex', 'gripper']} />)
    const text = 'Arm torque is off: elbow_flex, gripper released (Calibrate page)'
    expect(button('Arm home').disabled).toBe(true)
    expect(button('Set home').disabled).toBe(true)
    expect(screen.getByTestId('arm-actions-reason').textContent).toBe(text)
    fireEvent.mouseOver(button('Arm home').parentElement as HTMLElement)
    expect((await screen.findByRole('tooltip')).textContent).toBe(text)
  })
})

describe('ArmActions grasp', () => {
  it('has no Grasp button without a grasp state', () => {
    render(<Host />)
    expect(screen.queryByRole('button', { name: 'Grasp' })).toBeNull()
  })

  it('the Grasp menu lists every strategy', () => {
    render(<Host grasp={fakeGrasp()} />)
    fireEvent.click(button('Grasp'))
    for (const m of GRASP_MENU) expect(screen.getByRole('menuitem', { name: m.label })).toBeTruthy()
  })

  it('choosing a strategy opens the grasp flow and enters pick mode', () => {
    const grasp = fakeGrasp()
    const onOpenChange = vi.fn()
    render(<Host grasp={grasp} onOpenChange={onOpenChange} />)
    fireEvent.click(button('Grasp'))
    fireEvent.click(screen.getByRole('menuitem', { name: GRASP_MENU[0].label }))
    expect(grasp.startPick).toHaveBeenCalledWith(GRASP_MENU[0].strategy)
    expect(onOpenChange).toHaveBeenCalledWith(true)
  })

  it('shows the pick hint, a Cancel grasp pick button, and Esc cancels', () => {
    const grasp = fakeGrasp({ pickMode: true })
    render(<Host grasp={grasp} open />)
    expect(screen.getByTestId('arm-grasp-hint').textContent).toBe('Click the object in the arm view')
    fireEvent.click(button('Cancel grasp pick'))
    expect(grasp.cancelPick).toHaveBeenCalledTimes(1)
    fireEvent.keyDown(window, { key: 'Escape' })
    expect(grasp.cancelPick).toHaveBeenCalledTimes(2)
  })

  it('asks for a strategy while the flow is open without a target', () => {
    render(<Host grasp={fakeGrasp()} open />)
    expect(screen.getByTestId('arm-grasp-hint').textContent).toBe('Choose a strategy in the Grasp menu, then click the object')
  })

  it('is disabled with the released reason and cannot open the menu', () => {
    render(<Host grasp={fakeGrasp()} released joints={['gripper']} />)
    expect(button('Grasp').disabled).toBe(true)
    expect(screen.getByTestId('arm-actions-reason').textContent).toBe('Arm torque is off: gripper released (Calibrate page)')
  })

  it('is disabled with the in-flight reason while a plan runs, and the menu items are disabled', () => {
    render(<Host grasp={fakeGrasp({ planning: true })} />)
    expect(button('Grasp').disabled).toBe(true)
    expect(screen.getByTestId('arm-actions-reason').textContent).toBe('Arm action in flight')
  })

  it('shows STOP GRASP only while a grasp executes, and it stops', () => {
    const idle = render(<Host grasp={fakeGrasp()} />)
    expect(screen.queryByRole('button', { name: 'STOP GRASP' })).toBeNull()
    idle.unmount()
    const grasp = fakeGrasp({ executing: true })
    render(<Host grasp={grasp} />)
    fireEvent.click(button('STOP GRASP'))
    expect(grasp.stop).toHaveBeenCalledTimes(1)
  })
})
