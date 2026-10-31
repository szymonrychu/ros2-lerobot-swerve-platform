// @vitest-environment jsdom
import { cleanup, fireEvent, render, screen } from '@testing-library/react'
import { forwardRef, type ReactNode } from 'react'
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { ThemeProvider } from '@mui/material/styles'
import { theme } from '../theme'
import type { TabConfig } from '../types'
import ArmTab from '../tabs/ArmTab'
import ArmView from './ArmView.tsx'

const JOINTS = '/arm/joint_states'
const mocks = vi.hoisted(() => ({
  invalidate: vi.fn(),
  orbitEnabled: [] as boolean[],
  frameloop: [] as string[],
  armProps: [] as Record<string, unknown>[],
  modelRenders: [] as number[],
}))

vi.mock('@react-three/fiber', () => ({
  Canvas: ({ children, frameloop }: { children: ReactNode; frameloop?: string }) => {
    mocks.frameloop.push(frameloop ?? '')
    return <div data-testid="canvas">{children}</div>
  },
  useThree: () => ({ invalidate: mocks.invalidate, camera: { up: { set: vi.fn() } } }),
  useFrame: vi.fn(),
}))

vi.mock('@react-three/drei', () => ({
  OrbitControls: forwardRef<unknown, { enabled: boolean }>(function OrbitControls({ enabled }, _ref) {
    mocks.orbitEnabled.push(enabled)
    return null
  }),
}))

vi.mock('../components3d/InteractiveArm', () => ({
  InteractiveArm: (props: Record<string, unknown>) => {
    mocks.armProps.push(props)
    return null
  },
}))
vi.mock('../components3d/RobotModel', () => ({
  RobotModel: () => {
    mocks.modelRenders.push(1)
    return null
  },
}))

const TAB = {
  id: 'arm',
  type: 'arm',
  label: 'Arm',
  arm_urdf: 'arm.urdf',
  arm_command_topic: '/arm/cmd',
  arm_joint_states_topic: JOINTS,
} as TabConfig

function view(topicData: Record<string, unknown>, dragDisabledReason: string | null = null, onArmReady = vi.fn()) {
  return (
    <ThemeProvider theme={theme}>
      <ArmView tab={TAB} topicData={topicData} publish={vi.fn()} dragDisabledReason={dragDisabledReason} onArmReady={onArmReady} />
    </ThemeProvider>
  )
}

const last = <T,>(a: T[]): T => a[a.length - 1]

beforeEach(() => {
  mocks.invalidate.mockClear()
  mocks.orbitEnabled.length = 0
  mocks.frameloop.length = 0
  mocks.armProps.length = 0
  mocks.modelRenders.length = 0
})
afterEach(cleanup)

describe('ArmView', () => {
  it('renders a demand-frameloop canvas with the four mode buttons', () => {
    render(view({}))
    expect(last(mocks.frameloop)).toBe('demand')
    for (const name of ['Side L', 'Side R', 'Above', 'Gripper']) {
      expect(screen.getByRole('button', { name })).toBeTruthy()
    }
  })

  it('invalidates when joint states update', () => {
    const { rerender } = render(view({ [JOINTS]: { name: ['a'], position: [0] } }))
    mocks.invalidate.mockClear()
    rerender(view({ [JOINTS]: { name: ['a'], position: [0.5] } }))
    expect(mocks.invalidate).toHaveBeenCalled()
  })

  it('disables orbit controls while a mode is glued and enables them when freed', () => {
    render(view({}))
    expect(last(mocks.orbitEnabled)).toBe(false)
    fireEvent.click(screen.getByRole('button', { name: 'Above' }))
    expect(last(mocks.orbitEnabled)).toBe(false)
    fireEvent.click(screen.getByRole('button', { name: 'Above' }))
    expect(last(mocks.orbitEnabled)).toBe(true)
  })

  it('passes onRobotLoaded to the draggable arm', () => {
    render(view({}))
    expect(typeof last(mocks.armProps).onRobotLoaded).toBe('function')
  })

  it('renders the interactive arm and no reason when dragging is allowed', () => {
    render(view({}))
    expect(mocks.armProps.length).toBeGreaterThan(0)
    expect(mocks.modelRenders).toEqual([])
    expect(screen.queryByTestId('arm-drag-reason')).toBeNull()
  })

  it('renders the display-only arm and shows the reason when dragging is disabled', () => {
    render(view({}, 'Arm held by the agent or Take control - Release to drag'))
    expect(mocks.armProps).toEqual([])
    expect(mocks.modelRenders.length).toBeGreaterThan(0)
    expect(screen.getByTestId('arm-drag-reason').textContent).toBe('Arm held by the agent or Take control - Release to drag')
  })

  it('keeps the interactive arm mounted while it waits for joint states, so it can report ready', () => {
    const onArmReady = vi.fn()
    render(view({}, 'Waiting for arm joint states', onArmReady))
    expect(mocks.armProps.length).toBeGreaterThan(0)
    expect(screen.getByTestId('arm-drag-reason').textContent).toBe('Waiting for arm joint states')
    ;(last(mocks.armProps).onReady as (ready: boolean) => void)(true)
    expect(onArmReady).toHaveBeenCalledWith(true)
  })
})

describe('ArmTab', () => {
  it('mounts ArmView inside the arm-view-slot', async () => {
    window.matchMedia = ((q: string) => ({ matches: false, media: q, addEventListener: vi.fn(), removeEventListener: vi.fn(), addListener: vi.fn(), removeListener: vi.fn(), dispatchEvent: vi.fn(), onchange: null })) as typeof window.matchMedia
    render(
      <ThemeProvider theme={theme}>
        <ArmTab tab={TAB} topicData={{}} publish={vi.fn()} />
      </ThemeProvider>,
    )
    const view = await screen.findByTestId('arm-view')
    expect(screen.getByTestId('arm-view-slot').contains(view)).toBe(true)
  })
})
