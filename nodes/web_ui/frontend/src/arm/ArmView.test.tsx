// @vitest-environment jsdom
import { cleanup, fireEvent, render, screen } from '@testing-library/react'
import { forwardRef, type ReactNode } from 'react'
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import * as THREE from 'three'
import { ThemeProvider } from '@mui/material/styles'
import { theme } from '../theme'
import type { TabConfig } from '../types'
import ArmTab from '../tabs/ArmTab'
import ArmView from './ArmView.tsx'
import { ARM_CAMERA_MODES } from './armView.ts'

const JOINTS = '/arm/joint_states'
const canvasEl = vi.hoisted(() => {
  const el = document.createElement('canvas')
  el.getBoundingClientRect = () => ({ left: 0, top: 0, width: 200, height: 200, right: 200, bottom: 200, x: 0, y: 0, toJSON: () => ({}) })
  return el
})
const mocks = vi.hoisted(() => ({
  previews: [] as unknown[],
  invalidate: vi.fn(),
  orbitEnabled: [] as boolean[],
  frameloop: [] as string[],
  armProps: [] as Record<string, unknown>[],
  modelRenders: [] as number[],
}))

/** Camera 2 m above the arm root looking straight down. */
const topDownCamera = new THREE.PerspectiveCamera(50, 1, 0.01, 50)
topDownCamera.position.set(0, 2, 0)
topDownCamera.up.set(0, 0, -1)
topDownCamera.lookAt(0, 0, 0)
topDownCamera.updateMatrixWorld()

vi.mock('@react-three/fiber', () => ({
  Canvas: ({ children, frameloop }: { children: ReactNode; frameloop?: string }) => {
    mocks.frameloop.push(frameloop ?? '')
    return <div data-testid="canvas">{children}</div>
  },
  useThree: () => ({ invalidate: mocks.invalidate, camera: topDownCamera, gl: { domElement: canvasEl } }),
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
vi.mock('../grasp/GraspPreview', () => ({
  GraspPreview: ({ data }: { data: unknown }) => {
    mocks.previews.push(data)
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

interface ViewOptions {
  reason?: string | null
  onArmReady?: (ready: boolean) => void
  pickMode?: boolean
  onPick?: (point: { x: number; y: number }) => void
  graspPreview?: { box: null; waypoints: [] } | null
}

function view(topicData: Record<string, unknown>, reason: string | null = null, onArmReady: (ready: boolean) => void = vi.fn(), extra: ViewOptions = {}) {
  return (
    <ThemeProvider theme={theme}>
      <ArmView
        tab={TAB}
        topicData={topicData}
        publish={vi.fn()}
        dragDisabledReason={reason}
        onArmReady={onArmReady}
        pickMode={extra.pickMode ?? false}
        onPick={extra.onPick ?? vi.fn()}
        graspPreview={extra.graspPreview ?? null}
      />
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
  mocks.previews.length = 0
})
afterEach(cleanup)

describe('ArmView', () => {
  it('renders a demand-frameloop canvas with the three mode buttons', () => {
    render(view({}))
    expect(last(mocks.frameloop)).toBe('demand')
    for (const name of ['Side L', 'Side R', 'Above']) {
      expect(screen.getByRole('button', { name })).toBeTruthy()
    }
  })

  it('no gripper camera mode', () => {
    render(view({}))
    expect(screen.queryByRole('button', { name: 'Gripper' })).toBeNull()
    expect(ARM_CAMERA_MODES).toEqual(['side-left', 'side-right', 'above'])
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

describe('ArmView grasp pick', () => {
  const click = (down: [number, number], up: [number, number]) => {
    canvasEl.dispatchEvent(new MouseEvent('pointerdown', { clientX: down[0], clientY: down[1], button: 0, bubbles: true }))
    window.dispatchEvent(new MouseEvent('pointerup', { clientX: up[0], clientY: up[1], button: 0 }))
  }

  it('a click that does not move calls onPick with the base_link floor point', () => {
    const onPick = vi.fn()
    render(view({}, null, vi.fn(), { pickMode: true, onPick }))
    click([100, 100], [100, 100])
    expect(onPick).toHaveBeenCalledTimes(1)
    const p = onPick.mock.calls[0][0]
    expect(p.x).toBeCloseTo(0.0592)
    expect(p.y).toBeCloseTo(-0.05)
  })

  it('a press that moves (orbit) does not pick', () => {
    const onPick = vi.fn()
    render(view({}, null, vi.fn(), { pickMode: true, onPick }))
    click([100, 100], [160, 100])
    expect(onPick).not.toHaveBeenCalled()
  })

  it('does nothing outside pick mode', () => {
    const onPick = vi.fn()
    render(view({}, null, vi.fn(), { pickMode: false, onPick }))
    click([100, 100], [100, 100])
    expect(onPick).not.toHaveBeenCalled()
  })

  it('the arm is display-only in pick mode', () => {
    render(view({}, null, vi.fn(), { pickMode: true }))
    expect(mocks.armProps).toEqual([])
    expect(mocks.modelRenders.length).toBeGreaterThan(0)
  })

  it('draws the grasp preview when one is set', () => {
    const preview = { box: null, waypoints: [] as [] }
    render(view({}, null, vi.fn(), { graspPreview: preview }))
    expect(mocks.previews).toContain(preview)
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
