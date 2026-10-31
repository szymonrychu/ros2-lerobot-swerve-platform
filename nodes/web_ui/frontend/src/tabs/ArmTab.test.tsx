// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { act, cleanup, fireEvent, render, screen, waitFor, within } from '@testing-library/react'
import { ThemeProvider } from '@mui/material/styles'
import { theme } from '../theme'
import { ARM_COLORS } from '../map3d/palette'
import { TabConfig } from '../types'
import ArmTab from './ArmTab'
import { tabTopics } from '../hooks/subscription'
import { designViolations, SOURCES } from '../layout/designRules'

const servoMock = vi.hoisted(() => ({ groups: [] as unknown[], notConfigured: false }))
vi.mock('../calibrate/servoApi', async (orig) => ({
  ...(await orig<typeof import('../calibrate/servoApi')>()),
  useServoGroups: () => ({ groups: servoMock.groups, notConfigured: servoMock.notConfigured, error: null, loading: false }),
}))

/** Mock servo state: 'released' | 'enabled' | 'unknown' (no state) | 'none' (not configured). */
function setServo(id: string, kind: 'released' | 'enabled' | 'unknown' | 'none'): void {
  servoMock.notConfigured = kind === 'none'
  servoMock.groups = kind === 'none' ? [] : [{
    id, label: id[0].toUpperCase() + id.slice(1), neutral: true, age_s: 0.1, stale: false, calibration_stale: null,
    state: kind === 'unknown' ? null : { namespace: 'x', released: kind === 'released', joints: { a: kind === 'released' ? 0 : 1 }, stamp_ns: 1, neutral_enabled: true, neutral_capture: null, last_save: null },
  }]
}


const viewMock = vi.hoisted(() => ({
  props: null as null | {
    dragDisabledReason: string | null
    publish: (t: string, m: string, d: unknown) => void
    onArmReady: (r: boolean) => void
    pickMode: boolean
    onPick: (p: { x: number; y: number }) => void
    graspPreview: unknown
  },
}))
vi.mock('../arm/ArmView.tsx', () => ({
  default: (props: NonNullable<typeof viewMock.props>) => {
    viewMock.props = props
    return <div data-testid="arm-view">{props.dragDisabledReason}</div>
  },
}))

const FRONT = '/head/rgb/compressed'

const TAB = {
  id: 'arm',
  type: 'arm',
  label: 'Arm',
  head_camera_topic: FRONT,
  grasp_command_topic: '/grasp/command',
} as TabConfig

function setViewport(width: number): void {
  window.matchMedia = ((query: string) => {
    const min = /min-width:\s*([\d.]+)px/.exec(query)
    const max = /max-width:\s*([\d.]+)px/.exec(query)
    const matches = !((min && width < Number(min[1])) || (max && width > Number(max[1])))
    return { matches, media: query, addEventListener: vi.fn(), removeEventListener: vi.fn(), addListener: vi.fn(), removeListener: vi.fn(), dispatchEvent: vi.fn(), onchange: null }
  }) as typeof window.matchMedia
}

function renderTab(topicData: Record<string, unknown>, publish: () => void = vi.fn()) {
  return render(
    <ThemeProvider theme={theme}>
      <ArmTab tab={TAB} topicData={topicData} publish={publish} />
    </ThemeProvider>,
  )
}

afterEach(cleanup)

describe('ArmTab', () => {
  it('phone: stacks streams and the 3D view vertically', () => {
    setViewport(400)
    renderTab({ [FRONT]: { jpeg_b64: 'BBB' } })
    expect(screen.getByTestId('arm-layout').getAttribute('data-direction')).toBe('column')
    expect(screen.getByAltText('head camera').getAttribute('src')).toBe('data:image/jpeg;base64,BBB')
    expect(screen.getByTestId('arm-view-slot')).toBeTruthy()
    expect(screen.getByTestId('arm-jog-slot')).toBeTruthy()
    expect(screen.getByTestId('arm-actions-slot')).toBeTruthy()
  })

  it('puts Arm home and Set home in the actions slot', () => {
    setViewport(1400)
    renderTab({})
    const slot = screen.getByTestId('arm-actions-slot')
    expect(within(slot).getByRole('button', { name: 'Arm home' })).toBeTruthy()
    expect(within(slot).getByRole('button', { name: 'Set home' })).toBeTruthy()
  })

  it('disables Arm home and Set home with the joint names while the arm is released', () => {
    setViewport(1400)
    setServo('arm', 'released')
    renderTab({})
    const slot = screen.getByTestId('arm-actions-slot')
    expect((within(slot).getByRole('button', { name: 'Arm home' }) as HTMLButtonElement).disabled).toBe(true)
    expect((within(slot).getByRole('button', { name: 'Set home' }) as HTMLButtonElement).disabled).toBe(true)
    expect(within(slot).getByTestId('arm-actions-reason').textContent).toBe('Arm torque is off: a released (Calibrate page)')
  })

  it('desktop: places streams side by side with the 3D view slot', () => {
    setViewport(1400)
    renderTab({ [FRONT]: { jpeg_b64: 'BBB' } })
    expect(screen.getByTestId('arm-layout').getAttribute('data-direction')).toBe('row')
    expect(screen.getByTestId('arm-view-slot')).toBeTruthy()
  })

  it('shows the waiting message, never a placeholder image, for a stream without a frame', () => {
    setViewport(1400)
    renderTab({})
    expect(screen.getByText(`Waiting for ${FRONT}...`)).toBeTruthy()
    expect(screen.queryByAltText('head camera')).toBeNull()
  })

  it('only the head camera stream', () => {
    setViewport(1400)
    const legacy = { ...TAB, gripper_camera_topic: '/gripper' } as unknown as TabConfig
    render(
      <ThemeProvider theme={theme}>
        <ArmTab tab={legacy} topicData={{ '/gripper': { jpeg_b64: 'AAA' }, [FRONT]: { jpeg_b64: 'BBB' } }} publish={vi.fn()} />
      </ThemeProvider>,
    )
    expect(screen.queryByAltText('gripper camera')).toBeNull()
    expect(screen.queryByText('Waiting for /gripper...')).toBeNull()
    expect(screen.getAllByRole('img').map((i) => i.getAttribute('alt'))).toEqual(['head camera'])
    expect(tabTopics(legacy)).not.toContain('/gripper')
    expect(tabTopics(legacy)).toContain(FRONT)
  })

  it.each(['tabs/ArmTab.tsx', 'arm/ArmView.tsx', 'components3d/InteractiveArm.tsx'])('%s has no design violations', (path) => {
    expect(SOURCES[path], path).toBeTruthy()
    expect(designViolations(path, SOURCES[path])).toEqual([])
  })

  it('ArmView switches camera mode with a ToolGroup of ToolButtons, colours from the palette', () => {
    const src = SOURCES['arm/ArmView.tsx']
    expect(src).toContain('ToolGroup')
    expect(src).toContain('ToolButton')
    expect(src).not.toContain('ToggleButton')
  })

  it('arm colours in the palette are achromatic', () => {
    const channels = (n: number): number[] => [(n >> 16) & 255, (n >> 8) & 255, n & 255]
    const all = [...Object.values(ARM_COLORS.handle), ...Object.values(ARM_COLORS.endEffector), ...Object.values(ARM_COLORS.axis), ARM_COLORS.ghost, ARM_COLORS.mesh]
    for (const c of all) expect(new Set(channels(c)).size).toBe(1)
  })
})

describe('ArmTab released group', () => {
  const BANNER = 'Arm torque is off - controls disabled (Calibrate page)'
  const HELD = { '/filter/active_source': { data: 'autonomy' } }
  const JOGS = ['Up', 'Pan left', 'Gripper open']

  it('shows the banner, disables jog, Home and Take control, keeps Release enabled', () => {
    setViewport(1400)
    setServo('arm', 'released')
    renderTab(HELD)
    expect(screen.getByText(BANNER)).toBeTruthy()
    for (const name of [...JOGS, 'Home', 'Take control']) {
      expect((screen.getByRole('button', { name }) as HTMLButtonElement).disabled, name).toBe(true)
    }
    expect((screen.getByRole('button', { name: 'Release' }) as HTMLButtonElement).disabled).toBe(false)
  })

  it.each(['enabled', 'unknown', 'none'] as const)('no banner and controls enabled when %s', (kind) => {
    setViewport(1400)
    setServo('arm', kind)
    renderTab(HELD)
    expect(screen.queryByText(BANNER)).toBeNull()
    for (const name of [...JOGS, 'Home', 'Take control', 'Release']) {
      expect((screen.getByRole('button', { name }) as HTMLButtonElement).disabled, name).toBe(false)
    }
  })
})

describe('ArmTab drag reason', () => {
  const SOURCE = { '/filter/active_source': { data: 'filter' } }
  const reason = (): string | null => viewMock.props?.dragDisabledReason ?? null

  it('waits for joint states until the arm reports ready, then allows dragging', async () => {
    setViewport(1400)
    setServo('arm', 'enabled')
    renderTab(SOURCE)
    expect(reason()).toBe('Waiting for arm joint states')
    act(() => viewMock.props?.onArmReady(true))
    expect(reason()).toBeNull()
  })

  it('names the lease while the agent or Take control holds the arm', () => {
    setViewport(1400)
    setServo('arm', 'enabled')
    renderTab({ '/filter/active_source': { data: 'autonomy' } })
    act(() => viewMock.props?.onArmReady(true))
    expect(reason()).toBe('Arm held by the agent or Take control - Release to drag')
  })

  it('lists the released joints', () => {
    setViewport(1400)
    setServo('arm', 'released')
    renderTab(SOURCE)
    act(() => viewMock.props?.onArmReady(true))
    expect(reason()).toBe('Arm torque is off: a released (Calibrate page)')
  })

  it('never forwards a drag publish while released, forwards it otherwise', () => {
    setViewport(1400)
    const publish = vi.fn()
    setServo('arm', 'released')
    const { unmount } = renderTab(SOURCE, publish)
    viewMock.props?.publish('/cmd', 'sensor_msgs/JointState', {})
    expect(publish).not.toHaveBeenCalled()
    unmount()
    setServo('arm', 'enabled')
    renderTab(SOURCE, publish)
    viewMock.props?.publish('/cmd', 'sensor_msgs/JointState', {})
    expect(publish).toHaveBeenCalledTimes(1)
  })
})

describe('ArmTab grasp', () => {
  const fetchMock = vi.fn()
  const STREAM = '/web_ui/grasp_result'

  beforeEach(() => {
    fetchMock.mockReset()
    fetchMock.mockResolvedValue({ status: 200, json: async () => ({ ok: true, request_id: 'r', action: 'plan', result: { outcome: 'planned', reasons: [], plan: { strategy: 'scoop', feasible: true, waypoints: [] } } }) })
    vi.stubGlobal('fetch', fetchMock)
    setViewport(1400)
    setServo('arm', 'enabled')
  })
  afterEach(() => {
    vi.unstubAllGlobals()
  })

  const chooseFirstStrategy = () => {
    fireEvent.click(screen.getByRole('button', { name: 'Grasp' }))
    fireEvent.click(screen.getAllByRole('menuitem')[0])
  }

  it('choosing a strategy opens the grasp panel and arms the pick in the arm view', () => {
    renderTab({})
    expect(screen.queryByLabelText('Grasp panel')).toBeNull()
    chooseFirstStrategy()
    expect(screen.getByLabelText('Grasp panel')).toBeTruthy()
    expect(viewMock.props?.pickMode).toBe(true)
    expect(screen.getByTestId('arm-grasp-hint').textContent).toBe('Click the object in the arm view')
  })

  it('a pick plans at the base_link point and posts to /api/grasp for the arm tab', async () => {
    renderTab({})
    chooseFirstStrategy()
    act(() => viewMock.props?.onPick({ x: 0.3, y: 0.1 }))
    await waitFor(() => expect(fetchMock).toHaveBeenCalled())
    const [url, init] = fetchMock.mock.calls[0]
    expect(url).toBe('/api/grasp?tab=arm')
    const body = JSON.parse(init.body)
    expect(body.action).toBe('plan')
    expect(body.object.frame).toBe('base_link')
    expect(body.object.x).toBeCloseTo(0.3)
    expect(body.object.y).toBeCloseTo(0.1)
    expect(viewMock.props?.pickMode).toBe(false)
  })

  it('Esc leaves the pick mode', () => {
    renderTab({})
    chooseFirstStrategy()
    fireEvent.keyDown(window, { key: 'Escape' })
    expect(viewMock.props?.pickMode).toBe(false)
  })

  it('a released arm closes the grasp flow', () => {
    const view = renderTab({})
    chooseFirstStrategy()
    expect(screen.getByLabelText('Grasp panel')).toBeTruthy()
    setServo('arm', 'released')
    view.rerender(
      <ThemeProvider theme={theme}>
        <ArmTab tab={TAB} topicData={{}} publish={vi.fn()} />
      </ThemeProvider>,
    )
    expect(screen.queryByLabelText('Grasp panel')).toBeNull()
    expect(viewMock.props?.pickMode).toBe(false)
  })

  it('STOP GRASP shows only while a grasp executes and posts the stop for the arm tab', async () => {
    const view = renderTab({})
    expect(screen.queryByRole('button', { name: 'STOP GRASP' })).toBeNull()
    view.unmount()
    renderTab({ [STREAM]: { request_id: 'r', action: 'execute', state: 'running' } })
    fireEvent.click(screen.getByRole('button', { name: 'STOP GRASP' }))
    await waitFor(() => expect(fetchMock).toHaveBeenCalledWith('/api/grasp/stop?tab=arm', { method: 'POST' }))
  })

  it('the grasp button is disabled with the in-flight reason while a grasp executes', () => {
    renderTab({ [STREAM]: { request_id: 'r', action: 'execute', state: 'running' } })
    expect((screen.getByRole('button', { name: 'Grasp' }) as HTMLButtonElement).disabled).toBe(true)
    expect(screen.getByTestId('arm-actions-reason').textContent).toBe('Arm action in flight')
  })

  it('has no Grasp button when the tab has no grasp topic', () => {
    render(
      <ThemeProvider theme={theme}>
        <ArmTab tab={{ ...TAB, grasp_command_topic: undefined }} topicData={{}} publish={vi.fn()} />
      </ThemeProvider>,
    )
    expect(screen.queryByRole('button', { name: 'Grasp' })).toBeNull()
  })
})
