// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import { act, cleanup, render, screen } from '@testing-library/react'
import { ThemeProvider } from '@mui/material/styles'
import { theme } from '../theme'
import { ARM_COLORS } from '../map3d/palette'
import { TabConfig } from '../types'
import ArmTab from './ArmTab'
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
  props: null as null | { dragDisabledReason: string | null; publish: (t: string, m: string, d: unknown) => void; onArmReady: (r: boolean) => void },
}))
vi.mock('../arm/ArmView.tsx', () => ({
  default: (props: NonNullable<typeof viewMock.props>) => {
    viewMock.props = props
    return <div data-testid="arm-view">{props.dragDisabledReason}</div>
  },
}))

const GRIPPER = '/gripper/image_raw'
const FRONT = '/head_camera/image_raw/compressed'

const TAB = {
  id: 'arm',
  type: 'arm',
  label: 'Arm',
  gripper_camera_topic: GRIPPER,
  head_camera_topic: FRONT,
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
    renderTab({ [GRIPPER]: { jpeg_b64: 'AAA' }, [FRONT]: { jpeg_b64: 'BBB' } })
    expect(screen.getByTestId('arm-layout').getAttribute('data-direction')).toBe('column')
    expect(screen.getByAltText('gripper camera').getAttribute('src')).toBe('data:image/jpeg;base64,AAA')
    expect(screen.getByAltText('head camera').getAttribute('src')).toBe('data:image/jpeg;base64,BBB')
    expect(screen.getByTestId('arm-view-slot')).toBeTruthy()
    expect(screen.getByTestId('arm-jog-slot')).toBeTruthy()
  })

  it('desktop: places streams side by side with the 3D view slot', () => {
    setViewport(1400)
    renderTab({ [GRIPPER]: { jpeg_b64: 'AAA' }, [FRONT]: { jpeg_b64: 'BBB' } })
    expect(screen.getByTestId('arm-layout').getAttribute('data-direction')).toBe('row')
    expect(screen.getByTestId('arm-view-slot')).toBeTruthy()
  })

  it('shows the waiting message, never a placeholder image, for a stream without a frame', () => {
    setViewport(1400)
    renderTab({ [FRONT]: { jpeg_b64: 'BBB' } })
    expect(screen.getByText(`Waiting for ${GRIPPER}...`)).toBeTruthy()
    expect(screen.queryByAltText('gripper camera')).toBeNull()
    expect(screen.getByAltText('head camera')).toBeTruthy()
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
