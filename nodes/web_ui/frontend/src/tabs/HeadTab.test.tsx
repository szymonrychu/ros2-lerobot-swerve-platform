// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import { act, cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react'
import { ThemeProvider } from '@mui/material/styles'
import { theme } from '../theme'
import { TabConfig } from '../types'
import HeadTab from './HeadTab'
import { requestZoom } from '../camera/zoomApi'

vi.mock('../camera/zoomApi', async (orig) => ({
  ...(await orig<typeof import('../camera/zoomApi')>()),
  requestZoom: vi.fn(),
}))

const RGB = '/head_camera/image_raw/compressed'
const DEPTH = '/tof/depth/image_raw'
const AMP = '/tof/amplitude/image_raw'

const TAB = {
  id: 'head',
  type: 'head',
  label: 'Head',
  rgb_camera_topic: RGB,
  tof_depth_topic: DEPTH,
  tof_amplitude_topic: AMP,
  tof_range_m: 4,
} as TabConfig

function setViewport(width: number): void {
  window.matchMedia = ((query: string) => {
    const min = /min-width:\s*([\d.]+)px/.exec(query)
    const max = /max-width:\s*([\d.]+)px/.exec(query)
    const matches = !((min && width < Number(min[1])) || (max && width > Number(max[1])))
    return { matches, media: query, addEventListener: vi.fn(), removeEventListener: vi.fn(), addListener: vi.fn(), removeListener: vi.fn(), dispatchEvent: vi.fn(), onchange: null }
  }) as typeof window.matchMedia
}

function renderTab(topicData: Record<string, unknown>, publish = vi.fn(), tab: TabConfig = TAB) {
  return render(
    <ThemeProvider theme={theme}>
      <HeadTab tab={tab} topicData={topicData} publish={publish} />
    </ThemeProvider>,
  )
}

const FRAMES = {
  [RGB]: { jpeg_b64: 'RRR' },
  [DEPTH]: { depth_preview_b64: 'DDD' },
  [AMP]: { jpeg_b64: 'AAA' },
}

afterEach(cleanup)

describe('HeadTab', () => {
  it('renders the RGB stream and the depth preview', () => {
    setViewport(1400)
    renderTab(FRAMES)
    expect(screen.getByAltText('head camera').getAttribute('src')).toBe('data:image/jpeg;base64,RRR')
    expect(screen.getByAltText('depth').getAttribute('src')).toBe('data:image/jpeg;base64,DDD')
  })

  it('shows the amplitude preview only after the toggle, off by default', () => {
    setViewport(1400)
    renderTab(FRAMES)
    const toggle = screen.getByRole('checkbox', { name: 'amplitude' }) as HTMLInputElement
    expect(toggle.checked).toBe(false)
    expect(screen.queryByAltText('ToF amplitude')).toBeNull()
    expect(screen.queryByText(`Waiting for ${AMP}...`)).toBeNull()
    fireEvent.click(toggle)
    expect(screen.getByAltText('ToF amplitude').getAttribute('src')).toBe('data:image/jpeg;base64,AAA')
  })

  it('has no amplitude switch when the tab has no amplitude topic (depth only)', () => {
    setViewport(1400)
    const { tof_amplitude_topic: _unused, ...depthOnly } = TAB
    renderTab(FRAMES, vi.fn(), depthOnly as TabConfig)
    expect(screen.queryByRole('checkbox', { name: 'amplitude' })).toBeNull()
    expect(screen.queryByText(/Waiting for undefined/)).toBeNull()
    expect(screen.getByAltText('depth').getAttribute('src')).toBe('data:image/jpeg;base64,DDD')
  })

  it('shows Waiting for <topic> and no image for streams without a message', () => {
    setViewport(1400)
    renderTab({})
    fireEvent.click(screen.getByRole('checkbox', { name: 'amplitude' }))
    for (const topic of [RGB, DEPTH, AMP]) expect(screen.getByText(`Waiting for ${topic}...`)).toBeTruthy()
    expect(screen.queryAllByRole('img')).toEqual([])
  })

  describe('pan/tilt controls', () => {
    const CMD = '/head/joint_commands'
    const STATES = '/head/joint_states'
    const HEAD_TAB = {
      ...TAB,
      head_command_topic: CMD,
      head_joint_states_topic: STATES,
      head_pan_range_deg: [-90, 90],
      head_tilt_range_deg: [-30, 45],
      head_home_deg: [0, 30],
    } as TabConfig
    const SAMPLE = { [STATES]: { name: ['head_pan', 'head_tilt'], position: [Math.PI / 6, -Math.PI / 12] } }
    const DEG = Math.PI / 180

    function sliderInputs(): HTMLInputElement[] {
      return [screen.getByLabelText('pan'), screen.getByLabelText('tilt')] as HTMLInputElement[]
    }

    function dragPan(clientXs: number[]): void {
      const root = screen.getByTestId('head-pan-tilt').querySelector('.MuiSlider-root') as HTMLElement
      root.getBoundingClientRect = () => ({ left: 0, top: 0, width: 180, height: 10, right: 180, bottom: 10, x: 0, y: 0 }) as DOMRect
      fireEvent.mouseDown(root, { clientX: clientXs[0], clientY: 5 })
      clientXs.slice(1).forEach((x) => fireEvent.mouseMove(document, { clientX: x, clientY: 5, buttons: 1 }))
    }

    afterEach(() => {
      vi.useRealTimers()
    })

    it('disables both sliders and Home with a waiting line before the first sample', () => {
      setViewport(1400)
      renderTab(FRAMES, vi.fn(), HEAD_TAB)
      sliderInputs().forEach((i) => expect(i.disabled).toBe(true))
      expect(screen.getByText('waiting for /head/joint_states')).toBeTruthy()
    })

    it('enables the sliders with the measured degrees after the first sample', () => {
      setViewport(1400)
      renderTab({ ...FRAMES, ...SAMPLE }, vi.fn(), HEAD_TAB)
      const [pan, tilt] = sliderInputs()
      expect(pan.disabled).toBe(false)
      expect(Number(pan.value)).toBeCloseTo(30, 0)
      expect(Number(tilt.value)).toBeCloseTo(-15, 0)
      expect(screen.queryByText(/waiting for \/head\/joint_states/)).toBeNull()
      expect(screen.getByTestId('head-measured').textContent).toContain('30')
      expect(screen.getByTestId('head-measured').textContent).toContain('-15')
    })

    function release(): void {
      fireEvent.mouseUp(document, { clientX: 150, clientY: 5 })
    }

    it('release publishes radians with both joint names, once more after the drag', () => {
      setViewport(1400)
      vi.useFakeTimers()
      const publish = vi.fn()
      renderTab({ ...FRAMES, ...SAMPLE }, publish, HEAD_TAB)
      dragPan([90, 150])
      const beforeRelease = publish.mock.calls.length
      release()
      expect(publish.mock.calls.length).toBe(beforeRelease + 1)
      const last = publish.mock.calls[publish.mock.calls.length - 1]
      expect(last[0]).toBe(CMD)
      expect(last[1]).toBe('sensor_msgs/JointState')
      expect(last[2].name).toEqual(['head_pan', 'head_tilt'])
      expect(last[2].position[0]).toBeCloseTo(60 * DEG, 5)
      expect(last[2].position[1]).toBeCloseTo(-15 * DEG, 5)
    })

    it('drag publishes no faster than 5 Hz', () => {
      setViewport(1400)
      vi.useFakeTimers()
      const publish = vi.fn()
      renderTab({ ...FRAMES, ...SAMPLE }, publish, HEAD_TAB)
      dragPan([90, 100, 110, 120, 130])
      expect(publish.mock.calls.length).toBe(1)
      act(() => {
        vi.advanceTimersByTime(100)
      })
      dragPan([140])
      expect(publish.mock.calls.length).toBe(1)
      act(() => {
        vi.advanceTimersByTime(250)
      })
      expect(publish.mock.calls.length).toBe(2)
      release()
      expect(publish.mock.calls.length).toBe(3)
    })

    it('after release the other joint keeps its commanded value, clamped to the range', () => {
      setViewport(1400)
      vi.useFakeTimers()
      const publish = vi.fn()
      const over = { [STATES]: { name: ['head_pan', 'head_tilt'], position: [90.0002 * DEG, -15 * DEG] } }
      renderTab({ ...FRAMES, ...over }, publish, HEAD_TAB)
      fireEvent.click(screen.getByRole('button', { name: 'Home' }))
      publish.mockClear()
      // tilt commanded to 30 and still travelling (measured -15): a pan drag must not send the stale -15
      vi.advanceTimersByTime(300)
      dragPan([90])
      const msg = publish.mock.calls[0][2]
      expect(msg.position[1]).toBeCloseTo(30 * DEG, 5)
      expect(msg.position[0]).toBeLessThanOrEqual(90 * DEG + 1e-9)
    })

    it('a stale command (hold time passed, measurement never arrived) is replaced by the measured value', () => {
      setViewport(1400)
      vi.useFakeTimers()
      const publish = vi.fn()
      renderTab({ ...FRAMES, ...SAMPLE }, publish, HEAD_TAB)
      fireEvent.click(screen.getByRole('button', { name: 'Home' }))
      publish.mockClear()
      vi.advanceTimersByTime(4000)
      dragPan([90])
      expect(publish.mock.calls[0][2].position[1]).toBeCloseTo(-15 * DEG, 5)
    })

    it('inside the hold time the commanded value is still sent for the other joint', () => {
      setViewport(1400)
      vi.useFakeTimers()
      const publish = vi.fn()
      renderTab({ ...FRAMES, ...SAMPLE }, publish, HEAD_TAB)
      fireEvent.click(screen.getByRole('button', { name: 'Home' }))
      publish.mockClear()
      vi.advanceTimersByTime(1000)
      dragPan([90])
      expect(publish.mock.calls[0][2].position[1]).toBeCloseTo(30 * DEG, 5)
    })

    it('once the measurement arrived, a later external move is followed', () => {
      setViewport(1400)
      vi.useFakeTimers()
      const publish = vi.fn()
      const at = (tiltDeg: number) => ({ ...FRAMES, [STATES]: { name: ['head_pan', 'head_tilt'], position: [30 * DEG, tiltDeg * DEG] } })
      const view = (data: Record<string, unknown>) => (
        <ThemeProvider theme={theme}>
          <HeadTab tab={HEAD_TAB} topicData={data} publish={publish} />
        </ThemeProvider>
      )
      const { rerender } = render(view(at(-15)))
      fireEvent.click(screen.getByRole('button', { name: 'Home' }))
      rerender(view(at(30)))
      rerender(view(at(10)))
      publish.mockClear()
      vi.advanceTimersByTime(300)
      dragPan([90])
      expect(publish.mock.calls[0][2].position[1]).toBeCloseTo(10 * DEG, 5)
    })

    it('Home publishes the home pose', () => {
      setViewport(1400)
      const publish = vi.fn()
      renderTab({ ...FRAMES, ...SAMPLE }, publish, HEAD_TAB)
      fireEvent.click(screen.getByRole('button', { name: 'Home' }))
      expect(publish).toHaveBeenCalledTimes(1)
      const msg = publish.mock.calls[0][2]
      expect(publish.mock.calls[0][0]).toBe(CMD)
      expect(msg.name).toEqual(['head_pan', 'head_tilt'])
      expect(msg.position[0]).toBeCloseTo(0, 5)
      expect(msg.position[1]).toBeCloseTo(30 * DEG, 5)
    })

    it('shows no controls without head_command_topic and publishes nothing', () => {
      setViewport(1400)
      const publish = vi.fn()
      renderTab({ ...FRAMES, ...SAMPLE }, publish)
      expect(screen.queryByTestId('head-pan-tilt')).toBeNull()
      expect(screen.queryByRole('button', { name: 'Home' })).toBeNull()
      expect(publish).not.toHaveBeenCalled()
    })
  })

  it('phone: stacks the previews vertically', () => {
    setViewport(400)
    renderTab(FRAMES)
    expect(screen.getByTestId('head-layout').getAttribute('data-direction')).toBe('column')
  })

  it.each([700, 1400])('width %i: places the previews side by side', (width) => {
    setViewport(width)
    renderTab(FRAMES)
    expect(screen.getByTestId('head-layout').getAttribute('data-direction')).toBe('row')
  })

  it('renders no raw/fused toggle', () => {
    setViewport(1400)
    renderTab(FRAMES)
    expect(screen.queryByRole('button', { name: 'fused' })).toBeNull()
    expect(screen.queryByRole('button', { name: 'raw' })).toBeNull()
    expect(screen.getByAltText('depth').getAttribute('src')).toBe('data:image/jpeg;base64,DDD')
  })

  describe('click to zoom', () => {
    const ZOOM_TAB = { ...TAB, zoom_command_topic: '/head_camera/zoom/command' } as TabConfig
    const mockZoom = vi.mocked(requestZoom)

    function clickLive(): void {
      const img = screen.getByAltText('head camera') as HTMLImageElement
      Object.defineProperty(img, 'naturalWidth', { value: 1600, configurable: true })
      Object.defineProperty(img, 'naturalHeight', { value: 900, configurable: true })
      img.getBoundingClientRect = () => ({ left: 0, top: 0, width: 1600, height: 900 }) as DOMRect
      fireEvent.click(img, { clientX: 400, clientY: 450 })
    }

    afterEach(() => {
      mockZoom.mockReset()
    })

    it('shows a factor picker defaulting to 4 only with zoom topics', () => {
      setViewport(1400)
      renderTab(FRAMES, vi.fn(), ZOOM_TAB)
      const picker = screen.getByRole('radiogroup', { name: 'zoom factor' })
      expect(screen.getByRole('radio', { name: '4x' }).getAttribute('aria-checked')).toBe('true')
      expect(picker).toBeTruthy()
      cleanup()
      renderTab(FRAMES)
      expect(screen.queryByRole('radiogroup', { name: 'zoom factor' })).toBeNull()
      mockZoom.mockResolvedValue({ status: 'ok', jpeg_b64: 'ZZZ' })
      clickLive()
      expect(mockZoom).not.toHaveBeenCalled()
    })

    it('requests the clicked point with the chosen factor, ignoring clicks while pending', async () => {
      setViewport(1400)
      let resolve: (v: { status: string; jpeg_b64: string; zoom: number }) => void = () => {}
      mockZoom.mockReturnValue(new Promise((r) => { resolve = r }))
      renderTab(FRAMES, vi.fn(), ZOOM_TAB)
      fireEvent.click(screen.getByRole('radio', { name: '8x' }))
      clickLive()
      clickLive()
      expect(mockZoom).toHaveBeenCalledTimes(1)
      expect(mockZoom).toHaveBeenCalledWith('head', 0.25, 0.5, 8)
      expect(screen.getByText('Zooming...')).toBeTruthy()
      await act(async () => resolve({ status: 'ok', jpeg_b64: 'ZZZ', zoom: 8 }))
    })

    it('shows the crop with factor then returns to live', async () => {
      setViewport(1400)
      mockZoom.mockResolvedValue({ status: 'ok', jpeg_b64: 'ZZZ' })
      renderTab(FRAMES, vi.fn(), ZOOM_TAB)
      clickLive()
      await waitFor(() => expect(screen.getByAltText('head camera zoom').getAttribute('src')).toBe('data:image/jpeg;base64,ZZZ'))
      expect(screen.getByText(/4x/, { selector: '[data-testid="zoom-info"]' })).toBeTruthy()
      expect(screen.queryByText(/focus not confirmed/)).toBeNull()
      fireEvent.click(screen.getByRole('button', { name: 'Back to live' }))
      expect(screen.queryByAltText('head camera zoom')).toBeNull()
      expect(screen.getByAltText('head camera').getAttribute('src')).toBe('data:image/jpeg;base64,RRR')
    })

    it('notes unconfirmed focus on af_failed', async () => {
      setViewport(1400)
      mockZoom.mockResolvedValue({ status: 'af_failed', jpeg_b64: 'ZZZ' })
      renderTab(FRAMES, vi.fn(), ZOOM_TAB)
      clickLive()
      await waitFor(() => expect(screen.getByAltText('head camera zoom')).toBeTruthy())
      expect(screen.getByText(/focus not confirmed/)).toBeTruthy()
    })

    it.each([
      ['Zoom failed (HTTP 409): busy', 'Camera busy'],
      ['Zoom failed (HTTP 504): no answer', 'Zoom timed out'],
      ['Zoom failed (HTTP 502, timeout): no still frame within 5 s', 'Zoom timed out'],
      ['Zoom failed (HTTP 502, camera_error): sensor lost', 'Camera error'],
    ])('shows one short message on %s and keeps the live view', async (err, text) => {
      setViewport(1400)
      mockZoom.mockRejectedValue(new Error(err))
      renderTab(FRAMES, vi.fn(), ZOOM_TAB)
      clickLive()
      await waitFor(() => expect(screen.getByText(text)).toBeTruthy())
      expect(screen.queryByAltText('head camera zoom')).toBeNull()
      expect(screen.getByAltText('head camera').getAttribute('src')).toBe('data:image/jpeg;base64,RRR')
    })
  })
})
