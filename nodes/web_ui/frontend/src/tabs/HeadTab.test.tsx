// @vitest-environment jsdom
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { act, cleanup, fireEvent, render, screen, waitFor, within } from '@testing-library/react'
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

describe('RGB-D view', () => {
  const RGBD_RGB = '/head_rgbd/rgb/image_raw/compressed'
  const RGBD_DEPTH = '/head_rgbd/depth/image_raw'
  const RGBD_TAB = { ...TAB, rgbd_rgb_topic: RGBD_RGB, rgbd_depth_topic: RGBD_DEPTH, tof_range_m: 4 } as TabConfig

  function u16b64(values: number[]): string {
    const bytes = new Uint8Array(values.length * 2)
    const view = new DataView(bytes.buffer)
    values.forEach((v, i) => view.setUint16(i * 2, v, true))
    let s = ''
    bytes.forEach((b) => (s += String.fromCharCode(b)))
    return btoa(s)
  }

  // 2x2 registered depth: [no depth, 1.50 m / 4.00 m, 0.25 m]
  const depthAt = (stamp: number) => ({ depth_raw_b64: u16b64([0, 1500, 4000, 250]), width: 2, height: 2, stamp, frame_id: 'f' })
  const rgbAt = (stamp: number) => ({ jpeg_b64: `RGB${stamp}`, stamp })
  const frames = (rgbStamp: number, depthStamp: number) => ({ ...FRAMES, [RGBD_RGB]: rgbAt(rgbStamp), [RGBD_DEPTH]: depthAt(depthStamp) })

  const putImageData = vi.fn()
  beforeEach(() => {
    putImageData.mockClear()
    vi.spyOn(HTMLCanvasElement.prototype, 'getContext').mockImplementation((() => ({
      createImageData: (w: number, h: number) => ({ data: new Uint8ClampedArray(w * h * 4), width: w, height: h }),
      putImageData,
    })) as unknown as typeof HTMLCanvasElement.prototype.getContext)
    // a 100x100 box over the 2x2 depth: one pixel per 50x50 quadrant
    vi.spyOn(Element.prototype, 'getBoundingClientRect').mockReturnValue({ left: 0, top: 0, width: 100, height: 100, right: 100, bottom: 100, x: 0, y: 0 } as DOMRect)
  })
  afterEach(() => {
    vi.restoreAllMocks()
  })

  function wrapped(data: Record<string, unknown>, onRgbdWanted?: (wanted: boolean) => void) {
    return (
      <ThemeProvider theme={theme}>
        <HeadTab tab={RGBD_TAB} topicData={data} publish={vi.fn()} onRgbdWanted={onRgbdWanted} />
      </ThemeProvider>
    )
  }

  function openRgbd(data: Record<string, unknown>, onRgbdWanted = vi.fn()) {
    setViewport(1400)
    const utils = render(wrapped(data, onRgbdWanted))
    fireEvent.click(screen.getByRole('radio', { name: 'RGB-D' }))
    return { ...utils, onRgbdWanted }
  }

  it('has no view switch without both RGB-D topics, and the existing views are unchanged', () => {
    setViewport(1400)
    renderTab(FRAMES)
    expect(screen.queryByRole('radio', { name: 'RGB-D' })).toBeNull()
    expect(screen.getByAltText('depth').getAttribute('src')).toBe('data:image/jpeg;base64,DDD')
  })

  it('keeps the preview view by default and asks for the RGB-D topics only once the view is selected', () => {
    setViewport(1400)
    const onRgbdWanted = vi.fn()
    render(wrapped(frames(5, 5), onRgbdWanted))
    expect(screen.getByAltText('head camera')).toBeTruthy()
    expect(onRgbdWanted).toHaveBeenLastCalledWith(false)
    expect(onRgbdWanted).not.toHaveBeenCalledWith(true)
    fireEvent.click(screen.getByRole('radio', { name: 'RGB-D' }))
    expect(onRgbdWanted).toHaveBeenLastCalledWith(true)
    fireEvent.click(screen.getByRole('radio', { name: 'Preview' }))
    expect(onRgbdWanted).toHaveBeenLastCalledWith(false)
    expect(screen.getByAltText('head camera')).toBeTruthy()
  })

  it('stops asking for the topics when the tab unmounts', () => {
    const { unmount, onRgbdWanted } = openRgbd(frames(5, 5))
    expect(onRgbdWanted).toHaveBeenLastCalledWith(true)
    unmount()
    expect(onRgbdWanted).toHaveBeenLastCalledWith(false)
  })

  it('shows the RGB image under the depth overlay when the stamps are equal', () => {
    openRgbd(frames(5, 5))
    expect(screen.getByAltText('head RGB-D').getAttribute('src')).toBe('data:image/jpeg;base64,RGB5')
    const overlay = screen.getByTestId('rgbd-overlay') as HTMLCanvasElement
    expect([overlay.width, overlay.height]).toEqual([2, 2])
    const image = putImageData.mock.calls.map((c) => c[0] as { data: Uint8ClampedArray; width: number }).find((i) => i.width === 2)!
    expect(image.data[3]).toBe(0) // depth 0 is transparent
    expect(image.data[7]).toBe(255)
  })

  it('shows the waiting message, no image and no overlay when the stamps differ', () => {
    openRgbd(frames(5, 6))
    expect(screen.getByText(new RegExp(`Waiting for ${RGBD_RGB} and ${RGBD_DEPTH}`))).toBeTruthy()
    expect(screen.queryByAltText('head RGB-D')).toBeNull()
    expect(screen.queryByTestId('rgbd-overlay')).toBeNull()
  })

  it('shows the waiting message before any message arrives', () => {
    openRgbd({})
    expect(screen.getByText(/Waiting for/)).toBeTruthy()
    expect(screen.queryByAltText('head RGB-D')).toBeNull()
  })

  it('keeps the last matching pair while the next RGB frame is still waiting for its depth, then moves on', () => {
    const { rerender } = openRgbd(frames(5, 5))
    rerender(wrapped({ ...FRAMES, [RGBD_RGB]: rgbAt(6), [RGBD_DEPTH]: depthAt(5) }))
    expect(screen.getByAltText('head RGB-D').getAttribute('src')).toBe('data:image/jpeg;base64,RGB5')
    rerender(wrapped({ ...FRAMES, [RGBD_RGB]: rgbAt(6), [RGBD_DEPTH]: depthAt(6) }))
    expect(screen.getByAltText('head RGB-D').getAttribute('src')).toBe('data:image/jpeg;base64,RGB6')
  })

  it('drops the pair when a topic is cleared (no stale image)', () => {
    const { rerender } = openRgbd(frames(5, 5))
    rerender(wrapped({ ...FRAMES, [RGBD_RGB]: null, [RGBD_DEPTH]: depthAt(5) }))
    expect(screen.queryByAltText('head RGB-D')).toBeNull()
    expect(screen.getByText(/Waiting for/)).toBeTruthy()
  })

  it('reads the raw depth under the cursor and says "no depth" for invalid pixels', () => {
    openRgbd(frames(5, 5))
    const view = screen.getByTestId('rgbd-view')
    expect(screen.getByTestId('rgbd-readout').textContent).not.toMatch(/depth \d/)
    fireEvent.mouseMove(view, { clientX: 75, clientY: 25 }) // pixel (1, 0)
    expect(screen.getByTestId('rgbd-readout').textContent).toBe('depth 1.50 m')
    fireEvent.mouseMove(view, { clientX: 25, clientY: 75 }) // pixel (0, 1)
    expect(screen.getByTestId('rgbd-readout').textContent).toBe('depth 4.00 m')
    fireEvent.mouseMove(view, { clientX: 25, clientY: 25 }) // pixel (0, 0) is invalid
    expect(screen.getByTestId('rgbd-readout').textContent).toBe('no depth')
    fireEvent.mouseLeave(view)
    expect(screen.getByTestId('rgbd-readout').textContent).not.toMatch(/depth \d|no depth/)
  })

  it('has an opacity slider from 0 to 100 percent, default 50, applied to the overlay', () => {
    openRgbd(frames(5, 5))
    const slider = screen.getByLabelText('overlay opacity') as HTMLInputElement
    expect([slider.min, slider.max, slider.value]).toEqual(['0', '100', '50'])
    const overlay = screen.getByTestId('rgbd-overlay') as HTMLCanvasElement
    expect(overlay.style.opacity).toBe('0.5')
    fireEvent.change(slider, { target: { value: '80' } })
    expect(overlay.style.opacity).toBe('0.8')
    expect(screen.getByText('80 %')).toBeTruthy()
    fireEvent.change(slider, { target: { value: '0' } })
    expect(overlay.style.opacity).toBe('0')
  })

  it('shows a legend with the range in metres from tof_range_m', () => {
    openRgbd(frames(5, 5))
    const legend = screen.getByTestId('rgbd-legend')
    expect(legend.textContent).toContain('0.0 m')
    expect(legend.textContent).toContain('4.0 m')
    expect(within(legend).getByTestId('rgbd-legend-strip')).toBeTruthy()
  })
})
