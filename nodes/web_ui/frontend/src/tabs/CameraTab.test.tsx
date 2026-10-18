// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import { cleanup, render, screen } from '@testing-library/react'
import { ThemeProvider } from '@mui/material/styles'
import { theme } from '../theme'
import { TabConfig } from '../types'
import CameraTab from './CameraTab'
import { designViolations, SOURCES } from '../layout/designRules'

const FRONT = '/front_camera/image_raw/compressed'
const BACK = '/back_camera/image_raw/compressed'

const PAIR = {
  id: 'pi_cameras',
  type: 'camera',
  label: 'Front/Back',
  topics: [
    { topic: FRONT, fields: [], label: 'Front' },
    { topic: BACK, fields: [], label: 'Back' },
  ],
} as TabConfig

const SINGLE = { id: 'front', type: 'camera', label: 'Front', topic: '/front_camera/image_raw/compressed' } as TabConfig

function setViewport(width: number): void {
  window.matchMedia = ((query: string) => {
    const min = /min-width:\s*([\d.]+)px/.exec(query)
    const max = /max-width:\s*([\d.]+)px/.exec(query)
    const matches = !((min && width < Number(min[1])) || (max && width > Number(max[1])))
    return { matches, media: query, addEventListener: vi.fn(), removeEventListener: vi.fn(), addListener: vi.fn(), removeListener: vi.fn(), dispatchEvent: vi.fn(), onchange: null }
  }) as typeof window.matchMedia
}

function renderTab(tab: TabConfig, topicData: Record<string, unknown>) {
  return render(
    <ThemeProvider theme={theme}>
      <CameraTab tab={tab} topicData={topicData} publish={vi.fn()} />
    </ThemeProvider>,
  )
}

afterEach(cleanup)

describe('CameraTab', () => {
  it('renders the single stream of a tab with one topic, without captions', () => {
    setViewport(1400)
    renderTab(SINGLE, { [SINGLE.topic as string]: { jpeg_b64: 'SSS' } })
    expect(screen.getByAltText('camera feed').getAttribute('src')).toBe('data:image/jpeg;base64,SSS')
    expect(screen.queryByTestId('camera-grid')).toBeNull()
  })

  it('renders both labelled streams of a topics tab', () => {
    setViewport(1400)
    renderTab(PAIR, { [FRONT]: { jpeg_b64: 'FFF' }, [BACK]: { jpeg_b64: 'BBB' } })
    expect(screen.getByAltText('Front').getAttribute('src')).toBe('data:image/jpeg;base64,FFF')
    expect(screen.getByAltText('Back').getAttribute('src')).toBe('data:image/jpeg;base64,BBB')
    expect(screen.getByText('Front')).toBeTruthy()
    expect(screen.getByText('Back')).toBeTruthy()
  })

  it('shows the waiting line per stream while one camera has sent nothing', () => {
    setViewport(1400)
    renderTab(PAIR, { [FRONT]: { jpeg_b64: 'FFF' } })
    expect(screen.getByAltText('Front')).toBeTruthy()
    expect(screen.getByText(`Waiting for ${BACK}...`)).toBeTruthy()
  })

  it('stacks the streams on a phone and puts them side by side on a desktop', () => {
    setViewport(400)
    renderTab(PAIR, {})
    expect(screen.getByTestId('camera-grid').getAttribute('data-direction')).toBe('column')
    cleanup()
    setViewport(1400)
    renderTab(PAIR, {})
    expect(screen.getByTestId('camera-grid').getAttribute('data-direction')).toBe('row')
  })

  it('has no design violations', () => {
    expect(designViolations('tabs/CameraTab.tsx', SOURCES['tabs/CameraTab.tsx'])).toEqual([])
  })
})
