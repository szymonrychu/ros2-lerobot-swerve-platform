// @vitest-environment jsdom
import { cleanup, fireEvent, render, screen } from '@testing-library/react'
import { afterEach, describe, expect, it, vi } from 'vitest'
import { designViolations, SOURCES } from '../layout/designRules'
import { CameraStream } from './CameraStream'

describe('CameraStream', () => {
  afterEach(cleanup)

  it('shows one short loading line with three dots, no ellipsis character', () => {
    render(<CameraStream topic="/cam" data={undefined} />)
    const text = screen.getByText(/Waiting for/).textContent ?? ''
    expect(text).toBe('Waiting for /cam...')
    expect(text).not.toContain('…')
  })

  it('shows the error as one line when the frame carries an error and no image', () => {
    render(<CameraStream topic="/cam" data={{ jpeg_b64: null, error: 'decode failed' }} />)
    expect(screen.getByText('/cam: decode failed')).toBeTruthy()
  })

  it('renders the frame once a jpeg arrives', () => {
    render(<CameraStream topic="/cam" data={{ jpeg_b64: 'AAAA' }} alt="cam" />)
    expect(screen.getByAltText('cam').getAttribute('src')).toBe('data:image/jpeg;base64,AAAA')
  })

  function sizedImage(): HTMLElement {
    const img = screen.getByAltText('cam')
    Object.defineProperty(img, 'naturalWidth', { value: 1600 })
    Object.defineProperty(img, 'naturalHeight', { value: 900 })
    img.getBoundingClientRect = () => ({ left: 0, top: 0, width: 800, height: 450, right: 800, bottom: 450, x: 0, y: 0 }) as DOMRect
    return img
  }

  it('click calls onPointSelect with the mapped point and shows a crosshair', () => {
    const onPointSelect = vi.fn()
    render(<CameraStream topic="/cam" data={{ jpeg_b64: 'AAAA' }} alt="cam" onPointSelect={onPointSelect} />)
    const img = sizedImage()
    expect(img.style.cursor).toBe('crosshair')
    fireEvent.click(img, { clientX: 200, clientY: 225 })
    expect(onPointSelect).toHaveBeenCalledWith(0.25, 0.5)
  })

  it('does not call onPointSelect for a click outside the picture', () => {
    const onPointSelect = vi.fn()
    render(<CameraStream topic="/cam" data={{ jpeg_b64: 'AAAA' }} alt="cam" onPointSelect={onPointSelect} />)
    const img = sizedImage()
    img.getBoundingClientRect = () => ({ left: 0, top: 0, width: 800, height: 600, right: 800, bottom: 600, x: 0, y: 0 }) as DOMRect
    fireEvent.click(img, { clientX: 400, clientY: 5 })
    expect(onPointSelect).not.toHaveBeenCalled()
  })

  it('without onPointSelect the cursor is unchanged', () => {
    render(<CameraStream topic="/cam" data={{ jpeg_b64: 'AAAA' }} alt="cam" />)
    expect(screen.getByAltText('cam').style.cursor).toBe('')
  })

  it.each(['components/CameraStream.tsx', 'tabs/CameraTab.tsx'])('%s has no design violations', (path) => {
    expect(SOURCES[path], path).toBeTruthy()
    expect(designViolations(path, SOURCES[path])).toEqual([])
  })

  it('source has no ellipsis character', () => {
    expect(SOURCES['components/CameraStream.tsx']).not.toContain('…')
  })
})
