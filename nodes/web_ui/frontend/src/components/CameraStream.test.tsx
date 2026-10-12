// @vitest-environment jsdom
import { render, screen } from '@testing-library/react'
import { describe, expect, it } from 'vitest'
import { designViolations, SOURCES } from '../layout/designRules'
import { CameraStream } from './CameraStream'

describe('CameraStream', () => {
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

  it.each(['components/CameraStream.tsx', 'tabs/CameraTab.tsx'])('%s has no design violations', (path) => {
    expect(SOURCES[path], path).toBeTruthy()
    expect(designViolations(path, SOURCES[path])).toEqual([])
  })

  it('source has no ellipsis character', () => {
    expect(SOURCES['components/CameraStream.tsx']).not.toContain('…')
  })
})
