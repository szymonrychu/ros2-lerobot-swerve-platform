import { describe, expect, it } from 'vitest'
import { renderToStaticMarkup } from 'react-dom/server'
import { TiltBanner } from './TiltBanner'
import { tiltStatus } from '../tilt/tiltStatus'

describe('TiltBanner', () => {
  it('renders nothing at ok level', () => {
    expect(renderToStaticMarkup(<TiltBanner status={tiltStatus(undefined)} />)).toBe('')
  })

  it('renders the amber warning banner', () => {
    const html = renderToStaticMarkup(<TiltBanner status={tiltStatus({ level: 'warning', roll_deg: 12.5, pitch_deg: -3 })} />)
    expect(html).toContain('Tilt warning: roll 12.5 / pitch -3.0 deg - automatic navigation disabled, drive back manually')
    expect(html).toContain('MuiAlert-filledWarning')
    expect(html).toContain('role="alert"')
  })

  it('renders the red error banner', () => {
    const html = renderToStaticMarkup(<TiltBanner status={tiltStatus({ level: 'error', roll_deg: 25, pitch_deg: 4 })} />)
    expect(html).toContain('Tilt error: base drive disabled until the robot is level')
    expect(html).toContain('MuiAlert-filledError')
  })
})
