import { describe, expect, it } from 'vitest'
import { renderToStaticMarkup } from 'react-dom/server'
import { RobotIcon } from './RobotIcon'

describe('RobotIcon', () => {
  it('renders four rollers, straight and marked as no data before the first message', () => {
    const html = renderToStaticMarkup(<RobotIcon joints={undefined} />)
    expect(html.match(/data-testid="roller-/g)).toHaveLength(4)
    expect(html).toContain('no data yet')
    expect(html).toContain('rotate(0 ')
  })

  it('rotates the named roller from the joint states', () => {
    const html = renderToStaticMarkup(<RobotIcon joints={{ name: ['fl_steer'], position: [Math.PI / 2] }} />)
    expect(html).not.toContain('no data yet')
    expect(html).toMatch(/data-testid="roller-fl"[^>]*transform="rotate\(-90 /)
  })
})
