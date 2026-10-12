import { describe, expect, it } from 'vitest'
import { renderToStaticMarkup } from 'react-dom/server'
import { PhaseProgress } from './PhaseProgress'

describe('PhaseProgress', () => {
  it('shows attempt n/m, iteration k and the retry estimate', () => {
    const html = renderToStaticMarkup(<PhaseProgress attemptLabel="attempt 2/3" iterationLabel="iteration 4" pRetrySuccess={0.7} />)
    expect(html).toContain('attempt 2/3')
    expect(html).toContain('iteration 4')
    expect(html).toContain('p 0.70')
  })

  it('joins the parts with a hyphen and renders no Chip', () => {
    const html = renderToStaticMarkup(<PhaseProgress attemptLabel="attempt 2/3" iterationLabel="iteration 4" pRetrySuccess={0.7} />)
    expect(html).not.toContain('MuiChip')
    expect(html.replace(/<style[^>]*>.*?<\/style>/g, '').replace(/<[^>]+>/g, '')).toBe('attempt 2/3 - iteration 4 - p 0.70')
  })

  it('renders nothing for a legacy phase without the new fields', () => {
    expect(renderToStaticMarkup(<PhaseProgress attemptLabel={null} iterationLabel={null} pRetrySuccess={null} />)).toBe('')
  })
})
