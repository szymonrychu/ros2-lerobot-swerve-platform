// @vitest-environment jsdom
import { afterEach, describe, expect, it } from 'vitest'
import { cleanup, render, screen, within } from '@testing-library/react'
import { SequenceActivity, SkipReasons } from './SequenceActivity'
import stateFx from '../../../../head_rgbd_calibration/tests/fixtures/api/head_arm_state.json'
import type { HeadArmEvent, HeadArmSequence } from './headArmApi'

const ERROR_TEXT = 'the gripper closed past the handle contact: seat the handle up to the shoulder and start again'
const EVENTS: HeadArmEvent[] = [
  { time: '2026-11-09T10:02:00+00:00', phase: 'moving_to_base', level: 'error', message: `sequence error: ${ERROR_TEXT}` },
  { time: '2026-11-09T10:01:30+00:00', phase: 'waiting_board', level: 'info', message: 'board attached' },
  { time: '2026-11-09T10:01:00+00:00', phase: 'moving_to_base', level: 'info', message: 'sequence started: 24 poses' },
]
const seq = (over: Partial<HeadArmSequence>): HeadArmSequence => ({
  ...stateFx.sequence,
  grip: { ...stateFx.sequence.grip, source: stateFx.sequence.grip.source as 'config' | 'recorded' },
  running: false,
  phase: 'idle',
  status: 'idle',
  error: null,
  events: [],
  ...over,
})

afterEach(() => {
  cleanup()
})

describe('SequenceActivity', () => {
  it('shows the current status', () => {
    render(<SequenceActivity sequence={seq({ running: true, phase: 'capturing', status: 'tier 0: pose 2 of 4' })} />)
    expect(screen.getByText('Status (capturing): tier 0: pose 2 of 4')).toBeTruthy()
  })

  it('leaves the status out when the caller shows it in its progress line', () => {
    render(<SequenceActivity sequence={seq({ status: 'tier 0: pose 2 of 4' })} withStatus={false} />)
    expect(screen.queryByText(/tier 0: pose 2 of 4/)).toBeNull()
  })

  it('shows the sequence error in an error banner with the time and phase of the newest error event', () => {
    render(<SequenceActivity sequence={seq({ error: ERROR_TEXT, status: `error: ${ERROR_TEXT}`, events: EVENTS })} />)
    const banner = screen.getByRole('alert')
    expect(within(banner).getByText('Last head-arm error')).toBeTruthy()
    expect(banner.textContent).toContain(ERROR_TEXT)
    expect(banner.textContent).toContain('phase moving_to_base')
    expect(banner.textContent).toContain(new Date(EVENTS[0].time).toLocaleTimeString())
  })

  it('shows no banner without a sequence error', () => {
    render(<SequenceActivity sequence={seq({ events: EVENTS.slice(1) })} />)
    expect(screen.queryByRole('alert')).toBeNull()
  })

  it('lists the recent events newest first in a scrollable log', () => {
    render(<SequenceActivity sequence={seq({ error: ERROR_TEXT, events: EVENTS })} />)
    const log = screen.getByRole('list', { name: 'Recent head-arm events' })
    const items = within(log).getAllByRole('listitem')
    expect(items.map((i) => i.textContent)).toEqual([
      expect.stringContaining(`ERROR [moving_to_base] sequence error: ${ERROR_TEXT}`),
      expect.stringContaining('INFO [waiting_board] board attached'),
      expect.stringContaining('INFO [moving_to_base] sequence started: 24 poses'),
    ])
    expect(getComputedStyle(log).overflowY).toBe('auto')
  })

  it('says when there are no events yet', () => {
    render(<SequenceActivity sequence={seq({})} />)
    expect(screen.getByText('No head-arm events yet')).toBeTruthy()
  })
})

describe('SkipReasons', () => {
  it('names each skip reason with its count', () => {
    render(<SkipReasons skipped={{ 'inside D435 min-Z': 3, 'outside colour FOV': 2 }} />)
    expect(screen.getByText('inside D435 min-Z: 3')).toBeTruthy()
    expect(screen.getByText('outside colour FOV: 2')).toBeTruthy()
  })
})
