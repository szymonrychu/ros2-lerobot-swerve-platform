// @vitest-environment jsdom
import { afterEach, describe, expect, it } from 'vitest'
import { cleanup, render, screen, within } from '@testing-library/react'
import { FlowStep, FlowStepper, unlockedCount } from './FlowStepper'

afterEach(cleanup)

const step = (id: string, done: boolean, extra: Partial<FlowStep> = {}): FlowStep => ({
  id,
  title: `Title ${id}`,
  see: `See ${id}`,
  done,
  children: <button>{`Do ${id}`}</button>,
  ...extra,
})

describe('unlockedCount', () => {
  it('unlocks up to and including the first required step not done', () => {
    expect(unlockedCount([step('a', true), step('b', false), step('c', false)])).toBe(2)
    expect(unlockedCount([step('a', false), step('b', false)])).toBe(1)
    expect(unlockedCount([step('a', true), step('b', true)])).toBe(2)
  })
  it('an optional step never blocks the steps after it', () => {
    expect(unlockedCount([step('a', true), step('b', false, { optional: true }), step('c', false), step('d', false)])).toBe(3)
  })
})

describe('FlowStepper', () => {
  it('numbers the steps and shows the action and what to see of unlocked steps only', () => {
    render(<FlowStepper label="Flow" steps={[step('a', true), step('b', false), step('c', false)]} />)
    const items = within(screen.getByRole('list', { name: 'Flow' })).getAllByRole('listitem')
    expect(items).toHaveLength(3)
    expect(within(items[0]).getByText('1. Title a')).toBeTruthy()
    expect(within(items[0]).getByText('Done')).toBeTruthy()
    expect(within(items[1]).getByRole('button', { name: 'Do b' })).toBeTruthy()
    expect(within(items[1]).getByText('What you should see: See b')).toBeTruthy()
    expect(within(items[1]).getByText('Current')).toBeTruthy()
    expect(within(items[2]).queryByRole('button')).toBeNull()
    expect(within(items[2]).getByText('Finish step 2 first.')).toBeTruthy()
    expect(within(items[2]).getByText('Locked')).toBeTruthy()
  })

  it('shows the status line and the error of a step', () => {
    render(<FlowStepper label="Flow" steps={[step('a', false, { status: 'Pose 3 of 9', error: 'arm refused' })]} />)
    expect(screen.getByText('Pose 3 of 9')).toBeTruthy()
    expect(screen.getByText('arm refused')).toBeTruthy()
  })

  it('an always-open step is never locked', () => {
    render(<FlowStepper label="Flow" steps={[step('a', false), step('b', false), step('c', false, { alwaysOpen: true })]} />)
    expect(screen.getByRole('button', { name: 'Do c' })).toBeTruthy()
    expect(screen.queryByRole('button', { name: 'Do b' })).toBeNull()
  })

  it('marks an optional step', () => {
    render(<FlowStepper label="Flow" steps={[step('a', false, { optional: true }), step('b', false)]} />)
    expect(screen.getByText('Optional')).toBeTruthy()
    expect(screen.getByRole('button', { name: 'Do b' })).toBeTruthy()
  })
})
