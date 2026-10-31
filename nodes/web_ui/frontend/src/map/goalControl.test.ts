import { describe, expect, it } from 'vitest'
import { goalButtonState } from './goalControl'
import { tiltStatus } from '../tilt/tiltStatus'

const WARNING = tiltStatus({ level: 'warning', roll_deg: 12.5, pitch_deg: -3 })
const ERROR = tiltStatus({ level: 'error', roll_deg: 25, pitch_deg: 4 })
const OK = tiltStatus(undefined)

describe('goalButtonState', () => {
  it('is enabled with an empty tooltip in any view with a goal topic and a level robot', () => {
    expect(goalButtonState({ hasGoalTopic: true, tilt: OK })).toEqual({ disabled: false, tooltip: '' })
  })

  it('is disabled with the tilt text as tooltip during warning and error', () => {
    expect(goalButtonState({ hasGoalTopic: true, tilt: WARNING })).toEqual({
      disabled: true,
      tooltip: WARNING.navBlockedReason,
    })
    expect(goalButtonState({ hasGoalTopic: true, tilt: ERROR }).disabled).toBe(true)
    expect(goalButtonState({ hasGoalTopic: true, tilt: ERROR }).tooltip).toContain('Tilt error')
  })

  it('is disabled without a goal topic', () => {
    expect(goalButtonState({ hasGoalTopic: false, tilt: OK }).disabled).toBe(true)
  })
})
