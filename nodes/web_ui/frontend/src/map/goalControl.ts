import type { TiltStatus } from '../tilt/tiltStatus'

export interface GoalButtonInput {
  hasGoalTopic: boolean
  tilt: TiltStatus
}

/**
 * Disabled state and tooltip of the "Set goal" button. Goals can be set in the Top view and in the orbit view; only a
 * missing goal topic or a tilt warning/error disables the button.
 *
 * @param input - whether a goal topic is configured and the tilt status
 * @returns whether the button is disabled and the tooltip (the tilt text while blocked, otherwise empty)
 */
export function goalButtonState({ hasGoalTopic, tilt }: GoalButtonInput): { disabled: boolean; tooltip: string } {
  if (tilt.navBlocked) return { disabled: true, tooltip: tilt.navBlockedReason ?? '' }
  return { disabled: !hasGoalTopic, tooltip: '' }
}
