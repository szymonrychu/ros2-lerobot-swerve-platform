import type { TiltStatus } from '../tilt/tiltStatus'

const TOP_VIEW_HINT = 'Switch to Top view to set goals'

export interface GoalButtonInput {
  hasGoalTopic: boolean
  topView: boolean
  tilt: TiltStatus
}

/** Disabled state and tooltip of the "Set goal" button; a tilt warning/error takes precedence over the view hint. */
export function goalButtonState({ hasGoalTopic, topView, tilt }: GoalButtonInput): { disabled: boolean; tooltip: string } {
  if (tilt.navBlocked) return { disabled: true, tooltip: tilt.navBlockedReason ?? '' }
  return { disabled: !hasGoalTopic || !topView, tooltip: topView ? '' : TOP_VIEW_HINT }
}
