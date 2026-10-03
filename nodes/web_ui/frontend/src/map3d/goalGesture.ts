/**
 * Goal setting by press + drag in the locked top view. Positions are map-frame points picked on the ground;
 * the drag length is measured in screen px so the click/drag threshold feels the same at any zoom.
 * The pose itself comes from goalFromGesture (shared with the former 2D map).
 */
import { goalFromGesture, Pose2D, Vec2 } from '../map/mapMath'

export interface GoalDraft {
  press: Vec2 // map frame
  pressScreen: Vec2 // CSS px
  current: Vec2 // map frame
  dragPx: number // longest screen distance from the press so far
}

/**
 * Begin a goal gesture.
 *
 * @param pressMap - map point under the press
 * @param pressScreen - press position in CSS px
 * @returns new draft
 */
export function startGoalDraft(pressMap: Vec2, pressScreen: Vec2): GoalDraft {
  return { press: pressMap, pressScreen, current: pressMap, dragPx: 0 }
}

/**
 * Follow the pointer.
 *
 * @param draft - current draft
 * @param currentMap - map point under the pointer, or null when it misses the ground
 * @param currentScreen - pointer position in CSS px
 * @returns updated draft
 */
export function updateGoalDraft(draft: GoalDraft, currentMap: Vec2 | null, currentScreen: Vec2): GoalDraft {
  const dist = Math.hypot(currentScreen.x - draft.pressScreen.x, currentScreen.y - draft.pressScreen.y)
  return {
    ...draft,
    current: currentMap ?? draft.current,
    dragPx: Math.max(draft.dragPx, dist),
  }
}

/**
 * Pose the draft would publish if released now.
 *
 * @param draft - current draft
 * @param robot - robot pose in the map frame, or null
 * @returns goal pose
 */
export function draftGoalPose(draft: GoalDraft, robot: Pose2D | null): Pose2D {
  return goalFromGesture(draft.press, draft.current, draft.dragPx, robot)
}

/**
 * Finish the gesture.
 *
 * @param draft - current draft
 * @param releaseMap - map point under the release, or null when it misses the ground
 * @param releaseScreen - release position in CSS px
 * @param robot - robot pose in the map frame, or null
 * @returns goal pose to publish
 */
export function finishGoalDraft(
  draft: GoalDraft,
  releaseMap: Vec2 | null,
  releaseScreen: Vec2,
  robot: Pose2D | null,
): Pose2D {
  return draftGoalPose(updateGoalDraft(draft, releaseMap, releaseScreen), robot)
}
