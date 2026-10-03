import { describe, expect, it } from 'vitest'
import { draftGoalPose, finishGoalDraft, startGoalDraft, updateGoalDraft } from './goalGesture'

const ROBOT = { x: 0, y: 0, yaw: 0.5 }

describe('top-view goal gesture', () => {
  it('a plain tap sets the position and faces away from the robot', () => {
    const d = startGoalDraft({ x: 2, y: 2 }, { x: 100, y: 100 })
    const goal = finishGoalDraft(d, { x: 2, y: 2 }, { x: 101, y: 100 }, ROBOT)
    expect(goal.x).toBe(2)
    expect(goal.y).toBe(2)
    expect(goal.yaw).toBeCloseTo(Math.PI / 4)
  })

  it('a drag beyond the threshold sets the heading along the drag in map coordinates', () => {
    let d = startGoalDraft({ x: 1, y: 1 }, { x: 100, y: 100 })
    d = updateGoalDraft(d, { x: 1, y: 3 }, { x: 100, y: 40 })
    expect(d.dragPx).toBeCloseTo(60)
    const goal = finishGoalDraft(d, { x: 1, y: 3 }, { x: 100, y: 40 }, ROBOT)
    expect(goal).toEqual({ x: 1, y: 1, yaw: Math.PI / 2 })
  })

  it('tracks the longest drag so returning near the press still keeps the dragged heading', () => {
    let d = startGoalDraft({ x: 0, y: 0 }, { x: 0, y: 0 })
    d = updateGoalDraft(d, { x: -2, y: 0 }, { x: -50, y: 0 })
    d = updateGoalDraft(d, { x: -0.01, y: 0 }, { x: -2, y: 0 })
    expect(d.dragPx).toBeCloseTo(50)
    const goal = finishGoalDraft(d, { x: -0.01, y: 0 }, { x: -2, y: 0 }, null)
    expect(goal.yaw).toBeCloseTo(Math.PI)
  })

  it('a move that misses the ground keeps the previous point', () => {
    let d = startGoalDraft({ x: 0, y: 0 }, { x: 0, y: 0 })
    d = updateGoalDraft(d, null, { x: 30, y: 0 })
    expect(d.current).toEqual({ x: 0, y: 0 })
    expect(d.dragPx).toBeCloseTo(30)
  })

  it('draftGoalPose previews the pose the release would publish', () => {
    let d = startGoalDraft({ x: 0, y: 0 }, { x: 0, y: 0 })
    d = updateGoalDraft(d, { x: 0, y: -1 }, { x: 0, y: 40 })
    expect(draftGoalPose(d, ROBOT).yaw).toBeCloseTo(-Math.PI / 2)
  })
})
