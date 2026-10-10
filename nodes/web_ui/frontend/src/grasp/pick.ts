/**
 * Pick mode of the Grasp dropdown (pure): choosing a strategy item arms the map, the next click sets the object and
 * requests the plan, cancel leaves without planning. No React.
 */
import type { Vec2 } from '../map/mapMath'
import { GraspStrategy, STRATEGIES } from './grasp'

/** Dropdown items, in menu order: label and the strategy it sends. */
export const GRASP_MENU: readonly { label: string; strategy: GraspStrategy }[] = STRATEGIES.map((s) => ({
  label: s.label,
  strategy: s.value,
}))

/** Strategy of a dropdown item label, or null for an unknown label. */
export function strategyForMenuItem(label: string): GraspStrategy | null {
  return GRASP_MENU.find((m) => m.label === label)?.strategy ?? null
}

export type PickState = { kind: 'idle' } | { kind: 'picking'; strategy: GraspStrategy }
export const IDLE_PICK: PickState = { kind: 'idle' }

export type PickEvent =
  | { type: 'select'; strategy: GraspStrategy }
  | { type: 'click'; point: Vec2 }
  | { type: 'cancel' }

/** A plan to request: the clicked ground point (map frame) with the strategy armed when it was clicked. */
export interface PickPlan {
  strategy: GraspStrategy
  point: Vec2
}

/** Next state and the plan to request, if the event was the click that completes a pick. */
export function pickStep(state: PickState, event: PickEvent): { state: PickState; plan: PickPlan | null } {
  switch (event.type) {
    case 'select':
      return { state: { kind: 'picking', strategy: event.strategy }, plan: null }
    case 'click':
      return state.kind === 'picking'
        ? { state: IDLE_PICK, plan: { strategy: state.strategy, point: event.point } }
        : { state, plan: null }
    case 'cancel':
      return { state: IDLE_PICK, plan: null }
  }
}
