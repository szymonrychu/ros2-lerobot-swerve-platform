/**
 * Map toolbar group priorities. On a phone the low-priority groups move into a 'more' menu so the
 * toolbar stays on one row; tablet and desktop show every group.
 */
import type { LayoutClass } from '../layout/useLayout'

export type ToolbarPriority = 'high' | 'low'

export interface ToolbarGroup {
  id: string
  priority: ToolbarPriority
}

/** The stop control: STOP (navigation) always applies. */
export const STOP_GROUPS: readonly string[] = ['stop']

/** Groups that stay visible on every layout class: the stop and goal. */
export const ALWAYS_VISIBLE_GROUPS: readonly string[] = [...STOP_GROUPS, 'goal']

/** Toolbar groups in display order. Layers (with the POI and photo toggles), POI tools and camera view are low priority. */
export const TOOLBAR_GROUPS: readonly ToolbarGroup[] = [
  { id: 'stop', priority: 'high' },
  { id: 'cameraView', priority: 'low' },
  { id: 'goal', priority: 'high' },
  { id: 'poi', priority: 'low' },
  { id: 'layers', priority: 'low' },
  { id: 'mapFile', priority: 'low' },
]

/**
 * The stop controls; they always sit in the primary toolbar, never in the overflow menu.
 *
 * @returns ids of the stop groups to render.
 */
export function stopGroupIds(): string[] {
  return [...STOP_GROUPS]
}

/**
 * Split toolbar groups into the ones shown in the row and the ones in the 'more' menu.
 *
 * @param groups - toolbar groups in display order.
 * @param layout - current layout class.
 * @returns visible groups and overflow groups, both in the original order. Overflow is only used on phone,
 *   and never holds an always-visible group.
 */
export function splitToolbar<G extends ToolbarGroup>(
  groups: readonly G[],
  layout: LayoutClass,
): { visible: G[]; overflow: G[] } {
  if (layout !== 'phone') return { visible: [...groups], overflow: [] }
  const toOverflow = (g: G) => g.priority === 'low' && !ALWAYS_VISIBLE_GROUPS.includes(g.id)
  return { visible: groups.filter((g) => !toOverflow(g)), overflow: groups.filter(toOverflow) }
}

/**
 * Whether any of the given groups is rendered in the row (not in the overflow menu).
 *
 * @param overflow - groups moved to the 'more' menu.
 * @param ids - group ids to check.
 * @returns true when at least one id is not in overflow.
 */
export function anyInRow(overflow: readonly ToolbarGroup[], ids: readonly string[]): boolean {
  return ids.some((id) => !overflow.some((g) => g.id === id))
}
