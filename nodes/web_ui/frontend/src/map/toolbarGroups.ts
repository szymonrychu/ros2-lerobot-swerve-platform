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

/** Groups that stay visible on every layout class: STOP, goal and arm control. */
export const ALWAYS_VISIBLE_GROUPS: readonly string[] = ['stop', 'goal', 'grasp', 'armHome']

/** Toolbar groups in display order. Layers (with the POI and photo toggles), POI tools and camera view are low priority. */
export const TOOLBAR_GROUPS: readonly ToolbarGroup[] = [
  { id: 'stop', priority: 'high' },
  { id: 'cameraView', priority: 'low' },
  { id: 'goal', priority: 'high' },
  { id: 'poi', priority: 'low' },
  { id: 'layers', priority: 'low' },
  { id: 'mapFile', priority: 'low' },
  { id: 'grasp', priority: 'high' },
  { id: 'armHome', priority: 'high' },
]

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
