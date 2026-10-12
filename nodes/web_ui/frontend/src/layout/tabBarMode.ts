import type { LayoutClass } from './useLayout'

export interface TabBarMode {
  drawer: boolean
  scrollable: boolean
}

/**
 * Tab bar presentation per layout class.
 * @param layout - current layout class
 * @returns phone: drawer plus label-only scrollable tabs; tablet: scrollable label-only tabs; desktop: label-only tabs, no scrolling.
 */
export function tabBarMode(layout: LayoutClass): TabBarMode {
  switch (layout) {
    case 'phone':
      return { drawer: true, scrollable: true }
    case 'tablet':
      return { drawer: false, scrollable: true }
    case 'desktop':
      return { drawer: false, scrollable: false }
  }
}
