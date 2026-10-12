import type { LayoutClass } from './useLayout'

export interface TabBarMode {
  drawer: boolean
  showIcons: boolean
  showLabels: boolean
  scrollable: boolean
}

/**
 * Tab bar presentation per layout class.
 * @param layout - current layout class
 * @returns phone: drawer plus label-only scrollable tabs; tablet: scrollable icon+label tabs; desktop: full icon+label tabs.
 */
export function tabBarMode(layout: LayoutClass): TabBarMode {
  switch (layout) {
    case 'phone':
      return { drawer: true, showIcons: false, showLabels: true, scrollable: true }
    case 'tablet':
      return { drawer: false, showIcons: true, showLabels: true, scrollable: true }
    case 'desktop':
      return { drawer: false, showIcons: true, showLabels: true, scrollable: false }
  }
}
