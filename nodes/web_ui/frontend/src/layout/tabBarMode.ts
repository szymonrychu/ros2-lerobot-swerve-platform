import type { LayoutClass } from './useLayout'

export interface TabBarMode {
  /** The hamburger menu is present on every layout (it always holds Calibrate). */
  menu: boolean
  /** Phone only: the drawer also lists the tabs. */
  tabsInDrawer: boolean
  scrollable: boolean
}

/**
 * Tab bar presentation per layout class.
 * @param layout - current layout class
 * @returns phone: drawer with the tabs plus Calibrate and label-only scrollable tabs; tablet: drawer with Calibrate only, scrollable tabs; desktop: drawer with Calibrate only, label-only tabs, no scrolling.
 */
export function tabBarMode(layout: LayoutClass): TabBarMode {
  switch (layout) {
    case 'phone':
      return { menu: true, tabsInDrawer: true, scrollable: true }
    case 'tablet':
      return { menu: true, tabsInDrawer: false, scrollable: true }
    case 'desktop':
      return { menu: true, tabsInDrawer: false, scrollable: false }
  }
}
