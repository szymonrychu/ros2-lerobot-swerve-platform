import { describe, expect, it } from 'vitest'
import {
  initialTabIndex,
  orderTabs,
  readStoredTabId,
  SUPPORTED_TAB_TYPES,
  supportedTabs,
  TAB_STORAGE_KEY,
  writeStoredTabId,
} from './tabSelection'

const TABS = [
  { id: 'rgbd_camera', type: 'rgbd_camera' },
  { id: 'imu', type: 'imu_orientation' },
  { id: 'map', type: 'map_nav' },
  { id: 'map2', type: 'map_nav' },
]

function memoryStorage(initial: Record<string, string> = {}) {
  const data = new Map(Object.entries(initial))
  return {
    data,
    getItem: (k: string) => data.get(k) ?? null,
    setItem: (k: string, v: string) => {
      data.set(k, v)
    },
  }
}

const throwingStorage = {
  getItem: (): string | null => {
    throw new Error('SecurityError')
  },
  setItem: (): void => {
    throw new Error('QuotaExceededError')
  },
}

describe('initialTabIndex', () => {
  it('prefers the first map_nav tab when nothing is stored', () => {
    expect(initialTabIndex(TABS, null)).toBe(2)
  })

  it('restores the stored tab when it still exists', () => {
    expect(initialTabIndex(TABS, 'imu')).toBe(1)
  })

  it('ignores a stored tab that no longer exists and falls back to the map tab', () => {
    expect(initialTabIndex(TABS, 'removed_tab')).toBe(2)
  })

  it('falls back to the first tab when no map_nav tab is configured', () => {
    expect(initialTabIndex(TABS.slice(0, 2), null)).toBe(0)
    expect(initialTabIndex(TABS.slice(0, 2), 'gone')).toBe(0)
  })

  it('returns 0 for an empty tab list', () => {
    expect(initialTabIndex([], 'map')).toBe(0)
  })
})

describe('orderTabs', () => {
  it('lists map_nav tabs first and keeps the relative order of the rest', () => {
    expect(orderTabs(TABS).map((t) => t.id)).toEqual(['map', 'map2', 'rgbd_camera', 'imu'])
  })

  it('returns an equal list when no map tab exists and does not mutate its input', () => {
    const input = TABS.slice(0, 2)
    const out = orderTabs(input)
    expect(out.map((t) => t.id)).toEqual(['rgbd_camera', 'imu'])
    expect(input.map((t) => t.id)).toEqual(['rgbd_camera', 'imu'])
  })
})

describe('stored tab id', () => {
  it('round-trips through storage under the documented key', () => {
    const storage = memoryStorage()
    writeStoredTabId(storage, 'imu')
    expect(storage.data.get(TAB_STORAGE_KEY)).toBe('imu')
    expect(readStoredTabId(storage)).toBe('imu')
  })

  it('returns null when nothing is stored or storage is unavailable', () => {
    expect(readStoredTabId(memoryStorage())).toBeNull()
    expect(readStoredTabId(undefined)).toBeNull()
  })

  it('swallows storage errors on read and write', () => {
    expect(readStoredTabId(throwingStorage)).toBeNull()
    expect(() => writeStoredTabId(throwingStorage, 'imu')).not.toThrow()
    expect(() => writeStoredTabId(undefined, 'imu')).not.toThrow()
  })
})

describe('supportedTabs', () => {
  it('drops tab types merged into the 3D map tab or removed (nav_local, nav_gps, scene3d, robot_status, effector_graph)', () => {
    const tabs = [
      ...TABS,
      { id: 'local', type: 'nav_local' },
      { id: 'gps', type: 'nav_gps' },
      { id: 'scene', type: 'scene3d' },
      { id: 'status', type: 'robot_status' },
      { id: 'servos', type: 'effector_graph' },
      { id: 'cam', type: 'camera' },
      { id: 'graph', type: 'sensor_graph' },
    ]
    expect(supportedTabs(tabs).map((t) => t.id)).toEqual(['rgbd_camera', 'imu', 'map', 'map2', 'cam', 'graph'])
  })

  it('lists exactly the kept tab types', () => {
    expect([...SUPPORTED_TAB_TYPES].sort()).toEqual(
      ['camera', 'imu_orientation', 'map_nav', 'rgbd_camera', 'sensor_graph'].sort(),
    )
  })
})
