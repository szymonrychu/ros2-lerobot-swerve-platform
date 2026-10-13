import { describe, expect, it } from 'vitest'
import { needsColorSmall, subscribedTopics } from './subscription'
import { AppConfig, TabConfig } from '../types'

const map = { id: 'm', type: 'map_nav', label: 'Map', map_topic: '/map', goal_topic: '/goal' } as TabConfig
const rgbd = { id: 'r', type: 'rgbd_camera', label: 'RGBD', color_topic: '/c', depth_topic: '/d' } as TabConfig
const head = { id: 'h', type: 'head', label: 'Head', rgb_camera_topic: '/h/rgb' } as TabConfig

function cfg(tabs: TabConfig[], extra: Partial<AppConfig> = {}): AppConfig {
  return { http_port: 0, ws_broadcast_hz: 10, tabs, overlays: [{ topic: '/ov' }], ...extra } as unknown as AppConfig
}

describe('subscribedTopics', () => {
  it('skips the topics of inactive image tabs', () => {
    const t = subscribedTopics(cfg([map, rgbd, head]), map)
    expect(t).toContain('/map')
    expect(t).toContain('/ov')
    expect(t).not.toContain('/c')
    expect(t).not.toContain('/h/rgb')
  })
  it('includes the active image tab topics', () => {
    const t = subscribedTopics(cfg([map, rgbd, head]), rgbd)
    expect(t).toEqual(expect.arrayContaining(['/c', '/d', '/goal', '/map']))
  })
  it('always includes the topics the app reads outside the tabs', () => {
    const t = subscribedTopics(cfg([map]), map)
    expect(t).toEqual(expect.arrayContaining([
      '/web_ui/robot_pose', '/web_ui/gps_anchor', '/web_ui/gps_base_status', '/tilt_guard/state',
      '/web_ui/grasp_result', '/poi/list', '/poi/photos', '/filter/active_source',
    ]))
  })
  it('includes configured battery, rover GPS and per-tab named topics', () => {
    const poi = { id: 'p', type: 'poi', label: 'P', poi_list_topic: '/x/list', poi_photos_topic: '/x/photos' } as TabConfig
    const t = subscribedTopics(
      cfg([poi], { battery: { topic: '/bat' }, gps_status: { rover_topic: '/rover' } } as Partial<AppConfig>), poi,
    )
    expect(t).toEqual(expect.arrayContaining(['/bat', '/rover', '/x/list', '/x/photos']))
  })
})

describe('needsColorSmall', () => {
  it('is true only for the RGBD tab', () => {
    expect(needsColorSmall(rgbd)).toBe(true)
    expect(needsColorSmall(head)).toBe(false)
    expect(needsColorSmall(undefined)).toBe(false)
  })
})
