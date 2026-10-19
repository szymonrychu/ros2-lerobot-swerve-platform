import { describe, expect, it } from 'vitest'
import { subscribedTopics } from './subscription'
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
  it('subscribes the arm head_camera_topic and no front_camera_topic', () => {
    const arm = { id: 'a', type: 'arm', label: 'Arm', head_camera_topic: '/head_camera/image_raw/compressed', front_camera_topic: '/front' } as unknown as TabConfig
    const t = subscribedTopics(cfg([arm]), arm)
    expect(t).toContain('/head_camera/image_raw/compressed')
    expect(t).not.toContain('/front')
  })
  it('subscribes head_joint_states_topic for head and map_nav tabs', () => {
    const h = { ...head, head_joint_states_topic: '/head/joint_states' } as TabConfig
    const m = { ...map, head_joint_states_topic: '/head/joint_states' } as TabConfig
    expect(subscribedTopics(cfg([h]), h)).toContain('/head/joint_states')
    expect(subscribedTopics(cfg([m]), m)).toContain('/head/joint_states')
  })
})

describe('3D obstacles on demand', () => {
  const m = { ...map, obstacle_cloud_topic: '/tof/points_filtered', scan_points_topic: '/scan_filtered' } as TabConfig
  it('requests both topics only while the layer is on', () => {
    expect(subscribedTopics(cfg([m]), m)).not.toContain('/tof/points_filtered')
    expect(subscribedTopics(cfg([m]), m)).not.toContain('/scan_filtered')
    const on = subscribedTopics(cfg([m]), m, true)
    expect(on).toEqual(expect.arrayContaining(['/tof/points_filtered', '/scan_filtered']))
  })
  it('adds the map3d cloud topic only while the 3D map layer is on', () => {
    const m3 = { ...map, map3d_cloud_topic: '/map3d/cloud' } as TabConfig
    expect(subscribedTopics(cfg([m3]), m3)).not.toContain('/map3d/cloud')
    expect(subscribedTopics(cfg([m3]), m3, false, true)).toContain('/map3d/cloud')
  })
})
