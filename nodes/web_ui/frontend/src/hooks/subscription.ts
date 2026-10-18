import { LAYER_TOPIC_FIELDS } from '../map3d/layers'
import { AppConfig, TabConfig } from '../types'

/** Topics the app reads regardless of tab config (pose, tilt banner, GPS anchor/base, grasp stream, POI, source). */
export const ALWAYS_TOPICS = [
  '/web_ui/robot_pose', '/web_ui/gps_anchor', '/web_ui/gps_base_status', '/tilt_guard/state',
  '/web_ui/grasp_result', '/poi/list', '/poi/photos', '/filter/active_source',
]

/** Tab types that show camera frames: their image topics are only subscribed while the tab is open. */
export const IMAGE_TAB_TYPES = ['camera', 'rgbd_camera', 'head']

export function tabTopics(t: TabConfig): string[] {
  return [
    t.topic,
    ...(t.topics?.map((ts) => ts.topic) ?? []),
    t.color_topic, t.depth_topic, t.camera_info_topic,
    t.gripper_camera_topic, t.head_camera_topic,
    t.rgb_camera_topic, t.tof_depth_topic, t.tof_amplitude_topic,
    t.map_topic, t.global_plan_topic, t.local_plan_topic, t.footprint_topic,
    t.local_costmap_topic, t.base_joint_states_topic, t.arm_joint_states_topic, t.head_joint_states_topic,
    ...(t.type === 'map_nav' ? [t.goal_topic] : []),
    t.poi_list_topic, t.poi_photos_topic, t.grasp_result_topic, t.active_source_topic,
  ].filter((x): x is string => Boolean(x))
}

/**
 * Topics the backend should serialize for this client: everything the app reads (fixed topics, battery, GPS,
 * overlays and tab topics), except the topics only an inactive image tab (camera, RGBD, head) shows.
 * The 3D obstacle topics of the active map tab are added only while obstacles3d (its layer) is on.
 */
export function subscribedTopics(config: AppConfig, active: TabConfig | undefined, obstacles3d = false): string[] {
  const topics = new Set<string>(ALWAYS_TOPICS)
  config.overlays.forEach((o) => topics.add(o.topic))
  if (config.battery?.topic) topics.add(config.battery.topic)
  if (config.gps_status?.rover_topic) topics.add(config.gps_status.rover_topic)
  for (const t of config.tabs) {
    if (t === active || !IMAGE_TAB_TYPES.includes(t.type)) tabTopics(t).forEach((x) => topics.add(x))
  }
  if (obstacles3d && active?.type === 'map_nav') {
    for (const f of LAYER_TOPIC_FIELDS.obstacles3d ?? []) {
      const topic = active[f]
      if (typeof topic === 'string' && topic) topics.add(topic)
    }
  }
  return [...topics].sort()
}
