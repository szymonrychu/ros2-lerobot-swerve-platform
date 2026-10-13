import { TabConfig } from '../types'

/** Tab types that show camera frames: their image topics are only subscribed while the tab is open. */
export const IMAGE_TAB_TYPES = ['camera', 'rgbd_camera', 'head']

/** Tab types whose open tab needs the downscaled color_small JPEG. */
export const COLOR_SMALL_TAB_TYPES = ['rgbd_camera']

export function tabTopics(t: TabConfig): string[] {
  return [
    t.topic,
    ...(t.topics?.map((ts) => ts.topic) ?? []),
    t.color_topic, t.depth_topic, t.camera_info_topic,
    t.gripper_camera_topic, t.overview_camera_topic,
    t.rgb_camera_topic, t.tof_depth_topic, t.tof_amplitude_topic,
    t.map_topic, t.global_plan_topic, t.local_plan_topic, t.footprint_topic,
    t.local_costmap_topic, t.base_joint_states_topic, t.arm_joint_states_topic,
    ...(t.type === 'map_nav' ? [t.goal_topic] : []),
  ].filter((x): x is string => Boolean(x))
}

/**
 * Topics the backend should serialize for this client: everything the tabs and overlays use, except the topics
 * only an inactive image tab (camera, RGBD, head) shows.
 */
export function subscribedTopics(tabs: TabConfig[], overlayTopics: string[], active: TabConfig | undefined): string[] {
  const topics = new Set<string>(overlayTopics)
  for (const t of tabs) {
    if (t === active || !IMAGE_TAB_TYPES.includes(t.type)) tabTopics(t).forEach((x) => topics.add(x))
  }
  return [...topics].sort()
}

export function needsColorSmall(active: TabConfig | undefined): boolean {
  return active !== undefined && COLOR_SMALL_TAB_TYPES.includes(active.type)
}
