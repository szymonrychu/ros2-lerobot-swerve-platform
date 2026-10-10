import type { BatteryConfig } from './battery/batteryStatus'
import type { GpsStatusConfig } from './gps/gpsStatus'
import type { TileSourceInfo } from './map3d/tileSources'

export interface TabFieldSpec {
  path: string
  label: string
  color?: string
}

export interface TabTopicSpec {
  topic: string
  fields: TabFieldSpec[]
}

export interface TabConfig {
  id: string
  type: string
  label: string
  tile_version?: string | null // map_nav: default tile source version from /api/config (legacy single-source field)
  tile_sources?: TileSourceInfo[] // map_nav: public tile source list (id, label, max_zoom, attribution, version)
  default_tile_source?: string | null // map_nav: id of the source shown first
  tile_display_zoom?: number // map_nav: preferred GPS tile zoom (stretched above the source max zoom)
  topic?: string
  topics?: TabTopicSpec[]
  window_s?: number
  max_points?: number
  goal_topic?: string
  color_topic?: string
  depth_topic?: string
  camera_info_topic?: string
  // map_nav tab (backend fills defaults: /map, /plan, /optimal_trajectory, /goal_pose, map, base_link, /var/lib/ros2/maps/slam_map)
  map_topic?: string
  global_plan_topic?: string
  local_plan_topic?: string
  footprint_topic?: string // default /local_costmap/published_footprint (Nav2 robot footprint)
  map_frame?: string
  base_frame?: string
  map_save_path?: string
  map_reset_service?: string // default /slam_toolbox/reset
  navigate_action?: string // default /navigate_to_pose (Stop cancels <action>/_action/cancel_goal)
  // map_nav 3D view: robot model, local costmap and arm control
  local_costmap_topic?: string // map/costmap payload (png_b64 + placement) in map_frame, drawn above the SLAM map
  base_urdf?: string // swerve base URDF under /api/urdf/, placed at /web_ui/robot_pose
  base_joint_states_topic?: string // sensor_msgs/JointState driving the base URDF (wheel steer/drive)
  arm_urdf?: string // arm URDF under /api/urdf/
  arm_joint_states_topic?: string // sensor_msgs/JointState driving the arm URDF (follower positions)
  arm_offset?: [number, number, number] // arm mount in the base frame (ROS x, y, z metres)
  arm_command_topic?: string // sensor_msgs/JointState published by the draggable arm
  poi_list_topic?: string // default /poi/list (latched JSON list from poi_store)
  poi_command_topic?: string // default /poi/command (edits go through POST /api/poi)
  poi_result_topic?: string // default /poi/result
  grasp_command_topic?: string // default /grasp/command (grasp requests go through POST /api/grasp)
  grasp_result_topic?: string // default /grasp/result
  grasp_timeout_s?: number // seconds POST /api/grasp waits for a plan (default 30)
  agent_url?: string // agent_chat tab: claude_agent API base URL proxied by the backend (default http://127.0.0.1:18300)
}

export interface OverlayItem {
  topic: string
  field: string
  label: string
  format?: string
  unit?: string
}

export interface AppConfig {
  battery?: BatteryConfig | null // absent/null: battery chip and cut-off banner are off
  gps_status?: GpsStatusConfig | null // absent/null: GPS chips are off
  http_port: number
  ws_broadcast_hz: number
  tabs: TabConfig[]
  overlays: OverlayItem[]
}
