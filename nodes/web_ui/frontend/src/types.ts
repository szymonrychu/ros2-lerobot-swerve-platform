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
  topic?: string
  topics?: TabTopicSpec[]
  window_s?: number
  max_points?: number
  scan_topic?: string
  costmap_topic?: string
  odom_topic?: string
  goal_topic?: string
  fix_topic?: string
  tile_url?: string
  default_zoom?: number
  urdf_file?: string
  arm_urdf_file?: string
  arm_joint_topic?: string
  arm_offset?: [number, number, number]
  arm_command_topic?: string
  color_topic?: string
  depth_topic?: string
  camera_info_topic?: string
}

export interface OverlayItem {
  topic: string
  field: string
  label: string
  format?: string
  unit?: string
}

export interface AppConfig {
  http_port: number
  ws_broadcast_hz: number
  tabs: TabConfig[]
  overlays: OverlayItem[]
}
