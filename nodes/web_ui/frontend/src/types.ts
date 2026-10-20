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
  label?: string | null // camera tab: caption of this stream
}

/** Camera pose relative to its parent URDF link: translation in metres, roll/pitch/yaw in radians. */
export interface CameraMount {
  xyz: [number, number, number]
  rpy: [number, number, number]
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
  obstacle_cloud_topic?: string // ToF obstacle cloud (PointCloud2 payload + frame_pose), subscribed only while the 3D obstacles layer is on
  scan_points_topic?: string // lidar scan points (LaserScan payload + frame_pose), subscribed with the obstacle cloud
  map3d_cloud_topic?: string // RTAB-Map 3D map payload (map3d_cloud), subscribed only while the 3D map layer is on
  rgbd_cloud_topic?: string // head_rgbd_cpp XYZRGB cloud (map3d_cloud-style payload + frame_pose), subscribed only while the Head RGB-D layer is on
  map3d_status_topic?: string // map3d manager status (JSON String)
  map3d_result_topic?: string // map3d command results (JSON String)
  map3d_command_topic?: string // map3d commands (save, reset, ...)
  map3d_frame_id?: string // the only frame the 3D map layer accepts (drawn in the map frame)
  map3d_max_points?: number // backend cap of points sent to a browser
  map3d_voxel_m?: number // starting voxel edge of the cloud in metres
  lidar_wall_height_m?: number // height SLAM map walls are extruded to where no depth was measured
  base_urdf?: string // swerve base URDF under /api/urdf/, placed at /web_ui/robot_pose
  base_joint_states_topic?: string // sensor_msgs/JointState driving the base URDF (wheel steer/drive)
  arm_urdf?: string // arm URDF under /api/urdf/
  arm_joint_states_topic?: string // sensor_msgs/JointState driving the arm URDF (follower positions)
  arm_offset?: [number, number, number] // arm mount in the base frame (ROS x, y, z metres)
  arm_command_topic?: string // sensor_msgs/JointState published by the draggable arm
  poi_list_topic?: string // default /poi/list (latched JSON list from poi_store)
  poi_photos_topic?: string // default /poi/photos (latched photo store summary {revision, count, bytes, ...})
  poi_command_topic?: string // default /poi/command (edits go through POST /api/poi)
  poi_result_topic?: string // default /poi/result
  grasp_command_topic?: string // default /grasp/command (grasp requests go through POST /api/grasp)
  grasp_result_topic?: string // default /grasp/result
  grasp_timeout_s?: number // seconds POST /api/grasp waits for a plan (default 30)
  // arm tab: cameras, arm-glued 3D view and arm control (backend fills defaults for unset fields)
  gripper_camera_topic?: string // sensor_msgs/CompressedImage of the gripper camera
  head_camera_topic?: string // sensor_msgs/CompressedImage of the head camera
  rgb_camera_topic?: string // head tab: sensor_msgs/CompressedImage of the head RGB camera
  tof_depth_topic?: string // head tab: 16UC1 mm depth topic, ToF camera (payload carries depth_preview_b64)
  tof_amplitude_topic?: string // head tab: ToF amplitude topic, unset when the camera has none (payload carries jpeg_b64)
  rgbd_rgb_topic?: string // head tab: head_rgbd RGB (JPEG payload with stamp), subscribed only while the RGB-D view is open
  rgbd_depth_topic?: string // head tab: head_rgbd registered depth (raw uint16 mm payload with stamp), subscribed with rgbd_rgb_topic
  tof_range_m?: number // head tab: depth preview range in metres (also the far end of the RGB-D colormap)
  zoom_command_topic?: string // head tab: zoom request topic (click-to-zoom goes through POST /api/camera/zoom)
  zoom_result_topic?: string // head tab: zoom result topic
  zoom_image_topic?: string // head tab: zoom crop image topic
  zoom_timeout_s?: number // head tab: seconds POST /api/camera/zoom waits for the crop (default 8)
  head_command_topic?: string // head tab: sensor_msgs/JointState {head_pan, head_tilt} in radians; unset hides the pan/tilt controls
  head_joint_states_topic?: string // head, map_nav: sensor_msgs/JointState of the pan/tilt head (measured angles, drives the head URDF)
  head_pan_range_deg?: [number, number] // head tab: pan slider range in degrees
  head_tilt_range_deg?: [number, number] // head tab: tilt slider range in degrees
  head_home_deg?: [number, number] // head tab: Home pose, pan then tilt, degrees
  head_urdf?: string // map_nav: head URDF under /api/urdf/
  gripper_camera_mount?: CameraMount // gripper camera pose in the URDF link gripper_link (same as mcp_server)
  active_source_topic?: string // default /filter/active_source (which command source drives the arm)
  arm_home_service?: string // default /arm/home
  arm_set_home_service?: string // default /arm/set_home
  arm_take_control_service?: string // default /arm/take_control
  arm_release_control_service?: string // default /arm/release_control
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

/** A 2D pose in the map frame (map_manager Pose2D). */
export interface MapPose {
  x: number
  y: number
  yaw: number
}

export type LocalisationState = 'unknown' | 'loading' | 'localised' | 'not_localised' | 'ambiguous'

export interface MapLocalisation {
  state: LocalisationState
  ratio: number | null
  pose: MapPose | null
  message: string
  updated_at: number | null
}

/** One saved named map as listed by GET /api/maps. */
export interface MapSummary {
  name: string
  saved_at: number
  note: string
  poi_count: number
  /** Photos saved with the map; absent in summaries from an older map_manager. */
  photo_count?: number
  last_pose: MapPose
  valid: boolean
  error: string
}

/** The latched map_manager status served by GET /api/maps. */
export interface MapStatus {
  maps: MapSummary[]
  active_map: string | null
  load_seq: number
  localisation: MapLocalisation
}

/** map3d_cloud payload (web_ui.cloud_serializer.serialize_map3d_cloud): little-endian int16 centimetres and uint8 rgb, base64. */
export interface Map3dCloudPayload {
  frame_id: string
  stamp: number
  count: number
  source_count: number
  voxel_m: number
  dropped_out_of_range: number
  points_b64: string
  colors_b64?: string
}
/** map3d_status payload: the map3d manager status JSON. */
export interface Map3dStatusPayload {
  [key: string]: unknown
}

/** map3d_result payload: the result of one map3d command, matched by request_id. */
export interface Map3dResultPayload {
  request_id?: string
  ok?: boolean
  error?: string
  [key: string]: unknown
}
