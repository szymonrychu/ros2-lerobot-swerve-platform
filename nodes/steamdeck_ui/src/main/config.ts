import * as fs from "fs";
import * as path from "path";
import * as yaml from "js-yaml";

export interface OverlayItem {
  topic: string;
  field: string;
  label: string;
  format?: string;
  unit?: string;
}

export interface TabFieldSpec {
  path: string;
  label: string;
  color?: string;
}

export interface TabTopicSpec {
  topic: string;
  fields: TabFieldSpec[];
}

export interface TabConfig {
  id: string;
  type: "camera" | "sensor_graph" | "effector_graph" | "nav_local" | "nav_gps";
  label: string;
  // camera
  topic?: string;
  // graph tabs
  topics?: TabTopicSpec[];
  window_s?: number;
  max_points?: number;
  // nav_local
  scan_topic?: string;
  costmap_topic?: string;
  odom_topic?: string;
  goal_topic?: string;
  // nav_gps
  fix_topic?: string;
  tile_url?: string;
  default_zoom?: number;
}

export interface BridgeConfig {
  host: string;
  port: number;
  ros_static_peers: string;
  ros_domain_id?: string;
}

export interface AppConfig {
  bridge: BridgeConfig;
  tabs: TabConfig[];
  overlays: OverlayItem[];
}

const DEFAULT_CONFIG_PATHS = [
  "/etc/steamdeck-ui/config.yaml",
  path.join(__dirname, "../../config/default.yaml"),
];

export function loadConfig(configPath?: string): AppConfig {
  const searchPaths = configPath ? [configPath] : DEFAULT_CONFIG_PATHS;
  for (const p of searchPaths) {
    if (fs.existsSync(p)) {
      const raw = fs.readFileSync(p, "utf-8");
      const parsed = yaml.load(raw) as AppConfig;
      return validateConfig(parsed, p);
    }
  }
  throw new Error(`No config file found. Searched: ${searchPaths.join(", ")}`);
}

function validateConfig(cfg: AppConfig, filePath: string): AppConfig {
  if (!cfg || typeof cfg !== "object") {
    throw new Error(`Invalid config in ${filePath}: must be a YAML object`);
  }
  if (!cfg.bridge || typeof cfg.bridge.port !== "number") {
    throw new Error(`Invalid config in ${filePath}: bridge.port must be a number`);
  }
  if (!Array.isArray(cfg.tabs)) {
    throw new Error(`Invalid config in ${filePath}: tabs must be an array`);
  }
  if (!Array.isArray(cfg.overlays)) {
    cfg.overlays = [];
  }
  return cfg;
}
