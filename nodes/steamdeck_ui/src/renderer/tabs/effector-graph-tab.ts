import { SensorGraphTab } from "./sensor-graph-tab";
import type { TabConfig } from "../../main/config";

/**
 * Effector graph tab — identical to SensorGraphTab with effector-appropriate defaults.
 * Servo angles are in radians; Y axis auto-scales.
 */
export class EffectorGraphTab extends SensorGraphTab {
  constructor(config: TabConfig) {
    super({
      ...config,
      window_s: config.window_s ?? 10,
      max_points: config.max_points ?? 500,
    });
  }

  getTopics(): string[] {
    return [...new Set((this.topicSpecs ?? []).map((t) => t.topic))];
  }
}
