import type { TabConfig } from "../../main/config";

export abstract class TabBase {
  readonly id: string;
  readonly label: string;
  protected panel: HTMLElement;

  constructor(config: TabConfig) {
    this.id = config.id;
    this.label = config.label;
    this.panel = document.createElement("div");
    this.panel.className = "tab-panel";
    this.panel.dataset.tabId = this.id;
  }

  getPanel(): HTMLElement {
    return this.panel;
  }

  /** Called when the tab becomes visible. */
  activate(): void {}

  /** Called when the tab is hidden. */
  deactivate(): void {}

  /** Called with a parsed ROS2 message when a subscribed topic has new data. */
  abstract onMessage(topic: string, data: Record<string, unknown>): void;

  /** Returns all topic strings this tab subscribes to. */
  abstract getTopics(): string[];
}
