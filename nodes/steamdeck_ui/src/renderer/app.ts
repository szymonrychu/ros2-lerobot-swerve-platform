import type { AppConfig, TabConfig } from "../main/config";
import { TabBase } from "./tabs/tab-base";
import { CameraTab } from "./tabs/camera-tab";
import { SensorGraphTab } from "./tabs/sensor-graph-tab";
import { EffectorGraphTab } from "./tabs/effector-graph-tab";
import { NavLocalTab } from "./tabs/nav-local-tab";
import { NavGpsTab } from "./tabs/nav-gps-tab";
import { OverlayManager } from "./overlays/overlay-manager";

const BASE_RECONNECT_MS = 1000;
const MAX_RECONNECT_MS = 10000;
const DEBUG = new URLSearchParams(window.location.search).has("debug")
  || !!(window as unknown as Record<string, unknown>).__STEAMDECK_DEBUG__;

interface BridgeMessage {
  type: "topic_data";
  topic: string;
  data: Record<string, unknown>;
}

interface PublishRequest {
  type: "publish";
  topic: string;
  msg_type: string;
  data: Record<string, unknown>;
}

class App {
  private config!: AppConfig;
  private tabs: TabBase[] = [];
  private activeTabId: string | null = null;
  private ws: WebSocket | null = null;
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  private reconnectDelayMs = BASE_RECONNECT_MS;
  private lastHeartbeatMs = Date.now();
  private topicToTabs: Map<string, TabBase[]> = new Map();
  private overlayManager: OverlayManager | null = null;
  private statusDot!: HTMLElement;

  async init(): Promise<void> {
    this.config = await window.electronAPI.getConfig();
    this.buildUI();
    this.connect();
  }

  private buildUI(): void {
    const tabBar = document.getElementById("tab-bar") as HTMLElement;
    const content = document.getElementById("tab-content") as HTMLElement;
    const overlayBar = document.getElementById("overlay-bar") as HTMLElement;

    // Status dot
    this.statusDot = document.createElement("div");
    this.statusDot.id = "status-dot";
    tabBar.appendChild(this.statusDot);

    // Menu button (right-aligned)
    this.buildMenuButton(tabBar);

    // Build tabs
    for (const tabCfg of this.config.tabs) {
      const tab = this.createTab(tabCfg);
      if (!tab) continue;
      this.tabs.push(tab);

      // Register topic -> tab mapping
      for (const topic of tab.getTopics()) {
        const existing = this.topicToTabs.get(topic) ?? [];
        existing.push(tab);
        this.topicToTabs.set(topic, existing);
      }

      content.appendChild(tab.getPanel());
    }

    // Tab buttons — prepend before status dot
    for (const tab of this.tabs) {
      const btn = document.createElement("button");
      btn.className = "tab-btn";
      btn.textContent = tab.label;
      btn.dataset.tabId = tab.id;
      btn.addEventListener("click", () => this.activateTab(tab.id));
      tabBar.insertBefore(btn, this.statusDot);
    }

    // Overlays
    this.overlayManager = new OverlayManager(overlayBar, this.config.overlays);
    for (const topic of this.overlayManager.getTopics()) {
      // overlays also subscribe via the same topicToTabs mechanism (re-use a sentinel)
      const existing = this.topicToTabs.get(topic) ?? [];
      this.topicToTabs.set(topic, existing); // ensure key exists for subscription
    }

    // Activate first tab
    if (this.tabs.length > 0) this.activateTab(this.tabs[0].id);

    // Listen for publish requests from tabs
    document.addEventListener("ros-publish", (e: Event) => {
      const detail = (e as CustomEvent<PublishRequest["data"]>).detail as PublishRequest;
      this.send({ type: "publish", topic: detail.topic, msg_type: detail.msg_type, data: detail.data });
    });
  }

  private buildMenuButton(tabBar: HTMLElement): void {
    const btn = document.createElement("button");
    btn.id = "menu-btn";
    btn.className = "tab-btn";
    btn.textContent = "≡ MENU";
    btn.style.marginLeft = "auto";
    tabBar.appendChild(btn);

    const menu = document.createElement("div");
    menu.id = "app-menu";
    menu.style.cssText =
      "display:none;position:fixed;top:48px;right:0;background:#222;border:1px solid #444;" +
      "z-index:9999;min-width:160px;flex-direction:column;";
    document.body.appendChild(menu);

    const exitBtn = document.createElement("button");
    exitBtn.textContent = "Exit";
    exitBtn.style.cssText =
      "width:100%;padding:14px 20px;background:transparent;color:#fff;border:none;" +
      "border-bottom:1px solid #444;font-family:inherit;font-size:14px;text-align:left;cursor:pointer;";
    exitBtn.addEventListener("click", () => window.electronAPI.quit());
    menu.appendChild(exitBtn);

    btn.addEventListener("click", (e) => {
      e.stopPropagation();
      menu.style.display = menu.style.display === "flex" ? "none" : "flex";
    });
    document.addEventListener("click", () => {
      menu.style.display = "none";
    });
  }

  private createTab(cfg: TabConfig): TabBase | null {
    switch (cfg.type) {
      case "camera": return new CameraTab(cfg);
      case "sensor_graph": return new SensorGraphTab(cfg);
      case "effector_graph": return new EffectorGraphTab(cfg);
      case "nav_local": return new NavLocalTab(cfg);
      case "nav_gps": return new NavGpsTab(cfg);
      default: console.warn("Unknown tab type:", cfg.type); return null;
    }
  }

  private activateTab(id: string): void {
    if (this.activeTabId) {
      const prev = this.tabs.find((t) => t.id === this.activeTabId);
      prev?.deactivate();
      prev?.getPanel().classList.remove("active");
    }
    const tab = this.tabs.find((t) => t.id === id);
    if (!tab) return;
    this.activeTabId = id;
    tab.getPanel().classList.add("active");
    tab.activate();

    // Update button states
    document.querySelectorAll(".tab-btn").forEach((btn) => {
      const el = btn as HTMLElement;
      el.classList.toggle("active", el.dataset.tabId === id);
    });
  }

  private connect(): void {
    const { host, port } = this.config.bridge;
    const url = `ws://${host}:${port}`;
    this.ws = new WebSocket(url);

    this.ws.onopen = () => {
      console.log(`[bridge] connected to ${url}`);
      this.reconnectDelayMs = BASE_RECONNECT_MS;
      this.lastHeartbeatMs = Date.now();
      this.setStatus("connected");
      // Subscribe to all needed topics
      const topics = [...this.topicToTabs.keys()];
      if (this.overlayManager) {
        for (const t of this.overlayManager.getTopics()) {
          if (!topics.includes(t)) topics.push(t);
        }
      }
      this.send({ type: "subscribe", topics } as unknown as PublishRequest);
    };

    this.ws.onmessage = (ev) => {
      try {
        const msg = JSON.parse(ev.data as string) as BridgeMessage & { type: string; ts?: number };
        if (msg.type === "heartbeat") {
          this.lastHeartbeatMs = Date.now();
          return;
        }
        if (msg.type === "topic_data") {
          if (DEBUG) {
            console.log(`[bridge ${new Date().toISOString()}] ${msg.topic} (${(ev.data as string).length} bytes)`);
          }
          this.dispatchMessage(msg.topic, msg.data);
        }
      } catch { /* ignore parse errors */ }
    };

    setInterval(() => {
      if (Date.now() - this.lastHeartbeatMs > 10000) this.setStatus("stale");
    }, 3000);

    this.ws.onclose = () => {
      console.log("[bridge] disconnected, reconnecting...");
      this.setStatus("error");
      this.scheduleReconnect();
    };

    this.ws.onerror = () => {
      this.setStatus("error");
    };
  }

  private dispatchMessage(topic: string, data: Record<string, unknown>): void {
    const tabs = this.topicToTabs.get(topic) ?? [];
    for (const tab of tabs) tab.onMessage(topic, data);
    this.overlayManager?.onMessage(topic, data);
  }

  private send(msg: PublishRequest | Record<string, unknown>): void {
    if (this.ws?.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(msg));
    }
  }

  private scheduleReconnect(): void {
    if (this.reconnectTimer) return;
    this.reconnectTimer = setTimeout(() => {
      this.reconnectTimer = null;
      this.connect();
    }, this.reconnectDelayMs);
    this.reconnectDelayMs = Math.min(this.reconnectDelayMs * 2, MAX_RECONNECT_MS);
  }

  private setStatus(state: "connected" | "error" | "stale"): void {
    this.statusDot.className = state;
  }
}

document.addEventListener("DOMContentLoaded", () => {
  new App().init().catch(console.error);
});
