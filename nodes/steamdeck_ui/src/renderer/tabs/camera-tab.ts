import { TabBase } from "./tab-base";
import type { TabConfig } from "../../main/config";

export class CameraTab extends TabBase {
  private topic: string;
  private img: HTMLImageElement;
  private placeholder: HTMLElement;
  private frameCount = 0;

  constructor(config: TabConfig) {
    super(config);
    this.topic = config.topic ?? "";

    const container = document.createElement("div");
    container.className = "camera-container";

    this.img = document.createElement("img");
    this.img.className = "camera-img";
    this.img.alt = "camera feed";
    this.img.style.display = "none";

    this.placeholder = document.createElement("div");
    this.placeholder.className = "camera-placeholder";
    this.placeholder.textContent = `Waiting for ${this.topic}…`;

    container.appendChild(this.img);
    container.appendChild(this.placeholder);
    this.panel.appendChild(container);
  }

  getTopics(): string[] {
    return this.topic ? [this.topic] : [];
  }

  onMessage(topic: string, data: Record<string, unknown>): void {
    if (topic !== this.topic) return;
    const jpeg = data["jpeg_b64"] as string | undefined;
    if (!jpeg) return;

    this.frameCount++;
    if (this.frameCount === 1) {
      this.placeholder.style.display = "none";
      this.img.style.display = "block";
    }
    this.img.src = `data:image/jpeg;base64,${jpeg}`;
  }
}
