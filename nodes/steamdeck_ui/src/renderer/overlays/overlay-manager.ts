import { extractField, formatValue } from "../utils/field-extract";
import type { OverlayItem } from "../../main/config";

interface OverlayEntry {
  spec: OverlayItem;
  valueEl: HTMLElement;
}

export class OverlayManager {
  private entries: OverlayEntry[] = [];
  private topicMap: Map<string, OverlayEntry[]> = new Map();
  private bar: HTMLElement;

  constructor(container: HTMLElement, specs: OverlayItem[]) {
    this.bar = container;

    for (const spec of specs) {
      const item = document.createElement("div");
      item.className = "overlay-item";

      const label = document.createElement("span");
      label.className = "overlay-label";
      label.textContent = spec.label;

      const value = document.createElement("span");
      value.className = "overlay-value";
      value.textContent = "—";

      item.appendChild(label);
      item.appendChild(value);
      this.bar.appendChild(item);

      const entry: OverlayEntry = { spec, valueEl: value };
      this.entries.push(entry);

      const existing = this.topicMap.get(spec.topic) ?? [];
      existing.push(entry);
      this.topicMap.set(spec.topic, existing);
    }
  }

  getTopics(): string[] {
    return [...this.topicMap.keys()];
  }

  onMessage(topic: string, data: Record<string, unknown>): void {
    if ((window as unknown as Record<string, unknown>).__STEAMDECK_DEBUG__) {
      console.log(`[overlay ${new Date().toISOString()}] ${topic}`);
    }
    const entries = this.topicMap.get(topic);
    if (!entries) return;
    for (const entry of entries) {
      const val = extractField(data, entry.spec.field);
      if (val !== null) {
        entry.valueEl.textContent = formatValue(val, entry.spec.format, entry.spec.unit);
      }
    }
  }
}
