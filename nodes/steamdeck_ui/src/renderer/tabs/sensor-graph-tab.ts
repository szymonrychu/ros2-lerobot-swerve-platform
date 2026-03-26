import uPlot from "uplot";
import { TabBase } from "./tab-base";
import { extractField } from "../utils/field-extract";
import type { TabConfig, TabTopicSpec } from "../../main/config";

interface SeriesPoint {
  time: number;
  value: number;
}

export class SensorGraphTab extends TabBase {
  protected topicSpecs: TabTopicSpec[];
  protected windowS: number;
  protected maxPoints: number;
  private uplot: uPlot | null = null;
  private seriesData: SeriesPoint[][];
  private allTopics: string[];
  private container: HTMLElement;
  private redrawTimer: ReturnType<typeof setInterval> | null = null;

  constructor(config: TabConfig) {
    super(config);
    this.topicSpecs = config.topics ?? [];
    this.windowS = config.window_s ?? 10;
    this.maxPoints = config.max_points ?? 500;

    this.container = document.createElement("div");
    this.container.className = "graph-container";
    this.panel.appendChild(this.container);

    this.allTopics = [...new Set(this.topicSpecs.map((t) => t.topic))];
    const fieldCount = this.topicSpecs.reduce((n, t) => n + t.fields.length, 0);
    this.seriesData = Array.from({ length: fieldCount }, () => []);
  }

  getTopics(): string[] {
    return this.allTopics;
  }

  activate(): void {
    if (!this.uplot) requestAnimationFrame(() => this.initPlot());
    if (!this.redrawTimer) {
      this.redrawTimer = setInterval(() => this.redraw(Date.now() / 1000), 100);
    }
  }

  deactivate(): void {
    if (this.redrawTimer) {
      clearInterval(this.redrawTimer);
      this.redrawTimer = null;
    }
  }

  private buildSeriesDefs(): uPlot.Series[] {
    const defs: uPlot.Series[] = [{}]; // time axis placeholder
    for (const spec of this.topicSpecs) {
      for (const field of spec.fields) {
        defs.push({
          label: field.label,
          stroke: field.color ?? "#333333",
          width: 1.5,
          points: { show: false },
        });
      }
    }
    return defs;
  }

  private initPlot(): void {
    try {
      // Measure #tab-content — it always has a definite size from the app flex layout
      const tabContent = document.getElementById("tab-content");
      const w = tabContent ? tabContent.clientWidth - 32 : 1248;
      // Subtract padding (32) + legend rows (~20px each)
      const legendH = this.seriesData.length * 20 + 8;
      const h = (tabContent ? tabContent.clientHeight : 696) - 32 - legendH;
      console.log(`[graph] initPlot id=${this.id} w=${w} h=${h} legendH=${legendH}`);

      const now = Date.now() / 1000;
      const emptyData: number[][] = [
        [now - this.windowS, now],
        ...this.seriesData.map(() => [0, 0]),
      ];

      const opts: uPlot.Options = {
        width: w,
        height: h,
        series: this.buildSeriesDefs(),
        axes: [
          {
            stroke: "#444444",
            grid: { stroke: "#eeeeee" },
            ticks: { stroke: "#dddddd" },
          },
          {
            stroke: "#444444",
            grid: { stroke: "#eeeeee" },
            ticks: { stroke: "#dddddd" },
          },
        ],
        scales: { x: { time: false } },
        legend: { show: true },
      };

      this.uplot = new uPlot(opts, emptyData as uPlot.AlignedData, this.container);
    } catch (err) {
      console.error("[graph] initPlot failed:", err);
      const msg = document.createElement("div");
      msg.style.cssText = "color:#c00;padding:16px;font-size:12px;";
      msg.textContent = `Graph init error: ${err}`;
      this.container.appendChild(msg);
    }
  }

  onMessage(topic: string, data: Record<string, unknown>): void {
    if ((window as unknown as Record<string, unknown>).__STEAMDECK_DEBUG__) {
      console.log(`[graph ${new Date().toISOString()}] ${topic} id=${this.id}`);
    }
    const now = Date.now() / 1000;
    let seriesIdx = 0;
    for (const spec of this.topicSpecs) {
      if (spec.topic === topic) {
        for (const field of spec.fields) {
          const val = extractField(data, field.path);
          if (val !== null) {
            this.seriesData[seriesIdx].push({ time: now, value: val });
          }
          seriesIdx++;
        }
      } else {
        seriesIdx += spec.fields.length;
      }
    }
    this.trimData(now);
  }

  private trimData(now: number): void {
    const cutoff = now - this.windowS;
    for (const series of this.seriesData) {
      while (series.length > 0 && series[0].time < cutoff) series.shift();
      while (series.length > this.maxPoints) series.shift();
    }
  }

  private redraw(now: number): void {
    if (!this.uplot) return;
    const times = this.seriesData[0]?.map((p) => p.time) ?? [now - this.windowS, now];
    const data: uPlot.AlignedData = [
      times.length > 0 ? times : [now - this.windowS, now],
      ...this.seriesData.map((s) => s.map((p) => p.value)),
    ];
    this.uplot.setData(data);
    this.uplot.setScale("x", { min: now - this.windowS, max: now });
  }
}
