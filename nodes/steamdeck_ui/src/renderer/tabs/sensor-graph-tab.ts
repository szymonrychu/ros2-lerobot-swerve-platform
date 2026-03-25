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
    if (!this.uplot) this.initPlot();
  }

  deactivate(): void {
    // keep the plot alive to preserve data across tab switches
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
    const rect = this.container.getBoundingClientRect();
    const w = rect.width > 0 ? rect.width : 1200;
    const h = rect.height > 0 ? rect.height - 32 : 600;

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
          values: (_u, ticks) => ticks.map((t) => `${Math.round(t * 10) / 10}s`),
        },
        {
          stroke: "#444444",
          grid: { stroke: "#eeeeee" },
          ticks: { stroke: "#dddddd" },
        },
      ],
      scales: { x: { time: false } },
      cursor: { show: false },
      select: { show: false },
      legend: { show: true },
    };

    this.uplot = new uPlot(opts, emptyData as uPlot.AlignedData, this.container);
  }

  onMessage(topic: string, data: Record<string, unknown>): void {
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
    this.redraw(now);
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
