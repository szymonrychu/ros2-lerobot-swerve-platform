import { TabBase } from "./tab-base";
import { extractField } from "../utils/field-extract";
import type { TabConfig } from "../../main/config";

interface OccupancyGrid {
  info: {
    resolution: number;
    width: number;
    height: number;
    origin: { position: { x: number; y: number } };
  };
  data: number[];
}

interface RobotPose {
  x: number;
  y: number;
  yaw: number;
}

export class NavLocalTab extends TabBase {
  private scanTopic: string;
  private costmapTopic: string;
  private odomTopic: string;
  private goalTopic: string;
  private canvas: HTMLCanvasElement;
  private ctx: CanvasRenderingContext2D;
  private grid: OccupancyGrid | null = null;
  private scanPoints: [number, number][] = [];
  private robotPose: RobotPose | null = null;

  constructor(config: TabConfig) {
    super(config);
    this.scanTopic = config.scan_topic ?? "/controller/scan";
    this.costmapTopic = config.costmap_topic ?? "/controller/local_costmap";
    this.odomTopic = config.odom_topic ?? "/controller/odom";
    this.goalTopic = config.goal_topic ?? "/controller/goal_pose";

    const container = document.createElement("div");
    container.className = "nav-container";

    this.canvas = document.createElement("canvas");
    this.canvas.id = "nav-local-canvas";
    this.canvas.width = 1280;
    this.canvas.height = 720;
    this.ctx = this.canvas.getContext("2d") as CanvasRenderingContext2D;

    container.appendChild(this.canvas);
    this.panel.appendChild(container);

    this.canvas.addEventListener("click", (e) => this.onCanvasClick(e));
  }

  getTopics(): string[] {
    return [this.scanTopic, this.costmapTopic, this.odomTopic];
  }

  onMessage(topic: string, data: Record<string, unknown>): void {
    if (topic === this.costmapTopic) {
      this.grid = data as unknown as OccupancyGrid;
      this.render();
    } else if (topic === this.scanTopic) {
      this.parseScan(data);
      this.render();
    } else if (topic === this.odomTopic) {
      this.parsePose(data);
      this.render();
    }
  }

  private parseScan(data: Record<string, unknown>): void {
    const ranges = data["ranges"] as number[] | undefined;
    const angleMin = extractField(data, "angle_min") ?? 0;
    const angleIncrement = extractField(data, "angle_increment") ?? 0.01;
    if (!ranges) return;
    this.scanPoints = [];
    for (let i = 0; i < ranges.length; i++) {
      const r = ranges[i];
      if (r > 0 && isFinite(r) && r < 30) {
        const angle = angleMin + i * angleIncrement;
        this.scanPoints.push([r * Math.cos(angle), r * Math.sin(angle)]);
      }
    }
  }

  private parsePose(data: Record<string, unknown>): void {
    const x = extractField(data, "pose.pose.position.x");
    const y = extractField(data, "pose.pose.position.y");
    const qz = extractField(data, "pose.pose.orientation.z") ?? 0;
    const qw = extractField(data, "pose.pose.orientation.w") ?? 1;
    if (x !== null && y !== null) {
      this.robotPose = { x, y, yaw: 2 * Math.atan2(qz, qw) };
    }
  }

  private render(): void {
    const W = this.canvas.width;
    const H = this.canvas.height;
    this.ctx.clearRect(0, 0, W, H);
    this.ctx.fillStyle = "#888888";
    this.ctx.fillRect(0, 0, W, H);

    if (!this.grid) return;

    const { resolution, width, height, origin } = this.grid.info;
    const scale = Math.min(W / width, H / height);

    const offsetX = (W - width * scale) / 2;
    const offsetY = (H - height * scale) / 2;

    // Draw costmap
    const imgData = this.ctx.createImageData(width, height);
    for (let i = 0; i < this.grid.data.length; i++) {
      const v = this.grid.data[i];
      let r: number, g: number, b: number;
      if (v === -1) { r = 128; g = 128; b = 128; }
      else if (v === 0) { r = 240; g = 240; b = 240; }
      else { const t = 1 - v / 100; r = Math.round(t * 240); g = r; b = r; }
      imgData.data[i * 4] = r;
      imgData.data[i * 4 + 1] = g;
      imgData.data[i * 4 + 2] = b;
      imgData.data[i * 4 + 3] = 255;
    }

    const tmpCanvas = document.createElement("canvas");
    tmpCanvas.width = width;
    tmpCanvas.height = height;
    const tmpCtx = tmpCanvas.getContext("2d") as CanvasRenderingContext2D;
    tmpCtx.putImageData(imgData, 0, 0);
    this.ctx.drawImage(tmpCanvas, offsetX, offsetY, width * scale, height * scale);

    // Convert world coords to canvas pixels
    const toCanvas = (wx: number, wy: number): [number, number] => {
      const px = ((wx - origin.position.x) / resolution) * scale + offsetX;
      const py = H - (((wy - origin.position.y) / resolution) * scale + offsetY);
      return [px, py];
    };

    // Draw scan points
    this.ctx.fillStyle = "#333333";
    for (const [wx, wy] of this.scanPoints) {
      const rx = this.robotPose?.x ?? 0;
      const ry = this.robotPose?.y ?? 0;
      const yaw = this.robotPose?.yaw ?? 0;
      const worldX = rx + wx * Math.cos(yaw) - wy * Math.sin(yaw);
      const worldY = ry + wx * Math.sin(yaw) + wy * Math.cos(yaw);
      const [cx, cy] = toCanvas(worldX, worldY);
      this.ctx.fillRect(cx - 1, cy - 1, 2, 2);
    }

    // Draw robot
    if (this.robotPose) {
      const [cx, cy] = toCanvas(this.robotPose.x, this.robotPose.y);
      this.ctx.strokeStyle = "#111111";
      this.ctx.lineWidth = 2;
      this.ctx.beginPath();
      this.ctx.arc(cx, cy, 6, 0, Math.PI * 2);
      this.ctx.stroke();
      // Direction arrow
      const arrowLen = 14;
      this.ctx.beginPath();
      this.ctx.moveTo(cx, cy);
      this.ctx.lineTo(
        cx + arrowLen * Math.cos(-this.robotPose.yaw + Math.PI / 2),
        cy + arrowLen * Math.sin(-this.robotPose.yaw + Math.PI / 2)
      );
      this.ctx.stroke();
    }
  }

  private onCanvasClick(e: MouseEvent): void {
    if (!this.grid || !this.robotPose) return;
    const rect = this.canvas.getBoundingClientRect();
    const scaleX = this.canvas.width / rect.width;
    const scaleY = this.canvas.height / rect.height;
    const canvasX = (e.clientX - rect.left) * scaleX;
    const canvasY = (e.clientY - rect.top) * scaleY;

    const { resolution, width, height, origin } = this.grid.info;
    const scale = Math.min(this.canvas.width / width, this.canvas.height / height);
    const offsetX = (this.canvas.width - width * scale) / 2;
    const offsetY = (this.canvas.height - height * scale) / 2;

    const worldX = (canvasX - offsetX) / scale * resolution + origin.position.x;
    const worldY = (this.canvas.height - canvasY - offsetY) / scale * resolution + origin.position.y;

    this.publishGoal(worldX, worldY);
  }

  private publishGoal(x: number, y: number): void {
    const event = new CustomEvent("ros-publish", {
      detail: {
        topic: this.goalTopic,
        msg_type: "geometry_msgs/PoseStamped",
        data: {
          header: { frame_id: "map" },
          pose: {
            position: { x, y, z: 0 },
            orientation: { x: 0, y: 0, z: 0, w: 1 },
          },
        },
      },
    });
    document.dispatchEvent(event);
  }
}
