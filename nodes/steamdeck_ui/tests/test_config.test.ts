import * as fs from "fs";
import * as path from "path";
import * as os from "os";
import { loadConfig } from "../src/main/config";

function writeTmpConfig(data: string): string {
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), "steamdeck-ui-test-"));
  const filePath = path.join(dir, "config.yaml");
  fs.writeFileSync(filePath, data, "utf-8");
  return filePath;
}

describe("loadConfig", () => {
  test("loads a valid config file", () => {
    const p = writeTmpConfig(`
bridge:
  host: localhost
  port: 9090
tabs:
  - id: cam
    type: camera
    label: "Camera"
    topic: /controller/head_rgbd/image_raw
overlays: []
`);
    const cfg = loadConfig(p);
    expect(cfg.bridge.host).toBe("localhost");
    expect(cfg.bridge.port).toBe(9090);
    expect(cfg.tabs).toHaveLength(1);
    expect(cfg.tabs[0].type).toBe("camera");
  });

  test("throws when file does not exist", () => {
    expect(() => loadConfig("/nonexistent/config.yaml")).toThrow();
  });

  test("throws when bridge.port is missing", () => {
    const p = writeTmpConfig(`
bridge:
  host: localhost
tabs: []
overlays: []
`);
    expect(() => loadConfig(p)).toThrow();
  });

  test("tabs is array", () => {
    const p = writeTmpConfig(`
bridge:
  host: localhost
  port: 9090
tabs:
  - id: g
    type: sensor_graph
    label: IMU
    topics:
      - topic: /controller/imu/data
        fields:
          - path: linear_acceleration.x
            label: Ax
overlays: []
`);
    const cfg = loadConfig(p);
    expect(cfg.tabs[0].topics).toBeDefined();
    expect(cfg.tabs[0].topics![0].topic).toBe("/controller/imu/data");
  });

  test("missing overlays defaults to empty array", () => {
    const p = writeTmpConfig(`
bridge:
  host: localhost
  port: 9090
tabs: []
`);
    const cfg = loadConfig(p);
    expect(cfg.overlays).toEqual([]);
  });

  test("no config names the retired camera_0 gripper camera", () => {
    const root = path.resolve(__dirname, "..", "..", "..");
    const files = [
      path.join(root, "nodes/steamdeck_ui/config/default.yaml"),
      path.join(root, "nodes/steamdeck_ui/bridge/bridge_server.py"),
      path.join(root, "ansible/roles/steamdeck_ui/templates/config.yaml.j2"),
    ];
    for (const f of files) {
      expect(fs.readFileSync(f, "utf-8")).not.toContain("camera_0");
    }
    expect(loadConfig(files[0]).tabs.map((t) => t.topic)).not.toContain(
      "/controller/camera_0/image_raw",
    );
  });
});
