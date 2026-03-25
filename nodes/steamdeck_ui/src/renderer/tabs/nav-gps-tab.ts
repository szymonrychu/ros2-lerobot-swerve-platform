import L from "leaflet";
import { TabBase } from "./tab-base";
import type { TabConfig } from "../../main/config";

export class NavGpsTab extends TabBase {
  private fixTopic: string;
  private goalTopic: string;
  private tileUrl: string;
  private defaultZoom: number;
  private map: L.Map | null = null;
  private marker: L.CircleMarker | null = null;
  private mapContainer: HTMLElement;

  constructor(config: TabConfig) {
    super(config);
    this.fixTopic = config.fix_topic ?? "/controller/gps/fix";
    this.goalTopic = config.goal_topic ?? "/controller/goal_pose";
    this.tileUrl = config.tile_url ?? "https://{s}.basemaps.cartocdn.com/light_all/{z}/{x}/{y}{r}.png";
    this.defaultZoom = config.default_zoom ?? 18;

    const container = document.createElement("div");
    container.className = "nav-container";

    this.mapContainer = document.createElement("div");
    this.mapContainer.id = `nav-gps-map-${this.id}`;
    this.mapContainer.style.width = "100%";
    this.mapContainer.style.height = "100%";

    container.appendChild(this.mapContainer);
    this.panel.appendChild(container);
  }

  getTopics(): string[] {
    return [this.fixTopic];
  }

  activate(): void {
    if (!this.map) this.initMap();
  }

  private initMap(): void {
    this.map = L.map(this.mapContainer, {
      center: [51.5, 0],
      zoom: this.defaultZoom,
      zoomControl: true,
      attributionControl: false,
    });

    L.tileLayer(this.tileUrl, { maxZoom: 22 }).addTo(this.map);

    this.map.on("click", (e: L.LeafletMouseEvent) => {
      this.publishGeoGoal(e.latlng.lat, e.latlng.lng);
    });
  }

  onMessage(topic: string, data: Record<string, unknown>): void {
    if (topic !== this.fixTopic) return;
    const lat = data["latitude"] as number | undefined;
    const lng = data["longitude"] as number | undefined;
    const status = (data["status"] as Record<string, number> | undefined)?.["status"];
    if (lat === undefined || lng === undefined) return;
    if (!isFinite(lat) || !isFinite(lng)) return;

    if (!this.map) return;

    if (!this.marker) {
      this.marker = L.circleMarker([lat, lng], {
        radius: 8,
        color: "#222222",
        fillColor: status !== undefined && status >= 0 ? "#444444" : "#888888",
        fillOpacity: 1,
        weight: 2,
      }).addTo(this.map);
      this.map.setView([lat, lng], this.defaultZoom);
    } else {
      this.marker.setLatLng([lat, lng]);
      this.map.setView([lat, lng]);
    }
  }

  private publishGeoGoal(lat: number, lng: number): void {
    const event = new CustomEvent("ros-publish", {
      detail: {
        topic: this.goalTopic,
        msg_type: "geographic_msgs/GeoPoseStamped",
        data: {
          header: { frame_id: "map" },
          pose: {
            position: { latitude: lat, longitude: lng, altitude: 0 },
            orientation: { x: 0, y: 0, z: 0, w: 1 },
          },
        },
      },
    });
    document.dispatchEvent(event);
  }
}
