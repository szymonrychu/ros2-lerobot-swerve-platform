import { contextBridge, ipcRenderer } from "electron";
import type { AppConfig } from "./config";

contextBridge.exposeInMainWorld("electronAPI", {
  getConfig: (): Promise<AppConfig> => ipcRenderer.invoke("get-config"),
});

declare global {
  interface Window {
    electronAPI: {
      getConfig: () => Promise<AppConfig>;
    };
  }
}
