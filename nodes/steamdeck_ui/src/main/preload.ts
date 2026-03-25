import { contextBridge, ipcRenderer } from "electron";
import type { AppConfig } from "./config";

contextBridge.exposeInMainWorld("electronAPI", {
  getConfig: (): Promise<AppConfig> => ipcRenderer.invoke("get-config"),
  quit: (): void => ipcRenderer.send("quit"),
});

declare global {
  interface Window {
    electronAPI: {
      getConfig: () => Promise<AppConfig>;
      quit: () => void;
    };
  }
}
