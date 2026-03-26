import { app, BrowserWindow, ipcMain } from "electron";
import * as path from "path";
import { loadConfig, AppConfig } from "./config";

let config: AppConfig;

function createWindow(): void {
  const win = new BrowserWindow({
    width: 1280,
    height: 800,
    fullscreen: process.argv.includes("--fullscreen"),
    frame: false,
    backgroundColor: "#ffffff",
    webPreferences: {
      preload: path.join(__dirname, "preload.js"),
      contextIsolation: true,
      nodeIntegration: false,
    },
  });

  win.loadFile(path.join(__dirname, "../../src/renderer/index.html"));

  const debugMode = process.argv.includes("--dev") || process.argv.includes("--debug");
  if (debugMode) {
    win.webContents.openDevTools({ mode: "detach" });
  }
  if (process.argv.includes("--debug")) {
    win.webContents.on("did-finish-load", () => {
      win.webContents.executeJavaScript("window.__STEAMDECK_DEBUG__ = true;").catch(() => {});
    });
  }
}

function parseConfigPath(): string | undefined {
  const idx = process.argv.indexOf("--config");
  return idx !== -1 ? process.argv[idx + 1] : undefined;
}

app.whenReady().then(() => {
  const configPath = parseConfigPath();
  try {
    config = loadConfig(configPath);
  } catch (err) {
    console.error("Failed to load config:", err);
    app.quit();
    return;
  }

  ipcMain.handle("get-config", () => config);
  ipcMain.on("quit", () => app.quit());

  createWindow();

  app.on("activate", () => {
    if (BrowserWindow.getAllWindows().length === 0) createWindow();
  });
});

app.on("window-all-closed", () => {
  if (process.platform !== "darwin") app.quit();
});
