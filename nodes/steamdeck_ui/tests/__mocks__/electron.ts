// Minimal Electron mock for Jest
export const app = { whenReady: jest.fn(), on: jest.fn(), quit: jest.fn() };
export const BrowserWindow = jest.fn(() => ({
  loadFile: jest.fn(),
  webContents: { openDevTools: jest.fn() },
}));
export const ipcMain = { handle: jest.fn() };
export const ipcRenderer = { invoke: jest.fn() };
export const contextBridge = { exposeInMainWorld: jest.fn() };
