/** Browser storage used by the Agent tab, cleared when a new session starts. */

/** Every localStorage / sessionStorage key of the Agent tab starts with this. */
export const AGENT_STORAGE_PREFIX = 'web_ui.agent.'

function browserStorages(): Storage[] {
  const found: Storage[] = []
  for (const name of ['localStorage', 'sessionStorage'] as const) {
    try {
      found.push(window[name])
    } catch {
      // Storage blocked (private window, site data disabled): nothing to clear there.
    }
  }
  return found
}

/**
 * Remove the Agent tab's keys from the given storages (default: localStorage and sessionStorage). Never throws.
 * @param storages storages to clean
 */
export function clearAgentStorage(storages: Storage[] = browserStorages()): void {
  for (const storage of storages) {
    try {
      const keys: string[] = []
      for (let i = 0; i < storage.length; i++) {
        const key = storage.key(i)
        if (key !== null && key.startsWith(AGENT_STORAGE_PREFIX)) keys.push(key)
      }
      keys.forEach((key) => storage.removeItem(key))
    } catch {
      // Unreadable storage: skip it.
    }
  }
}
