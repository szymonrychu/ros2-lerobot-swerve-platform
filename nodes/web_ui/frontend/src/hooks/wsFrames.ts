/** Parsing of WebSocket frames: topic envelopes and error frames. */

export type WsFrame =
  | { kind: 'envelope'; topic: string; data: unknown }
  | { kind: 'error'; source?: string; message: string }

/**
 * Parse one text frame from the backend.
 * @param raw frame text
 * @returns an envelope ({topic, data}), an error frame ({type: "error", message}), or null if malformed/unknown
 */
export function parseWsFrame(raw: string): WsFrame | null {
  try {
    const frame = JSON.parse(raw) as { type?: unknown; topic?: unknown; data?: unknown; source?: unknown; message?: unknown }
    if (frame.type === 'error' && typeof frame.message === 'string') {
      return { kind: 'error', source: typeof frame.source === 'string' ? frame.source : undefined, message: frame.message }
    }
    if (typeof frame.topic === 'string' && frame.topic) return { kind: 'envelope', topic: frame.topic, data: frame.data }
  } catch {
    // malformed frame
  }
  return null
}
