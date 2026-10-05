import { parseAgentFrame } from './agentModel'
import type { ParsedFrame } from './agentModel'

export const RECONNECT_DELAYS_MS = [1000, 2000, 4000, 8000, 15000, 30000]

/**
 * Delay before reconnect attempt number `attempt` (0-based), capped at the last entry of RECONNECT_DELAYS_MS.
 * @param attempt number of consecutive failed attempts
 * @returns delay in milliseconds
 */
export function reconnectDelay(attempt: number): number {
  return RECONNECT_DELAYS_MS[Math.min(attempt, RECONNECT_DELAYS_MS.length - 1)]
}

export interface SocketLike {
  onopen: (() => void) | null
  onmessage: ((evt: { data: unknown }) => void) | null
  onclose: (() => void) | null
  onerror: (() => void) | null
  close: () => void
}

export interface AgentSocketOptions {
  makeSocket: () => SocketLike
  schedule: (fn: () => void, ms: number) => unknown
  cancel: (handle: unknown) => void
  onOpen: () => void
  onFrame: (parsed: ParsedFrame) => void
  onClose: () => void
}

/**
 * Keep a socket to /ws/agent connected, reconnecting with backoff. The backoff resets only when a real frame (the
 * history, an event) arrives, not on open: the proxy accepts the browser socket even while the agent is down and then
 * closes it with an error frame, so an open alone does not mean the agent is reachable.
 * @param opts socket factory, timer functions and callbacks
 * @returns a function that closes the socket and stops reconnecting
 */
export function runAgentSocket(opts: AgentSocketOptions): () => void {
  let socket: SocketLike | null = null
  let timer: unknown
  let attempt = 0
  let stopped = false

  const connect = () => {
    const ws = opts.makeSocket()
    socket = ws
    ws.onopen = () => opts.onOpen()
    ws.onmessage = (evt) => {
      const parsed = parseAgentFrame(evt.data as string)
      if (!parsed) return
      if (parsed.kind === 'frame') attempt = 0
      opts.onFrame(parsed)
    }
    ws.onclose = () => {
      if (stopped) return
      opts.onClose()
      const delay = reconnectDelay(attempt)
      attempt++
      timer = opts.schedule(connect, delay)
    }
    ws.onerror = () => ws.close()
  }

  connect()
  return () => {
    stopped = true
    opts.cancel(timer)
    socket?.close()
  }
}
