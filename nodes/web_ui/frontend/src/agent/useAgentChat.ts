import { useCallback, useEffect, useRef, useState } from 'react'
import log from '../logging'
import { fetchAgentState } from './agentApi'
import { applyFrame, applyStateSnapshot, emptyChat, parseAgentFrame } from './agentModel'
import type { AgentInfo, ChatState } from './agentModel'

export const RECONNECT_DELAYS_MS = [1000, 2000, 4000, 8000, 15000]

export interface UseAgentChat {
  chat: ChatState
  info: AgentInfo | null
  connected: boolean
  refreshInfo: () => void
}

/**
 * Subscribe to /ws/agent: the history is replayed on every connect (deduplicated by seq), then events stream in.
 * The socket reconnects with backoff when it drops or the proxy reports the agent as disconnected.
 * @returns transcript state, last agent state snapshot, connection flag and a state refresh trigger
 */
export function useAgentChat(): UseAgentChat {
  const [chat, setChat] = useState<ChatState>(emptyChat)
  const [info, setInfo] = useState<AgentInfo | null>(null)
  const [connected, setConnected] = useState(false)
  const unmounted = useRef(false)

  const refreshInfo = useCallback(() => {
    void fetchAgentState().then((s) => {
      if (unmounted.current || !s) return
      setInfo(s)
      setChat((prev) => applyStateSnapshot(prev, s))
    })
  }, [])

  useEffect(() => {
    unmounted.current = false
    let ws: WebSocket | null = null
    let timer: ReturnType<typeof setTimeout> | undefined
    let attempt = 0

    const connect = () => {
      const proto = location.protocol === 'https:' ? 'wss' : 'ws'
      ws = new WebSocket(`${proto}://${location.host}/ws/agent`)
      ws.onopen = () => {
        log.info('[agent] WebSocket connected')
        attempt = 0
        setConnected(true)
        refreshInfo()
      }
      ws.onmessage = (evt) => {
        const parsed = parseAgentFrame(evt.data as string)
        if (parsed?.kind === 'frame') setChat((prev) => applyFrame(prev, parsed.frame))
        else if (parsed?.kind === 'proxy_error') {
          log.warn('[agent] proxy error:', parsed.message)
          setConnected(false)
        }
      }
      ws.onclose = () => {
        if (unmounted.current) return
        setConnected(false)
        const delay = RECONNECT_DELAYS_MS[Math.min(attempt, RECONNECT_DELAYS_MS.length - 1)]
        attempt++
        log.info('[agent] WebSocket closed, reconnecting in', delay, 'ms')
        timer = setTimeout(connect, delay)
      }
      ws.onerror = () => ws?.close()
    }

    connect()
    return () => {
      unmounted.current = true
      clearTimeout(timer)
      ws?.close()
    }
  }, [refreshInfo])

  return { chat, info, connected, refreshInfo }
}
