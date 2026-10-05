import { useCallback, useEffect, useRef, useState } from 'react'
import log from '../logging'
import { fetchAgentState } from './agentApi'
import { applyFrame, applyStateSnapshot, emptyChat } from './agentModel'
import { runAgentSocket } from './agentSocket'
import type { SocketLike } from './agentSocket'
import type { AgentInfo, ChatState } from './agentModel'

export interface UseAgentChat {
  chat: ChatState
  info: AgentInfo | null
  connected: boolean
  refreshInfo: () => void
}

/**
 * Subscribe to /ws/agent: the history is replayed on every connect (deduplicated by seq), then events stream in.
 * The socket reconnects with backoff (1 s up to 30 s) when it drops or the proxy reports the agent as disconnected;
 * the backoff resets only after the first real frame arrives (see runAgentSocket).
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
    const proto = location.protocol === 'https:' ? 'wss' : 'ws'
    const stop = runAgentSocket({
      makeSocket: () => new WebSocket(`${proto}://${location.host}/ws/agent`) as unknown as SocketLike,
      schedule: (fn, ms) => setTimeout(fn, ms),
      cancel: (handle) => clearTimeout(handle as ReturnType<typeof setTimeout>),
      onOpen: () => {
        log.info('[agent] WebSocket connected')
        setConnected(true)
        refreshInfo()
      },
      onFrame: (parsed) => {
        if (parsed.kind === 'frame') setChat((prev) => applyFrame(prev, parsed.frame))
        else {
          log.warn('[agent] proxy error:', parsed.message)
          setConnected(false)
        }
      },
      onClose: () => {
        if (unmounted.current) return
        setConnected(false)
        log.info('[agent] WebSocket closed, reconnecting with backoff')
      },
    })
    return () => {
      unmounted.current = true
      stop()
    }
  }, [refreshInfo])

  return { chat, info, connected, refreshInfo }
}
