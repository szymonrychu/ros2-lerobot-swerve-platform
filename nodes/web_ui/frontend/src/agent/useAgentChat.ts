import { useCallback, useEffect, useRef, useState } from 'react'
import log from '../logging'
import { fetchAgentHistory, fetchAgentState } from './agentApi'
import { clearAgentStorage } from './agentCache'
import { HISTORY_PAGE_SIZE, MAX_ITEMS, applyFrame, applyStateSnapshot, emptyChat, prependHistory } from './agentModel'
import { runAgentSocket } from './agentSocket'
import type { SocketLike } from './agentSocket'
import type { AgentInfo, ChatState } from './agentModel'

export interface UseAgentChat {
  chat: ChatState
  info: AgentInfo | null
  connected: boolean
  refreshInfo: () => void
  /** Fetch the next older page and prepend it (no-op while loading or at the start of the session). */
  loadOlder: () => void
  loadingOlder: boolean
  /** The tab calls this with the follow state: the oldest items are only dropped from memory while following. */
  setFollowing: (following: boolean) => void
  /** Drop every cached item, cursor and stored key of the tab (after a successful reset). */
  clearCache: () => void
}

/**
 * Subscribe to /ws/agent: the history is replayed on every connect (deduplicated by seq), then events stream in.
 * The socket reconnects with backoff (1 s up to 30 s) when it drops or the proxy reports the agent as disconnected;
 * the backoff resets only after the first real frame arrives (see runAgentSocket).
 * Only the newest page arrives on connect; older pages load on demand (loadOlder) and the in-memory window is capped
 * at MAX_ITEMS (the oldest items are dropped while the view follows the end).
 * @returns transcript state, last agent state snapshot, connection flag, refresh/paging/cache controls
 */
export function useAgentChat(): UseAgentChat {
  const [chat, setChat] = useState<ChatState>(emptyChat)
  const [info, setInfo] = useState<AgentInfo | null>(null)
  const [connected, setConnected] = useState(false)
  const [loadingOlder, setLoadingOlder] = useState(false)
  const unmounted = useRef(false)
  const following = useRef(true)
  const loading = useRef(false)
  // Bumped when the transcript is replaced or cleared, so a page fetched for the old one is discarded.
  const epoch = useRef(0)
  const chatRef = useRef(chat)
  chatRef.current = chat

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
        if (parsed.kind === 'frame') {
          if (parsed.frame.type === 'history') epoch.current++
          setChat((prev) => applyFrame(prev, parsed.frame, following.current ? MAX_ITEMS : null))
        }
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

  const loadOlder = useCallback(() => {
    const { firstSeq, hasMore } = chatRef.current
    if (loading.current || !hasMore || firstSeq === null) return
    loading.current = true
    setLoadingOlder(true)
    const started = epoch.current
    void fetchAgentHistory(firstSeq, HISTORY_PAGE_SIZE).then((page) => {
      loading.current = false
      if (unmounted.current) return
      setLoadingOlder(false)
      if (page === null) log.warn('[agent] loading older history failed')
      else if (started === epoch.current) setChat((prev) => prependHistory(prev, page.events, page.has_more))
    })
  }, [])

  const setFollowing = useCallback((value: boolean) => {
    following.current = value
  }, [])

  const clearCache = useCallback(() => {
    epoch.current++
    loading.current = false
    setLoadingOlder(false)
    following.current = true
    clearAgentStorage()
    setChat(emptyChat())
  }, [])

  return { chat, info, connected, refreshInfo, loadOlder, loadingOlder, setFollowing, clearCache }
}
