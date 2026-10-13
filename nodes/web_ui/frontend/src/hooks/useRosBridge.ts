import { useCallback, useEffect, useRef, useState } from 'react'
import log from '../logging'
import { parseWsFrame } from './wsFrames'

export interface TopicData {
  [topic: string]: unknown
}

/** Error frame sent by the backend when it rejects a command ({"type":"error","source","message"}). */
export interface BridgeError {
  source?: string
  message: string
}

export interface UseRosBridgeReturn {
  topicData: TopicData
  connected: boolean
  publish: (topic: string, msgType: string, data: unknown) => void
}

const RECONNECT_DELAYS = [1000, 2000, 4000, 8000, 15000]

/**
 * topics: what the backend should serialize for this client; colorSmall: whether the open tab needs the downscaled
 * color JPEG. The subscription is sent on connect and whenever either changes.
 */
export function useRosBridge(
  topics: string[],
  colorSmall: boolean,
  onError?: (error: BridgeError) => void
): UseRosBridgeReturn {
  const [topicData, setTopicData] = useState<TopicData>({})
  const [connected, setConnected] = useState(false)
  const wsRef = useRef<WebSocket | null>(null)
  const reconnectAttempt = useRef(0)
  const unmounted = useRef(false)
  const onErrorRef = useRef(onError)
  onErrorRef.current = onError
  const subscriptionRef = useRef('')
  subscriptionRef.current = JSON.stringify({ type: 'subscribe', topics, color_small: colorSmall })
  const sendSubscription = useCallback(() => {
    if (wsRef.current?.readyState === WebSocket.OPEN) wsRef.current.send(subscriptionRef.current)
  }, [])

  const connect = useCallback(() => {
    const proto = location.protocol === 'https:' ? 'wss' : 'ws'
    const url = `${proto}://${location.host}/ws`
    log.info('[bridge] connecting to', url)

    const ws = new WebSocket(url)
    wsRef.current = ws

    ws.onopen = () => {
      log.info('[bridge] WebSocket connected to', url)
      setConnected(true)
      reconnectAttempt.current = 0
      sendSubscription()
    }

    ws.onmessage = (evt) => {
      log.debug('[bridge] recv', evt.data.length, 'bytes')
      const frame = parseWsFrame(evt.data as string)
      if (frame?.kind === 'envelope') {
        log.debug('[bridge] topic updated:', frame.topic)
        setTopicData((prev) => ({ ...prev, [frame.topic]: frame.data }))
      } else if (frame?.kind === 'error') {
        log.warn('[bridge] error frame:', frame.message)
        onErrorRef.current?.({ source: frame.source, message: frame.message })
      }
    }

    ws.onclose = () => {
      if (unmounted.current) return
      setConnected(false)
      const delay = RECONNECT_DELAYS[Math.min(reconnectAttempt.current, RECONNECT_DELAYS.length - 1)]
      reconnectAttempt.current++
      log.info('[bridge] WebSocket closed, reconnecting in', delay, 'ms')
      setTimeout(connect, delay)
    }

    ws.onerror = () => {
      ws.close()
    }
  }, [sendSubscription])

  useEffect(() => {
    unmounted.current = false
    connect()
    return () => {
      unmounted.current = true
      wsRef.current?.close()
    }
  }, [connect])

  const subscriptionKey = subscriptionRef.current
  useEffect(() => {
    if (subscriptionKey) sendSubscription()
  }, [subscriptionKey, sendSubscription])

  const publish = useCallback((topic: string, msgType: string, data: unknown) => {
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      const frame = JSON.stringify({ type: 'publish', topic, msg_type: msgType, data })
      wsRef.current.send(frame)
      log.debug('[bridge] send publish:', topic)
    }
  }, [])

  return { topicData, connected, publish }
}
