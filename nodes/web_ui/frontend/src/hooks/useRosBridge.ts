import { useCallback, useEffect, useRef, useState } from 'react'
import log from '../logging'

export interface TopicData {
  [topic: string]: unknown
}

export interface UseRosBridgeReturn {
  topicData: TopicData
  connected: boolean
  publish: (topic: string, msgType: string, data: unknown) => void
}

const RECONNECT_DELAYS = [1000, 2000, 4000, 8000, 15000]

export function useRosBridge(_topics: string[]): UseRosBridgeReturn {
  const [topicData, setTopicData] = useState<TopicData>({})
  const [connected, setConnected] = useState(false)
  const wsRef = useRef<WebSocket | null>(null)
  const reconnectAttempt = useRef(0)
  const unmounted = useRef(false)

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
    }

    ws.onmessage = (evt) => {
      log.debug('[bridge] ←', evt.data.length, 'bytes')
      try {
        const envelope = JSON.parse(evt.data as string) as { topic: string; data: unknown }
        if (envelope.topic) {
          log.debug('[bridge] topic updated:', envelope.topic)
          setTopicData((prev) => ({ ...prev, [envelope.topic]: envelope.data }))
        }
      } catch {
        // ignore malformed frames
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
  }, [])

  useEffect(() => {
    unmounted.current = false
    connect()
    return () => {
      unmounted.current = true
      wsRef.current?.close()
    }
  }, [connect])

  const publish = useCallback((topic: string, msgType: string, data: unknown) => {
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      const frame = JSON.stringify({ type: 'publish', topic, msg_type: msgType, data })
      wsRef.current.send(frame)
      log.debug('[bridge] → publish:', topic)
    }
  }, [])

  return { topicData, connected, publish }
}
