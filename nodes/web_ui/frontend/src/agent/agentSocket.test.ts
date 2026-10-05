import { describe, expect, it } from 'vitest'
import { RECONNECT_DELAYS_MS, reconnectDelay, runAgentSocket } from './agentSocket'
import type { SocketLike } from './agentSocket'

class FakeSocket implements SocketLike {
  onopen: (() => void) | null = null
  onmessage: ((evt: { data: unknown }) => void) | null = null
  onclose: (() => void) | null = null
  onerror: (() => void) | null = null
  closed = false
  close() {
    this.closed = true
  }
}

function setup() {
  const sockets: FakeSocket[] = []
  const delays: number[] = []
  let pending: (() => void) | null = null
  const frames: string[] = []
  const stop = runAgentSocket({
    makeSocket: () => {
      const s = new FakeSocket()
      sockets.push(s)
      return s
    },
    schedule: (fn, ms) => {
      delays.push(ms)
      pending = fn
      return fn
    },
    cancel: () => {
      pending = null
    },
    onOpen: () => {},
    onFrame: (p) => frames.push(p.kind),
    onClose: () => {},
  })
  const last = () => sockets[sockets.length - 1]
  const reconnect = () => {
    const fn = pending
    pending = null
    fn?.()
  }
  return { sockets, delays, frames, stop, last, reconnect, hasPending: () => pending !== null }
}

const HISTORY = JSON.stringify({ type: 'history', events: [] })
const PROXY_ERROR = JSON.stringify({ type: 'error', message: 'agent disconnected' })

describe('reconnect delays', () => {
  it('doubles up to a 30 s cap', () => {
    expect(RECONNECT_DELAYS_MS[RECONNECT_DELAYS_MS.length - 1]).toBe(30000)
    expect([0, 1, 2, 3, 4, 5, 6, 50].map(reconnectDelay)).toEqual([1000, 2000, 4000, 8000, 15000, 30000, 30000, 30000])
  })
})

describe('runAgentSocket backoff', () => {
  it('keeps backing off when the socket opens but the agent is down (proxy error, no history)', () => {
    const t = setup()
    for (let i = 0; i < 4; i++) {
      t.last().onopen?.()
      t.last().onmessage?.({ data: PROXY_ERROR })
      t.last().onclose?.()
      t.reconnect()
    }
    expect(t.delays).toEqual([1000, 2000, 4000, 8000])
  })

  it('resets the backoff after the first real frame', () => {
    const t = setup()
    for (let i = 0; i < 3; i++) {
      t.last().onclose?.()
      t.reconnect()
    }
    t.last().onopen?.()
    t.last().onmessage?.({ data: HISTORY })
    t.last().onclose?.()
    expect(t.delays).toEqual([1000, 2000, 4000, 1000])
  })

  it('ignores malformed frames and does not reset on them', () => {
    const t = setup()
    t.last().onclose?.()
    t.reconnect()
    t.last().onmessage?.({ data: 'not json' })
    t.last().onclose?.()
    expect(t.delays).toEqual([1000, 2000])
  })

  it('stops reconnecting after the cleanup function runs', () => {
    const t = setup()
    t.stop()
    expect(t.last().closed).toBe(true)
    t.last().onclose?.()
    expect(t.hasPending()).toBe(false)
  })
})
