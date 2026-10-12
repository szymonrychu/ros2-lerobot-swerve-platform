// @vitest-environment jsdom
import { act, cleanup, render } from '@testing-library/react'
import { useRef } from 'react'
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { useInvalidateOnResize } from './useInvalidateOnResize'

const mocks = vi.hoisted(() => ({ invalidate: vi.fn() }))

vi.mock('@react-three/fiber', () => ({ invalidate: mocks.invalidate }))

/** Controllable ResizeObserver stand-in: tests push sizes through `resize`. */
class FakeResizeObserver {
  static instances: FakeResizeObserver[] = []
  observed: Element[] = []
  disconnect = vi.fn()
  constructor(private readonly callback: ResizeObserverCallback) {
    FakeResizeObserver.instances.push(this)
  }
  observe(el: Element): void {
    this.observed.push(el)
  }
  unobserve(): void {}
  resize(width: number, height: number): void {
    const entries = this.observed.map((target) => ({ target, contentRect: { width, height } }) as unknown as ResizeObserverEntry)
    act(() => this.callback(entries, this as unknown as ResizeObserver))
  }
}

function Probe() {
  const ref = useRef<HTMLDivElement>(null)
  useInvalidateOnResize(ref)
  return <div ref={ref} data-testid="box" />
}

beforeEach(() => {
  FakeResizeObserver.instances = []
  mocks.invalidate.mockClear()
  vi.stubGlobal('ResizeObserver', FakeResizeObserver)
})
afterEach(() => {
  cleanup()
  vi.unstubAllGlobals()
})

describe('useInvalidateOnResize', () => {
  it('observes the container', () => {
    const { getByTestId } = render(<Probe />)
    expect(FakeResizeObserver.instances).toHaveLength(1)
    expect(FakeResizeObserver.instances[0].observed).toEqual([getByTestId('box')])
  })

  it('invalidates once per size change and never without a change', () => {
    render(<Probe />)
    const ro = FakeResizeObserver.instances[0]
    ro.resize(800, 600) // first report: the initial size, not a change
    expect(mocks.invalidate).not.toHaveBeenCalled()
    ro.resize(800, 600)
    expect(mocks.invalidate).not.toHaveBeenCalled()
    ro.resize(500, 600) // sidebar opened
    expect(mocks.invalidate).toHaveBeenCalledTimes(1)
    ro.resize(500, 600)
    expect(mocks.invalidate).toHaveBeenCalledTimes(1)
    ro.resize(500, 420)
    expect(mocks.invalidate).toHaveBeenCalledTimes(2)
    ro.resize(800, 600) // sidebar closed
    expect(mocks.invalidate).toHaveBeenCalledTimes(3)
  })

  it('disconnects the observer on unmount', () => {
    const { unmount } = render(<Probe />)
    const ro = FakeResizeObserver.instances[0]
    expect(ro.disconnect).not.toHaveBeenCalled()
    unmount()
    expect(ro.disconnect).toHaveBeenCalledTimes(1)
  })
})
