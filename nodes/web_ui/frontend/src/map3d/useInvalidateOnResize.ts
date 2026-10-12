/**
 * Redraw an on-demand R3F canvas when its container changes size (a docked sidebar opening or closing beside
 * the map). Resizing clears the WebGL drawing buffer, and with frameloop="demand" nothing else asks for the
 * next frame, so the canvas would stay blank or stretched until the next topic update.
 */
import { useEffect } from 'react'
import type { RefObject } from 'react'
import { invalidate } from '@react-three/fiber'

/**
 * Observe the element behind `ref` and request one R3F frame per size change. The first report is the
 * initial size and requests nothing; a report with an unchanged size requests nothing either.
 *
 * @param ref - ref of the canvas container element.
 * @returns nothing; the observer is disconnected on unmount.
 */
export function useInvalidateOnResize(ref: RefObject<Element>): void {
  useEffect(() => {
    const el = ref.current
    if (!el) return
    let last: { width: number; height: number } | null = null
    const observer = new ResizeObserver((entries) => {
      const entry = entries[entries.length - 1]
      if (!entry) return
      const { width, height } = entry.contentRect
      const changed = last !== null && (last.width !== width || last.height !== height)
      last = { width, height }
      if (changed) invalidate()
    })
    observer.observe(el)
    return () => observer.disconnect()
  }, [ref])
}
