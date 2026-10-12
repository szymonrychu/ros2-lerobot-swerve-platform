import { describe, expect, it } from 'vitest'
import { createOrbitHold } from './orbitHold'

describe('createOrbitHold', () => {
  it('restores enabled=true after a drag when orbit was enabled before', () => {
    const c = { enabled: true }
    const hold = createOrbitHold()
    hold.hold(c)
    expect(c.enabled).toBe(false)
    hold.release(c)
    expect(c.enabled).toBe(true)
  })

  it('keeps orbit disabled after a drag when a glued mode had disabled it', () => {
    const c = { enabled: false }
    const hold = createOrbitHold()
    hold.hold(c)
    hold.release(c)
    expect(c.enabled).toBe(false)
  })

  it('release without a hold leaves the controls untouched, and null controls are ignored', () => {
    const c = { enabled: false }
    createOrbitHold().release(c)
    expect(c.enabled).toBe(false)
    expect(() => createOrbitHold().hold(null)).not.toThrow()
  })
})
