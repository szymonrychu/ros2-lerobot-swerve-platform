import { describe, expect, it } from 'vitest'
import * as THREE from 'three'
import { orbitBindings } from './cameraBindings'

describe('orbitBindings', () => {
  it('top view: left and one finger pan', () => {
    expect(orbitBindings({ topView: true, goalMode: false })).toEqual({
      mouseButtons: { LEFT: THREE.MOUSE.PAN, MIDDLE: THREE.MOUSE.DOLLY, RIGHT: THREE.MOUSE.PAN },
      touches: { ONE: THREE.TOUCH.PAN, TWO: THREE.TOUCH.DOLLY_PAN },
    })
  })

  it('top view goal mode: left and one finger belong to the goal gesture', () => {
    expect(orbitBindings({ topView: true, goalMode: true })).toEqual({
      mouseButtons: { MIDDLE: THREE.MOUSE.DOLLY, RIGHT: THREE.MOUSE.PAN },
      touches: { TWO: THREE.TOUCH.DOLLY_PAN },
    })
  })

  it('orbit view: left and one finger rotate', () => {
    expect(orbitBindings({ topView: false, goalMode: false })).toEqual({
      mouseButtons: { LEFT: THREE.MOUSE.ROTATE, MIDDLE: THREE.MOUSE.DOLLY, RIGHT: THREE.MOUSE.PAN },
      touches: { ONE: THREE.TOUCH.ROTATE, TWO: THREE.TOUCH.DOLLY_PAN },
    })
  })

  it('orbit view goal mode: no left button and no one-finger action, right rotates, middle pans, two fingers dolly and pan', () => {
    const { mouseButtons, touches } = orbitBindings({ topView: false, goalMode: true })
    expect('LEFT' in mouseButtons).toBe(false)
    expect('ONE' in touches).toBe(false)
    expect(mouseButtons.RIGHT).toBe(THREE.MOUSE.ROTATE)
    expect(mouseButtons.MIDDLE).toBe(THREE.MOUSE.PAN)
    expect(touches.TWO).toBe(THREE.TOUCH.DOLLY_PAN)
  })
})
