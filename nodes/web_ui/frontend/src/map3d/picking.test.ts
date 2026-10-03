import { describe, expect, it } from 'vitest'
import * as THREE from 'three'
import { pickGround, rayGroundIntersection, screenToNdc } from './picking'

describe('screenToNdc', () => {
  it('maps the canvas centre to (0, 0) and corners to +-1 with y up', () => {
    expect(screenToNdc(50, 25, 100, 50)).toEqual({ x: 0, y: 0 })
    expect(screenToNdc(0, 0, 100, 50)).toEqual({ x: -1, y: 1 })
    expect(screenToNdc(100, 50, 100, 50)).toEqual({ x: 1, y: -1 })
  })

  it('returns null for an empty canvas', () => {
    expect(screenToNdc(1, 1, 0, 10)).toBeNull()
  })
})

describe('rayGroundIntersection', () => {
  it('hits the ground plane straight below', () => {
    const hit = rayGroundIntersection([1, 5, -2], [0, -1, 0], 0)
    expect(hit).toEqual({ x: 1, y: 2 })
  })

  it('hits along a slanted ray and honours the ground height', () => {
    const hit = rayGroundIntersection([0, 2, 0], [1, -1, 0], 1)
    expect(hit!.x).toBeCloseTo(1)
    expect(hit!.y).toBeCloseTo(0)
  })

  it('returns null for rays parallel to or pointing away from the ground', () => {
    expect(rayGroundIntersection([0, 1, 0], [1, 0, 0], 0)).toBeNull()
    expect(rayGroundIntersection([0, 1, 0], [0, 1, 0], 0)).toBeNull()
  })
})

describe('pickGround', () => {
  it('maps the screen centre of a top-down camera to the map point below it', () => {
    const cam = new THREE.PerspectiveCamera(50, 2, 0.1, 100)
    cam.position.set(3, 10, -4) // above ROS (3, 4)
    cam.up.set(0, 0, -1)
    cam.lookAt(3, 0, -4)
    cam.updateMatrixWorld()
    const hit = pickGround(cam, 100, 50, 200, 100)
    expect(hit!.x).toBeCloseTo(3)
    expect(hit!.y).toBeCloseTo(4)
  })

  it('screen right is ROS +x and screen up is ROS +y in top view', () => {
    const cam = new THREE.OrthographicCamera(-10, 10, 5, -5, 0.1, 100)
    cam.position.set(0, 10, 0)
    cam.up.set(0, 0, -1)
    cam.lookAt(0, 0, 0)
    cam.updateMatrixWorld()
    const right = pickGround(cam, 200, 50, 200, 100)!
    const top = pickGround(cam, 100, 0, 200, 100)!
    expect(right.x).toBeCloseTo(10)
    expect(right.y).toBeCloseTo(0)
    expect(top.x).toBeCloseTo(0)
    expect(top.y).toBeCloseTo(5)
  })

  it('returns null when the camera looks above the horizon', () => {
    const cam = new THREE.PerspectiveCamera(50, 1, 0.1, 100)
    cam.position.set(0, 1, 0)
    cam.lookAt(0, 5, -1)
    cam.updateMatrixWorld()
    expect(pickGround(cam, 50, 50, 100, 100)).toBeNull()
  })
})
