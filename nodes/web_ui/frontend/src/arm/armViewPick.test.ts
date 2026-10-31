import { describe, expect, it } from 'vitest'
import * as THREE from 'three'
import { armViewGroundY, armViewHitToBaseLink, BASE_LINK_POSE, pickArmViewGround } from './armViewPick'

const OFFSET: [number, number, number] = [0.0592, -0.05, 0.1]

describe('armViewGroundY', () => {
  it('is the floor under the arm root: base_link z = 0 is -mount z in the arm frame', () => {
    expect(armViewGroundY(OFFSET)).toBeCloseTo(-0.1)
  })
})

describe('armViewHitToBaseLink', () => {
  it('adds the mount x, y to a hit in the arm frame', () => {
    const p = armViewHitToBaseLink({ x: 0.2, y: 0.1 }, OFFSET)
    expect(p?.x).toBeCloseTo(0.2592)
    expect(p?.y).toBeCloseTo(0.05)
  })

  it('keeps a hit above the horizon null', () => {
    expect(armViewHitToBaseLink(null, OFFSET)).toBeNull()
  })
})

describe('BASE_LINK_POSE', () => {
  it('is the identity pose: the picked point is already in base_link', () => {
    expect(BASE_LINK_POSE).toEqual({ x: 0, y: 0, yaw: 0 })
  })
})

describe('pickArmViewGround', () => {
  it('maps the centre of a top-down camera over the arm root to the mount position in base_link', () => {
    const cam = new THREE.PerspectiveCamera(50, 1, 0.01, 50)
    cam.position.set(0, 2, 0)
    cam.up.set(0, 0, -1)
    cam.lookAt(0, 0, 0)
    cam.updateMatrixWorld()
    const p = pickArmViewGround(cam, 100, 100, 200, 200, OFFSET)
    expect(p?.x).toBeCloseTo(0.0592)
    expect(p?.y).toBeCloseTo(-0.05)
  })

  it('is null above the horizon', () => {
    const cam = new THREE.PerspectiveCamera(50, 1, 0.01, 50)
    cam.position.set(0, 1, 2)
    cam.lookAt(0, 3, 0)
    cam.updateMatrixWorld()
    expect(pickArmViewGround(cam, 100, 100, 200, 200, OFFSET)).toBeNull()
  })
})
