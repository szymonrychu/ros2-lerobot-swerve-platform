import { describe, expect, it } from 'vitest'
import { rosToThree, rosYawToThreeY, threeToRos, threeYToRosYaw } from './coords'

describe('rosToThree / threeToRos', () => {
  it('maps ROS (x, y, z) to three (x, z, -y)', () => {
    expect(rosToThree({ x: 1, y: 2, z: 3 })).toEqual([1, 3, -2])
  })

  it('defaults z to 0 for 2D map points', () => {
    expect(rosToThree({ x: 4, y: -5 })).toEqual([4, 0, 5])
  })

  it('applies an extra height (three y) when given', () => {
    expect(rosToThree({ x: 0, y: 1 }, 0.02)).toEqual([0, 0.02, -1])
  })

  it('maps three (x, y, z) back to ROS (x, -z, y)', () => {
    expect(threeToRos([1, 3, -2])).toEqual({ x: 1, y: 2, z: 3 })
  })

  it('round-trips arbitrary points', () => {
    const p = { x: -1.25, y: 7.5, z: 0.3 }
    const back = threeToRos(rosToThree(p))
    expect(back.x).toBeCloseTo(p.x)
    expect(back.y).toBeCloseTo(p.y)
    expect(back.z).toBeCloseTo(p.z)
  })
})

describe('yaw conversion', () => {
  it('ROS yaw about +z equals three rotation about +y (proper rotation, same sign)', () => {
    expect(rosYawToThreeY(0.7)).toBeCloseTo(0.7)
    expect(threeYToRosYaw(-1.2)).toBeCloseTo(-1.2)
  })

  it('a heading of +90 deg in ROS (facing +y) faces three -z', () => {
    const yaw = Math.PI / 2
    const ry = rosYawToThreeY(yaw)
    // three: rotating +x about +y by ry gives (cos ry, 0, -sin ry)
    const dir = [Math.cos(ry), 0, -Math.sin(ry)]
    const expected = rosToThree({ x: Math.cos(yaw), y: Math.sin(yaw) })
    expect(dir[0]).toBeCloseTo(expected[0])
    expect(dir[2]).toBeCloseTo(expected[2])
  })
})
