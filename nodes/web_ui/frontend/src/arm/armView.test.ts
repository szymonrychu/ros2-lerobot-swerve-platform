import { Object3D, PerspectiveCamera, Vector3 } from 'three'
import { describe, expect, it } from 'vitest'
import type { URDFRobot } from 'urdf-loader'
import { armCameraPose } from '../map3d/armCamera'
import { applyArmCamera } from './armView.ts'

const MOUNT = { xyz: [0, 0, 0] as [number, number, number], rpy: [0, 0, 0] as [number, number, number] }

/** Stub robot with shoulder_link and gripper_link whose world matrices follow a "joint" value. */
function stubRobot(): { robot: URDFRobot; setPan: (v: number) => void } {
  const shoulder = new Object3D()
  const gripper = new Object3D()
  shoulder.add(gripper)
  const root = new Object3D()
  root.add(shoulder)
  const robot = Object.assign(root, { links: { shoulder_link: shoulder, gripper_link: gripper } }) as unknown as URDFRobot
  return {
    robot,
    setPan: (v: number) => {
      shoulder.position.set(v, 0, 0)
      gripper.position.set(0, v, 0)
    },
  }
}

describe('applyArmCamera', () => {
  it('sets camera position, up and target from the shoulder link and follows a joint change', () => {
    const { robot, setPan } = stubRobot()
    const camera = new PerspectiveCamera()
    const target = new Vector3()
    setPan(0)
    applyArmCamera(camera, target, robot, 'side-left', MOUNT)
    const first = camera.position.clone()
    setPan(1)
    applyArmCamera(camera, target, robot, 'side-left', MOUNT)
    expect(camera.position.x - first.x).toBeCloseTo(1)
    const expected = armCameraPose('side-left', robot.links.shoulder_link.matrixWorld, robot.links.gripper_link.matrixWorld, MOUNT)
    expect(camera.position.distanceTo(expected.position)).toBeLessThan(1e-9)
    expect(target.distanceTo(expected.target)).toBeLessThan(1e-9)
    expect(camera.up.distanceTo(expected.up)).toBeLessThan(1e-9)
  })

  it('gripper mode follows the gripper link', () => {
    const { robot, setPan } = stubRobot()
    const camera = new PerspectiveCamera()
    const target = new Vector3()
    setPan(0)
    applyArmCamera(camera, target, robot, 'gripper', MOUNT)
    setPan(0.5)
    applyArmCamera(camera, target, robot, 'gripper', MOUNT)
    expect(camera.position.y).toBeCloseTo(0.5)
  })

  it('returns false and leaves the camera when a link is missing', () => {
    const camera = new PerspectiveCamera()
    camera.position.set(1, 2, 3)
    const robot = { links: {}, updateWorldMatrix: () => undefined } as unknown as URDFRobot
    expect(applyArmCamera(camera, new Vector3(), robot, 'above', MOUNT)).toBe(false)
    expect(camera.position.toArray()).toEqual([1, 2, 3])
  })
})
