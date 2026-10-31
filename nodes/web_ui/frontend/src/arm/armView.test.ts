import { Euler, Matrix4, Object3D, PerspectiveCamera, Quaternion, Vector3 } from 'three'
import { describe, expect, it } from 'vitest'
import type { URDFRobot } from 'urdf-loader'
import { URDF_TO_THREE_ROTATION_X } from '../map3d/coords'
import { applyArmCamera, armCameraPose, type ArmCameraMode } from './armView.ts'

/** URDF origin transform (xyz, rpy = fixed-axis roll, pitch, yaw: R = Rz Ry Rx). */
function urdfOrigin(xyz: number[], rpy: number[]): Matrix4 {
  const m = new Matrix4().makeRotationFromQuaternion(new Quaternion().setFromEuler(new Euler(rpy[0], rpy[1], rpy[2], 'ZYX')))
  return m.setPosition(xyz[0], xyz[1], xyz[2])
}

/** shoulder_link world matrix (three space) for a given shoulder_pan, following so101_arm.urdf. */
function shoulderWorld(pan: number): Matrix4 {
  const rot = new Matrix4().makeRotationX(URDF_TO_THREE_ROTATION_X)
  const placed = new Matrix4().makeTranslation(1, 2, 0.5).multiply(rot)
  const joint = urdfOrigin([0.0388353, 0, 0.0624], [Math.PI, 0, -Math.PI])
  const panRot = new Matrix4().makeRotationZ(pan)
  return placed.multiply(joint).multiply(panRot)
}

function relative(shoulder: Matrix4, p: Vector3): Vector3 {
  return p.clone().applyMatrix4(shoulder.clone().invert())
}

describe('armCameraPose side and above modes', () => {
  const modes: ArmCameraMode[] = ['side-left', 'side-right', 'above']

  it.each(modes)('%s pose relative to shoulder_link does not change with shoulder_pan', (mode) => {
    const a = armCameraPose(mode, shoulderWorld(0))
    const b = armCameraPose(mode, shoulderWorld(1.1))
    const sa = shoulderWorld(0)
    const sb = shoulderWorld(1.1)
    for (const key of ['position', 'target'] as const) {
      const ra = relative(sa, a[key])
      const rb = relative(sb, b[key])
      expect(rb.distanceTo(ra)).toBeLessThan(1e-9)
    }
    const ua = a.up.clone().transformDirection(sa.clone().invert())
    const ub = b.up.clone().transformDirection(sb.clone().invert())
    expect(ub.distanceTo(ua)).toBeLessThan(1e-9)
    expect(a.position.distanceTo(b.position)).toBeGreaterThan(0)
  })

  it('side-left looks from the robot left, side-right from the right (three: left = -z for a robot facing +x)', () => {
    const s = shoulderWorld(0)
    const left = armCameraPose('side-left', s)
    const right = armCameraPose('side-right', s)
    const origin = new Vector3().setFromMatrixPosition(s)
    // Arm forward at pan 0 is base +x, i.e. three +x. ROS left (+y) is three -z.
    expect(left.position.z).toBeLessThan(origin.z)
    expect(right.position.z).toBeGreaterThan(origin.z)
    expect(left.up.y).toBeCloseTo(1, 9)
    expect(right.up.y).toBeCloseTo(1, 9)
  })

  it('keeps the arm at opposite screen sides for side-left vs side-right', () => {
    const s = shoulderWorld(0)
    const forward = new Vector3(1, 0, 0) // arm direction at pan 0 in three space
    const screenX = (mode: ArmCameraMode): number => {
      const pose = armCameraPose(mode, s)
      const dir = pose.target.clone().sub(pose.position).normalize()
      const camRight = dir.clone().cross(pose.up).normalize()
      return forward.dot(camRight)
    }
    expect(screenX('side-left')).toBeLessThan(-0.9)
    expect(screenX('side-right')).toBeGreaterThan(0.9)
  })

  it('above sits over the shoulder (three +y) and looks down', () => {
    const s = shoulderWorld(0)
    const pose = armCameraPose('above', s)
    const dir = pose.target.clone().sub(pose.position).normalize()
    expect(dir.y).toBeLessThan(-0.99)
    expect(pose.position.y).toBeGreaterThan(new Vector3().setFromMatrixPosition(s).y)
    // arm points to the top of the screen: up vector is the arm forward direction (three +x)
    expect(pose.up.x).toBeCloseTo(1, 9)
  })
})


/** Stub robot with a shoulder_link whose world matrix follows a "joint" value. */
function stubRobot(): { robot: URDFRobot; setPan: (v: number) => void } {
  const shoulder = new Object3D()
  const root = new Object3D()
  root.add(shoulder)
  const robot = Object.assign(root, { links: { shoulder_link: shoulder } }) as unknown as URDFRobot
  return {
    robot,
    setPan: (v: number) => {
      shoulder.position.set(v, 0, 0)
    },
  }
}

describe('applyArmCamera', () => {
  it('sets camera position, up and target from the shoulder link and follows a joint change', () => {
    const { robot, setPan } = stubRobot()
    const camera = new PerspectiveCamera()
    const target = new Vector3()
    setPan(0)
    applyArmCamera(camera, target, robot, 'side-left')
    const first = camera.position.clone()
    setPan(1)
    applyArmCamera(camera, target, robot, 'side-left')
    expect(camera.position.x - first.x).toBeCloseTo(1)
    const expected = armCameraPose('side-left', robot.links.shoulder_link.matrixWorld)
    expect(camera.position.distanceTo(expected.position)).toBeLessThan(1e-9)
    expect(target.distanceTo(expected.target)).toBeLessThan(1e-9)
    expect(camera.up.distanceTo(expected.up)).toBeLessThan(1e-9)
  })

  it('returns false and leaves the camera when shoulder_link is missing', () => {
    const camera = new PerspectiveCamera()
    camera.position.set(1, 2, 3)
    const robot = { links: {}, updateWorldMatrix: () => undefined } as unknown as URDFRobot
    expect(applyArmCamera(camera, new Vector3(), robot, 'above')).toBe(false)
    expect(camera.position.toArray()).toEqual([1, 2, 3])
  })
})
