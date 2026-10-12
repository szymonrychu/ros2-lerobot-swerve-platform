import { Euler, Matrix4, Quaternion, Vector3 } from 'three'
import { describe, expect, it } from 'vitest'
import { armCameraPose, type ArmCameraMode } from './armCamera'
import { URDF_TO_THREE_ROTATION_X } from './coords'

const MOUNT = { xyz: [-0.0046, 0.0364, -0.0364] as [number, number, number], rpy: [-1.5622, 1.195, -1.4466] as [number, number, number] }

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

const GRIPPER = new Matrix4().makeTranslation(0.3, 0.2, 0.4).multiply(new Matrix4().makeRotationY(0.7))

describe('armCameraPose side and above modes', () => {
  const modes: ArmCameraMode[] = ['side-left', 'side-right', 'above']

  it.each(modes)('%s pose relative to shoulder_link does not change with shoulder_pan', (mode) => {
    const a = armCameraPose(mode, shoulderWorld(0), GRIPPER, MOUNT)
    const b = armCameraPose(mode, shoulderWorld(1.1), GRIPPER, MOUNT)
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
    const left = armCameraPose('side-left', s, GRIPPER, MOUNT)
    const right = armCameraPose('side-right', s, GRIPPER, MOUNT)
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
      const pose = armCameraPose(mode, s, GRIPPER, MOUNT)
      const dir = pose.target.clone().sub(pose.position).normalize()
      const camRight = dir.clone().cross(pose.up).normalize()
      return forward.dot(camRight)
    }
    expect(screenX('side-left')).toBeLessThan(-0.9)
    expect(screenX('side-right')).toBeGreaterThan(0.9)
  })

  it('above sits over the shoulder (three +y) and looks down', () => {
    const s = shoulderWorld(0)
    const pose = armCameraPose('above', s, GRIPPER, MOUNT)
    const dir = pose.target.clone().sub(pose.position).normalize()
    expect(dir.y).toBeLessThan(-0.99)
    expect(pose.position.y).toBeGreaterThan(new Vector3().setFromMatrixPosition(s).y)
    // arm points to the top of the screen: up vector is the arm forward direction (three +x)
    expect(pose.up.x).toBeCloseTo(1, 9)
  })
})

describe('armCameraPose gripper mode', () => {
  it('composes gripper_link with the mount and looks along the optical axis (body +x)', () => {
    const pose = armCameraPose('gripper', shoulderWorld(0), GRIPPER, MOUNT)
    // Hand-computed mount rotation R = Rz(yaw) Ry(pitch) Rx(roll).
    const [r, p, y] = MOUNT.rpy
    const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(y), sy = Math.sin(y)
    const xAxis = new Vector3(cy * cp, sy * cp, -sp)
    const zAxis = new Vector3(cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr)
    const origin = new Vector3(...MOUNT.xyz).applyMatrix4(GRIPPER)
    const dir = xAxis.clone().transformDirection(GRIPPER)
    const up = zAxis.clone().transformDirection(GRIPPER)
    expect(pose.position.distanceTo(origin)).toBeLessThan(1e-9)
    expect(pose.target.clone().sub(pose.position).normalize().distanceTo(dir)).toBeLessThan(1e-9)
    expect(pose.up.distanceTo(up)).toBeLessThan(1e-9)
  })

  it('does not depend on the shoulder matrix', () => {
    const a = armCameraPose('gripper', shoulderWorld(0), GRIPPER, MOUNT)
    const b = armCameraPose('gripper', shoulderWorld(1), GRIPPER, MOUNT)
    expect(a.position.distanceTo(b.position)).toBe(0)
  })
})
