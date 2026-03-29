import type { URDFRobot, URDFJoint } from 'urdf-loader'
import * as THREE from 'three'

export interface CCDIKOptions {
  chainJointNames: string[]
  endEffector: THREE.Object3D
  maxIterations?: number
  tolerance?: number
}

const _toEE = new THREE.Vector3()
const _toTarget = new THREE.Vector3()
const _worldAxis = new THREE.Vector3()
const _tempQuat = new THREE.Quaternion()
const _cross = new THREE.Vector3()

export function solveCCDIK(
  robot: URDFRobot,
  target: THREE.Vector3,
  options: CCDIKOptions,
): void {
  const { chainJointNames, endEffector } = options
  const maxIterations = options.maxIterations ?? 10
  const tolerance = options.tolerance ?? 0.001

  for (let iter = 0; iter < maxIterations; iter++) {
    for (let i = chainJointNames.length - 1; i >= 0; i--) {
      const jointName = chainJointNames[i]
      const joint = robot.joints[jointName] as unknown as URDFJoint
      if (!joint) continue

      const eePos = endEffector.getWorldPosition(new THREE.Vector3())
      const jointPos = joint.getWorldPosition(new THREE.Vector3())

      if (eePos.distanceTo(target) < tolerance) return

      joint.getWorldQuaternion(_tempQuat)
      _worldAxis.set(0, 0, 1).applyQuaternion(_tempQuat).normalize()

      _toEE.copy(eePos).sub(jointPos)
      _toTarget.copy(target).sub(jointPos)

      _toEE.addScaledVector(_worldAxis, -_toEE.dot(_worldAxis))
      _toTarget.addScaledVector(_worldAxis, -_toTarget.dot(_worldAxis))

      const eeLen = _toEE.length()
      const tLen = _toTarget.length()
      if (eeLen < 1e-6 || tLen < 1e-6) continue

      _cross.crossVectors(_toEE, _toTarget)
      const sign = Math.sign(_cross.dot(_worldAxis))
      const cosAngle = THREE.MathUtils.clamp(_toEE.dot(_toTarget) / (eeLen * tLen), -1, 1)
      const delta = sign * Math.acos(cosAngle)

      if (Math.abs(delta) < 1e-8) continue

      const currentAngle = (joint.jointValue as number[])[0] ?? 0
      robot.setJointValue(jointName, currentAngle + delta)

      robot.updateMatrixWorld(true)
    }
  }
}
