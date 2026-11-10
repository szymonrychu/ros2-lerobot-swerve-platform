// @vitest-environment jsdom
import { act, cleanup, render } from '@testing-library/react'
import { useEffect, type RefObject } from 'react'
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import * as THREE from 'three'
import type { URDFRobot } from 'urdf-loader'
import type { OrbitControls as OrbitControlsImpl } from 'three-stdlib'
import { InteractiveArm } from './InteractiveArm'

const COMMAND_TOPIC = '/arm/cmd'
const NAMES = ['shoulder_pan', 'gripper']
const MEASURED = [0.8, 0.4]
const OLD_OFFSETS = { shoulder_pan: 0.2, gripper: 0.1 }
const NEW_OFFSETS = { shoulder_pan: 0.5, gripper: 0.3 }

const canvasEl = vi.hoisted(() => {
  const el = document.createElement('canvas')
  el.getBoundingClientRect = () => ({ left: 0, top: 0, width: 200, height: 200, right: 200, bottom: 200, x: 0, y: 0, toJSON: () => ({}) })
  return el
})
const mocks = vi.hoisted(() => ({ robot: null as unknown, setValues: [] as Array<[string, number]> }))

/** Camera 2 m in front of the arm looking at its root. */
const camera = new THREE.PerspectiveCamera(50, 1, 0.01, 50)
camera.position.set(0, 0, 2)
camera.lookAt(0, 0, 0)
camera.updateMatrixWorld()

vi.mock('@react-three/fiber', () => ({
  useThree: () => ({ invalidate: vi.fn(), camera, gl: { domElement: canvasEl } }),
}))

vi.mock('./RobotModel', () => ({
  RobotModel: ({ ghost, onRobotLoaded }: { ghost?: boolean; onRobotLoaded?: (r: unknown) => void }) => {
    useEffect(() => {
      if (ghost) onRobotLoaded?.(mocks.robot)
      // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [])
    return null
  },
}))

/** A ghost robot with two revolute joints and no links (no spheres or arrows are made). */
function fakeRobot(): URDFRobot {
  const robot = new THREE.Object3D() as unknown as URDFRobot & THREE.Object3D
  const joints: Record<string, THREE.Object3D> = {}
  NAMES.forEach((name, i) => {
    const j = new THREE.Object3D() as THREE.Object3D & { jointType: string }
    j.jointType = 'revolute'
    j.position.set(i * 0.1, 0, 0)
    robot.add(j)
    joints[name] = j
  })
  Object.assign(robot, {
    joints,
    links: {},
    setJointValue: (name: string, value: number) => mocks.setValues.push([name, value]),
  })
  robot.updateMatrixWorld(true)
  return robot
}

function urdfStates(offsets: Record<string, number>) {
  return { name: NAMES, position: MEASURED.map((m, i) => m + offsets[NAMES[i]]) }
}

function pointer(type: string, x: number, y: number): MouseEvent {
  return new MouseEvent(type, { clientX: x, clientY: y, button: 0, bubbles: true })
}

describe('InteractiveArm', () => {
  let now = 1000

  beforeEach(() => {
    mocks.robot = fakeRobot()
    mocks.setValues = []
    now = 1000
    vi.spyOn(Date, 'now').mockImplementation(() => now)
    // Every pointer hits the gripper ring.
    vi.spyOn(THREE.Raycaster.prototype, 'intersectObjects').mockImplementation(
      (targets: THREE.Object3D[]) => {
        const ring = targets.find((t) => t.parent === (mocks.robot as URDFRobot).joints.gripper)
        return ring ? [{ object: ring } as THREE.Intersection] : []
      },
    )
  })

  afterEach(() => {
    cleanup()
    vi.restoreAllMocks()
  })

  it('keeps the offsets the commanded angles were taken with when a calibration arrives mid-drag', () => {
    const publish = vi.fn()
    const orbitRef = { current: null } as RefObject<OrbitControlsImpl | null>
    const props = { urdfFile: 'arm.urdf', commandTopic: COMMAND_TOPIC, publish, orbitRef }
    const { rerender } = render(<InteractiveArm {...props} liveJointStates={urdfStates(OLD_OFFSETS)} jointOffsets={OLD_OFFSETS} />)

    act(() => {
      canvasEl.dispatchEvent(pointer('pointerdown', 150, 100))
      window.dispatchEvent(pointer('pointermove', 100, 150))
    })
    // A new calibration (and the live states drawn with it) arrives mid-drag.
    rerender(<InteractiveArm {...props} liveJointStates={urdfStates(NEW_OFFSETS)} jointOffsets={NEW_OFFSETS} />)
    act(() => {
      now += 100
      window.dispatchEvent(pointer('pointermove', 50, 100))
      window.dispatchEvent(pointer('pointerup', 50, 100))
    })

    const msgs = publish.mock.calls.map((c) => c[2] as { name: string[]; position: number[] })
    expect(publish.mock.calls.every((c) => c[0] === COMMAND_TOPIC)).toBe(true)
    expect(msgs).toHaveLength(3)
    const gripperSets = mocks.setValues.filter(([n]) => n === 'gripper').map(([, v]) => v)
    expect(gripperSets).toHaveLength(2)
    const expectedGripper = [gripperSets[0], gripperSets[1], gripperSets[1]]
    msgs.forEach((msg, i) => {
      expect(msg.name).toEqual(NAMES)
      // Untouched joint: always its measured angle.
      expect(msg.position[0]).toBeCloseTo(MEASURED[0], 9)
      // Dragged joint: the ghost's URDF angle less the offset it was dragged with.
      expect(msg.position[1]).toBeCloseTo(expectedGripper[i] - OLD_OFFSETS.gripper, 9)
    })
  })
})
