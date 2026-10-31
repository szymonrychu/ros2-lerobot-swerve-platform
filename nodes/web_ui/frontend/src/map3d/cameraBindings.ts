/** Mouse and touch bindings of the map's OrbitControls for the four combinations of view and goal mode. */
import * as THREE from 'three'

export interface OrbitBindings {
  mouseButtons: Partial<Record<'LEFT' | 'MIDDLE' | 'RIGHT', THREE.MOUSE>>
  touches: Partial<Record<'ONE' | 'TWO', THREE.TOUCH>>
}

/**
 * Bindings of the camera controls. In goal mode the left button and one finger belong to the press-drag goal gesture,
 * so the camera keeps only the other inputs (the wheel always zooms).
 *
 * @param topView - locked top-down camera instead of free orbit
 * @param goalMode - the one-finger / left-button gesture is the goal gesture
 * @returns the OrbitControls mouseButtons and touches
 */
export function orbitBindings({ topView, goalMode }: { topView: boolean; goalMode: boolean }): OrbitBindings {
  if (topView) {
    return goalMode
      ? { mouseButtons: { MIDDLE: THREE.MOUSE.DOLLY, RIGHT: THREE.MOUSE.PAN }, touches: { TWO: THREE.TOUCH.DOLLY_PAN } }
      : {
          mouseButtons: { LEFT: THREE.MOUSE.PAN, MIDDLE: THREE.MOUSE.DOLLY, RIGHT: THREE.MOUSE.PAN },
          touches: { ONE: THREE.TOUCH.PAN, TWO: THREE.TOUCH.DOLLY_PAN },
        }
  }
  return goalMode
    ? { mouseButtons: { MIDDLE: THREE.MOUSE.PAN, RIGHT: THREE.MOUSE.ROTATE }, touches: { TWO: THREE.TOUCH.DOLLY_PAN } }
    : {
        mouseButtons: { LEFT: THREE.MOUSE.ROTATE, MIDDLE: THREE.MOUSE.DOLLY, RIGHT: THREE.MOUSE.PAN },
        touches: { ONE: THREE.TOUCH.ROTATE, TWO: THREE.TOUCH.DOLLY_PAN },
      }
}
