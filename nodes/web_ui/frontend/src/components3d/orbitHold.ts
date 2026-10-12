export interface Enableable {
  enabled: boolean
}

export interface OrbitHold {
  hold: (controls: Enableable | null | undefined) => void
  release: (controls: Enableable | null | undefined) => void
}

/**
 * Disables orbit controls for a drag and later restores the state they had before it, so a
 * controller that keeps them off (a glued camera mode) is not re-enabled by drag end.
 *
 * @returns hold(controls) disables and remembers the prior state; release(controls) restores it
 */
export function createOrbitHold(): OrbitHold {
  let previous: boolean | null = null
  return {
    hold(controls) {
      if (!controls) return
      if (previous === null) previous = controls.enabled
      controls.enabled = false
    },
    release(controls) {
      if (!controls || previous === null) return
      controls.enabled = previous
      previous = null
    },
  }
}
