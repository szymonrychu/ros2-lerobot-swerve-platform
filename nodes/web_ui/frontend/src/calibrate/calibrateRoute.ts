/** Hash of the hidden Calibrate page. It is a route, never a tab. */
export const CALIBRATE_HASH = '#/calibrate'

/** The calibration flows of the page, in page order; `#/calibrate/<flow>` opens one. */
export const FLOW_IDS = ['servo-calibration', 'imu', 'camera-ground', 'arm-camera'] as const
export type FlowId = (typeof FLOW_IDS)[number]

/**
 * True when the location hash addresses the Calibrate page: `#/calibrate`, a trailing slash or a flow segment.
 *
 * @param hash - location hash
 * @returns whether the page is addressed
 */
export function isCalibrateHash(hash: string): boolean {
  return hash === CALIBRATE_HASH || hash.startsWith(`${CALIBRATE_HASH}/`)
}

/**
 * The flow a Calibrate hash deep-links to.
 *
 * @param hash - location hash
 * @returns the flow id, or null for the bare page, an unknown flow or another route
 */
export function calibrateFlow(hash: string): FlowId | null {
  if (!hash.startsWith(`${CALIBRATE_HASH}/`)) return null
  const flow = hash.slice(CALIBRATE_HASH.length + 1)
  return (FLOW_IDS as readonly string[]).includes(flow) ? (flow as FlowId) : null
}

/**
 * Deep link of one flow.
 *
 * @param flow - flow id
 * @returns `#/calibrate/<flow>`
 */
export function flowHash(flow: FlowId): string {
  return `${CALIBRATE_HASH}/${flow}`
}

/** Open the Calibrate page. */
export function openCalibrate(): void {
  window.location.hash = CALIBRATE_HASH
}

/** Leave the Calibrate page: clear the hash without leaving a bare '#' or reloading. */
export function clearCalibrateHash(): void {
  window.history.replaceState(null, '', window.location.pathname + window.location.search)
  window.dispatchEvent(new HashChangeEvent('hashchange'))
}
