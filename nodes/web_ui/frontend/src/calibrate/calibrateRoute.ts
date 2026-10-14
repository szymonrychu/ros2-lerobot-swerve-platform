/** Hash of the hidden Calibrate page. It is a route, never a tab. */
export const CALIBRATE_HASH = '#/calibrate'

/** True when the location hash addresses the Calibrate page (a trailing slash is tolerated). */
export function isCalibrateHash(hash: string): boolean {
  return hash === CALIBRATE_HASH || hash === `${CALIBRATE_HASH}/`
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
