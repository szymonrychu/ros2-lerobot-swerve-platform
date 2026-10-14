import { describe, expect, it } from 'vitest'
import { CALIBRATE_HASH, isCalibrateHash } from './calibrateRoute'

describe('calibrateRoute', () => {
  it('builds the hash', () => {
    expect(CALIBRATE_HASH).toBe('#/calibrate')
  })
  it('parses the calibrate hash', () => {
    expect(isCalibrateHash('#/calibrate')).toBe(true)
    expect(isCalibrateHash('#/calibrate/')).toBe(true)
  })
  it('rejects everything else', () => {
    expect(isCalibrateHash('')).toBe(false)
    expect(isCalibrateHash('#')).toBe(false)
    expect(isCalibrateHash('#/calibrated')).toBe(false)
    expect(isCalibrateHash('#/map')).toBe(false)
  })
})
