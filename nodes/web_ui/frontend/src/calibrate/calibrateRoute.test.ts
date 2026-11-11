import { describe, expect, it } from 'vitest'
import { CALIBRATE_HASH, FLOW_IDS, calibrateFlow, flowHash, isCalibrateHash } from './calibrateRoute'

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
  it('lists the four flows in page order', () => {
    expect(FLOW_IDS).toEqual(['servo-calibration', 'imu', 'camera-ground', 'arm-camera'])
  })
  it('a flow deep link is a calibrate hash and names its flow', () => {
    for (const flow of FLOW_IDS) {
      expect(flowHash(flow)).toBe(`#/calibrate/${flow}`)
      expect(isCalibrateHash(flowHash(flow))).toBe(true)
      expect(calibrateFlow(flowHash(flow))).toBe(flow)
    }
  })
  it('an unknown flow is still the page but names no flow', () => {
    expect(isCalibrateHash('#/calibrate/nope')).toBe(true)
    expect(calibrateFlow('#/calibrate/nope')).toBeNull()
    expect(calibrateFlow('#/calibrate')).toBeNull()
    expect(calibrateFlow('#/map')).toBeNull()
  })
})
