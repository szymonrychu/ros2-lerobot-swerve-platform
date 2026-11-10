/**
 * GET /api/arm/calibration bodies of the backend contract fixtures (nodes/web_ui/tests/fixtures/api/arm_calibration_*.json,
 * pinned by the backend's test_arm_calibration.py) and never copied: a contract change on the backend shows up here.
 * Imported by tests only.
 */
import acceptedJson from '../../../tests/fixtures/api/arm_calibration_accepted.json'
import configuredJson from '../../../tests/fixtures/api/arm_calibration_configured.json'
import type { ArmCalibrationBody, ServedArmCalibration } from './armCalibrationApi'

/** Nothing accepted: the configured client.yml values. */
export const configuredBody = configuredJson as ArmCalibrationBody
/** An accepted calibration. */
export const acceptedBody = acceptedJson as ArmCalibrationBody

/** The configured values as served. */
export const CONFIGURED = configuredBody.calibration as ServedArmCalibration
/** The accepted calibration as served. */
export const ACCEPTED = acceptedBody.calibration as ServedArmCalibration
