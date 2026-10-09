/** Pure GPS chip logic: level, colour and text of the robot (rover) and base station chips. */

export interface GpsStatusConfig {
  rover_topic: string | null
  base_url: string | null
  base_poll_hz: number
  stale_after_s: number
  base_timeout_s: number
}

/** Synthetic WS topic the backend stores the polled base status under. */
export const GPS_BASE_STATUS_KEY = '/web_ui/gps_base_status'

/** Rover status topic payload (gps_rtk rover node). */
export interface RoverGpsPayload {
  role: 'rover'
  quality: number
  fix: string
  num_satellites: number | null
  hdop: number | null
  diff_age_s: number | null
  ntrip_connected: boolean
  ntrip_rx_bytes: number
}

/** Base status as polled by the backend: a reachable sample or an observed failure. */
export interface BaseGpsSample {
  reachable: true
  role: 'base'
  quality: number
  fix: string
  num_satellites: number | null
  hdop: number | null
  ntrip_clients: number
  rtcm_tx_frames: number
  rtcm_tx_bytes: number
  rtcm_types: number[]
  /** Base antenna position from its GGA (absent in statuses from older gps_rtk builds). */
  latitude?: number
  longitude?: number
  altitude?: number
  stale?: boolean
}
export type BaseGpsPayload = BaseGpsSample | { reachable: false; error: string }

export type GpsLevel =
  | 'rtk_fixed'
  | 'rtk_float'
  | 'dgps'
  | 'gps'
  | 'no_fix'
  | 'other'
  | 'stale'
  | 'unreachable'
  | 'no_data'
export type GpsColor = 'success' | 'warning' | 'info' | 'error' | 'default'

export interface GpsChipStatus {
  level: GpsLevel
  color: GpsColor
  label: string
  /** Label without satellites / link detail, used on narrow screens. */
  shortLabel: string
  tooltip: string
}

const QUALITY_LEVELS: Record<number, { level: GpsLevel; color: GpsColor }> = {
  0: { level: 'no_fix', color: 'error' },
  1: { level: 'gps', color: 'info' },
  2: { level: 'dgps', color: 'warning' },
  4: { level: 'rtk_fixed', color: 'success' },
  5: { level: 'rtk_float', color: 'warning' },
}
const OTHER_QUALITY = { level: 'other' as GpsLevel, color: 'default' as GpsColor }

function simple(prefix: string, level: GpsLevel, color: GpsColor, text: string, tooltip: string): GpsChipStatus {
  const label = `${prefix}: ${text}`
  return { level, color, label, shortLabel: label, tooltip }
}

function fresh(nowMs: number, rxAtMs: number | null, cfg: GpsStatusConfig): boolean {
  return rxAtMs !== null && nowMs - rxAtMs <= cfg.stale_after_s * 1000
}

function satsText(sats: number | null): string | null {
  return sats === null ? null : `${sats} sats`
}

function hdopText(hdop: number | null): string {
  return hdop === null ? 'HDOP ?' : `HDOP ${hdop}`
}

/**
 * Robot (rover) chip.
 * @param payload latest rover status (undefined/null when none received)
 * @param cfg gps_status config from /api/config (null when off)
 * @param nowMs current time in ms
 * @param rxAtMs time in ms the payload arrived, or null if never
 * @returns no data without a sample; red "stale" when older than stale_after_s; otherwise colour by fix quality
 */
export function roverGpsStatus(
  payload: RoverGpsPayload | null | undefined,
  cfg: GpsStatusConfig | null | undefined,
  nowMs: number,
  rxAtMs: number | null,
): GpsChipStatus {
  const prefix = 'Robot'
  if (!payload || !cfg || rxAtMs === null) return simple(prefix, 'no_data', 'default', 'no data', 'Robot GPS: no data yet')
  if (!fresh(nowMs, rxAtMs, cfg)) {
    return simple(prefix, 'stale', 'error', 'stale', `Robot GPS: no message for more than ${cfg.stale_after_s} s`)
  }
  const { level, color } = QUALITY_LEVELS[payload.quality] ?? OTHER_QUALITY
  const sats = satsText(payload.num_satellites)
  const tooltip = [
    `Robot GPS: ${payload.fix}`,
    sats ?? 'satellites ?',
    hdopText(payload.hdop),
    payload.ntrip_connected ? 'NTRIP connected' : 'NTRIP disconnected',
    payload.diff_age_s === null ? null : `correction age ${payload.diff_age_s.toFixed(1)} s`,
    `NTRIP rx ${payload.ntrip_rx_bytes} B`,
  ]
    .filter((l): l is string => l !== null)
    .join('\n')
  return {
    level,
    color,
    label: [`${prefix}: ${payload.fix}`, sats].filter(Boolean).join(' - '),
    shortLabel: `${prefix}: ${payload.fix}`,
    tooltip,
  }
}

/**
 * Base station chip, with the rover-to-base link state taken from the rover status.
 * @param base latest polled base status (undefined/null when none received)
 * @param rover latest rover status, used for link state and correction age
 * @param cfg gps_status config from /api/config (null when off)
 * @param nowMs current time in ms
 * @param baseRxAtMs time in ms the base status arrived, or null if never
 * @param roverRxAtMs time in ms the rover status arrived, or null if never
 * @returns "unreachable" (red) when the poll failed; stale when flagged or old; otherwise colour by fix quality
 *   (red when the rover reports its NTRIP link down) with "link OK <age>s" / "link down" appended
 */
export function baseGpsStatus(
  base: BaseGpsPayload | null | undefined,
  rover: RoverGpsPayload | null | undefined,
  cfg: GpsStatusConfig | null | undefined,
  nowMs: number,
  baseRxAtMs: number | null,
  roverRxAtMs: number | null,
): GpsChipStatus {
  const prefix = 'Base'
  if (!base || !cfg || baseRxAtMs === null) return simple(prefix, 'no_data', 'default', 'no data', 'Base GPS: no data yet')
  if (!base.reachable) {
    return simple(prefix, 'unreachable', 'error', 'unreachable', `Base station unreachable: ${base.error}`)
  }
  if (base.stale || !fresh(nowMs, baseRxAtMs, cfg)) {
    return simple(prefix, 'stale', 'error', 'stale', `Base GPS: no new status for more than ${cfg.stale_after_s} s`)
  }
  const roverKnown = !!rover && fresh(nowMs, roverRxAtMs, cfg)
  let link: string | null = null
  if (roverKnown && rover) {
    if (!rover.ntrip_connected) link = 'link down'
    else link = rover.diff_age_s === null ? 'link OK' : `link OK ${rover.diff_age_s.toFixed(1)}s`
  }
  const linkDown = link === 'link down'
  const quality = QUALITY_LEVELS[base.quality] ?? OTHER_QUALITY
  const sats = satsText(base.num_satellites)
  const tooltip = [
    `Base GPS: ${base.fix}`,
    sats ?? 'satellites ?',
    hdopText(base.hdop),
    `${base.ntrip_clients} NTRIP ${base.ntrip_clients === 1 ? 'client' : 'clients'}`,
    `RTCM tx ${base.rtcm_tx_bytes} B (${base.rtcm_tx_frames} frames)`,
    `RTCM types ${base.rtcm_types.join(', ') || 'none'}`,
    roverKnown && rover ? `Rover link: ${link}, NTRIP rx ${rover.ntrip_rx_bytes} B` : 'Rover link: unknown',
  ].join('\n')
  return {
    level: quality.level,
    color: linkDown ? 'error' : quality.color,
    label: [`${prefix}: ${base.fix}`, sats, link].filter(Boolean).join(' - '),
    shortLabel: `${prefix}: ${base.fix}`,
    tooltip,
  }
}
