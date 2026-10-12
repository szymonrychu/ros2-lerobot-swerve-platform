import { describe, expect, it } from 'vitest'
import { renderToStaticMarkup } from 'react-dom/server'
import { GpsChip } from './GpsChip'
import { baseGpsStatus, roverGpsStatus, type GpsStatusConfig, type RoverGpsPayload } from '../gps/gpsStatus'

const CFG: GpsStatusConfig = {
  rover_topic: '/client/gps/status',
  base_url: 'http://server:5016/status',
  base_poll_hz: 1,
  stale_after_s: 5,
  base_timeout_s: 2,
}
const NOW = 100_000
const NO_FIX: RoverGpsPayload = {
  role: 'rover',
  quality: 0,
  fix: 'No fix',
  num_satellites: 7,
  hdop: 3.5,
  diff_age_s: null,
  ntrip_connected: true,
  ntrip_rx_bytes: 900,
  nmea_silent: false,
  nmea_received: true,
  nmea_age_s: 0.1,
}
const SILENT: RoverGpsPayload = {
  ...NO_FIX,
  quality: null,
  fix: 'Receiver silent',
  num_satellites: null,
  hdop: null,
  nmea_silent: true,
  nmea_age_s: 12.3,
}

describe('GpsChip', () => {
  it('renders no fix with satellites and an error mark', () => {
    const html = renderToStaticMarkup(<GpsChip status={roverGpsStatus(NO_FIX, CFG, NOW, NOW)} role="robot" />)
    expect(html).toContain('Robot: No fix - 7 sats')
    expect(html).not.toContain('<svg')
    expect(html).toContain('data-testid="status-mark"')
  })

  it('renders receiver silent with an error mark', () => {
    const html = renderToStaticMarkup(<GpsChip status={roverGpsStatus(SILENT, CFG, NOW, NOW)} role="robot" />)
    expect(html).toContain('Robot: receiver silent')
    expect(html).not.toContain('<svg')
    expect(html).toContain('data-testid="status-mark"')
  })

  it('renders no data distinctly from silent', () => {
    const html = renderToStaticMarkup(<GpsChip status={roverGpsStatus(undefined, CFG, NOW, null)} role="robot" />)
    expect(html).toContain('Robot: no data')
    expect(html).not.toContain('data-testid="status-mark"')
  })

  it('renders a reachable base without a status yet', () => {
    const status = baseGpsStatus({ reachable: true, has_status: false, reason: 'no status yet' }, NO_FIX, CFG, NOW, NOW, NOW)
    const html = renderToStaticMarkup(<GpsChip status={status} role="base" />)
    expect(html).toContain('Base: no status yet')
    expect(html).not.toContain('<svg')
  })
})
