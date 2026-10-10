import { describe, expect, it } from 'vitest'
import {
  enuToLatLon,
  latLonToEnu,
  latLonToMap,
  latLonToTile,
  mapToLatLon,
  tileSizeMeters,
  tilesAround,
  tileToLatLon,
  tileUrl,
  anchorSummary,
  validAnchor,
} from './geo'

// Exact payload shape of '/web_ui/gps_anchor' (web_ui backend GpsAnchorEstimator): map (0, 0) sits at (lat, lon).
const ANCHOR = { lat: 52.2297, lon: 21.0122, heading_rad: 0, residual_m: 0.12, n_points: 42 }

describe('Web Mercator tile math', () => {
  it('puts lat/lon (0, 0) in the middle of the world', () => {
    const t = latLonToTile(0, 0, 1)
    expect(t.x).toBeCloseTo(1)
    expect(t.y).toBeCloseTo(1)
  })

  it('matches the OSM reference tile for a known city', () => {
    // Warsaw centre at z=19 lies in OSM tile x=292745, y=172635 (slippy map formula, computed independently)
    const t = latLonToTile(52.2297, 21.0122, 19)
    expect(Math.floor(t.x)).toBe(292745)
    expect(Math.floor(t.y)).toBe(172635)
  })

  it('tileToLatLon is the inverse of latLonToTile', () => {
    const t = latLonToTile(52.2297, 21.0122, 18)
    const ll = tileToLatLon(t.x, t.y, 18)
    expect(ll.latitude).toBeCloseTo(52.2297, 8)
    expect(ll.longitude).toBeCloseTo(21.0122, 8)
  })

  it('tile size shrinks with latitude and zoom', () => {
    expect(tileSizeMeters(0, 0)).toBeCloseTo(40075016.686, 0)
    expect(tileSizeMeters(60, 1)).toBeCloseTo(40075016.686 / 2 / 2, 0)
  })

  it('builds the backend tile proxy url', () => {
    expect(tileUrl(19, 1, 2)).toBe('/api/tiles/19/1/2.png')
    expect(tileUrl(19, 1, 2, 'ab12cd34')).toBe('/api/tiles/19/1/2.png?v=ab12cd34')
    expect(tileUrl(19, 1, 2, null)).toBe('/api/tiles/19/1/2.png')
    expect(tileUrl(19, 1, 2, 'ab12cd34', 'satellite')).toBe('/api/tiles/satellite/19/1/2.png?v=ab12cd34')
    expect(tileUrl(19, 1, 2, null, 'street')).toBe('/api/tiles/street/19/1/2.png')
  })
})

describe('local ENU <-> lat/lon', () => {
  it('one degree of latitude north is about 111.3 km', () => {
    const e = latLonToEnu(1, 0, 0, 0)
    expect(e.x).toBeCloseTo(0)
    expect(e.y / 1000).toBeCloseTo(111.32, 1)
  })

  it('east offsets shrink with cos(latitude)', () => {
    const e = latLonToEnu(60, 0.001, 60, 0)
    expect(e.x).toBeCloseTo(111.32 * 0.5, 0)
  })

  it('round-trips', () => {
    const ll = enuToLatLon(123.4, -56.7, 52.2, 21.0)
    const e = latLonToEnu(ll.latitude, ll.longitude, 52.2, 21.0)
    expect(e.x).toBeCloseTo(123.4, 6)
    expect(e.y).toBeCloseTo(-56.7, 6)
  })
})

describe('anchor placement (map frame <-> lat/lon)', () => {
  it('the anchor lat/lon lands on the map origin (0, 0)', () => {
    const p = latLonToMap(ANCHOR, ANCHOR.lat, ANCHOR.lon)
    expect(p.x).toBeCloseTo(0)
    expect(p.y).toBeCloseTo(0)
    const ll = mapToLatLon(ANCHOR, 0, 0)
    expect(ll.latitude).toBeCloseTo(ANCHOR.lat, 10)
    expect(ll.longitude).toBeCloseTo(ANCHOR.lon, 10)
  })

  it('with heading 0 the map +x axis points east and +y north', () => {
    const east = mapToLatLon(ANCHOR, 10, 0)
    expect(east.latitude).toBeCloseTo(ANCHOR.lat, 8)
    expect(east.longitude).toBeGreaterThan(ANCHOR.lon)
    const north = mapToLatLon(ANCHOR, 0, 10)
    expect(north.latitude).toBeGreaterThan(ANCHOR.lat)
  })

  it('heading rotates the map frame: heading +90 deg means map +x points north', () => {
    const a = { ...ANCHOR, heading_rad: Math.PI / 2 }
    const ll = mapToLatLon(a, 10, 0) // 10 m along map +x
    const enu = latLonToEnu(ll.latitude, ll.longitude, a.lat, a.lon)
    expect(enu.x).toBeCloseTo(0, 6)
    expect(enu.y).toBeCloseTo(10, 6)
    const back = latLonToMap(a, ll.latitude, ll.longitude)
    expect(back.x).toBeCloseTo(10, 6)
    expect(back.y).toBeCloseTo(0, 6)
  })
})

const COMPASS = { lat: 54.6, lon: 18.3, heading_rad: 1.2, residual_m: null, n_points: 1, source: 'compass' }

describe('compass anchor', () => {
  it('accepts a compass payload with residual_m null and keeps the source', () => {
    expect(validAnchor(COMPASS)).toEqual({ lat: 54.6, lon: 18.3, heading_rad: 1.2, n_points: 1, source: 'compass' })
  })

  it('keeps source fit and drops unknown sources', () => {
    expect(validAnchor({ ...ANCHOR, source: 'fit' })?.source).toBe('fit')
    expect(validAnchor({ ...ANCHOR, source: 'magic' })?.source).toBeUndefined()
  })

  it('summarises the anchor source for display', () => {
    expect(anchorSummary({ lat: 1, lon: 2, heading_rad: 0, source: 'compass', n_points: 1 })).toBe('compass heading')
    expect(anchorSummary({ lat: 1, lon: 2, heading_rad: 0, source: 'fit', n_points: 42, residual_m: 0.123 })).toBe(
      'drive fit, 42 pts, residual 0.12 m',
    )
    expect(anchorSummary({ lat: 1, lon: 2, heading_rad: 0, source: 'fit' })).toBe('drive fit')
    expect(anchorSummary({ lat: 1, lon: 2, heading_rad: 0 })).toBe('drive fit')
  })
})

describe('validAnchor', () => {
  it('accepts the backend payload {lat, lon, heading_rad, residual_m, n_points}', () => {
    expect(validAnchor(ANCHOR)).toEqual(ANCHOR)
  })

  it('rejects null (no fix yet), missing and non-finite fields', () => {
    expect(validAnchor(null)).toBeNull()
    expect(validAnchor(undefined)).toBeNull()
    expect(validAnchor({ ...ANCHOR, lat: NaN })).toBeNull()
    expect(validAnchor({ ...ANCHOR, lon: '21' })).toBeNull()
    expect(validAnchor({ ...ANCHOR, heading_rad: Infinity })).toBeNull()
    const { heading_rad: _h, ...noHeading } = ANCHOR
    expect(validAnchor(noHeading)).toBeNull()
    expect(validAnchor({ lat: 1, lon: 2 })).toBeNull()
  })

  it('rejects latitudes outside [-90, 90] and longitudes outside [-180, 180]', () => {
    expect(validAnchor({ ...ANCHOR, lat: 91 })).toBeNull()
    expect(validAnchor({ ...ANCHOR, lon: -181 })).toBeNull()
  })

  it('treats residual_m and n_points as optional diagnostics', () => {
    expect(validAnchor({ lat: 1, lon: 2, heading_rad: 0.5 })).toEqual({ lat: 1, lon: 2, heading_rad: 0.5 })
    expect(validAnchor({ ...ANCHOR, residual_m: 'x', n_points: null })).toEqual({ lat: ANCHOR.lat, lon: ANCHOR.lon, heading_rad: 0 })
  })

  it('does not accept the old invented {latitude, longitude, x, y, heading} shape', () => {
    expect(validAnchor({ latitude: 52.2, longitude: 21.0, x: 0, y: 0, heading: 0 })).toBeNull()
  })
})

describe('tilesAround', () => {
  it('returns (2r+1)^2 tiles centred on the tile under the point', () => {
    const tiles = tilesAround(ANCHOR, { x: 0, y: 0 }, 19, 1)
    expect(tiles).toHaveLength(9)
    const centre = latLonToTile(ANCHOR.lat, ANCHOR.lon, 19)
    const mid = tiles.find((t) => t.x === Math.floor(centre.x) && t.y === Math.floor(centre.y))
    expect(mid).toBeDefined()
    expect(mid!.url).toBe(`/api/tiles/19/${mid!.x}/${mid!.y}.png`)
  })

  it('overzooms: at display zoom 19 with max zoom 18 each tile crops a quadrant of its z18 parent', () => {
    const tiles = tilesAround(ANCHOR, { x: 0, y: 0 }, 19, 1, null, 'satellite', 18)
    expect(tiles).toHaveLength(9)
    for (const t of tiles) {
      expect(t.z).toBe(19)
      expect(t.url).toBe(`/api/tiles/satellite/18/${t.x >> 1}/${t.y >> 1}.png`)
      expect(t.crop.u1 - t.crop.u0).toBe(0.5)
      expect(t.crop.u0).toBe((t.x & 1) / 2)
      expect(t.crop.v0).toBe((t.y & 1) / 2)
    }
  })

  it('uses full-tile crops when the display zoom is within the max zoom', () => {
    const tiles = tilesAround(ANCHOR, { x: 0, y: 0 }, 18, 1, null, 'street', 18)
    expect(tiles.every((t) => t.crop.u0 === 0 && t.crop.v1 === 1 && t.url.includes('/18/'))).toBe(true)
  })

  it('appends the tile version to every tile url when given', () => {
    const tiles = tilesAround(ANCHOR, { x: 0, y: 0 }, 19, 1, 'ab12cd34')
    expect(tiles.every((t) => t.url.endsWith('.png?v=ab12cd34'))).toBe(true)
  })

  it('places each tile so that its centre maps back to the tile centre lat/lon', () => {
    const tiles = tilesAround(ANCHOR, { x: 0, y: 0 }, 19, 1)
    for (const t of tiles) {
      const ll = mapToLatLon(ANCHOR, t.center.x, t.center.y)
      const expected = tileToLatLon(t.x + 0.5, t.y + 0.5, 19)
      expect(ll.latitude).toBeCloseTo(expected.latitude, 6)
      expect(ll.longitude).toBeCloseTo(expected.longitude, 6)
    }
  })

  it('tiles are about tileSizeMeters wide, adjacent, and rotated by -heading', () => {
    const a = { ...ANCHOR, heading_rad: 0.3 }
    const tiles = tilesAround(a, { x: 0, y: 0 }, 19, 1)
    const size = tileSizeMeters(a.lat, 19)
    for (const t of tiles) {
      expect(t.width).toBeCloseTo(size, 0)
      expect(t.height).toBeCloseTo(size, 0)
      expect(t.yaw).toBeCloseTo(-0.3)
    }
    const c = tiles.find((t) => t.dx === 0 && t.dy === 0)!
    const e = tiles.find((t) => t.dx === 1 && t.dy === 0)!
    expect(Math.hypot(e.center.x - c.center.x, e.center.y - c.center.y)).toBeCloseTo(size, 0)
  })

  it('clamps tile indices to the world', () => {
    const polar = { ...ANCHOR, lat: 0, lon: -179.9999 }
    const tiles = tilesAround(polar, { x: 0, y: 0 }, 2, 1)
    for (const t of tiles) {
      expect(t.x).toBeGreaterThanOrEqual(0)
      expect(t.x).toBeLessThan(4)
    }
  })
})
