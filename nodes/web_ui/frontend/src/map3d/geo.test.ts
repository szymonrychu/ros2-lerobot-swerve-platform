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
  validAnchor,
} from './geo'

const ANCHOR = { latitude: 52.2297, longitude: 21.0122, x: 10, y: -5, heading: 0, frame_id: 'map' }

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
  it('the anchor lat/lon lands on the anchor map point', () => {
    const p = latLonToMap(ANCHOR, ANCHOR.latitude, ANCHOR.longitude)
    expect(p.x).toBeCloseTo(10)
    expect(p.y).toBeCloseTo(-5)
  })

  it('with heading 0 the map +x axis points east and +y north', () => {
    const east = mapToLatLon(ANCHOR, 20, -5)
    expect(east.latitude).toBeCloseTo(ANCHOR.latitude, 8)
    expect(east.longitude).toBeGreaterThan(ANCHOR.longitude)
    const north = mapToLatLon(ANCHOR, 10, 5)
    expect(north.latitude).toBeGreaterThan(ANCHOR.latitude)
  })

  it('heading rotates the map frame: heading +90 deg means map +x points north', () => {
    const a = { ...ANCHOR, heading: Math.PI / 2 }
    const ll = mapToLatLon(a, 20, -5) // 10 m along map +x
    const enu = latLonToEnu(ll.latitude, ll.longitude, a.latitude, a.longitude)
    expect(enu.x).toBeCloseTo(0, 6)
    expect(enu.y).toBeCloseTo(10, 6)
    const back = latLonToMap(a, ll.latitude, ll.longitude)
    expect(back.x).toBeCloseTo(20, 6)
    expect(back.y).toBeCloseTo(-5, 6)
  })
})

describe('validAnchor', () => {
  it('accepts a full anchor and rejects missing or non-finite ones', () => {
    expect(validAnchor(ANCHOR)).toEqual(ANCHOR)
    expect(validAnchor(null)).toBeNull()
    expect(validAnchor(undefined)).toBeNull()
    expect(validAnchor({ ...ANCHOR, latitude: NaN })).toBeNull()
    expect(validAnchor({ latitude: 1, longitude: 2 })).toBeNull()
  })

  it('defaults a missing heading to 0', () => {
    const { heading: _h, ...noHeading } = ANCHOR
    expect(validAnchor(noHeading)!.heading).toBe(0)
  })
})

describe('tilesAround', () => {
  it('returns (2r+1)^2 tiles centred on the tile under the point', () => {
    const tiles = tilesAround(ANCHOR, { x: 10, y: -5 }, 19, 1)
    expect(tiles).toHaveLength(9)
    const centre = latLonToTile(ANCHOR.latitude, ANCHOR.longitude, 19)
    const mid = tiles.find((t) => t.x === Math.floor(centre.x) && t.y === Math.floor(centre.y))
    expect(mid).toBeDefined()
    expect(mid!.url).toBe(`/api/tiles/19/${mid!.x}/${mid!.y}.png`)
  })

  it('places each tile so that its centre maps back to the tile centre lat/lon', () => {
    const tiles = tilesAround(ANCHOR, { x: 10, y: -5 }, 19, 1)
    for (const t of tiles) {
      const ll = mapToLatLon(ANCHOR, t.center.x, t.center.y)
      const expected = tileToLatLon(t.x + 0.5, t.y + 0.5, 19)
      expect(ll.latitude).toBeCloseTo(expected.latitude, 6)
      expect(ll.longitude).toBeCloseTo(expected.longitude, 6)
    }
  })

  it('tiles are about tileSizeMeters wide, adjacent, and rotated by -heading', () => {
    const a = { ...ANCHOR, heading: 0.3 }
    const tiles = tilesAround(a, { x: 10, y: -5 }, 19, 1)
    const size = tileSizeMeters(a.latitude, 19)
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
    const polar = { ...ANCHOR, latitude: 0, longitude: -179.9999 }
    const tiles = tilesAround(polar, { x: 10, y: -5 }, 2, 1)
    for (const t of tiles) {
      expect(t.x).toBeGreaterThanOrEqual(0)
      expect(t.x).toBeLessThan(4)
    }
  })
})
