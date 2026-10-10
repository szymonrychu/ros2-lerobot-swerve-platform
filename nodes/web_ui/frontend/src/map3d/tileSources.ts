/** Tile source selection and overzoom (parent tile + UV sub-rectangle) math for the GPS map layer. */
import type { TabStorage } from '../tabSelection'

export const TILE_SOURCE_STORAGE_KEY = 'web_ui.map3d.tile_source'

/** Public tile source description from /api/config (no URL or key). */
export interface TileSourceInfo {
  id: string
  label: string
  max_zoom: number
  attribution?: string | null
  version?: string | null
}

/** Sub-rectangle of a tile image as fractions from the top-left corner (v grows downward). */
export interface TileCrop {
  u0: number
  v0: number
  u1: number
  v1: number
}

/** The tile actually requested for a display tile, and the part of it the display tile shows. */
export interface OverzoomTile extends TileCrop {
  z: number
  x: number
  y: number
}

/**
 * Parent tile at the source's max zoom for a display tile, with the sub-rectangle to show.
 *
 * @param z - display zoom
 * @param x - display tile column
 * @param y - display tile row
 * @param maxZoom - highest zoom the source serves
 * @returns the tile itself (full crop) when z <= maxZoom, else its ancestor at maxZoom and the 1/2^(z-maxZoom) square
 */
export function overzoomTile(z: number, x: number, y: number, maxZoom: number): OverzoomTile {
  if (z <= maxZoom) return { z, x, y, u0: 0, v0: 0, u1: 1, v1: 1 }
  const shift = z - maxZoom
  const span = 2 ** shift
  const px = Math.floor(x / span)
  const py = Math.floor(y / span)
  const u0 = (x - px * span) / span
  const v0 = (y - py * span) / span
  return { z: maxZoom, x: px, y: py, u0, v0, u1: u0 + 1 / span, v1: v0 + 1 / span }
}

/**
 * Texture coordinates for a PlaneGeometry (vertex order top-left, top-right, bottom-left, bottom-right) that
 * shows only a crop of the texture. Texture v is bottom-up, the crop is top-down.
 *
 * @param crop - sub-rectangle of the image
 * @returns [u, v] pairs flattened for the four plane vertices
 */
export function cropUvs(crop: TileCrop): number[] {
  const top = 1 - crop.v0
  const bottom = 1 - crop.v1
  return [crop.u0, top, crop.u1, top, crop.u0, bottom, crop.u1, bottom]
}

/**
 * Choose the active tile source.
 *
 * @param sources - sources from /api/config
 * @param stored - id stored in the browser, or null
 * @param defaultId - tab default_tile_source, or null
 * @returns stored source if it still exists, else the default, else the first; null when there are no sources
 */
export function pickTileSource(
  sources: TileSourceInfo[],
  stored: string | null,
  defaultId: string | null,
): TileSourceInfo | null {
  return (
    sources.find((s) => s.id === stored) ?? sources.find((s) => s.id === defaultId) ?? sources[0] ?? null
  )
}

/**
 * Read the stored source id; storage may be missing or throw.
 *
 * @param storage - localStorage or a stand-in, or undefined
 * @returns stored id or null
 */
export function readStoredTileSource(storage: TabStorage | undefined): string | null {
  try {
    return storage?.getItem(TILE_SOURCE_STORAGE_KEY) ?? null
  } catch {
    return null
  }
}

/**
 * Persist the chosen source id; failures are ignored.
 *
 * @param storage - localStorage or a stand-in, or undefined
 * @param id - source id
 */
export function writeStoredTileSource(storage: TabStorage | undefined, id: string): void {
  try {
    storage?.setItem(TILE_SOURCE_STORAGE_KEY, id)
  } catch {
    // Storage unavailable: the choice resets next visit.
  }
}
