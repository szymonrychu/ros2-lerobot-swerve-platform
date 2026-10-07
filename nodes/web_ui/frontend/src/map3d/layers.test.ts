import { describe, expect, it } from 'vitest'
import {
  DEFAULT_LAYERS,
  LAYER_KEYS,
  LAYER_STORAGE_KEY,
  parseLayerState,
  readLayerState,
  writeLayerState,
} from './layers'

function memoryStorage(initial: Record<string, string> = {}) {
  const data = new Map(Object.entries(initial))
  return {
    data,
    getItem: (k: string) => data.get(k) ?? null,
    setItem: (k: string, v: string) => {
      data.set(k, v)
    },
  }
}

const throwingStorage = {
  getItem: (): string | null => {
    throw new Error('SecurityError')
  },
  setItem: (): void => {
    throw new Error('QuotaExceededError')
  },
}

describe('layer defaults', () => {
  it('has a toggle for every required layer and robot part', () => {
    expect([...LAYER_KEYS].sort()).toEqual(
      [
        'slamMap',
        'localCostmap',
        'gpsMap',
        'globalPlan',
        'localPlan',
        'goal',
        'footprint',
        'robotBase',
        'robotWheels',
        'robotArm',
        'pois',
      ].sort(),
    )
  })

  it('shows everything except the GPS map by default', () => {
    for (const k of LAYER_KEYS) expect(DEFAULT_LAYERS[k]).toBe(k !== 'gpsMap')
  })
})

describe('parseLayerState', () => {
  it('returns the defaults for null, garbage or non-object JSON', () => {
    expect(parseLayerState(null)).toEqual(DEFAULT_LAYERS)
    expect(parseLayerState('{not json')).toEqual(DEFAULT_LAYERS)
    expect(parseLayerState('[1,2]')).toEqual(DEFAULT_LAYERS)
    expect(parseLayerState('42')).toEqual(DEFAULT_LAYERS)
  })

  it('takes stored booleans and ignores unknown keys and wrong types', () => {
    const s = parseLayerState(JSON.stringify({ slamMap: false, gpsMap: true, bogus: false, goal: 'no' }))
    expect(s.slamMap).toBe(false)
    expect(s.gpsMap).toBe(true)
    expect(s.goal).toBe(true)
    expect('bogus' in s).toBe(false)
  })

  it('returns a fresh object (defaults are not mutated)', () => {
    const s = parseLayerState(null)
    s.slamMap = false
    expect(DEFAULT_LAYERS.slamMap).toBe(true)
  })
})

describe('layer persistence', () => {
  it('round-trips through storage under the documented key', () => {
    const storage = memoryStorage()
    writeLayerState(storage, { ...DEFAULT_LAYERS, robotArm: false })
    expect(storage.data.has(LAYER_STORAGE_KEY)).toBe(true)
    expect(readLayerState(storage).robotArm).toBe(false)
  })

  it('falls back to defaults and never throws when storage fails or is missing', () => {
    expect(readLayerState(throwingStorage)).toEqual(DEFAULT_LAYERS)
    expect(readLayerState(undefined)).toEqual(DEFAULT_LAYERS)
    expect(() => writeLayerState(throwingStorage, DEFAULT_LAYERS)).not.toThrow()
    expect(() => writeLayerState(undefined, DEFAULT_LAYERS)).not.toThrow()
  })
})
