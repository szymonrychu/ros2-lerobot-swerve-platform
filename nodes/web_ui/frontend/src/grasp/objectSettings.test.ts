import { describe, expect, it } from 'vitest'
import {
  DEFAULT_OBJECT_SETTINGS,
  GRIP_PROFILE_KEY,
  loadGripProfile,
  loadObjectSettings,
  OBJECT_SETTINGS_KEY,
  saveGripProfile,
  saveObjectSettings,
} from './objectSettings'

function memory(initial: Record<string, string> = {}) {
  const data = { ...initial }
  return { data, getItem: (k: string) => data[k] ?? null, setItem: (k: string, v: string) => void (data[k] = v) }
}

const throwing = {
  getItem: (): string | null => {
    throw new Error('blocked')
  },
  setItem: (): void => {
    throw new Error('blocked')
  },
}

describe('object settings persistence', () => {
  it('defaults to 4 cm each and no gap', () => {
    expect(DEFAULT_OBJECT_SETTINGS).toEqual({ width: '0.04', depth: '0.04', height: '0.04', gapBelow: '0' })
  })

  it('round-trips through storage', () => {
    const s = memory()
    saveObjectSettings(s, { width: '0.05', depth: '0.06', height: '0.07', gapBelow: '0.01' })
    expect(loadObjectSettings(s)).toEqual({ width: '0.05', depth: '0.06', height: '0.07', gapBelow: '0.01' })
  })

  it('falls back to the defaults when storage is missing, empty, corrupt or throws', () => {
    expect(loadObjectSettings(undefined)).toEqual(DEFAULT_OBJECT_SETTINGS)
    expect(loadObjectSettings(memory())).toEqual(DEFAULT_OBJECT_SETTINGS)
    expect(loadObjectSettings(memory({ [OBJECT_SETTINGS_KEY]: '{not json' }))).toEqual(DEFAULT_OBJECT_SETTINGS)
    expect(loadObjectSettings(throwing)).toEqual(DEFAULT_OBJECT_SETTINGS)
  })

  it('keeps valid stored fields and defaults the rest', () => {
    const s = memory({ [OBJECT_SETTINGS_KEY]: JSON.stringify({ width: '0.09', depth: 5 }) })
    expect(loadObjectSettings(s)).toEqual({ ...DEFAULT_OBJECT_SETTINGS, width: '0.09' })
  })

  it('saving never throws when storage does', () => {
    expect(() => saveObjectSettings(throwing, DEFAULT_OBJECT_SETTINGS)).not.toThrow()
    expect(() => saveObjectSettings(undefined, DEFAULT_OBJECT_SETTINGS)).not.toThrow()
  })
})

describe('grip strength persistence', () => {
  it('defaults to normal and round-trips a valid profile', () => {
    expect(loadGripProfile(undefined)).toBe('normal')
    const s = memory()
    saveGripProfile(s, 'gentle')
    expect(s.data[GRIP_PROFILE_KEY]).toBe('gentle')
    expect(loadGripProfile(s)).toBe('gentle')
  })

  it('ignores unknown stored values and storage that throws', () => {
    expect(loadGripProfile(memory({ [GRIP_PROFILE_KEY]: 'crushing' }))).toBe('normal')
    expect(loadGripProfile(throwing)).toBe('normal')
    expect(() => saveGripProfile(throwing, 'firm')).not.toThrow()
    expect(() => saveGripProfile(undefined, 'firm')).not.toThrow()
  })
})
