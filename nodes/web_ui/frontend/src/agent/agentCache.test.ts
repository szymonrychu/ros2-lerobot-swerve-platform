import { describe, expect, it } from 'vitest'
import { AGENT_STORAGE_PREFIX, clearAgentStorage } from './agentCache'

class MemStorage {
  private data = new Map<string, string>()
  get length() { return this.data.size }
  key(i: number) { return [...this.data.keys()][i] ?? null }
  getItem(k: string) { return this.data.get(k) ?? null }
  setItem(k: string, v: string) { this.data.set(k, v) }
  removeItem(k: string) { this.data.delete(k) }
  keys() { return [...this.data.keys()] }
}

describe('clearAgentStorage', () => {
  it('removes only the agent keys from both storages', () => {
    const local = new MemStorage()
    const session = new MemStorage()
    local.setItem(`${AGENT_STORAGE_PREFIX}draft`, 'x')
    local.setItem(`${AGENT_STORAGE_PREFIX}scroll`, 'y')
    local.setItem('other', 'z')
    session.setItem(`${AGENT_STORAGE_PREFIX}cursor`, '1')
    clearAgentStorage([local as unknown as Storage, session as unknown as Storage])
    expect(local.keys()).toEqual(['other'])
    expect(session.keys()).toEqual([])
  })

  it('survives a storage that throws', () => {
    const broken = new Proxy({}, { get() { throw new Error('denied') } }) as Storage
    expect(() => clearAgentStorage([broken])).not.toThrow()
  })
})
