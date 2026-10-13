import { describe, expect, it } from 'vitest'
import { needsColorSmall, subscribedTopics } from './subscription'
import { TabConfig } from '../types'

const map = { id: 'm', type: 'map_nav', label: 'Map', map_topic: '/map', goal_topic: '/goal' } as TabConfig
const rgbd = { id: 'r', type: 'rgbd_camera', label: 'RGBD', color_topic: '/c', depth_topic: '/d' } as TabConfig
const head = { id: 'h', type: 'head', label: 'Head', rgb_camera_topic: '/h/rgb' } as TabConfig

describe('subscribedTopics', () => {
  it('skips the topics of inactive image tabs', () => {
    expect(subscribedTopics([map, rgbd, head], ['/ov'], map)).toEqual(['/goal', '/map', '/ov'])
  })
  it('includes the active image tab topics', () => {
    expect(subscribedTopics([map, rgbd, head], [], rgbd)).toEqual(['/c', '/d', '/goal', '/map'])
  })
})

describe('needsColorSmall', () => {
  it('is true only for the RGBD tab', () => {
    expect(needsColorSmall(rgbd)).toBe(true)
    expect(needsColorSmall(head)).toBe(false)
    expect(needsColorSmall(undefined)).toBe(false)
  })
})
