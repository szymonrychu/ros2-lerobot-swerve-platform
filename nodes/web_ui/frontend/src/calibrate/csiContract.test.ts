/**
 * Pins the CSI intrinsics client to the node's contract fixtures: every fixture passes the client's runtime checks and
 * every object the client checks has exactly the fixture's keys, so a key added, renamed or dropped on either side
 * fails here.
 */
import { afterEach, describe, expect, it, vi } from 'vitest'
import { CSI_KEYS, getCsiCameras, getCsiState } from './csiApi'
import camerasFx from '../../../../head_rgbd_calibration/tests/fixtures/api/csi_cameras.json'
import stateFx from '../../../../head_rgbd_calibration/tests/fixtures/api/csi_state.json'
import runningFx from '../../../../head_rgbd_calibration/tests/fixtures/api/csi_state_running.json'

const keys = (v: object) => Object.keys(v).sort()

function stub(body: unknown) {
  vi.stubGlobal('fetch', vi.fn(async () => new Response(JSON.stringify(body))))
}

afterEach(() => {
  vi.unstubAllGlobals()
})

describe('CSI client against the node contract fixtures', () => {
  it('accepts every node fixture', async () => {
    stub(camerasFx)
    expect(await getCsiCameras()).toEqual({ ok: true, value: camerasFx })
    for (const fx of [stateFx, runningFx]) {
      stub(fx)
      expect(await getCsiState('front')).toEqual({ ok: true, value: fx })
    }
  })

  it.each([
    ['cameras', camerasFx],
    ['state', stateFx],
    ['state', runningFx],
    ['capture', stateFx.capture],
    ['capture', runningFx.capture],
    ['session', stateFx.session],
    ['result', stateFx.session.results[0]],
    ['applied', stateFx.applied],
  ] as const)('the %s shape has exactly the fixture keys', (shape, fixture) => {
    expect([...CSI_KEYS[shape]].sort()).toEqual(keys(fixture))
  })
})
