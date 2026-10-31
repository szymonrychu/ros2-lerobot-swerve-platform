// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from "vitest";
import {
  allowBigJoints,
  applyHeadArm,
  boardAttached,
  computeArm,
  computeHeadArm,
  getHeadArmResult,
  getHeadArmState,
  listHeadArmSessions,
  moveToBase,
  park,
  stopSequence,
} from "./headArmApi";
import stateFx from "../../../../head_rgbd_calibration/tests/fixtures/api/head_arm_state.json";
import resultFx from "../../../../head_rgbd_calibration/tests/fixtures/api/head_arm_result.json";
import sessionsFx from "../../../../head_rgbd_calibration/tests/fixtures/api/head_arm_sessions.json";
import applyFx from "../../../../head_rgbd_calibration/tests/fixtures/api/floor_apply.json";

const B = "/api/calibration/rgbd/head-arm";
/** result.json of a passed arm offsets fit (kind "arm"), as arm_results.compute_arm writes it. */
const ARM_RESULT = {
  id: "r-20261031-120000",
  session: "calib-20261031-110000",
  kind: "arm",
  floor_session: "floor-20261030-090000",
  floor: { session: "floor-20261030-090000", result: "r-20261030-091000" },
  created: "2026-10-31T12:00:00+01:00",
  passed: true,
  failed_limits: [],
  error: null,
  rms_px: 0.61,
  holdout_rms_px: 0.72,
  holdout_tool_mm: 1.9,
  wrist_roll_fitted: false,
  tool_fitted: false,
  tool_offset_m: [0.0, 0.0, -0.0981],
  offsets: {
    shoulder_pan: { value: 0.02, sigma: 0.001, current: 0.0182 },
    wrist_roll: { value: 0.0, sigma: null, current: 0.0 },
  },
  head: { mount_x_m: { value: 0.156, sigma: 0.0001, urdf: 0.155 } },
  limits: [{ name: "rms_px", value: 0.61, limit: 1.5, ok: true }],
  snippet: "    joint_offsets_rad:\n      shoulder_pan: 0.0200\n",
};
/** An arm result that stopped on an input error: only the base keys and the error. */
const ARM_ERROR = {
  id: "r-20261031-120100",
  session: "calib-20261031-110000",
  kind: "arm",
  floor_session: null,
  floor: null,
  created: "2026-10-31T12:01:00+01:00",
  passed: false,
  failed_limits: [],
  error: "apply a head calibration first",
};
const JSON_HEADERS = { "Content-Type": "application/json" };

function stub(status: number, body: unknown) {
  const f = vi.fn(async () => new Response(JSON.stringify(body), { status }));
  vi.stubGlobal("fetch", f);
  return f;
}
afterEach(() => {
  vi.unstubAllGlobals();
});

describe("headArmApi", () => {
  it("getHeadArmState accepts the node fixture and applied null", async () => {
    const f = stub(200, stateFx);
    expect(await getHeadArmState()).toEqual({ ok: true, value: stateFx });
    expect(f).toHaveBeenCalledWith(`${B}/state`, { cache: "no-store" });
    stub(200, { sequence: stateFx.sequence, applied: null });
    expect((await getHeadArmState()).ok).toBe(true);
  });
  it("getHeadArmState accepts the idle sequence with null tier and arm_pose", async () => {
    const idle = { ...stateFx.sequence, tier: null, arm_pose: null, skipped: {} };
    stub(200, { sequence: idle, applied: null });
    expect((await getHeadArmState()).ok).toBe(true);
  });
  it("sequence steps post their path and body", async () => {
    const f = stub(200, stateFx.sequence);
    expect(await moveToBase(true)).toEqual({
      ok: true,
      value: stateFx.sequence,
    });
    expect(f).toHaveBeenLastCalledWith(`${B}/sequence/base`, {
      method: "POST",
      headers: JSON_HEADERS,
      body: JSON.stringify({ confirm: true }),
    });
    await boardAttached();
    expect(f).toHaveBeenLastCalledWith(`${B}/sequence/board-attached`, {
      method: "POST",
    });
    await allowBigJoints(false);
    expect(f).toHaveBeenLastCalledWith(`${B}/sequence/big-joints`, {
      method: "POST",
      headers: JSON_HEADERS,
      body: JSON.stringify({ confirm: false }),
    });
    await stopSequence();
    expect(f).toHaveBeenLastCalledWith(`${B}/sequence/stop`, {
      method: "POST",
    });
    await park(true);
    expect(f).toHaveBeenLastCalledWith(`${B}/park`, {
      method: "POST",
      headers: JSON_HEADERS,
      body: JSON.stringify({ confirm: true, board_on: false }),
    });
    await park(true, true);
    expect(f).toHaveBeenLastCalledWith(`${B}/park`, {
      method: "POST",
      headers: JSON_HEADERS,
      body: JSON.stringify({ confirm: true, board_on: true }),
    });
  });
  it("listHeadArmSessions accepts the sessions fixture", async () => {
    const f = stub(200, sessionsFx);
    expect(await listHeadArmSessions()).toEqual({
      ok: true,
      value: sessionsFx,
    });
    expect(f).toHaveBeenCalledWith(`${B}/sessions`, { cache: "no-store" });
  });
  it("computeHeadArm and getHeadArmResult accept the result fixture", async () => {
    const f = stub(200, resultFx);
    expect(await computeHeadArm("a/b")).toEqual({ ok: true, value: resultFx });
    expect(f).toHaveBeenCalledWith(`${B}/sessions/a%2Fb/compute`, {
      method: "POST",
    });
    expect(await getHeadArmResult("s", "r")).toEqual({
      ok: true,
      value: resultFx,
    });
    expect(f).toHaveBeenLastCalledWith(`${B}/sessions/s/results/r`, {
      cache: "no-store",
    });
  });
  it("computeArm posts the floor session", async () => {
    const f = stub(200, ARM_RESULT);
    expect(await computeArm("a/b", "floor-20261030-090000")).toEqual({
      ok: true,
      value: ARM_RESULT,
    });
    expect(f).toHaveBeenLastCalledWith(`${B}/sessions/a%2Fb/compute-arm`, {
      method: "POST",
      headers: JSON_HEADERS,
      body: JSON.stringify({ floor_session: "floor-20261030-090000" }),
    });
    await computeArm("s");
    expect(f).toHaveBeenLastCalledWith(`${B}/sessions/s/compute-arm`, {
      method: "POST",
      headers: JSON_HEADERS,
      body: JSON.stringify({ floor_session: null }),
    });
  });
  it("computeArm and getHeadArmResult accept an arm result, also one that stopped on an error", async () => {
    stub(200, ARM_ERROR);
    expect(await computeArm("s")).toEqual({ ok: true, value: ARM_ERROR });
    stub(200, ARM_RESULT);
    expect(await getHeadArmResult("s", "r")).toEqual({
      ok: true,
      value: ARM_RESULT,
    });
    const { snippet: _s, ...rest } = ARM_RESULT;
    stub(200, { ...rest, snippet: 3 });
    const bad = await getHeadArmResult("s", "r");
    expect(bad.ok).toBe(false);
    if (!bad.ok) expect(bad.message).toContain("snippet");
    const { floor_session: _f, ...noFloor } = ARM_ERROR;
    stub(200, noFloor);
    const missing = await computeArm("s");
    expect(missing.ok).toBe(false);
    if (!missing.ok) expect(missing.message).toContain("floor_session");
  });
  it("applyHeadArm accepts the apply answer", async () => {
    const f = stub(200, applyFx);
    expect(await applyHeadArm("s", "r")).toEqual({ ok: true, value: applyFx });
    expect(f).toHaveBeenCalledWith(`${B}/sessions/s/results/r/apply`, {
      method: "POST",
    });
  });
  it("rejects a body missing a key with an error naming it", async () => {
    const { phase: _p, ...noPhase } = stateFx.sequence;
    stub(200, { sequence: noPhase, applied: null });
    const a = await getHeadArmState();
    expect(a.ok).toBe(false);
    if (!a.ok) expect(a.message).toContain("phase");
    const { rms_px: _r, ...noRms } = resultFx;
    stub(200, noRms);
    const b = await computeHeadArm("s");
    expect(b.ok).toBe(false);
    if (!b.ok) expect(b.message).toContain("rms_px");
    stub(200, [{ id: "x" }]);
    const c = await listHeadArmSessions();
    expect(c.ok).toBe(false);
    if (!c.ok) expect(c.message).toContain("created");
  });
  it("error bodies surface message", async () => {
    stub(400, { message: "the base move needs confirm=true" });
    expect(await moveToBase(false)).toEqual({
      ok: false,
      status: 400,
      message: "the base move needs confirm=true",
    });
    stub(409, { reason: "busy" });
    expect(await park(true)).toEqual({
      ok: false,
      status: 409,
      message: "busy",
    });
  });
});
