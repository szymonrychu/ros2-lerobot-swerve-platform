// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from "vitest";
import {
  allowBigJoints,
  applyHeadArm,
  boardAttached,
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
      body: JSON.stringify({ board_removed: true }),
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
