/** Typed fetch calls for the Head from arm steps (node routes /api/head-arm/* behind /api/calibration/rgbd/head-arm/*). */
import {
  GET,
  POST,
  call,
  isArrayOf,
  isNullOr,
  isNum,
  isObj,
  isStr,
  seg,
  type AppliedHeadChain,
  type FloorApiResult,
  type FloorApplied,
} from "./headFloorApi";

const BASE = "/api/calibration/rgbd/head-arm";

/** Progress of the guided arm sequence. `skipped` maps each skip reason to its count; `arm_pose` is the 0-based pose index; tier and arm_pose are null while idle. */
export interface HeadArmSequence {
  running: boolean;
  phase: string;
  session_id: string | null;
  tier: number | null;
  arm_pose: number | null;
  total_poses: number;
  captured: number;
  skipped: Record<string, number>;
  status: string;
  error: string | null;
  /** Joints the tier-2 confirm names (elbow_flex, shoulder_lift, shoulder_pan). */
  big_joints: string[];
  /** Epoch seconds since the sequence waits for a user confirm; null while it does not wait. */
  waiting_since: number | null;
}

/** Body of GET /head-arm/state; `applied` is null while no head chain is applied. */
export interface HeadArmState {
  sequence: HeadArmSequence;
  applied: AppliedHeadChain | null;
}

/** One fitted parameter with its start value and the floor-fit cross-check (null when the floor fit has none). */
export interface HeadArmParam {
  value: number;
  sigma: number;
  start: number;
  floor: number | null;
  floor_delta: number | null;
}

export interface HeadArmPerView {
  view_id: string;
  arm_pose: number;
  rms_px: number;
  held_out: boolean;
}

/** result.json of one head-from-arm fit. */
export interface HeadArmResult {
  id: string;
  session: string;
  kind: string;
  passed: boolean;
  failed_limits: string[];
  error: string | null;
  rms_px: number;
  holdout_rms_px: number;
  holdout_board_mm: number;
  views: number;
  arm_poses: number;
  pan_span_deg: number;
  tilt_span_deg: number;
  max_correlation: { pair: string[]; value: number } | null;
  params: Record<string, HeadArmParam>;
  board_mount: { xyz: number[]; rpy: number[] };
  per_view: HeadArmPerView[];
  floor: { session: string; result: string } | null;
}

/** One head_arm session of GET /head-arm/sessions. */
export interface HeadArmSession {
  id: string;
  created: string;
  views: number;
  arm_poses: number;
  results: string[];
}

type Check = (v: unknown) => boolean;
const isBool: Check = (v) => typeof v === "boolean";
const isNumArray: Check = isArrayOf(isNum);
const isFloor: Check = isNullOr(
  (f) => isObj(f) && isStr(f.session) && isStr(f.result),
);

/** Name of the first key of `v` that is missing or has the wrong type, or null when all fit. */
function badKey(v: unknown, shape: Record<string, Check>): string | null {
  if (!isObj(v)) return "(body)";
  return Object.keys(shape).find((k) => !(k in v) || !shape[k](v[k])) ?? null;
}

const SEQUENCE: Record<string, Check> = {
  running: isBool,
  phase: isStr,
  session_id: isNullOr(isStr),
  tier: isNullOr(isNum),
  arm_pose: isNullOr(isNum),
  total_poses: isNum,
  captured: isNum,
  skipped: isObj,
  status: isStr,
  error: isNullOr(isStr),
  big_joints: isArrayOf(isStr),
  waiting_since: isNullOr(isNum),
};
const PARAM: Record<string, Check> = {
  value: isNum,
  sigma: isNum,
  start: isNum,
  floor: isNullOr(isNum),
  floor_delta: isNullOr(isNum),
};
const RESULT: Record<string, Check> = {
  id: isStr,
  session: isStr,
  kind: isStr,
  passed: isBool,
  failed_limits: isArrayOf(isStr),
  error: isNullOr(isStr),
  rms_px: isNum,
  holdout_rms_px: isNum,
  holdout_board_mm: isNum,
  views: isNum,
  arm_poses: isNum,
  pan_span_deg: isNum,
  tilt_span_deg: isNum,
  max_correlation: isNullOr(
    (m) => isObj(m) && isArrayOf(isStr)(m.pair) && isNum(m.value),
  ),
  params: (p) =>
    isObj(p) && Object.values(p).every((x) => badKey(x, PARAM) === null),
  board_mount: (b) => isObj(b) && isNumArray(b.xyz) && isNumArray(b.rpy),
  per_view: isArrayOf(
    (w) =>
      badKey(w, {
        view_id: isStr,
        arm_pose: isNum,
        rms_px: isNum,
        held_out: isBool,
      }) === null,
  ),
  floor: isFloor,
};
const SESSION: Record<string, Check> = {
  id: isStr,
  created: isStr,
  views: isNum,
  arm_poses: isNum,
  results: isArrayOf(isStr),
};

/** First problem of a state body as a key name (`sequence.phase`), or null. */
function badState(v: unknown): string | null {
  if (!isObj(v)) return "(body)";
  if (!("sequence" in v)) return "sequence";
  const seq = badKey(v.sequence, SEQUENCE);
  if (seq) return `sequence.${seq}`;
  if (!("applied" in v)) return "applied";
  return null;
}

function badSessions(v: unknown): string | null {
  if (!Array.isArray(v)) return "(body)";
  for (const s of v) {
    const k = badKey(s, SESSION);
    if (k) return k;
  }
  return null;
}

async function request<T>(
  url: string,
  init: RequestInit,
  bad: (v: unknown) => string | null,
): Promise<FloorApiResult<T>> {
  const r = await call<unknown>(url, init, () => true);
  if (!r.ok) return r;
  const key = bad(r.value);
  return key === null
    ? { ok: true, value: r.value as T }
    : {
        ok: false,
        status: 200,
        message: `Unexpected head-from-arm response from the robot: missing or invalid ${key}`,
      };
}

const post = (body?: Record<string, boolean>): RequestInit =>
  body
    ? {
        ...POST,
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
      }
    : POST;
const sessionPath = (sid: string) => `${BASE}/sessions/${seg(sid)}`;
const badSequence = (v: unknown) => badKey(v, SEQUENCE);
const badResult = (v: unknown) => badKey(v, RESULT);
const badApplied = (v: unknown) =>
  isObj(v) &&
  isObj(v.head_chain) &&
  isStr(v.head_chain.result) &&
  isArrayOf(isObj)(v.units)
    ? null
    : "head_chain";

export const getHeadArmState = () =>
  request<HeadArmState>(`${BASE}/state`, GET, badState);
/** Moves the arm to the base pose; `confirm` is the user's answer to the confirm naming the joints that move. */
export const moveToBase = (confirm: boolean) =>
  request<HeadArmSequence>(
    `${BASE}/sequence/base`,
    post({ confirm }),
    badSequence,
  );
export const boardAttached = () =>
  request<HeadArmSequence>(
    `${BASE}/sequence/board-attached`,
    POST,
    badSequence,
  );
/** Answers the tier-2 confirm (elbow_flex, shoulder_lift, shoulder_pan); false skips tier 2. */
export const allowBigJoints = (confirm: boolean) =>
  request<HeadArmSequence>(
    `${BASE}/sequence/big-joints`,
    post({ confirm }),
    badSequence,
  );
export const stopSequence = () =>
  request<HeadArmSequence>(`${BASE}/sequence/stop`, POST, badSequence);
/** Parks the arm (the board may stay on when its park path is clear); `confirm` is the user's confirm. */
export const park = (confirm: boolean) =>
  request<HeadArmSequence>(
    `${BASE}/park`,
    post({ confirm }),
    badSequence,
  );
export const listHeadArmSessions = () =>
  request<HeadArmSession[]>(`${BASE}/sessions`, GET, badSessions);
export const computeHeadArm = (sid: string) =>
  request<HeadArmResult>(`${sessionPath(sid)}/compute`, POST, badResult);
export const getHeadArmResult = (sid: string, rid: string) =>
  request<HeadArmResult>(
    `${sessionPath(sid)}/results/${seg(rid)}`,
    GET,
    badResult,
  );
export const applyHeadArm = (sid: string, rid: string) =>
  request<FloorApplied>(
    `${sessionPath(sid)}/results/${seg(rid)}/apply`,
    POST,
    badApplied,
  );
