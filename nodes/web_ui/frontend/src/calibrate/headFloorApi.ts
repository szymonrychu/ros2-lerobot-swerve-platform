/** Typed fetch calls for the Head floor fit step (node routes /api/floor/* behind /api/calibration/rgbd/floor/*). */

const BASE = "/api/calibration/rgbd/floor";

/** Sweep progress of the node: pose `pose` of `total`, counts of kept and skipped poses. */
export interface SweepState {
  running: boolean;
  session_id: string | null;
  pose: number;
  total: number;
  observations: number;
  skipped: number;
  status: string;
  error: string | null;
}

/** The currently applied head chain (head_chain.json record) with its joint overrides. */
export interface AppliedHeadChain {
  session: string;
  result: string;
  note: string;
  /** Which fit wrote the record; absent in records written before the head-from-arm fit. */
  source?: "floor" | "head_arm";
  /** The floor fit used as prior or cross-check (floor records: the fit itself). */
  floor?: { session: string; result: string } | null;
  joints: Record<string, { xyz?: number[]; rpy?: number[] }>;
}

/** Body of GET /floor/state; `applied` is null while no head chain is applied. */
export interface FloorState {
  sweep: SweepState;
  applied: AppliedHeadChain | null;
}

/** One fitted parameter: fitted value, its sigma and the URDF (current) value. */
export interface FloorParam {
  value: number;
  sigma: number;
  urdf: number;
}

export interface FloorPose {
  pose: number;
  pan_deg: number;
  tilt_deg: number;
  observations: number;
  height_residual_mm: number;
  normal_residual_deg: number;
}

export interface SkippedPose {
  pose: number;
  pan_deg: number;
  tilt_deg: number;
  reason: string;
}

/** floor_result.json: one floor fit. */
export interface FloorResult {
  id: string;
  session: string;
  created: string;
  passed: boolean;
  failed_limits: string[];
  params: {
    mount_z_m: FloorParam;
    mount_roll_deg: FloorParam;
    mount_pitch_deg: FloorParam;
    tilt_zero_deg: FloorParam;
    /** Absent in results fitted before the tilt roll was added. */
    tilt_roll_deg?: FloorParam;
  };
  /** RMS over the poses of the pose-mean height residual: the value the max_height_rms_mm limit checks. */
  height_rms_mm: number;
  /** RMS over the poses of the pose-mean normal angle: the value the max_normal_rms_deg limit checks. */
  normal_rms_deg: number;
  /** RMS of every observation's height residual (reported, not limited); absent in older results. */
  observation_height_rms_mm?: number;
  /** RMS of every observation's normal angle (reported, not limited); absent in older results. */
  observation_normal_rms_deg?: number;
  poses: FloorPose[];
  skipped: SkippedPose[];
}

/** Answer of apply: the applied record and the restart state of the consumer units. */
export interface FloorApplied {
  head_chain: {
    session: string;
    result: string;
    applied_at: string;
    source?: "floor" | "head_arm";
    floor?: { session: string; result: string } | null;
  };
  units: { name: string; restarted: boolean; active_state?: string }[];
}

export type FloorApiResult<T> =
  { ok: true; value: T } | { ok: false; status: number; message: string };

export const isObj = (v: unknown): v is Record<string, unknown> =>
  typeof v === "object" && v !== null && !Array.isArray(v);
export const isStr = (v: unknown) => typeof v === "string";
export const isNum = (v: unknown) =>
  typeof v === "number" && Number.isFinite(v);
export const isNullOr = (f: (v: unknown) => boolean) => (v: unknown) =>
  v === null || f(v);
export const isOptional = (f: (v: unknown) => boolean) => (v: unknown) =>
  v === undefined || f(v);
export const isArrayOf = (f: (v: unknown) => boolean) => (v: unknown) =>
  Array.isArray(v) && v.every(f);

const isSweep = (v: unknown) =>
  isObj(v) &&
  typeof v.running === "boolean" &&
  isNullOr(isStr)(v.session_id) &&
  isNum(v.pose) &&
  isNum(v.total) &&
  isNum(v.observations) &&
  isNum(v.skipped) &&
  isStr(v.status) &&
  isNullOr(isStr)(v.error);

const isApplied = (v: unknown) =>
  isObj(v) &&
  isStr(v.session) &&
  isStr(v.result) &&
  isStr(v.note) &&
  isObj(v.joints) &&
  isOptional(isStr)(v.source) &&
  isOptional(isNullOr((f) => isObj(f) && isStr(f.session) && isStr(f.result)))(
    v.floor,
  );

const isFloorState = (v: unknown) =>
  isObj(v) && isSweep(v.sweep) && isNullOr(isApplied)(v.applied);

const isParam = (v: unknown) =>
  isObj(v) && isNum(v.value) && isNum(v.sigma) && isNum(v.urdf);

const isPose = (v: unknown) =>
  isObj(v) &&
  isNum(v.pose) &&
  isNum(v.pan_deg) &&
  isNum(v.tilt_deg) &&
  isNum(v.observations) &&
  isNum(v.height_residual_mm) &&
  isNum(v.normal_residual_deg);

const isSkipped = (v: unknown) =>
  isObj(v) &&
  isNum(v.pose) &&
  isNum(v.pan_deg) &&
  isNum(v.tilt_deg) &&
  isStr(v.reason);

const isFloorResult = (v: unknown) =>
  isObj(v) &&
  isStr(v.id) &&
  isStr(v.session) &&
  isStr(v.created) &&
  typeof v.passed === "boolean" &&
  isArrayOf(isStr)(v.failed_limits) &&
  isObj(v.params) &&
  isParam(v.params.mount_z_m) &&
  isParam(v.params.mount_roll_deg) &&
  isParam(v.params.mount_pitch_deg) &&
  isParam(v.params.tilt_zero_deg) &&
  isOptional(isParam)(v.params.tilt_roll_deg) &&
  isNum(v.height_rms_mm) &&
  isNum(v.normal_rms_deg) &&
  isOptional(isNum)(v.observation_height_rms_mm) &&
  isOptional(isNum)(v.observation_normal_rms_deg) &&
  isArrayOf(isPose)(v.poses) &&
  isArrayOf(isSkipped)(v.skipped);

const isFloorApplied = (v: unknown) =>
  isObj(v) &&
  isObj(v.head_chain) &&
  isStr(v.head_chain.session) &&
  isStr(v.head_chain.result) &&
  isStr(v.head_chain.applied_at) &&
  isArrayOf(
    (u) => isObj(u) && isStr(u.name) && typeof u.restarted === "boolean",
  )(v.units);

export async function call<T>(
  url: string,
  init: RequestInit,
  valid: (v: unknown) => boolean,
): Promise<FloorApiResult<T>> {
  let res: Response;
  try {
    res = await fetch(url, init);
  } catch (e) {
    return {
      ok: false,
      status: 0,
      message: e instanceof Error ? e.message : String(e),
    };
  }
  let body: unknown = null;
  try {
    body = await res.json();
  } catch {
    body = null;
  }
  if (res.ok) {
    return valid(body)
      ? { ok: true, value: body as T }
      : {
          ok: false,
          status: res.status,
          message: "Unexpected floor fit response from the robot",
        };
  }
  const b = (body ?? {}) as { message?: unknown; reason?: unknown };
  const message =
    typeof b.message === "string"
      ? b.message
      : typeof b.reason === "string"
        ? b.reason
        : `HTTP ${res.status}`;
  return { ok: false, status: res.status, message };
}

export const GET: RequestInit = { cache: "no-store" };
export const POST: RequestInit = { method: "POST" };
export const seg = encodeURIComponent;
const sessionPath = (sid: string) => `${BASE}/sessions/${seg(sid)}`;

export const getFloorState = () =>
  call<FloorState>(`${BASE}/state`, GET, isFloorState);
export const startSweep = () =>
  call<SweepState>(`${BASE}/sweep/start`, POST, isSweep);
export const stopSweep = () =>
  call<SweepState>(`${BASE}/sweep/stop`, POST, isSweep);
export const computeFloor = (sid: string) =>
  call<FloorResult>(`${sessionPath(sid)}/compute`, POST, isFloorResult);
export const getFloorResult = (sid: string, rid: string) =>
  call<FloorResult>(
    `${sessionPath(sid)}/results/${seg(rid)}`,
    GET,
    isFloorResult,
  );
export const applyFloor = (sid: string, rid: string) =>
  call<FloorApplied>(
    `${sessionPath(sid)}/results/${seg(rid)}/apply`,
    POST,
    isFloorApplied,
  );
