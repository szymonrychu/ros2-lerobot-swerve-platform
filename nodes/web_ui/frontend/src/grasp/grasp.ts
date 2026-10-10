/**
 * Pure helpers of the Map tab Grasp panel: form -> mcp_server grasp request (validation), plan gating, answer and
 * stream parsing, and the frame / scene transforms of the 3D preview. No React, no three.js objects.
 *
 * Contract: nodes/mcp_server/README.md "Web-UI contract"; the backend relays it as POST /api/grasp[/stop].
 */
import type { Pose2D, Vec2 } from '../map/mapMath'
import { rosToThree } from '../map3d/coords'
import type { ThreeTuple } from '../map3d/coords'

/** Arm mount in base_link [x, y, z] metres (tab arm_offset); the mount has no yaw. */
export type ArmMount = readonly [number, number, number]

/** Mount used when a tab has no arm_offset: the arm mount measured on the robot 2026-10-10 (mcp_server arm.base_in_base_link). */
export const DEFAULT_ARM_OFFSET: [number, number, number] = [0.0592, -0.05, 0.100]

export const MAX_OBJECT_SIZE_M = 0.5
export const MAX_TILT_DEG = 45
export const MAX_SURFACE_Z_M = 1
const DEG = Math.PI / 180

export type GraspFrame = 'arm' | 'base_link'
export type GraspStrategy = 'auto' | 'scoop' | 'angled' | 'top_down'
export type GraspAction = 'plan' | 'execute' | 'release'
export type GraspOutcomeName = 'planned' | 'infeasible' | 'grasped' | 'missed' | 'aborted' | 'released'

export const STRATEGIES: { value: GraspStrategy; label: string }[] = [
  { value: 'auto', label: 'Auto' },
  { value: 'scoop', label: 'Scoop' },
  { value: 'angled', label: 'Angled' },
  { value: 'top_down', label: 'Top down' },
]

/** An advanced grasp parameter (mcp_server `grasp` config field) with the bounds the server enforces. */
export interface ParamDef {
  key: string
  label: string
  unit: string
  min: number
  max: number
  /** Server default shown as the placeholder (blank = use it). */
  default: number
}

export const ADVANCED_PARAMS: ParamDef[] = [
  { key: 'approach_distance_m', label: 'Approach distance', unit: 'm', min: 0, max: 0.2, default: 0.04 },
  { key: 'pre_grasp_clearance_m', label: 'Pre-grasp clearance', unit: 'm', min: 0, max: 0.3, default: 0.05 },
  { key: 'slide_speed_scale', label: 'Slide speed scale', unit: '', min: 0.01, max: 0.5, default: 0.15 },
  { key: 'lift_height_m', label: 'Lift height', unit: 'm', min: 0, max: 0.3, default: 0.05 },
  { key: 'retreat_distance_m', label: 'Retreat distance', unit: 'm', min: 0, max: 0.3, default: 0.05 },
  { key: 'jaw_thickness_m', label: 'Jaw thickness', unit: 'm', min: 0.001, max: 0.05, default: 0.008 },
  { key: 'jaw_open_margin_m', label: 'Jaw open margin', unit: 'm', min: 0, max: 0.05, default: 0.015 },
  { key: 'below_object_offset_m', label: 'Scoop depth below object', unit: 'm', min: 0, max: 0.05, default: 0.005 },
  { key: 'skim_clearance_m', label: 'Skim clearance', unit: 'm', min: 0, max: 0.05, default: 0.003 },
  { key: 'max_object_width_m', label: 'Max object width', unit: 'm', min: 0.01, max: 0.12, default: 0.08 },
  { key: 'close_effort_threshold', label: 'Close effort threshold', unit: '', min: 1, max: 1000, default: 300 },
  { key: 'hold_effort_min', label: 'Hold effort minimum', unit: '', min: 0, max: 1000, default: 100 },
  { key: 'min_hold_gap_rad', label: 'Min hold gap', unit: 'rad', min: 0, max: 1.5, default: 0.08 },
  { key: 'interpolation_step_m', label: 'Interpolation step', unit: 'm', min: 0.001, max: 0.05, default: 0.005 },
  { key: 'max_joint_jump_rad', label: 'Max joint jump', unit: 'rad', min: 0.01, max: 1, default: 0.25 },
  { key: 'scoop_pitch_deg', label: 'Scoop pitch', unit: 'deg', min: -10, max: 60, default: 0 },
  { key: 'scoop_max_pitch_deg', label: 'Scoop max pitch', unit: 'deg', min: 0, max: 60, default: 25 },
  { key: 'scoop_pitch_step_deg', label: 'Scoop pitch step', unit: 'deg', min: 0.5, max: 30, default: 5 },
  { key: 'angled_pitch_deg', label: 'Angled pitch (default)', unit: 'deg', min: 0, max: 90, default: 45 },
  { key: 'stall_shoulder_lift_rad', label: 'Stall shoulder lift', unit: 'rad', min: 0, max: 2, default: 1.85 },
  { key: 'stretched_elbow_max_rad', label: 'Stretched elbow max', unit: 'rad', min: -2, max: 2, default: 0 },
  { key: 'release_open_fraction', label: 'Release opening', unit: '0..1', min: 0.05, max: 1, default: 0.6 },
  { key: 'release_lift_m', label: 'Release lift', unit: 'm', min: 0, max: 0.3, default: 0.05 },
]

/** Panel inputs, all as the text the user typed (blank = unset). Lengths in m, angles in deg. */
export interface GraspForm {
  frame: GraspFrame
  x: string
  y: string
  supportZ: string
  width: string
  depth: string
  height: string
  /** Clear height under the object bottom (Scoop only). */
  gapBelow: string
  yaw: string
  strategy: GraspStrategy
  pitchDeg: string
  params: Record<string, string>
  surfaceZ: string
  tiltRoll: string
  tiltPitch: string
}

export const DEFAULT_FORM: GraspForm = {
  frame: 'base_link',
  x: '',
  y: '',
  supportZ: '0',
  width: '0.04',
  depth: '0.04',
  height: '0.04',
  gapBelow: '0',
  yaw: '',
  strategy: 'auto',
  pitchDeg: '',
  params: {},
  surfaceZ: '',
  tiltRoll: '',
  tiltPitch: '',
}

export interface GraspObject {
  frame: GraspFrame
  x: number
  y: number
  support_z: number
  width_m: number
  depth_m: number
  height_m: number
  yaw?: number
  gap_below_m?: number
}

export interface GraspRequest {
  action: GraspAction
  object?: GraspObject
  strategy?: GraspStrategy
  params?: Record<string, number>
  approach_pitch_deg?: number
  surface_z_m?: number
  tilt_override_deg?: { roll: number; pitch: number }
}

export type BuildResult<T> = { ok: true; value: T } | { ok: false; errors: string[] }
export type RequestResult = { ok: true; request: GraspRequest } | { ok: false; errors: string[] }

/** Number from a text field; null for blank or not a finite number. */
export function parseNumber(text: string): number | null {
  if (text.trim() === '') return null
  const n = Number(text)
  return Number.isFinite(n) ? n : null
}

function isBlank(text: string): boolean {
  return text.trim() === ''
}

/** Read an optional ranged number: blank gives undefined, junk or out of range pushes an error. */
function optional(text: string, label: string, min: number, max: number, errors: string[]): number | undefined {
  if (isBlank(text)) return undefined
  const n = parseNumber(text)
  if (n === null || n < min || n > max) {
    errors.push(`${label} must be a number from ${min} to ${max}`)
    return undefined
  }
  return n
}

/** Validate the object fields (frame, position, support height, size, yaw). */
export function buildObject(form: GraspForm): BuildResult<GraspObject> {
  const errors: string[] = []
  const x = parseNumber(form.x)
  const y = parseNumber(form.y)
  const supportZ = parseNumber(form.supportZ)
  if (x === null) errors.push('x must be a number')
  if (y === null) errors.push('y must be a number')
  if (supportZ === null) errors.push('support height must be a number')
  const size = (text: string, label: string): number => {
    const n = parseNumber(text)
    if (n === null || n <= 0 || n > MAX_OBJECT_SIZE_M) {
      errors.push(`${label} must be a number above 0 and at most ${MAX_OBJECT_SIZE_M} m`)
      return 0
    }
    return n
  }
  const width = size(form.width, 'width')
  const depth = size(form.depth, 'depth')
  const height = size(form.height, 'height')
  let gap: number | undefined
  if (form.strategy === 'scoop') {
    gap = isBlank(form.gapBelow) ? 0 : (parseNumber(form.gapBelow) ?? undefined)
    if (gap === undefined || gap < 0 || gap > MAX_OBJECT_SIZE_M) {
      errors.push(`gap below must be a number from 0 to ${MAX_OBJECT_SIZE_M} m`)
      gap = undefined
    }
  }
  let yaw: number | undefined
  if (!isBlank(form.yaw)) {
    const deg = parseNumber(form.yaw)
    if (deg === null) errors.push('yaw must be a number (deg)')
    else yaw = deg * DEG
  }
  if (errors.length > 0 || x === null || y === null || supportZ === null) return { ok: false, errors }
  const object: GraspObject = {
    frame: form.frame,
    x,
    y,
    support_z: supportZ,
    width_m: width,
    depth_m: depth,
    height_m: height,
  }
  if (yaw !== undefined) object.yaw = yaw
  if (gap !== undefined) object.gap_below_m = gap
  return { ok: true, value: object }
}

/**
 * Validate the form and build the contract request for an action.
 *
 * plan/execute carry the object and strategy; release carries only params and overrides. Blank optional fields are
 * left out so the server default applies. Every problem is reported, not only the first.
 */
export function buildGraspRequest(action: GraspAction, form: GraspForm): RequestResult {
  const errors: string[] = []
  const request: GraspRequest = { action }
  if (action !== 'release') {
    const object = buildObject(form)
    if (object.ok) request.object = object.value
    else errors.push(...object.errors)
    request.strategy = form.strategy
    if (form.strategy === 'angled') {
      const pitch = optional(form.pitchDeg, 'approach pitch (deg)', 0, 90, errors)
      if (pitch !== undefined) request.approach_pitch_deg = pitch
    }
  }
  const params: Record<string, number> = {}
  for (const def of ADVANCED_PARAMS) {
    const value = optional(form.params[def.key] ?? '', def.key, def.min, def.max, errors)
    if (value !== undefined) params[def.key] = value
  }
  if (Object.keys(params).length > 0) request.params = params
  const surface = optional(form.surfaceZ, 'surface height (m)', -MAX_SURFACE_Z_M, MAX_SURFACE_Z_M, errors)
  if (surface !== undefined) request.surface_z_m = surface
  if (!isBlank(form.tiltRoll) || !isBlank(form.tiltPitch)) {
    const before = errors.length
    const roll = optional(form.tiltRoll, 'tilt roll (deg)', -MAX_TILT_DEG, MAX_TILT_DEG, errors)
    const pitch = optional(form.tiltPitch, 'tilt pitch (deg)', -MAX_TILT_DEG, MAX_TILT_DEG, errors)
    if (errors.length === before) request.tilt_override_deg = { roll: roll ?? 0, pitch: pitch ?? 0 }
  }
  return errors.length > 0 ? { ok: false, errors } : { ok: true, request }
}

/** Identity of a plan: the request with the action fixed to plan, so execute matches the plan it follows. */
export function planKey(request: GraspRequest): string {
  return JSON.stringify({ ...request, action: 'plan' })
}

/** A parsed plan answer with the key of the inputs it was planned for. */
export interface PlanState {
  key: string
  outcome: GraspAnswer
}

/** Execute is allowed only after a feasible plan for exactly the current (valid) inputs. */
export function canExecute(plan: PlanState | null, form: GraspForm): boolean {
  if (plan === null || plan.outcome.plan?.feasible !== true) return false
  const request = buildGraspRequest('plan', form)
  return request.ok && planKey(request.request) === plan.key
}

// --- frames ---------------------------------------------------------------------------------------------------

export interface Vec3 {
  x: number
  y: number
  z: number
}

/** Arm-frame point (arm base, z up) to base_link. */
export function armPointToBaseLink(p: Vec3, mount: ArmMount): Vec3 {
  return { x: p.x + mount[0], y: p.y + mount[1], z: p.z + mount[2] }
}

/** base_link point to the arm frame. */
export function baseLinkPointToArm(p: Vec3, mount: ArmMount): Vec3 {
  return { x: p.x - mount[0], y: p.y - mount[1], z: p.z - mount[2] }
}

/** Map-frame point to base_link for the robot pose (x, y, yaw in the map). */
export function mapToBaseLink(p: Vec2, pose: Pose2D): Vec2 {
  const dx = p.x - pose.x
  const dy = p.y - pose.y
  const c = Math.cos(pose.yaw)
  const s = Math.sin(pose.yaw)
  return { x: c * dx + s * dy, y: -s * dx + c * dy }
}

/**
 * The form for a map click: the object stands on the floor (support z 0 in base_link, whose origin is on the floor)
 * at the clicked ground point converted from the map frame with the robot pose.
 *
 * @param form - current form (size, overrides are kept)
 * @param strategy - strategy chosen in the Grasp dropdown
 * @param click - clicked ground point in the map frame
 * @param pose - robot pose in the map, or null when unknown
 * @returns the form with frame, x, y, support z and strategy set, or null when the pose is unknown
 */
export function formFromClick(form: GraspForm, strategy: GraspStrategy, click: Vec2, pose: Pose2D | null): GraspForm | null {
  if (pose === null) return null
  const p = mapToBaseLink(click, pose)
  return { ...form, frame: 'base_link', x: p.x.toFixed(3), y: p.y.toFixed(3), supportZ: '0', strategy }
}

// --- scene transforms -----------------------------------------------------------------------------------------

/** One planned waypoint of the answer (tool point in the arm frame). */
export interface GraspWaypoint {
  label: string
  x: number
  y: number
  z: number
  pitch: number
  roll: number
  gripper: number | null
  gripper_action: string
  speed_scale: number
  linear: boolean
}

export interface SceneWaypoint {
  label: string
  /** Position in the robot (base_link) group's three.js coordinates. */
  position: ThreeTuple
}

/** Waypoints (arm frame) as positions in the robot group (base_link), through the arm mount. */
export function waypointScenePoints(waypoints: GraspWaypoint[], mount: ArmMount): SceneWaypoint[] {
  return waypoints.map((w) => ({ label: w.label, position: rosToThree(armPointToBaseLink(w, mount)) }))
}

export interface ObjectBox {
  position: ThreeTuple
  /** three.js box size: [width (ROS x), height (ROS z), depth (ROS y)] before the yaw. */
  size: ThreeTuple
  rotationY: number
}

/** Object box in the robot group (base_link) three.js coordinates; no yaw = width across the approach. */
export function objectBox(object: GraspObject, mount: ArmMount): ObjectBox {
  const base =
    object.frame === 'arm'
      ? armPointToBaseLink({ x: object.x, y: object.y, z: object.support_z }, mount)
      : { x: object.x, y: object.y, z: object.support_z }
  const inArm = baseLinkPointToArm(base, mount)
  const yaw = object.yaw ?? Math.atan2(inArm.y, inArm.x) + Math.PI / 2
  return {
    position: rosToThree({ x: base.x, y: base.y, z: base.z + object.height_m / 2 }),
    size: [object.width_m, object.height_m, object.depth_m],
    rotationY: yaw,
  }
}

/** The object for the preview box, or null while the object fields are incomplete. */
export function previewObject(form: GraspForm): GraspObject | null {
  const object = buildObject(form)
  return object.ok ? object.value : null
}

// --- answers --------------------------------------------------------------------------------------------------

export interface PlanAttempt {
  strategy?: string
  feasible?: boolean
  reasons: string[]
}

export interface GraspPlanSummary {
  strategy: string
  feasible: boolean
  reasons: string[]
  approachPitchDeg?: number
  wristRollRad?: number
  openingM?: number
  skim: boolean
  waypoints: GraspWaypoint[]
  slowZone: unknown[]
  attempts: PlanAttempt[]
}

export interface GraspStep {
  label: string
  status: string
  message: string
}

/** A normalised mcp_server answer (or backend error / accepted reply). */
export interface GraspAnswer {
  ok: boolean
  accepted: boolean
  requestId: string | null
  action: string | null
  error?: string
  outcome?: GraspOutcomeName
  reasons: string[]
  plan?: GraspPlanSummary
  steps: GraspStep[]
  gripperPositionRad?: number
  gripperEffort?: number
}

const OUTCOMES: readonly string[] = ['planned', 'infeasible', 'grasped', 'missed', 'aborted', 'released']

type Rec = Record<string, unknown>

function isRec(v: unknown): v is Rec {
  return typeof v === 'object' && v !== null && !Array.isArray(v)
}

function strings(v: unknown): string[] {
  return Array.isArray(v) ? v.filter((s): s is string => typeof s === 'string') : []
}

function num(v: unknown): number | undefined {
  return typeof v === 'number' && Number.isFinite(v) ? v : undefined
}

function parseWaypoint(v: unknown): GraspWaypoint | null {
  if (!isRec(v)) return null
  const x = num(v.x)
  const y = num(v.y)
  const z = num(v.z)
  if (typeof v.label !== 'string' || x === undefined || y === undefined || z === undefined) return null
  return {
    label: v.label,
    x,
    y,
    z,
    pitch: num(v.pitch) ?? 0,
    roll: num(v.roll) ?? 0,
    gripper: num(v.gripper) ?? null,
    gripper_action: typeof v.gripper_action === 'string' ? v.gripper_action : 'keep',
    speed_scale: num(v.speed_scale) ?? 0,
    linear: v.linear === true,
  }
}

function parsePlan(v: unknown): GraspPlanSummary | undefined {
  if (!isRec(v)) return undefined
  return {
    strategy: typeof v.strategy === 'string' ? v.strategy : '',
    feasible: v.feasible === true,
    reasons: strings(v.reasons),
    approachPitchDeg: num(v.approach_pitch_deg),
    wristRollRad: num(v.wrist_roll_rad),
    openingM: num(v.opening_m),
    skim: v.skim === true,
    waypoints: (Array.isArray(v.waypoints) ? v.waypoints : [])
      .map(parseWaypoint)
      .filter((w): w is GraspWaypoint => w !== null),
    slowZone: Array.isArray(v.slow_zone) ? v.slow_zone : [],
    attempts: (Array.isArray(v.attempts) ? v.attempts : []).filter(isRec).map((a) => ({
      strategy: typeof a.strategy === 'string' ? a.strategy : undefined,
      feasible: typeof a.feasible === 'boolean' ? a.feasible : undefined,
      reasons: strings(a.reasons),
    })),
  }
}

/**
 * Normalise a POST /api/grasp answer: the mcp_server answer ({ok, request_id, action, result | error}), the backend
 * accepted reply ({ok, accepted, ...}) or the backend action error ({ok: false, message}). Never throws.
 */
export function parseGraspResponse(body: unknown, status: number): GraspAnswer {
  if (!isRec(body)) {
    return { ok: false, accepted: false, requestId: null, action: null, error: `HTTP ${status}`, reasons: [], steps: [] }
  }
  const ok = body.ok === true
  const answer: GraspAnswer = {
    ok,
    accepted: body.accepted === true,
    requestId: typeof body.request_id === 'string' ? body.request_id : null,
    action: typeof body.action === 'string' ? body.action : null,
    reasons: [],
    steps: [],
  }
  if (!ok) {
    const text = typeof body.error === 'string' ? body.error : typeof body.message === 'string' ? body.message : ''
    answer.error = text || `HTTP ${status}`
  }
  const result = body.result
  if (isRec(result)) {
    if (typeof result.outcome === 'string' && OUTCOMES.includes(result.outcome)) {
      answer.outcome = result.outcome as GraspOutcomeName
    }
    answer.reasons = strings(result.reasons)
    answer.plan = parsePlan(result.plan)
    answer.steps = (Array.isArray(result.steps) ? result.steps : []).filter(isRec).map((s) => ({
      label: typeof s.label === 'string' ? s.label : '',
      status: typeof s.status === 'string' ? s.status : '',
      message: typeof s.message === 'string' ? s.message : '',
    }))
    answer.gripperPositionRad = num(result.gripper_position_rad)
    answer.gripperEffort = num(result.gripper_effort)
  }
  return answer
}

/** State of a running or finished execute/release as streamed on /web_ui/grasp_result. */
export interface GraspStream {
  requestId: string
  action: string | null
  state: 'running' | 'done'
  outcome: GraspAnswer | null
}

/** Parse the /web_ui/grasp_result payload; null for anything that is not a running/done state. */
export function parseGraspStream(data: unknown): GraspStream | null {
  if (!isRec(data) || typeof data.request_id !== 'string') return null
  if (data.state !== 'running' && data.state !== 'done') return null
  return {
    requestId: data.request_id,
    action: typeof data.action === 'string' ? data.action : null,
    state: data.state,
    outcome: data.state === 'done' ? parseGraspResponse(data, 200) : null,
  }
}

export type Severity = 'success' | 'warning' | 'error'

const OUTCOME_TEXT: Record<GraspOutcomeName, { text: string; severity: Severity }> = {
  planned: { text: 'Plan feasible (dry run, nothing moved)', severity: 'success' },
  infeasible: { text: 'Infeasible: nothing moved', severity: 'error' },
  grasped: { text: 'Grasped: holding the object', severity: 'success' },
  missed: { text: 'Missed: closed on nothing, opened and retreated', severity: 'warning' },
  aborted: { text: 'Aborted: stopped and held', severity: 'error' },
  released: { text: 'Released: gripper opened', severity: 'success' },
}

export function describeOutcome(outcome: GraspOutcomeName): { text: string; severity: Severity } {
  return OUTCOME_TEXT[outcome]
}

/** Lines for the execute confirmation: what the plan will do. */
export function summarizePlan(plan: GraspPlanSummary): string[] {
  const lines = [`Strategy: ${plan.strategy}`]
  if (plan.approachPitchDeg !== undefined) lines.push(`Approach pitch: ${Math.round(plan.approachPitchDeg)} deg`)
  if (plan.wristRollRad !== undefined) lines.push(`Wrist roll: ${Math.round(plan.wristRollRad / DEG)} deg`)
  if (plan.openingM !== undefined) lines.push(`Jaw opening: ${Math.round(plan.openingM * 1000)} mm`)
  lines.push(`Waypoints: ${plan.waypoints.map((w) => w.label).join(' > ') || 'none'}`)
  if (plan.skim) lines.push('Scoop skims the surface')
  if (plan.slowZone.length > 0) lines.push(`${plan.slowZone.length} step(s) in the floor slow zone (slowed, not blocked)`)
  return lines
}
