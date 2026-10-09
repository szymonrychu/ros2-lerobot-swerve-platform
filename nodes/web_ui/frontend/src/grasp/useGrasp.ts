/**
 * Grasp panel state of the map tab: the form, the last plan (and whether it still matches the inputs), the execute
 * confirmation, the two-click release, stop, and the execute/release state streamed on /web_ui/grasp_result.
 *
 * Execute is only ever sent from confirmExecute(), which re-checks that the plan is feasible and was made for exactly
 * the current inputs. Stop is never gated.
 */
import { useCallback, useEffect, useMemo, useState } from 'react'
import type { Vec2 } from '../map/mapMath'
import { confirmClick, RESET_CONFIRM_MS } from '../map/mapActions'
import type { ActionResult } from '../map/mapActions'
import { postGrasp, postGraspStop } from './graspApi'
import {
  ArmMount,
  buildGraspRequest,
  canExecute,
  convertFormFrame,
  DEFAULT_FORM,
  formFromPick,
  GraspAnswer,
  GraspForm,
  GraspFrame,
  GraspObject,
  GraspStream,
  objectBox,
  ObjectBox,
  parseGraspStream,
  planKey,
  PlanState,
  previewObject,
  SceneWaypoint,
  waypointScenePoints,
} from './grasp'

/** Synthetic WS topic carrying the state of the running execute/release (see the backend bridge). */
export const GRASP_STREAM_TOPIC = '/web_ui/grasp_result'

/** What the 3D scene draws for the grasp: the object box and the planned tool path (robot group coordinates). */
export interface GraspPreviewData {
  box: ObjectBox | null
  waypoints: SceneWaypoint[]
}

export interface GraspPanelState {
  form: GraspForm
  setForm: (patch: Partial<GraspForm>) => void
  setFrame: (frame: GraspFrame) => void
  setParam: (key: string, value: string) => void
  plan: PlanState | null
  /** The plan was made for exactly the current inputs. */
  planFresh: boolean
  planning: boolean
  errors: string[]
  /** Validation errors are shown after the first attempt. */
  showErrors: boolean
  canExecute: boolean
  confirming: boolean
  executing: boolean
  stream: GraspStream | null
  lastAnswer: GraspAnswer | null
  releaseArmed: boolean
  pickMode: boolean
  setPickMode: (on: boolean) => void
  applyPick: (point: Vec2) => void
  preview: GraspPreviewData | null
  requestPlan: () => void
  requestExecute: () => void
  cancelConfirm: () => void
  confirmExecute: () => void
  release: () => void
  stop: () => void
}

export interface GraspArgs {
  tabId: string
  topicData: Record<string, unknown>
  mount: ArmMount
  notify: (result: ActionResult) => void
}

export function useGrasp({ tabId, topicData, mount, notify }: GraspArgs): GraspPanelState {
  const [form, setFormState] = useState<GraspForm>(DEFAULT_FORM)
  const [plan, setPlan] = useState<PlanState | null>(null)
  const [planning, setPlanning] = useState(false)
  const [starting, setStarting] = useState(false)
  const [confirming, setConfirming] = useState(false)
  const [showErrors, setShowErrors] = useState(false)
  const [lastAnswer, setLastAnswer] = useState<GraspAnswer | null>(null)
  const [releaseArmedAt, setReleaseArmedAt] = useState<number | null>(null)
  const [pickMode, setPickMode] = useState(false)

  const stream = useMemo(() => parseGraspStream(topicData[GRASP_STREAM_TOPIC]), [topicData[GRASP_STREAM_TOPIC]])
  const executing = starting || stream?.state === 'running'

  const planRequest = useMemo(() => buildGraspRequest('plan', form), [form])
  const errors = planRequest.ok ? [] : planRequest.errors
  const planFresh = plan !== null && planRequest.ok && planKey(planRequest.request) === plan.key
  const executable = useMemo(() => canExecute(plan, form), [plan, form])

  // Changing any input after a plan drops the confirmation (the plan no longer matches).
  useEffect(() => {
    if (!executable) setConfirming(false)
  }, [executable])

  useEffect(() => {
    if (releaseArmedAt === null) return
    const timer = setTimeout(() => setReleaseArmedAt(null), RESET_CONFIRM_MS)
    return () => clearTimeout(timer)
  }, [releaseArmedAt])

  const setForm = useCallback((patch: Partial<GraspForm>) => setFormState((f) => ({ ...f, ...patch })), [])
  const setFrame = useCallback(
    (frame: GraspFrame) => setFormState((f) => convertFormFrame(f, frame, mount)),
    [mount],
  )
  const setParam = useCallback(
    (key: string, value: string) => setFormState((f) => ({ ...f, params: { ...f.params, [key]: value } })),
    [],
  )
  const applyPick = useCallback(
    (point: Vec2) => {
      setFormState((f) => formFromPick(f, point, mount))
      setPickMode(false)
    },
    [mount],
  )

  const requestPlan = useCallback(() => {
    setShowErrors(true)
    const built = buildGraspRequest('plan', form)
    if (!built.ok) return
    const key = planKey(built.request)
    setPlanning(true)
    setConfirming(false)
    void postGrasp(tabId, built.request).then((answer) => {
      setPlanning(false)
      setLastAnswer(answer)
      if (answer.ok && answer.plan) setPlan({ key, outcome: answer })
      else {
        setPlan(null)
        notify({ state: 'error', message: `Grasp plan: ${answer.error ?? 'no plan returned'}` })
      }
    })
  }, [form, tabId, notify])

  const requestExecute = useCallback(() => {
    setShowErrors(true)
    if (canExecute(plan, form)) setConfirming(true)
  }, [plan, form])

  const cancelConfirm = useCallback(() => setConfirming(false), [])

  const confirmExecute = useCallback(() => {
    const built = buildGraspRequest('execute', form)
    if (!built.ok || !canExecute(plan, form)) {
      setConfirming(false)
      return
    }
    setConfirming(false)
    setStarting(true)
    void postGrasp(tabId, built.request).then((answer) => {
      setStarting(false)
      setLastAnswer(answer)
      if (answer.ok) notify({ state: 'ok', message: 'Grasp started; Stop is always available' })
      else notify({ state: 'error', message: `Grasp execute: ${answer.error ?? 'rejected'}` })
    })
  }, [form, plan, tabId, notify])

  const release = useCallback(() => {
    const next = confirmClick(releaseArmedAt, Date.now())
    setReleaseArmedAt(next.armedAt)
    if (!next.fire) return
    setShowErrors(true)
    const built = buildGraspRequest('release', form)
    if (!built.ok) return
    setStarting(true)
    void postGrasp(tabId, built.request).then((answer) => {
      setStarting(false)
      setLastAnswer(answer)
      if (answer.ok) notify({ state: 'ok', message: 'Release started' })
      else notify({ state: 'error', message: `Grasp release: ${answer.error ?? 'rejected'}` })
    })
  }, [releaseArmedAt, form, tabId, notify])

  const stop = useCallback(() => {
    setConfirming(false)
    void postGraspStop(tabId).then((answer) => {
      setLastAnswer(answer)
      if (answer.ok) notify({ state: 'ok', message: 'Grasp stopped' })
      else notify({ state: 'error', message: `Grasp stop: ${answer.error ?? 'failed'}` })
    })
  }, [tabId, notify])

  const object: GraspObject | null = useMemo(() => previewObject(form), [form])
  const preview = useMemo<GraspPreviewData | null>(() => {
    const box = object ? objectBox(object, mount) : null
    const waypoints = planFresh && plan?.outcome.plan ? waypointScenePoints(plan.outcome.plan.waypoints, mount) : []
    return box || waypoints.length > 0 ? { box, waypoints } : null
  }, [object, mount, planFresh, plan])

  return {
    form,
    setForm,
    setFrame,
    setParam,
    plan,
    planFresh,
    planning,
    errors,
    showErrors,
    canExecute: executable && !executing && !planning,
    confirming,
    executing,
    stream,
    lastAnswer,
    releaseArmed: releaseArmedAt !== null,
    pickMode,
    setPickMode,
    applyPick,
    preview,
    requestPlan,
    requestExecute,
    cancelConfirm,
    confirmExecute,
    release,
    stop,
  }
}
