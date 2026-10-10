/**
 * Grasp state of the map tab: the dropdown pick mode (a map click sets the object and plans at once), the form, the
 * last plan (and whether it still matches the inputs), the execute confirmation, the two-click release, stop, and the
 * execute/release state streamed on /web_ui/grasp_result.
 *
 * Execute is only ever sent from confirmExecute(), which re-checks that the plan is feasible and was made for exactly
 * the current inputs. Stop is never gated.
 */
import { useCallback, useEffect, useMemo, useState } from 'react'
import type { Pose2D, Vec2 } from '../map/mapMath'
import { browserStorage } from '../tabSelection'
import { confirmClick, RESET_CONFIRM_MS } from '../map/mapActions'
import type { ActionResult } from '../map/mapActions'
import { postGrasp, postGraspStop } from './graspApi'
import {
  ArmMount,
  buildGraspRequest,
  canExecute,
  DEFAULT_FORM,
  formFromClick,
  GraspAnswer,
  GraspForm,
  GraspObject,
  GraspStrategy,
  GraspStream,
  objectBox,
  ObjectBox,
  parseGraspStream,
  parseNumber,
  planKey,
  PlanState,
  previewObject,
  SceneWaypoint,
  waypointScenePoints,
} from './grasp'
import { loadGripProfile, loadObjectSettings, saveGripProfile, saveObjectSettings } from './objectSettings'
import { IDLE_PICK, pickStep, PickState } from './pick'

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
  /** Armed by a dropdown item: the next map click sets the object. */
  pickMode: boolean
  /** Strategy armed in pick mode. */
  pickStrategy: GraspStrategy | null
  startPick: (strategy: GraspStrategy) => void
  cancelPick: () => void
  /** The pick click: a ground point in the map frame and the robot pose in the map (null when unknown). */
  applyClick: (point: Vec2, pose: Pose2D | null) => void
  /** An object was clicked, so the plan can be redone for changed settings. */
  hasTarget: boolean
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
  const [form, setFormState] = useState<GraspForm>(() => ({
    ...DEFAULT_FORM,
    ...loadObjectSettings(browserStorage()),
    gripProfile: loadGripProfile(browserStorage()),
  }))
  const [plan, setPlan] = useState<PlanState | null>(null)
  const [planning, setPlanning] = useState(false)
  const [starting, setStarting] = useState(false)
  const [confirming, setConfirming] = useState(false)
  const [showErrors, setShowErrors] = useState(false)
  const [lastAnswer, setLastAnswer] = useState<GraspAnswer | null>(null)
  const [releaseArmedAt, setReleaseArmedAt] = useState<number | null>(null)
  const [pick, setPick] = useState<PickState>(IDLE_PICK)

  const stream = useMemo(() => parseGraspStream(topicData[GRASP_STREAM_TOPIC]), [topicData[GRASP_STREAM_TOPIC]])
  const executing = starting || stream?.state === 'running'
  const { width, depth, height, gapBelow } = form
  useEffect(() => saveObjectSettings(browserStorage(), { width, depth, height, gapBelow }), [width, depth, height, gapBelow])
  useEffect(() => saveGripProfile(browserStorage(), form.gripProfile), [form.gripProfile])

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
  const setParam = useCallback(
    (key: string, value: string) => setFormState((f) => ({ ...f, params: { ...f.params, [key]: value } })),
    [],
  )

  const runPlan = useCallback(
    (planForm: GraspForm) => {
      setShowErrors(true)
      const built = buildGraspRequest('plan', planForm)
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
    },
    [tabId, notify],
  )
  const requestPlan = useCallback(() => runPlan(form), [runPlan, form])

  const startPick = useCallback(
    (strategy: GraspStrategy) => {
      if (executing || planning) return
      setConfirming(false)
      setPick((p) => pickStep(p, { type: 'select', strategy }).state)
    },
    [executing, planning],
  )
  const cancelPick = useCallback(() => setPick((p) => pickStep(p, { type: 'cancel' }).state), [])
  const applyClick = useCallback(
    (point: Vec2, pose: Pose2D | null) => {
      const step = pickStep(pick, { type: 'click', point })
      setPick(step.state)
      if (!step.plan) return
      const next = formFromClick(form, step.plan.strategy, step.plan.point, pose)
      if (!next) {
        notify({ state: 'error', message: 'Grasp: robot pose unknown, cannot place the object' })
        return
      }
      setFormState(next)
      runPlan(next)
    },
    [pick, form, runPlan, notify],
  )

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
    pickMode: pick.kind === 'picking',
    pickStrategy: pick.kind === 'picking' ? pick.strategy : null,
    startPick,
    cancelPick,
    applyClick,
    hasTarget: parseNumber(form.x) !== null && parseNumber(form.y) !== null,
    preview,
    requestPlan,
    requestExecute,
    cancelConfirm,
    confirmExecute,
    release,
    stop,
  }
}
