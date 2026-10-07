/**
 * POI state and gestures of the map tab: the latched list from poi_store, the selection, the add-point / add-area
 * modes, dragging of the selected POI and every edit through POST /api/poi.
 *
 * Gestures run on the map container: add modes (top view) use a click that does not move; with no mode active a
 * press on a POI selects it (capture phase, so the camera does not pan) and, in the top view, a press on the
 * already selected POI drags it. A dragged POI is shown at its new place until the store's list catches up.
 */
import { MutableRefObject, RefObject, useCallback, useEffect, useMemo, useRef, useState } from 'react'
import type { Vec2 } from '../map/mapMath'
import type { SceneController } from '../map3d/MapScene'
import { postPoi } from './api'
import {
  addVertex,
  applyOverride,
  canFinishArea,
  CLICK_MAX_DRAG_PX,
  defaultName,
  dragCommand,
  dragPreview,
  DragState,
  HIT_TOLERANCE_PX,
  isClick,
  newAreaFields,
  newPointFields,
  PoiMode,
  startDrag,
  updateDrag,
} from './editor'
import { distance, hitTest } from './geometry'
import { parsePoiList } from './style'
import type { Poi, PoiCommand, PoiResult } from './types'

const OVERRIDE_TIMEOUT_MS = 3000
const FALLBACK_TOLERANCE_M = 0.15

interface Pending {
  poi: Poi
  baseRevision: number
}

export interface PoiEditorArgs {
  tabId: string
  listTopic: string | undefined
  commandAvailable: boolean
  topicData: Record<string, unknown>
  containerRef: RefObject<HTMLDivElement>
  controllerRef: MutableRefObject<SceneController | null>
  topView: boolean
  visible: boolean
  onResult: (result: PoiResult, what: string) => void
}

export interface PoiEditor {
  pois: Poi[]
  revision: number | null
  mode: PoiMode
  setMode: (mode: PoiMode) => void
  draft: Vec2[]
  finishArea: () => void
  cancelDraft: () => void
  selectedId: string | null
  selected: Poi | null
  select: (id: string | null) => void
  focus: (poi: Poi) => void
  send: (command: PoiCommand, what: string) => Promise<PoiResult>
  dragging: boolean
  busy: boolean
}

function isCanvasTarget(e: Event): boolean {
  return e.target instanceof Element && e.target.closest('canvas') !== null
}

export function usePoiEditor(args: PoiEditorArgs): PoiEditor {
  const { tabId, listTopic, commandAvailable, topicData, containerRef, controllerRef, topView, visible, onResult } = args
  const list = useMemo(() => parsePoiList(listTopic ? topicData[listTopic] : undefined), [topicData, listTopic])
  const storePois = useMemo(() => list?.pois ?? [], [list])
  const revision = list?.revision ?? null

  const [mode, setModeState] = useState<PoiMode>('none')
  const [draft, setDraft] = useState<Vec2[]>([])
  const [selectedId, setSelectedId] = useState<string | null>(null)
  const [drag, setDrag] = useState<DragState | null>(null)
  const [pending, setPending] = useState<Pending | null>(null)
  const [busy, setBusy] = useState(false)

  const storeRef = useRef<Poi[]>([])
  storeRef.current = storePois
  const revisionRef = useRef<number | null>(revision)
  revisionRef.current = revision
  const modeRef = useRef(mode)
  modeRef.current = mode
  const draftRef = useRef(draft)
  draftRef.current = draft
  const selectedRef = useRef(selectedId)
  selectedRef.current = selectedId
  const dragRef = useRef<DragState | null>(null)
  const onResultRef = useRef(onResult)
  onResultRef.current = onResult

  const send = useCallback(
    async (command: PoiCommand, what: string): Promise<PoiResult> => {
      setBusy(true)
      const result = await postPoi(tabId, command)
      setBusy(false)
      onResultRef.current(result, what)
      return result
    },
    [tabId],
  )

  const setMode = useCallback((next: PoiMode) => {
    setModeState(next)
    setDraft([])
  }, [])
  const cancelDraft = useCallback(() => {
    setModeState('none')
    setDraft([])
  }, [])
  const finishArea = useCallback(() => {
    const vertices = draftRef.current
    if (!canFinishArea(vertices)) return
    const fields = newAreaFields(vertices, defaultName('area', storeRef.current))
    setModeState('none')
    setDraft([])
    void send({ op: 'add', poi: fields }, 'Area added').then((r) => r.poi && setSelectedId(r.poi.id))
  }, [send])

  const displayed = useMemo(() => {
    const dragged = drag ? storePois.find((p) => p.id === drag.target.id) : undefined
    if (drag && dragged) return applyOverride(storePois, dragPreview(dragged, drag))
    return applyOverride(storePois, pending?.poi ?? null)
  }, [storePois, drag, pending])

  // A moved POI stays where it was dropped until the store publishes a newer list.
  useEffect(() => {
    if (pending && revision !== null && revision > pending.baseRevision) setPending(null)
  }, [pending, revision])
  useEffect(() => {
    if (!pending) return
    const timer = setTimeout(() => setPending(null), OVERRIDE_TIMEOUT_MS)
    return () => clearTimeout(timer)
  }, [pending])

  // The selected POI was deleted (by anyone): drop the selection.
  useEffect(() => {
    if (selectedId && list && !list.pois.some((p) => p.id === selectedId)) setSelectedId(null)
  }, [list, selectedId])

  // Add modes only exist in the top view with the layer shown.
  useEffect(() => {
    if ((!topView || !visible || !commandAvailable) && mode !== 'none') cancelDraft()
  }, [topView, visible, commandAvailable, mode, cancelDraft])

  const focus = useCallback(
    (poi: Poi) => {
      setSelectedId(poi.id)
      controllerRef.current?.centerOn(poi)
    },
    [controllerRef],
  )

  // Keyboard: Enter finishes the area being drawn, Escape cancels the mode or the drag.
  useEffect(() => {
    if (mode === 'none' && !drag) return
    const onKey = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        dragRef.current = null
        setDrag(null)
        cancelDraft()
      } else if (e.key === 'Enter' && modeRef.current === 'add_area') finishArea()
    }
    window.addEventListener('keydown', onKey)
    return () => window.removeEventListener('keydown', onKey)
  }, [mode, drag, cancelDraft, finishArea])

  // Add modes: a click that does not move places a point / an area vertex; a double click finishes the area.
  useEffect(() => {
    const el = containerRef.current
    if (!el || mode === 'none') return
    const pointers = new Set<number>()
    let down: Vec2 | null = null
    const screen = (e: PointerEvent): Vec2 => ({ x: e.clientX, y: e.clientY })
    const onDown = (e: PointerEvent) => {
      if (!isCanvasTarget(e)) return
      pointers.add(e.pointerId)
      down = pointers.size === 1 && e.button === 0 ? screen(e) : null
    }
    const onUp = (e: PointerEvent) => {
      pointers.delete(e.pointerId)
      const start = down
      down = null
      if (!start || !isClick(start, screen(e))) return
      const at = controllerRef.current?.pick(e.clientX, e.clientY)
      if (!at) return
      if (modeRef.current === 'add_point') {
        setModeState('none')
        void send({ op: 'add', poi: newPointFields(at, defaultName('point', storeRef.current)) }, 'Point added').then(
          (r) => r.poi && setSelectedId(r.poi.id),
        )
      } else {
        setDraft((d) => addVertex(d, at))
      }
    }
    const onCancel = (e: PointerEvent) => {
      pointers.delete(e.pointerId)
      down = null
    }
    const onDouble = () => {
      if (modeRef.current === 'add_area') finishArea()
    }
    el.addEventListener('pointerdown', onDown)
    el.addEventListener('dblclick', onDouble)
    window.addEventListener('pointerup', onUp)
    window.addEventListener('pointercancel', onCancel)
    return () => {
      el.removeEventListener('pointerdown', onDown)
      el.removeEventListener('dblclick', onDouble)
      window.removeEventListener('pointerup', onUp)
      window.removeEventListener('pointercancel', onCancel)
    }
  }, [mode, containerRef, controllerRef, send, finishArea])

  // No mode: select by pressing a POI, drag the selected one (top view).
  useEffect(() => {
    const el = containerRef.current
    if (!el || mode !== 'none' || !visible) return
    const pointers = new Set<number>()
    let emptyDown: Vec2 | null = null
    const screen = (e: PointerEvent): Vec2 => ({ x: e.clientX, y: e.clientY })
    const tolerance = (e: PointerEvent, at: Vec2): number => {
      const beside = controllerRef.current?.pick(e.clientX + HIT_TOLERANCE_PX, e.clientY)
      return beside ? distance(at, beside) : FALLBACK_TOLERANCE_M
    }
    const onDown = (e: PointerEvent) => {
      if (!isCanvasTarget(e)) return
      pointers.add(e.pointerId)
      emptyDown = null
      if (pointers.size > 1) {
        dragRef.current = null
        setDrag(null)
        return
      }
      if (e.button !== 0 || !controllerRef.current) return
      const at = controllerRef.current.pick(e.clientX, e.clientY)
      if (!at) return
      const hit = hitTest(storeRef.current, at, tolerance(e, at), selectedRef.current)
      if (!hit) {
        emptyDown = screen(e)
        return
      }
      e.stopPropagation() // the camera must not pan under a POI press
      const wasSelected = hit.id === selectedRef.current
      setSelectedId(hit.id)
      if (wasSelected && topView) {
        dragRef.current = startDrag(hit, at, screen(e))
        setDrag(dragRef.current)
      }
    }
    const onMove = (e: PointerEvent) => {
      const d = dragRef.current
      if (!d || !controllerRef.current) return
      dragRef.current = updateDrag(d, controllerRef.current.pick(e.clientX, e.clientY), screen(e))
      setDrag(dragRef.current)
    }
    const onUp = (e: PointerEvent) => {
      pointers.delete(e.pointerId)
      const d = dragRef.current
      dragRef.current = null
      if (d) {
        setDrag(null)
        const poi = storeRef.current.find((p) => p.id === d.target.id)
        const final = controllerRef.current ? updateDrag(d, controllerRef.current.pick(e.clientX, e.clientY), screen(e)) : d
        if (poi && final.movedPx >= CLICK_MAX_DRAG_PX) {
          setPending({ poi: dragPreview(poi, final), baseRevision: revisionRef.current ?? -1 })
          void send(dragCommand(poi, final), 'POI moved').then((r) => !r.ok && setPending(null))
        }
        return
      }
      const start = emptyDown
      emptyDown = null
      if (start && isClick(start, screen(e))) setSelectedId(null)
    }
    const onCancel = (e: PointerEvent) => {
      pointers.delete(e.pointerId)
      dragRef.current = null
      setDrag(null)
      emptyDown = null
    }
    el.addEventListener('pointerdown', onDown, true)
    window.addEventListener('pointermove', onMove)
    window.addEventListener('pointerup', onUp)
    window.addEventListener('pointercancel', onCancel)
    return () => {
      el.removeEventListener('pointerdown', onDown, true)
      window.removeEventListener('pointermove', onMove)
      window.removeEventListener('pointerup', onUp)
      window.removeEventListener('pointercancel', onCancel)
    }
  }, [mode, visible, topView, containerRef, controllerRef, send])

  const selected = useMemo(() => displayed.find((p) => p.id === selectedId) ?? null, [displayed, selectedId])

  return {
    pois: displayed,
    revision,
    mode,
    setMode,
    draft,
    finishArea,
    cancelDraft,
    selectedId,
    selected,
    select: setSelectedId,
    focus,
    send,
    dragging: drag !== null,
    busy,
  }
}
