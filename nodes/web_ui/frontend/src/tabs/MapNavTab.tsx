/**
 * Map tab: one 3D view in the ROS map frame merging the SLAM map, local costmap, GPS tiles, Nav2 plans/goal/
 * footprint and the robot (swerve base + draggable arm). Free orbit by default; "Top view" locks a top-down
 * camera in which goals are set by press + drag. The 3D scene is a separate lazily loaded chunk.
 */
import { lazy, Suspense, useCallback, useEffect, useMemo, useRef, useState } from 'react'
import type { ReactNode } from 'react'
import Alert from '@mui/material/Alert'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import ButtonGroup from '@mui/material/ButtonGroup'
import Checkbox from '@mui/material/Checkbox'
import CircularProgress from '@mui/material/CircularProgress'
import Collapse from '@mui/material/Collapse'
import Divider from '@mui/material/Divider'
import FormControlLabel from '@mui/material/FormControlLabel'
import IconButton from '@mui/material/IconButton'
import Paper from '@mui/material/Paper'
import Snackbar from '@mui/material/Snackbar'
import Stack from '@mui/material/Stack'
import Tooltip from '@mui/material/Tooltip'
import Typography from '@mui/material/Typography'
import useMediaQuery from '@mui/material/useMediaQuery'
import { useTheme } from '@mui/material/styles'
import StopCircleIcon from '@mui/icons-material/StopCircle'
import PlaceIcon from '@mui/icons-material/Place'
import MyLocationIcon from '@mui/icons-material/MyLocation'
import FitScreenIcon from '@mui/icons-material/FitScreen'
import SaveIcon from '@mui/icons-material/Save'
import RestartAltIcon from '@mui/icons-material/RestartAlt'
import LayersIcon from '@mui/icons-material/Layers'
import VerticalAlignBottomIcon from '@mui/icons-material/VerticalAlignBottom'
import ThreeDRotationIcon from '@mui/icons-material/ThreeDRotation'
import HomeIcon from '@mui/icons-material/Home'
import AddLocationAltIcon from '@mui/icons-material/AddLocationAlt'
import PolylineIcon from '@mui/icons-material/Polyline'
import CheckIcon from '@mui/icons-material/Check'
import CloseIcon from '@mui/icons-material/Close'
import BookmarkAddIcon from '@mui/icons-material/BookmarkAdd'
import log from '../logging'
import { Pose2D, Vec2, yawToQuaternion } from '../map/mapMath'
import { ActionResult, confirmClick, isCleared, parseActionResult, RESET_CONFIRM_MS } from '../map/mapActions'
import { validAnchor } from '../map3d/geo'
import { draftGoalPose, finishGoalDraft, GoalDraft, startGoalDraft, updateGoalDraft } from '../map3d/goalGesture'
import { Bounds, mapBounds } from '../map3d/groundMath'
import { LAYER_LABELS, LayerKey, LayerState, readLayerState, writeLayerState } from '../map3d/layers'
import type { SceneController } from '../map3d/MapScene'
import type { GridImageMsg, PathMsg } from '../map3d/sceneLayers'
import { browserStorage } from '../tabSelection'
import { CANVAS_BG, MONO_FONT } from '../theme'
import { TabConfig } from '../types'
import { PoiEditorPanel, PoiListPanel } from '../poi/PoiPanels'
import { canFinishArea } from '../poi/editor'
import { usePoiEditor } from '../poi/usePoiEditor'
import type { PoiResult } from '../poi/types'

const MapScene = lazy(() => import('../map3d/MapScene'))

const ROBOT_POSE_TOPIC = '/web_ui/robot_pose'
const GPS_ANCHOR_TOPIC = '/web_ui/gps_anchor'
const ROBOT_FIT_SPAN_M = 4 // view size around the robot when there is no map to fit
const SNACKBAR_MS: Record<ActionResult['state'], number | null> = { idle: null, busy: null, ok: 4000, error: 10000 }

/** Swatch colours (mirror sceneLayers.SCENE_COLORS; duplicated so the toolbar does not pull in the 3D chunk). */
const LAYER_SWATCH: Partial<Record<LayerKey, string>> = {
  localCostmap: '#c158dc',
  globalPlan: '#2ecc40',
  localPlan: '#ff851b',
  goal: '#ff4136',
  footprint: '#2f9bff',
  pois: '#ffb000',
}

const MAP_LEGEND: [string, string][] = [
  ['Free', '#fefefe'],
  ['Occupied', '#000000'],
  ['Unknown', '#cdcdcd'],
  ['Robot front', '#ffdc00'],
  ['Goal being set', '#ffdc00'],
]

const LAYER_GROUPS: { title: string; keys: LayerKey[] }[] = [
  { title: 'Maps', keys: ['slamMap', 'localCostmap', 'gpsMap'] },
  { title: 'Navigation', keys: ['globalPlan', 'localPlan', 'goal', 'footprint'] },
  { title: 'Robot model', keys: ['robotBase', 'robotWheels', 'robotArm'] },
  { title: 'Points of interest', keys: ['pois'] },
]

interface Props {
  tab: TabConfig
  topicData: Record<string, unknown>
  publish: (topic: string, msgType: string, data: unknown) => void
}

interface FramedPose extends Pose2D {
  frame_id: string
}

interface JointStates {
  name?: string[]
  position?: number[]
}

function inFrame<T extends { frame_id?: string }>(data: T | null | undefined, mapFrame: string): data is T {
  return Boolean(data) && (!data!.frame_id || data!.frame_id === mapFrame)
}

function Swatch({ color }: { color: string }) {
  return (
    <Box
      component="span"
      sx={{ width: 14, height: 14, bgcolor: color, border: 1, borderColor: 'grey.600', flexShrink: 0, display: 'inline-block' }}
    />
  )
}

function Centered({ children }: { children: ReactNode }) {
  return (
    <Box sx={{ height: '100%', display: 'flex', alignItems: 'center', justifyContent: 'center' }}>{children}</Box>
  )
}

export default function MapNavTab({ tab, topicData, publish }: Props) {
  const containerRef = useRef<HTMLDivElement>(null)
  const controllerRef = useRef<SceneController | null>(null)
  const pointersRef = useRef<Set<number>>(new Set())
  const draftRef = useRef<GoalDraft | null>(null)
  const fittedRef = useRef(false)

  const [topView, setTopView] = useState(false)
  const [goalMode, setGoalMode] = useState(false)
  const [draft, setDraft] = useState<GoalDraft | null>(null)
  const [action, setAction] = useState<ActionResult>({ state: 'idle', message: '' })
  const [resetArmedAt, setResetArmedAt] = useState<number | null>(null)
  const [setHomeArmedAt, setSetHomeArmedAt] = useState<number | null>(null)
  const [armReady, setArmReady] = useState(false)
  const [layers, setLayers] = useState<LayerState>(() => readLayerState(browserStorage()))
  const muiTheme = useTheme()
  const narrow = useMediaQuery(muiTheme.breakpoints.down('sm'))
  const [panelOpen, setPanelOpen] = useState<boolean | null>(null) // null = follow screen size
  const [poiListOpen, setPoiListOpen] = useState<boolean | null>(null) // null = open on wide screens only
  const showPanel = panelOpen ?? !narrow
  const showPoiList = poiListOpen ?? !narrow

  const mapFrame = tab.map_frame ?? 'map'
  // Each value keeps its identity until its own topic updates, so the memoised scene skips unrelated traffic.
  // null = the backend cleared its cache (map reset / navigation stopped); undefined = nothing received yet.
  const mapMsg = topicData[tab.map_topic ?? ''] as GridImageMsg | null | undefined
  const costmapMsg = topicData[tab.local_costmap_topic ?? ''] as GridImageMsg | null | undefined
  const globalPath = topicData[tab.global_plan_topic ?? ''] as PathMsg | null | undefined
  const localPath = topicData[tab.local_plan_topic ?? ''] as PathMsg | null | undefined
  const goalMsg = topicData[tab.goal_topic ?? ''] as FramedPose | null | undefined
  const poseMsg = topicData[ROBOT_POSE_TOPIC] as FramedPose | null | undefined
  const footprint = topicData[tab.footprint_topic ?? ''] as PathMsg | null | undefined
  const anchorMsg = topicData[GPS_ANCHOR_TOPIC]
  const baseJoints = topicData[tab.base_joint_states_topic ?? ''] as JointStates | undefined
  const armJoints = topicData[tab.arm_joint_states_topic ?? ''] as JointStates | undefined
  const robotPose = inFrame(poseMsg, mapFrame) ? poseMsg : null
  const goal = inFrame(goalMsg, mapFrame) ? goalMsg : null
  const anchor = useMemo(() => validAnchor(anchorMsg), [anchorMsg])
  // The anchor is the GPS placement of the map frame origin itself, so it needs no frame check.
  const gpsAvailable = anchor !== null
  const map = isCleared(mapMsg) ? null : mapMsg
  const mapBox = useMemo<Bounds | null>(() => (map ? mapBounds(map) : null), [map])

  useEffect(() => writeLayerState(browserStorage(), layers), [layers])

  // Map reset: fit the next map that arrives.
  useEffect(() => {
    if (isCleared(mapMsg)) {
      log.info('[map] map cleared by backend')
      fittedRef.current = false
    }
  }, [mapMsg])

  // Fit the first map once the scene is up.
  useEffect(() => {
    if (fittedRef.current || !mapBox) return
    const timer = setInterval(() => {
      if (!controllerRef.current) return
      controllerRef.current.fit(mapBox)
      fittedRef.current = true
      clearInterval(timer)
    }, 100)
    return () => clearInterval(timer)
  }, [mapBox])

  // Armed two-step buttons disarm after the confirm window.
  useEffect(() => {
    if (resetArmedAt === null) return
    const timer = setTimeout(() => setResetArmedAt(null), RESET_CONFIRM_MS)
    return () => clearTimeout(timer)
  }, [resetArmedAt])
  useEffect(() => {
    if (setHomeArmedAt === null) return
    const timer = setTimeout(() => setSetHomeArmedAt(null), RESET_CONFIRM_MS)
    return () => clearTimeout(timer)
  }, [setHomeArmedAt])

  // Goal setting only exists in the locked top view.
  useEffect(() => {
    if (!topView) setGoalMode(false)
  }, [topView])
  useEffect(() => {
    if (!goalMode) {
      draftRef.current = null
      setDraft(null)
    }
  }, [goalMode])

  const onPoiResult = useCallback((result: PoiResult, what: string) => {
    setAction(result.ok ? { state: 'ok', message: what } : { state: 'error', message: `POI: ${result.message}` })
  }, [])
  const poi = usePoiEditor({
    tabId: tab.id,
    listTopic: tab.poi_list_topic,
    commandAvailable: Boolean(tab.poi_command_topic),
    topicData,
    containerRef,
    controllerRef,
    topView,
    visible: layers.pois,
    onResult: onPoiResult,
  })
  const poiAdding = poi.mode !== 'none'
  // On a phone the layers panel would cover the editor: close it when a POI is selected.
  useEffect(() => {
    if (narrow && poi.selectedId) setPanelOpen(false)
  }, [narrow, poi.selectedId])
  // Goal setting and POI adding both own the one-finger gesture: only one at a time.
  useEffect(() => {
    if (poiAdding) setGoalMode(false)
  }, [poiAdding])

  const robotPoseRef = useRef(robotPose)
  robotPoseRef.current = robotPose

  const sendGoal = useCallback(
    (g: Pose2D) => {
      if (!tab.goal_topic) return
      log.info('[map] goal', g.x.toFixed(2), g.y.toFixed(2), 'yaw', g.yaw.toFixed(2), 'in', mapFrame)
      publish(tab.goal_topic, 'geometry_msgs/PoseStamped', {
        header: { frame_id: mapFrame, stamp: { sec: 0, nanosec: 0 } }, // backend stamps with the node clock
        pose: { position: { x: g.x, y: g.y, z: 0 }, orientation: yawToQuaternion(g.yaw) },
      })
    },
    [tab.goal_topic, mapFrame, publish],
  )

  // Goal gesture: listeners on the container (bubble phase). Arm handles stop propagation on the canvas in the
  // capture phase, so a press on them never starts a goal; OrbitControls ignores one-finger/left presses in goal mode.
  useEffect(() => {
    const el = containerRef.current
    if (!el || !goalMode) return
    const screen = (e: PointerEvent): Vec2 => ({ x: e.clientX, y: e.clientY })
    const onDown = (e: PointerEvent) => {
      pointersRef.current.add(e.pointerId)
      if (pointersRef.current.size > 1) {
        // Second finger: pinch/pan the view, abandon the goal.
        draftRef.current = null
        setDraft(null)
        return
      }
      if (e.button !== 0 || !controllerRef.current) return
      const at = controllerRef.current.pick(e.clientX, e.clientY)
      if (!at) return
      draftRef.current = startGoalDraft(at, screen(e))
      setDraft(draftRef.current)
    }
    const onMove = (e: PointerEvent) => {
      const d = draftRef.current
      if (!d || !controllerRef.current) return
      draftRef.current = updateGoalDraft(d, controllerRef.current.pick(e.clientX, e.clientY), screen(e))
      setDraft(draftRef.current)
    }
    const onUp = (e: PointerEvent, commit: boolean) => {
      pointersRef.current.delete(e.pointerId)
      const d = draftRef.current
      draftRef.current = null
      setDraft(null)
      if (!d || !commit || !controllerRef.current) return
      const release = controllerRef.current.pick(e.clientX, e.clientY)
      sendGoal(finishGoalDraft(d, release, screen(e), robotPoseRef.current))
      setGoalMode(false)
    }
    const up = (e: PointerEvent) => onUp(e, true)
    const cancel = (e: PointerEvent) => onUp(e, false)
    el.addEventListener('pointerdown', onDown)
    window.addEventListener('pointermove', onMove)
    window.addEventListener('pointerup', up)
    window.addEventListener('pointercancel', cancel)
    return () => {
      el.removeEventListener('pointerdown', onDown)
      window.removeEventListener('pointermove', onMove)
      window.removeEventListener('pointerup', up)
      window.removeEventListener('pointercancel', cancel)
      pointersRef.current.clear()
    }
  }, [goalMode, sendGoal])

  const draftGoal = useMemo(() => (draft ? draftGoalPose(draft, robotPose) : null), [draft, robotPose])

  const runAction = async (path: string, busyMessage: string) => {
    setAction({ state: 'busy', message: busyMessage })
    try {
      const resp = await fetch(`${path}?tab=${encodeURIComponent(tab.id)}`, { method: 'POST' })
      setAction(parseActionResult(resp.status, await resp.json()))
    } catch (err) {
      setAction({ state: 'error', message: `Request failed: ${String(err)}` })
    }
  }

  const onResetClick = () => {
    const next = confirmClick(resetArmedAt, Date.now())
    setResetArmedAt(next.armedAt)
    if (next.fire) void runAction('/api/map/reset', 'Resetting map...')
  }

  const onSetHomeClick = () => {
    const next = confirmClick(setHomeArmedAt, Date.now())
    setSetHomeArmedAt(next.armedAt)
    if (next.fire) void runAction('/api/arm/set_home', 'Saving arm home pose...')
  }

  const toggleLayer = (k: LayerKey) => setLayers((s) => ({ ...s, [k]: !s[k] }))
  const onArmReady = useCallback((ready: boolean) => setArmReady(ready), [])

  const centerOnRobot = () => robotPose && controllerRef.current?.centerOn(robotPose)
  const fit = () => {
    if (mapBox) controllerRef.current?.fit(mapBox)
    else if (robotPose) {
      const h = ROBOT_FIT_SPAN_M / 2
      controllerRef.current?.fit({ minX: robotPose.x - h, maxX: robotPose.x + h, minY: robotPose.y - h, maxY: robotPose.y + h })
    }
  }

  const busy = action.state === 'busy'
  const resetArmed = resetArmedAt !== null
  const setHomeArmed = setHomeArmedAt !== null
  const snackbarOpen = action.state !== 'idle' && action.message !== ''
  const hasArm = Boolean(tab.arm_urdf)

  return (
    <Box sx={{ width: '100%', height: '100%', display: 'flex', flexDirection: 'column' }}>
      <Paper
        square
        elevation={0}
        sx={{ flexShrink: 0, px: { xs: 1, sm: 1.5 }, py: 1, borderBottom: 1, borderColor: 'divider' }}
      >
        <Stack direction="row" useFlexGap sx={{ flexWrap: 'wrap', alignItems: 'center', gap: 1 }}>
          <Button
            variant="contained"
            color="error"
            size="large"
            startIcon={<StopCircleIcon />}
            onClick={() => void runAction('/api/nav/stop', 'Stopping...')}
            title="Cancel the current Nav2 goal; Nav2 stops the robot"
            sx={{ px: 3, fontWeight: 800, letterSpacing: '0.05em' }}
          >
            STOP
          </Button>
          <Button
            variant={topView ? 'contained' : 'outlined'}
            startIcon={topView ? <VerticalAlignBottomIcon /> : <ThreeDRotationIcon />}
            onClick={() => setTopView((v) => !v)}
            aria-pressed={topView}
            title={topView ? 'Unlock the camera (free orbit)' : 'Lock a top-down camera (needed to set goals)'}
          >
            {topView ? 'Top view' : 'Orbit'}
          </Button>
          <Tooltip title={topView ? '' : 'Switch to Top view to set goals'}>
            <span>
              <Button
                variant={goalMode ? 'contained' : 'outlined'}
                color="secondary"
                startIcon={<PlaceIcon />}
                onClick={() => {
                  poi.setMode('none')
                  setGoalMode((m) => !m)
                }}
                disabled={!tab.goal_topic || !topView}
                aria-pressed={goalMode}
                title="Press on the map to set the goal position; drag to set its heading"
              >
                {goalMode ? 'Tap map to set goal' : 'Set goal'}
              </Button>
            </span>
          </Tooltip>
          {tab.poi_command_topic && (
            <ButtonGroup variant="outlined" aria-label="Points of interest">
              <Tooltip title={topView ? '' : 'Switch to Top view to add POIs'}>
                <span>
                  <Button
                    variant={poi.mode === 'add_point' ? 'contained' : 'outlined'}
                    startIcon={<AddLocationAltIcon />}
                    disabled={!topView}
                    aria-pressed={poi.mode === 'add_point'}
                    onClick={() => {
                      setGoalMode(false)
                      setLayers((s) => ({ ...s, pois: true }))
                      poi.setMode(poi.mode === 'add_point' ? 'none' : 'add_point')
                    }}
                    title="Click the map to place a point of interest"
                  >
                    {poi.mode === 'add_point' ? 'Click map' : 'Add point'}
                  </Button>
                </span>
              </Tooltip>
              <Tooltip title={topView ? '' : 'Switch to Top view to add POIs'}>
                <span>
                  <Button
                    variant={poi.mode === 'add_area' ? 'contained' : 'outlined'}
                    startIcon={<PolylineIcon />}
                    disabled={!topView}
                    aria-pressed={poi.mode === 'add_area'}
                    onClick={() => {
                      setGoalMode(false)
                      setLayers((s) => ({ ...s, pois: true }))
                      poi.setMode(poi.mode === 'add_area' ? 'none' : 'add_area')
                    }}
                    title="Click the vertices of the area; double-click or Enter finishes"
                  >
                    {poi.mode === 'add_area' ? `Area: ${poi.draft.length} vertices` : 'Add area'}
                  </Button>
                </span>
              </Tooltip>
              {poi.mode === 'add_area' && (
                <Button startIcon={<CheckIcon />} disabled={!canFinishArea(poi.draft)} onClick={poi.finishArea}>
                  Finish
                </Button>
              )}
              {poiAdding && (
                <Button startIcon={<CloseIcon />} color="inherit" onClick={poi.cancelDraft}>
                  Cancel
                </Button>
              )}
            </ButtonGroup>
          )}
          <Stack direction="row" role="group" aria-label="Map view" sx={{ border: 1, borderColor: 'divider', borderRadius: 1 }}>
            <Tooltip title="Center on robot">
              <span>
                <IconButton aria-label="Center on robot" disabled={!robotPose} onClick={centerOnRobot}>
                  <MyLocationIcon />
                </IconButton>
              </span>
            </Tooltip>
            <Tooltip title="Fit map">
              <span>
                <IconButton aria-label="Fit map" disabled={!mapBox && !robotPose} onClick={fit}>
                  <FitScreenIcon />
                </IconButton>
              </span>
            </Tooltip>
            <Tooltip title={showPanel ? 'Hide layers and legend' : 'Show layers and legend'}>
              <IconButton
                aria-label={showPanel ? 'Hide layers and legend' : 'Show layers and legend'}
                aria-pressed={showPanel}
                color={showPanel ? 'primary' : 'default'}
                onClick={() => setPanelOpen(!showPanel)}
              >
                <LayersIcon />
              </IconButton>
            </Tooltip>
          </Stack>
          <ButtonGroup variant="outlined" aria-label="Map file">
            <Button startIcon={<SaveIcon />} disabled={busy} onClick={() => void runAction('/api/map/save', 'Saving map...')}>
              Save map
            </Button>
            <Button
              variant={resetArmed ? 'contained' : 'outlined'}
              color={resetArmed ? 'warning' : 'primary'}
              startIcon={<RestartAltIcon />}
              disabled={busy}
              onClick={onResetClick}
              title="Discard the current SLAM map and start a new one (the saved map file is kept). Click twice to confirm."
            >
              {resetArmed ? 'Confirm reset' : 'Reset map'}
            </Button>
          </ButtonGroup>
          {hasArm && (
            <ButtonGroup variant="outlined" aria-label="Arm home">
              <Button
                startIcon={<HomeIcon />}
                disabled={busy}
                onClick={() => void runAction('/api/arm/home', 'Moving arm home...')}
                title="Move the arm to its saved home pose"
              >
                Arm home
              </Button>
              <Button
                variant={setHomeArmed ? 'contained' : 'outlined'}
                color={setHomeArmed ? 'warning' : 'primary'}
                startIcon={<BookmarkAddIcon />}
                disabled={busy}
                onClick={onSetHomeClick}
                title="Save the arm's current pose as its home pose. Click twice to confirm."
              >
                {setHomeArmed ? 'Confirm home' : 'Set home'}
              </Button>
            </ButtonGroup>
          )}
        </Stack>
      </Paper>

      <Box
        ref={containerRef}
        sx={{
          flex: 1,
          minHeight: 0,
          position: 'relative',
          overflow: 'hidden',
          bgcolor: CANVAS_BG,
          cursor: goalMode || poiAdding ? 'crosshair' : poi.dragging ? 'grabbing' : 'default',
          touchAction: 'none',
        }}
        onContextMenu={(e) => e.preventDefault()}
      >
        <Suspense
          fallback={
            <Centered>
              <CircularProgress size={24} />
            </Centered>
          }
        >
          <MapScene
            baseUrdf={tab.base_urdf}
            armUrdf={tab.arm_urdf}
            armOffset={tab.arm_offset}
            armCommandTopic={tab.arm_command_topic}
            publish={publish}
            onArmReady={onArmReady}
            map={map}
            costmap={isCleared(costmapMsg) ? null : costmapMsg}
            globalPath={globalPath}
            localPath={localPath}
            goal={goal}
            draftGoal={draftGoal}
            footprint={footprint}
            robotPose={robotPose}
            baseJoints={baseJoints}
            armJoints={armJoints}
            anchor={anchor}
            layers={layers}
            topView={topView}
            goalMode={goalMode || poiAdding}
            pois={poi.pois}
            selectedPoiId={poi.selectedId}
            poiDraft={poi.draft}
            controllerRef={controllerRef}
            mapFrame={mapFrame}
          />
        </Suspense>

        {!map && (
          <Typography
            variant="body2"
            color="text.secondary"
            sx={{ position: 'absolute', top: 8, left: 8, pointerEvents: 'none' }}
          >
            Waiting for map on {tab.map_topic ?? '?'}...
          </Typography>
        )}

        {tab.poi_list_topic && (
          <Box
            sx={{
              position: 'absolute',
              top: 8,
              left: 8,
              width: { xs: showPanel ? 0 : 'calc(100% - 16px)', sm: 300 },
              maxWidth: 'calc(100% - 16px)',
              display: { xs: showPanel ? 'none' : 'flex', sm: 'flex' },
              flexDirection: 'column',
              gap: 1,
              zIndex: 1,
              pointerEvents: 'none',
              '& > *': { pointerEvents: 'auto' },
            }}
          >
            <PoiListPanel
              pois={poi.pois}
              selectedId={poi.selectedId}
              open={showPoiList}
              onToggle={() => setPoiListOpen(!showPoiList)}
              onFocus={poi.focus}
            />
            {poi.selected && (
              <PoiEditorPanel
                key={poi.selected.id}
                poi={poi.selected}
                busy={poi.busy}
                onClose={() => poi.select(null)}
                onSend={poi.send}
              />
            )}
          </Box>
        )}

        <Collapse
          in={showPanel}
          sx={{ position: 'absolute', top: 8, right: 8, width: { xs: 'calc(100% - 16px)', sm: 260 }, zIndex: 1 }}
          onPointerDown={(e) => e.stopPropagation()}
        >
          <Paper
            variant="outlined"
            sx={{ maxHeight: { xs: '55vh', sm: '70vh' }, overflowY: 'auto', bgcolor: 'rgba(22, 27, 34, 0.92)' }}
          >
            <Box sx={{ px: 1.5, py: 1 }}>
              {LAYER_GROUPS.map((g) => (
                <Box key={g.title} sx={{ mb: 0.5 }}>
                  <Typography variant="overline" color="text.secondary" sx={{ lineHeight: 2 }}>
                    {g.title}
                  </Typography>
                  {g.keys.map((k) => {
                    const disabled = k === 'gpsMap' && !gpsAvailable
                    return (
                      <Box key={k}>
                        <FormControlLabel
                          sx={{ display: 'flex', mr: 0, minHeight: 40 }}
                          disabled={disabled}
                          control={<Checkbox size="small" checked={layers[k] && !disabled} onChange={() => toggleLayer(k)} />}
                          label={
                            <Box component="span" sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                              {LAYER_SWATCH[k] && <Swatch color={LAYER_SWATCH[k]!} />}
                              <Typography variant="body2" component="span">
                                {LAYER_LABELS[k]}
                              </Typography>
                            </Box>
                          }
                        />
                        {disabled && (
                          <Typography variant="caption" color="text.secondary" sx={{ display: 'block', ml: 4, mt: -0.5 }}>
                            Needs a GPS fix: waiting for {GPS_ANCHOR_TOPIC}.
                          </Typography>
                        )}
                        {k === 'localCostmap' && !tab.local_costmap_topic && (
                          <Typography variant="caption" color="text.secondary" sx={{ display: 'block', ml: 4, mt: -0.5 }}>
                            No costmap topic configured.
                          </Typography>
                        )}
                      </Box>
                    )
                  })}
                </Box>
              ))}
              <Divider sx={{ my: 1 }} />
              <Typography variant="overline" color="text.secondary" sx={{ lineHeight: 2 }}>
                Legend
              </Typography>
              {MAP_LEGEND.map(([label, color]) => (
                <Box key={label} sx={{ display: 'flex', alignItems: 'center', gap: 1, lineHeight: '22px' }}>
                  <Swatch color={color} />
                  <Typography variant="body2">{label}</Typography>
                </Box>
              ))}
              <Typography variant="body2" color="text.secondary" sx={{ mt: 0.5, fontFamily: MONO_FONT }}>
                {robotPose ? `Robot ${robotPose.x.toFixed(2)}, ${robotPose.y.toFixed(2)}` : `No ${mapFrame} pose (robot at origin)`}
              </Typography>
              {tab.arm_command_topic && (
                <Typography variant="body2" color={armReady ? 'success.main' : 'warning.main'} sx={{ mt: 0.5 }}>
                  {armReady ? 'Drag arm rings/handles to move the arm' : 'Waiting for arm servo positions...'}
                </Typography>
              )}
              <Typography variant="caption" color="text.secondary" sx={{ display: 'block', mt: 0.5 }}>
                {topView
                  ? goalMode
                    ? 'Press to place the goal, drag to set heading. Two fingers / right button pan, wheel or pinch zooms.'
                    : 'Top view: drag to pan, wheel or pinch to zoom.'
                  : 'Orbit: drag to rotate, right button / two fingers pan, wheel or pinch zooms.'}
              </Typography>
            </Box>
          </Paper>
        </Collapse>

        <Snackbar
          open={snackbarOpen}
          autoHideDuration={SNACKBAR_MS[action.state]}
          onClose={(_e, reason) => reason !== 'clickaway' && setAction({ state: 'idle', message: '' })}
          anchorOrigin={{ vertical: 'bottom', horizontal: 'center' }}
          sx={{ position: 'absolute' }}
        >
          <Alert
            onPointerDown={(e) => e.stopPropagation()}
            variant="filled"
            severity={action.state === 'ok' ? 'success' : action.state === 'error' ? 'error' : 'info'}
            onClose={busy ? undefined : () => setAction({ state: 'idle', message: '' })}
            sx={{ width: '100%' }}
          >
            {action.message}
          </Alert>
        </Snackbar>
      </Box>
    </Box>
  )
}
