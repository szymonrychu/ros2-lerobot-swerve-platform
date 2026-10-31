/**
 * Map tab: one 3D view in the ROS map frame merging the SLAM map, local costmap, GPS tiles, Nav2 plans/goal/
 * footprint and the robot (swerve base + display-only arm). Free orbit by default; "Top view" locks a top-down
 * camera in which goals are set by press + drag. The 3D scene is a separate lazily loaded chunk.
 */
import { lazy, Suspense, useCallback, useEffect, useMemo, useRef, useState } from 'react'
import type { ReactNode } from 'react'
import Alert from '@mui/material/Alert'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import CircularProgress from '@mui/material/CircularProgress'
import Menu from '@mui/material/Menu'
import MenuItem from '@mui/material/MenuItem'
import Paper from '@mui/material/Paper'
import Snackbar from '@mui/material/Snackbar'
import Stack from '@mui/material/Stack'
import Typography from '@mui/material/Typography'
import PlaceIcon from '@mui/icons-material/Place'
import MoreVertIcon from '@mui/icons-material/MoreVert'
import MyLocationIcon from '@mui/icons-material/MyLocation'
import FitScreenIcon from '@mui/icons-material/FitScreen'
import SaveAsIcon from '@mui/icons-material/SaveAs'
import FolderOpenIcon from '@mui/icons-material/FolderOpen'
import SaveIcon from '@mui/icons-material/Save'
import RestartAltIcon from '@mui/icons-material/RestartAlt'
import LayersIcon from '@mui/icons-material/Layers'
import FormatListBulletedIcon from '@mui/icons-material/FormatListBulleted'
import PhotoLibraryIcon from '@mui/icons-material/PhotoLibrary'
import VerticalAlignBottomIcon from '@mui/icons-material/VerticalAlignBottom'
import ThreeDRotationIcon from '@mui/icons-material/ThreeDRotation'
import AddLocationAltIcon from '@mui/icons-material/AddLocationAlt'
import PolylineIcon from '@mui/icons-material/Polyline'
import CheckIcon from '@mui/icons-material/Check'
import CloseIcon from '@mui/icons-material/Close'
import log from '../logging'
import { Pose2D, Vec2, yawToQuaternion } from '../map/mapMath'
import { ActionResult, isCleared, postTabAction } from '../map/mapActions'
import { baseMarkerLabel, basePlacement } from '../gps/basePlacement'
import { BaseGpsPayload, GPS_BASE_STATUS_KEY, isBaseSample } from '../gps/gpsStatus'
import { anchorSummary, validAnchor } from '../map3d/geo'
import { ReleasedBannerView, releasedLabel } from '../calibrate/ReleasedBanner'
import { useServoGroups } from '../calibrate/servoApi'
import { goalButtonState } from '../map/goalControl'
import { anyInRow, splitToolbar, stopGroupIds, TOOLBAR_GROUPS } from '../map/toolbarGroups'
import { StatusItem } from '../components/StatusItem'
import { LayersPanel } from '../map/LayersPanel'
import { MapStatusLine, mapStatusText } from '../map/MapStatusLine'
import { MapsDialog, SaveMapAsDialog } from '../map/MapsDialog'
import { useMapStatus } from '../map/mapsApi'
import { ToolButton, ToolGroup } from '../components/ToolButton'
import { ConfirmDialog } from '../layout/ConfirmDialog'
import { MapSidebar } from '../layout/MapSidebar'
import { useLayout } from '../layout/useLayout'
import { tiltStatus } from '../tilt/tiltStatus'
import type { TiltStatus } from '../tilt/tiltStatus'
import { draftGoalPose, finishGoalDraft, GoalDraft, startGoalDraft, updateGoalDraft } from '../map3d/goalGesture'
import type { GroundState } from '../map3d/groundLayer'
import { Bounds, mapBounds } from '../map3d/groundMath'
import {
  pickTileSource,
  readStoredTileSource,
  writeStoredTileSource,
} from '../map3d/tileSources'
import { LayerKey, LayerState, readLayerState, writeLayerState } from '../map3d/layers'
import type { SceneController } from '../map3d/MapScene'
import type { GridImageMsg, PathMsg } from '../map3d/sceneLayers'
import { browserStorage } from '../tabSelection'
import { CANVAS_BG } from '../theme'
import { Map3dStatusPayload, TabConfig } from '../types'
import { PhotoList } from '../photos/PhotoList'
import { PhotoPanel } from '../photos/PhotoPanel'
import type { Photo } from '../photos/api'
import { ageCutoff, type PhotoAge } from '../photos/geometry'
import type { PendingFocus } from '../poi/poiItems'
import type { PhotoLayerData } from '../photos/usePhotoLayer'
import { consumeFocus, whenController } from '../map3d/mapFocus'
import { PoiEditorPanel, PoiListPanel } from '../poi/PoiPanels'
import { canFinishArea } from '../poi/editor'
import { usePoiEditor } from '../poi/usePoiEditor'
import type { PoiResult } from '../poi/types'

const MapScene = lazy(() => import('../map3d/MapScene'))

const ROBOT_POSE_TOPIC = '/web_ui/robot_pose'
const GPS_ANCHOR_TOPIC = '/web_ui/gps_anchor'
const STOP_SX = { px: 3, flexShrink: 0, fontWeight: 500, letterSpacing: 0 }
const TOUCH_TARGET = '2.75rem' // 44 px minimum touch target
const ROBOT_FIT_SPAN_M = 4 // view size around the robot when there is no map to fit
const ALERT_SEVERITY = { idle: 'info', busy: 'info', ok: 'success', warning: 'warning', error: 'error' } as const
const SNACKBAR_MS: Record<ActionResult['state'], number | null> = { idle: null, busy: null, ok: 4000, warning: 10000, error: 10000 }

interface Props {
  tab: TabConfig
  topicData: Record<string, unknown>
  publish: (topic: string, msgType: string, data: unknown) => void
  /** Tilt guard state; at warning/error goal setting is disabled. */
  tilt?: TiltStatus
  /** Photos App fetches for the POI tab and the map photo layer (newest first, all ages). */
  photos: PhotoLayerData
  /** Map focus requested from the POI tab; the map clears it through onFocusConsumed once handled. */
  pendingFocus?: PendingFocus | null
  onFocusConsumed?: () => void
  /** Reports whether the 3D obstacles layer needs its topics; false again when it is switched off or the tab unmounts. */
  onObstaclesWanted?: (wanted: boolean) => void
  onMap3dCloudWanted?: (wanted: boolean) => void
  /** Reports whether the Head RGB-D layer needs its cloud topic; false again when it is switched off or the tab unmounts. */
  onHeadRgbdCloudWanted?: (wanted: boolean) => void
  /** Reports whether the Ground layer needs its cloud and state topics; false again when it is switched off or the tab unmounts. */
  onGroundWanted?: (wanted: boolean) => void
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

function Centered({ children }: { children: ReactNode }) {
  return (
    <Box sx={{ height: '100%', display: 'flex', alignItems: 'center', justifyContent: 'center' }}>{children}</Box>
  )
}

export default function MapNavTab({
  tab,
  topicData,
  publish,
  tilt = tiltStatus(undefined),
  photos,
  pendingFocus = null,
  onFocusConsumed,
  onObstaclesWanted,
  onMap3dCloudWanted,
  onHeadRgbdCloudWanted,
  onGroundWanted,
}: Props) {
  const containerRef = useRef<HTMLDivElement>(null)
  const controllerRef = useRef<SceneController | null>(null)
  const pointersRef = useRef<Set<number>>(new Set())
  const draftRef = useRef<GoalDraft | null>(null)
  const fittedRef = useRef(false)

  const [topView, setTopView] = useState(false)
  const [goalMode, setGoalMode] = useState(false)
  const [draft, setDraft] = useState<GoalDraft | null>(null)
  const [action, setAction] = useState<ActionResult>({ state: 'idle', message: '' })
  const [resetOpen, setResetOpen] = useState(false)
  const [layers, setLayers] = useState<LayerState>(() => readLayerState(browserStorage()))
  const [storedTileSource, setStoredTileSource] = useState<string | null>(() => readStoredTileSource(browserStorage()))
  const [tileCaption, setTileCaption] = useState<string | null>(null)
  const tileSources = useMemo(() => tab.tile_sources ?? [], [tab.tile_sources])
  const tileSource = useMemo(
    () => pickTileSource(tileSources, storedTileSource, tab.default_tile_source ?? null),
    [tileSources, storedTileSource, tab.default_tile_source],
  )
  const layout = useLayout()
  const narrow = layout === 'phone'
  const { overflow } = splitToolbar(TOOLBAR_GROUPS, layout)
  const inRow = (id: string) => !overflow.some((g) => g.id === id)
  const inMenu = (id: string) => overflow.some((g) => g.id === id)
  const [moreAnchor, setMoreAnchor] = useState<HTMLElement | null>(null)
  const closeMore = () => setMoreAnchor(null)
  const [panelOpen, setPanelOpen] = useState<boolean | null>(null) // null = follow screen size
  const [poiListOpen, setPoiListOpen] = useState<boolean | null>(null) // null = open on wide screens only
  const [photoListOpen, setPhotoListOpen] = useState(false) // the photo list only shows when asked for
  const showPanel = panelOpen ?? !narrow
  const showPoiList = poiListOpen ?? !narrow
  const hasPoi = Boolean(tab.poi_list_topic)

  const mapFrame = tab.map_frame ?? 'map'
  // Each value keeps its identity until its own topic updates, so the memoised scene skips unrelated traffic.
  // null = the backend cleared its cache (map reset / navigation stopped); undefined = nothing received yet.
  const mapMsg = topicData[tab.map_topic ?? ''] as GridImageMsg | null | undefined
  const costmapMsg = topicData[tab.local_costmap_topic ?? ''] as GridImageMsg | null | undefined
  const obstaclesConfigured = Boolean(tab.obstacle_cloud_topic && tab.scan_points_topic)
  const obstaclesWanted = layers.obstacles3d && obstaclesConfigured
  const obstacleCloud = topicData[tab.obstacle_cloud_topic ?? '']
  const scanPoints = topicData[tab.scan_points_topic ?? '']
  const map3dCloudWanted = layers.map3dCloud && Boolean(tab.map3d_cloud_topic)
  const map3dCloud = topicData[tab.map3d_cloud_topic ?? '']
  const map3dStatus = topicData[tab.map3d_status_topic ?? '']
  const rgbdCloudConfigured = Boolean(tab.rgbd_cloud_topic)
  const headRgbdCloudWanted = layers.headRgbd && rgbdCloudConfigured
  const headRgbdCloud = topicData[tab.rgbd_cloud_topic ?? '']
  const groundConfigured = Boolean(tab.ground_cloud_topic && tab.ground_state_topic)
  const groundWanted = layers.ground && groundConfigured
  const groundCloud = topicData[tab.ground_cloud_topic ?? '']
  const groundState = topicData[tab.ground_state_topic ?? ''] as GroundState | null | undefined
  const globalPath = topicData[tab.global_plan_topic ?? ''] as PathMsg | null | undefined
  const localPath = topicData[tab.local_plan_topic ?? ''] as PathMsg | null | undefined
  const goalMsg = topicData[tab.goal_topic ?? ''] as FramedPose | null | undefined
  const poseMsg = topicData[ROBOT_POSE_TOPIC] as FramedPose | null | undefined
  const footprint = topicData[tab.footprint_topic ?? ''] as PathMsg | null | undefined
  const anchorMsg = topicData[GPS_ANCHOR_TOPIC]
  const baseJoints = topicData[tab.base_joint_states_topic ?? ''] as JointStates | undefined
  const armJoints = topicData[tab.arm_joint_states_topic ?? ''] as JointStates | undefined
  const headJoints = topicData[tab.head_joint_states_topic ?? ''] as JointStates | undefined
  const robotPose = inFrame(poseMsg, mapFrame) ? poseMsg : null
  const goal = inFrame(goalMsg, mapFrame) ? goalMsg : null
  const anchor = useMemo(() => validAnchor(anchorMsg), [anchorMsg])
  // The anchor is the GPS placement of the map frame origin itself, so it needs no frame check.
  const gpsAvailable = anchor !== null
  const baseStatus = topicData[GPS_BASE_STATUS_KEY] as BaseGpsPayload | null | undefined
  const basePosition = useMemo(() => basePlacement(anchor, baseStatus), [anchor, baseStatus])
  const baseLabel = isBaseSample(baseStatus) ? baseMarkerLabel(baseStatus) : 'Base'
  const map = isCleared(mapMsg) ? null : mapMsg
  const mapBox = useMemo<Bounds | null>(() => (map ? mapBounds(map) : null), [map])

  useEffect(() => writeLayerState(browserStorage(), layers), [layers])
  useEffect(() => {
    onObstaclesWanted?.(obstaclesWanted)
    return () => onObstaclesWanted?.(false)
  }, [obstaclesWanted, onObstaclesWanted])
  useEffect(() => {
    onMap3dCloudWanted?.(map3dCloudWanted)
    return () => onMap3dCloudWanted?.(false)
  }, [map3dCloudWanted, onMap3dCloudWanted])
  useEffect(() => {
    onHeadRgbdCloudWanted?.(headRgbdCloudWanted)
    return () => onHeadRgbdCloudWanted?.(false)
  }, [headRgbdCloudWanted, onHeadRgbdCloudWanted])
  useEffect(() => {
    onGroundWanted?.(groundWanted)
    return () => onGroundWanted?.(false)
  }, [groundWanted, onGroundWanted])

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
    return whenController(
      () => controllerRef.current,
      (c) => {
        c.fit(mapBox)
        fittedRef.current = true
      },
    )
  }, [mapBox])

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
  const photoSummary = topicData[tab.poi_photos_topic ?? '/poi/photos'] as { revision?: unknown } | null | undefined
  const photoRevision = typeof photoSummary?.revision === 'number' ? photoSummary.revision : null
  const [photoAge, setPhotoAge] = useState<PhotoAge>('7d')
  // App's photo store (one fetch path for the POI tab and the map), narrowed to the layer's age filter.
  const layerPhotos = useMemo<PhotoLayerData>(() => {
    const cutoff = ageCutoff(photoAge, Date.now() / 1000)
    const shown = cutoff === null ? photos.photos : photos.photos.filter((p) => p.ts >= cutoff)
    return { photos: shown, total: cutoff === null ? photos.total : shown.length, error: photos.error }
  }, [photos, photoAge])
  const [pickedPhoto, setPickedPhoto] = useState<Photo | null>(null)
  // The open photo follows the refetched data (name, note and links change after a save).
  const openPhoto = pickedPhoto && (photos.photos.find((p) => p.id === pickedPhoto.id) ?? pickedPhoto)
  const selectPoi = poi.select
  // One item open at a time: opening a photo deselects the POI, selecting a POI closes the photo.
  const selectPhotoOnMap = useCallback(
    (id: string) => {
      selectPoi(null)
      setPickedPhoto(photos.photos.find((p) => p.id === id) ?? null)
    },
    [photos.photos, selectPoi],
  )
  useEffect(() => {
    if (poi.selectedId) setPickedPhoto(null)
  }, [poi.selectedId])

  // "Show on map" from the POI tab (the request survives the tab switch in App): make the target visible, then
  // once the scene controller exists centre on it, open it in the sidebar (bottom sheet on a phone) and clear the request.
  const onFocusConsumedRef = useRef(onFocusConsumed)
  onFocusConsumedRef.current = onFocusConsumed
  useEffect(() => {
    if (!pendingFocus) return
    if (pendingFocus.kind === 'poi') {
      setLayers((s) => (s.pois ? s : { ...s, pois: true }))
      return
    }
    setLayers((s) => (s.photos ? s : { ...s, photos: true }))
    const cutoff = ageCutoff(photoAge, Date.now() / 1000)
    const target = photos.photos.find((p) => p.id === pendingFocus.id)
    if (target && cutoff !== null && target.ts < cutoff) setPhotoAge('all')
  }, [pendingFocus, photos.photos, photoAge])
  useEffect(() => {
    if (!pendingFocus) return
    return consumeFocus(pendingFocus, poi.pois, photos.photos, () => controllerRef.current, () => {
      fittedRef.current = true // a map arriving later must not move the view off the focused item
      if (pendingFocus.kind === 'photo') selectPhotoOnMap(pendingFocus.id)
      else selectPoi(pendingFocus.id)
      onFocusConsumedRef.current?.()
    })
  }, [pendingFocus, poi.pois, photos.photos, selectPhotoOnMap, selectPoi])
  const servo = useServoGroups()
  const baseReleased = releasedLabel(servo, 'base') !== null
  const goalButton = goalButtonState({ hasGoalTopic: Boolean(tab.goal_topic), topView, tilt })
  const goalBlocked = tilt.navBlocked || baseReleased
  useEffect(() => {
    if (goalBlocked) setGoalMode(false)
  }, [goalBlocked])
  const robotPoseRef = useRef(robotPose)
  robotPoseRef.current = robotPose

  const sendGoal = useCallback(
    (g: Pose2D) => {
      if (!tab.goal_topic || goalBlocked) return
      log.info('[map] goal', g.x.toFixed(2), g.y.toFixed(2), 'yaw', g.yaw.toFixed(2), 'in', mapFrame)
      publish(tab.goal_topic, 'geometry_msgs/PoseStamped', {
        header: { frame_id: mapFrame, stamp: { sec: 0, nanosec: 0 } }, // backend stamps with the node clock
        pose: { position: { x: g.x, y: g.y, z: 0 }, orientation: yawToQuaternion(g.yaw) },
      })
    },
    [tab.goal_topic, mapFrame, publish, goalBlocked],
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

  const { status: mapsStatus, refresh: refreshMaps } = useMapStatus(tab.id)
  const [mapsOpen, setMapsOpen] = useState(false)
  const [saveAsOpen, setSaveAsOpen] = useState(false)

  const runAction = async (path: string, busyMessage: string) => {
    setAction({ state: 'busy', message: busyMessage })
    setAction(await postTabAction(path, tab.id))
  }

  const togglePoiMode = (mode: 'add_point' | 'add_area') => {
    setGoalMode(false)
    setLayers((s) => ({ ...s, pois: true }))
    poi.setMode(poi.mode === mode ? 'none' : mode)
  }

  const toggleLayer = (k: LayerKey) => setLayers((s) => ({ ...s, [k]: !s[k] }))

  const centerOnRobot = () => robotPose && controllerRef.current?.centerOn(robotPose)
  const fit = () => {
    if (mapBox) controllerRef.current?.fit(mapBox)
    else if (robotPose) {
      const h = ROBOT_FIT_SPAN_M / 2
      controllerRef.current?.fit({ minX: robotPose.x - h, maxX: robotPose.x + h, minY: robotPose.y - h, maxY: robotPose.y + h })
    }
  }

  const busy = action.state === 'busy'
  const stopIds = stopGroupIds()
  const snackbarOpen = action.state !== 'idle' && action.message !== ''
  const hintItems: string[] = []
  if (goalMode) hintItems.push('Press on the map to set the goal, drag to set its heading')
  if (poi.mode === 'add_point') hintItems.push('Click the map to place a point of interest')
  if (poi.mode === 'add_area') hintItems.push(`Area: ${poi.draft.length} vertices, double-click or Enter finishes`)

  const poseItem = (
    <StatusItem
      key="pose"
      label="Robot"
      value={robotPose ? `${robotPose.x.toFixed(2)}, ${robotPose.y.toFixed(2)}` : `no ${mapFrame} pose`}
      tooltip={robotPose ? undefined : 'No pose in the map frame: the robot is drawn at the origin'}
    />
  )
  const mapText = mapStatusText(mapsStatus)
  const statusItems = [...hintItems, poseItem, ...(mapText ? [mapText] : [])]

  // POI list, POI editor, photo panel and photo list. Phone: a column spanning the map, so the panels' bottom
  // sheets anchor to its bottom edge (hidden while the layers sheet is up). Wide: sections of the sidebar.
  const poiPanels = hasPoi && (
    <>
      {(narrow || showPoiList) && (
        <PoiListPanel
          pois={poi.pois}
          photos={photos.photos}
          selectedId={poi.selectedId}
          open={showPoiList && !(narrow && poi.selected)}
          onToggle={() => setPoiListOpen(!showPoiList)}
          onFocus={poi.focus}
          tabId={tab.id}
        />
      )}
      {poi.selected && (
        <PoiEditorPanel
          key={poi.selected.id}
          poi={poi.selected}
          busy={poi.busy}
          onClose={() => poi.select(null)}
          onSend={poi.send}
        />
      )}
      {openPhoto && (
        <PhotoPanel
          key={openPhoto.id}
          tabId={tab.id}
          photo={openPhoto}
          pois={poi.pois}
          onClose={() => setPickedPhoto(null)}
          onDeleted={() => setPickedPhoto(null)}
        />
      )}
      {photoListOpen && (
        <PhotoList
          onSelect={setPickedPhoto}
          tabId={tab.id}
          revision={photoRevision}
          selectedPoiId={poi.selectedId}
          selectedPoiName={poi.selected ? poi.selected.name || '(unnamed)' : null}
        />
      )}
    </>
  )
  const panels = (
    <>
      {poiPanels && narrow && (
        <Box
          sx={{
            position: 'absolute',
            inset: 0,
            p: 1,
            boxSizing: 'border-box',
            display: showPanel ? 'none' : 'flex',
            flexDirection: 'column',
            gap: 1,
            zIndex: 1,
            pointerEvents: 'none',
            '& > *': { pointerEvents: 'auto' },
          }}
        >
          {poiPanels}
        </Box>
      )}
      {!narrow && poiPanels}
      <LayersPanel
        open={showPanel}
        onClose={() => setPanelOpen(false)}
        layers={layers}
        onToggle={toggleLayer}
        gpsAvailable={gpsAvailable}
        rtkAvailable={basePosition !== null}
        costmapConfigured={Boolean(tab.local_costmap_topic)}
        obstaclesConfigured={obstaclesConfigured}
        rgbdCloudConfigured={rgbdCloudConfigured}
        groundConfigured={groundConfigured}
        groundState={groundState}
        tileSources={tileSources}
        tileSource={tileSource}
        onTileSource={(id) => {
          setStoredTileSource(id)
          writeStoredTileSource(browserStorage(), id)
        }}
        tileCaption={tileCaption}
        anchorSummary={anchor ? anchorSummary(anchor) : null}
        photoAge={photoAge}
        onPhotoAge={setPhotoAge}
        photoStatus={{ shown: layerPhotos.photos.length, total: layerPhotos.total, error: layerPhotos.error }}
        topView={topView}
        goalMode={goalMode}
        map3dStatus={map3dStatus as Map3dStatusPayload | null | undefined}
      />
    </>
  )
  // Wide layouts: the sidebar exists only while a panel is open, so with every panel closed the map is full width.
  const sidebarOpen =
    showPanel || (hasPoi && (showPoiList || poi.selected !== null || openPhoto !== null || photoListOpen))

  return (
    <Box sx={{ width: '100%', height: '100%', display: 'flex', flexDirection: 'column' }}>
      <Paper
        square
        elevation={0}
        sx={{ flexShrink: 0, px: { xs: 1, sm: 2 }, py: 1, borderBottom: 1, borderColor: 'divider' }}
      >
        <ReleasedBannerView groupId="base" servo={servo} />
        <Stack
          direction="row"
          useFlexGap
          sx={{ flexWrap: narrow ? 'nowrap' : 'wrap', alignItems: 'center', gap: 1, overflowX: narrow ? 'auto' : 'visible' }}
        >
          {stopIds.includes('stop') && (
            <Button variant="contained" color="error" onClick={() => void runAction('/api/nav/stop', 'Stopping...')} sx={STOP_SX}>
              STOP
            </Button>
          )}
          {anyInRow(overflow, ['cameraView', 'layers']) && (
            <ToolGroup label="Map view">
              {inRow('cameraView') && (
                <>
                  <ToolButton label="Top view" icon={topView ? <VerticalAlignBottomIcon /> : <ThreeDRotationIcon />} pressed={topView} onClick={() => setTopView((v) => !v)} />
                  <ToolButton label="Center on robot" icon={<MyLocationIcon />} disabled={!robotPose} onClick={centerOnRobot} />
                  <ToolButton label="Fit map" icon={<FitScreenIcon />} disabled={!mapBox && !robotPose} onClick={fit} />
                </>
              )}
              {inRow('layers') && (
                <ToolButton
                  label={showPanel ? 'Hide layers and legend' : 'Show layers and legend'}
                  icon={<LayersIcon />}
                  pressed={showPanel}
                  onClick={() => setPanelOpen(!showPanel)}
                />
              )}
              {inRow('layers') && hasPoi && !narrow && (
                <ToolButton
                  label={showPoiList ? 'Hide POI list' : 'Show POI list'}
                  icon={<FormatListBulletedIcon />}
                  pressed={showPoiList}
                  onClick={() => setPoiListOpen(!showPoiList)}
                />
              )}
              {inRow('layers') && hasPoi && (
                <ToolButton
                  label={photoListOpen ? 'Hide photo list' : 'Show photo list'}
                  icon={<PhotoLibraryIcon />}
                  pressed={photoListOpen}
                  onClick={() => setPhotoListOpen((o) => !o)}
                />
              )}
            </ToolGroup>
          )}
          <ToolGroup label="Goal">
            <ToolButton
              label={goalButton.disabled && goalButton.tooltip ? goalButton.tooltip : 'Set goal'}
              icon={<PlaceIcon />}
              pressed={goalMode}
              disabled={goalButton.disabled || baseReleased}
              onClick={() => {
                poi.setMode('none')
                setGoalMode((m) => !m)
              }}
            />
          </ToolGroup>
          {inRow('poi') && tab.poi_command_topic && (
            <ToolGroup label="Points of interest">
              <ToolButton
                label={topView ? 'Add point' : 'Add point (needs Top view)'}
                icon={<AddLocationAltIcon />}
                disabled={!topView}
                pressed={poi.mode === 'add_point'}
                onClick={() => togglePoiMode('add_point')}
              />
              <ToolButton
                label={topView ? 'Add area' : 'Add area (needs Top view)'}
                icon={<PolylineIcon />}
                disabled={!topView}
                pressed={poi.mode === 'add_area'}
                onClick={() => togglePoiMode('add_area')}
              />
              {poi.mode === 'add_area' && (
                <ToolButton label="Finish area" icon={<CheckIcon />} disabled={!canFinishArea(poi.draft)} onClick={poi.finishArea} />
              )}
              {poiAdding && <ToolButton label="Cancel placing" icon={<CloseIcon />} onClick={poi.cancelDraft} />}
            </ToolGroup>
          )}
          {inRow('mapFile') && (
            <ToolGroup label="Map file">
              <ToolButton label="Save map" icon={<SaveIcon />} disabled={busy} onClick={() => void runAction('/api/map/save', 'Saving map...')} />
              <ToolButton label="Save map as" icon={<SaveAsIcon />} disabled={busy} onClick={() => setSaveAsOpen(true)} />
              <ToolButton label="Maps" icon={<FolderOpenIcon />} onClick={() => setMapsOpen(true)} />
              <ToolButton label="Reset map" icon={<RestartAltIcon />} disabled={busy} onClick={() => setResetOpen(true)} />
            </ToolGroup>
          )}
          {overflow.length > 0 && (
            <>
              <ToolButton label="More map controls" icon={<MoreVertIcon />} expanded={moreAnchor !== null} onClick={(e) => setMoreAnchor(e.currentTarget)} />
              <Menu anchorEl={moreAnchor} open={moreAnchor !== null} onClose={closeMore}>
                {inMenu('cameraView') && [
                  <MenuItem
                    key="orbit"
                    selected={topView}
                    sx={{ minHeight: TOUCH_TARGET }}
                    onClick={() => {
                      setTopView((v) => !v)
                      closeMore()
                    }}
                  >
                    {topView ? 'Unlock camera (Orbit)' : 'Top view'}
                  </MenuItem>,
                  <MenuItem
                    key="center"
                    disabled={!robotPose}
                    sx={{ minHeight: TOUCH_TARGET }}
                    onClick={() => {
                      centerOnRobot()
                      closeMore()
                    }}
                  >
                    Center on robot
                  </MenuItem>,
                  <MenuItem
                    key="fit"
                    disabled={!mapBox && !robotPose}
                    sx={{ minHeight: TOUCH_TARGET }}
                    onClick={() => {
                      fit()
                      closeMore()
                    }}
                  >
                    Fit map
                  </MenuItem>,
                ]}
                {inMenu('poi') && tab.poi_command_topic && [
                  <MenuItem
                    key="add_point"
                    selected={poi.mode === 'add_point'}
                    disabled={!topView}
                    sx={{ minHeight: TOUCH_TARGET }}
                    onClick={() => {
                      togglePoiMode('add_point')
                      closeMore()
                    }}
                  >
                    {topView ? 'Add point' : 'Add point (needs Top view)'}
                  </MenuItem>,
                  <MenuItem
                    key="add_area"
                    selected={poi.mode === 'add_area'}
                    disabled={!topView}
                    sx={{ minHeight: TOUCH_TARGET }}
                    onClick={() => {
                      togglePoiMode('add_area')
                      closeMore()
                    }}
                  >
                    {topView ? 'Add area' : 'Add area (needs Top view)'}
                  </MenuItem>,
                  poi.mode === 'add_area' && (
                  <MenuItem
                    key="finish"
                    disabled={!canFinishArea(poi.draft)}
                    sx={{ minHeight: TOUCH_TARGET }}
                    onClick={() => {
                      poi.finishArea()
                      closeMore()
                    }}
                  >
                    Finish area ({poi.draft.length} vertices)
                  </MenuItem>
                  ),
                  poiAdding && (
                  <MenuItem
                    key="cancel"
                    sx={{ minHeight: TOUCH_TARGET }}
                    onClick={() => {
                      poi.cancelDraft()
                      closeMore()
                    }}
                  >
                    Cancel
                  </MenuItem>
                  ),
                ]}
                {inMenu('layers') && (
                  <MenuItem
                    selected={showPanel}
                    sx={{ minHeight: TOUCH_TARGET }}
                    onClick={() => {
                      setPanelOpen(!showPanel)
                      closeMore()
                    }}
                  >
                    {showPanel ? 'Hide layers and legend' : 'Show layers and legend'}
                  </MenuItem>
                )}
                {inMenu('layers') && hasPoi && (
                  <MenuItem
                    selected={photoListOpen}
                    sx={{ minHeight: TOUCH_TARGET }}
                    onClick={() => {
                      setPhotoListOpen((o) => !o)
                      closeMore()
                    }}
                  >
                    {photoListOpen ? 'Hide photo list' : 'Show photo list'}
                  </MenuItem>
                )}
                {inMenu('mapFile') && [
                  <MenuItem
                    key="save"
                    disabled={busy}
                    sx={{ minHeight: TOUCH_TARGET }}
                    onClick={() => {
                      closeMore()
                      void runAction('/api/map/save', 'Saving map...')
                    }}
                  >
                    Save map
                  </MenuItem>,
                  <MenuItem
                    key="save-as"
                    disabled={busy}
                    sx={{ minHeight: TOUCH_TARGET }}
                    onClick={() => {
                      closeMore()
                      setSaveAsOpen(true)
                    }}
                  >
                    Save map as
                  </MenuItem>,
                  <MenuItem
                    key="maps"
                    sx={{ minHeight: TOUCH_TARGET }}
                    onClick={() => {
                      closeMore()
                      setMapsOpen(true)
                    }}
                  >
                    Maps
                  </MenuItem>,
                  <MenuItem
                    key="reset"
                    disabled={busy}
                    sx={{ minHeight: TOUCH_TARGET }}
                    onClick={() => {
                      closeMore()
                      setResetOpen(true)
                    }}
                  >
                    Reset map
                  </MenuItem>,
                ]}
              </Menu>
            </>
          )}
        </Stack>
      </Paper>

      <Box sx={{ flex: 1, minHeight: 0, display: 'flex', flexDirection: 'row' }}>
        <Box
          ref={containerRef}
          data-testid="map-container"
          sx={{
            flex: 1,
            minWidth: 0,
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
              tileSource={tileSource}
              tileZoom={tab.tile_display_zoom}
              onTileCaption={setTileCaption}
              baseUrdf={tab.base_urdf}
              armUrdf={tab.arm_urdf}
              headUrdf={tab.head_urdf}
              armOffset={tab.arm_offset}
              map={map}
              costmap={isCleared(costmapMsg) ? null : costmapMsg}
              obstacleCloud={obstacleCloud}
              scanPoints={scanPoints}
              map3dCloud={map3dCloud}
              headRgbdCloud={headRgbdCloud}
              groundCloud={groundCloud}
              groundState={groundState}
              map3dFrameId={tab.map3d_frame_id}
              lidarWallHeightM={tab.lidar_wall_height_m}
              globalPath={globalPath}
              localPath={localPath}
              goal={goal}
              draftGoal={draftGoal}
              footprint={footprint}
              robotPose={robotPose}
              baseJoints={baseJoints}
              armJoints={armJoints}
              headJoints={headJoints}
              anchor={anchor}
              basePosition={basePosition}
              baseLabel={baseLabel}
              layers={layers}
              topView={topView}
              goalMode={goalMode || poiAdding}
              pois={poi.pois}
              selectedPoiId={poi.selectedId}
              poiDraft={poi.draft}
              photos={layerPhotos.photos}
              selectedPhotoId={openPhoto?.id ?? null}
              onSelectPhoto={goalMode || poiAdding ? null : selectPhotoOnMap}
              controllerRef={controllerRef}
              mapFrame={mapFrame}
            />
          </Suspense>
          {layers.gpsMap && anchor && tileSource?.attribution && (
            <Typography
              variant="caption"
              sx={{
                position: 'absolute',
                right: 4,
                bottom: 2,
                px: 0.5,
                color: 'text.secondary',
                bgcolor: 'background.paper',
                pointerEvents: 'none',
                fontSize: '0.65rem',
              }}
            >
              {tileSource.attribution}
            </Typography>
          )}

          {!map && (
            <Typography
              variant="body2"
              color="text.secondary"
              sx={{ position: 'absolute', top: 1, left: 1, pointerEvents: 'none' }}
            >
              Waiting for map on {tab.map_topic ?? '?'}...
            </Typography>
          )}

          <MapStatusLine items={statusItems} />

          <Snackbar
            open={snackbarOpen}
            autoHideDuration={SNACKBAR_MS[action.state]}
            onClose={(_e, reason) => reason !== 'clickaway' && setAction({ state: 'idle', message: '' })}
            anchorOrigin={{ vertical: 'bottom', horizontal: 'center' }}
            sx={{ position: 'absolute' }}
          >
            <Alert
              onPointerDown={(e) => e.stopPropagation()}
              variant="standard"
              severity={ALERT_SEVERITY[action.state]}
              onClose={busy ? undefined : () => setAction({ state: 'idle', message: '' })}
              sx={{ width: '100%' }}
            >
              {action.message}
            </Alert>
          </Snackbar>
          {narrow && <MapSidebar>{panels}</MapSidebar>}
        </Box>
        {!narrow && sidebarOpen && <MapSidebar>{panels}</MapSidebar>}
      </Box>
      <SaveMapAsDialog
        open={saveAsOpen}
        tabId={tab.id}
        existing={(mapsStatus?.maps ?? []).map((m) => m.name)}
        onClose={() => setSaveAsOpen(false)}
        onSaved={(result) => {
          setAction(result)
          void refreshMaps()
        }}
      />
      <MapsDialog
        open={mapsOpen}
        tabId={tab.id}
        status={mapsStatus}
        onClose={() => setMapsOpen(false)}
        onChanged={() => void refreshMaps()}
      />
      <ConfirmDialog
        open={resetOpen}
        title="Reset map?"
        body="Discards the current SLAM map and starts a new one, and deletes the live POIs and photos. Saved maps are kept."
        confirmLabel="Reset map"
        onCancel={() => setResetOpen(false)}
        onConfirm={() => {
          setResetOpen(false)
          void runAction('/api/map/reset', 'Resetting map...')
        }}
      />
    </Box>
  )
}
