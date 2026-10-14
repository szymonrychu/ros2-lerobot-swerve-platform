import { lazy, Suspense, useCallback, useEffect, useMemo, useState } from 'react'
import type { ReactElement, ReactNode } from 'react'
import Alert from '@mui/material/Alert'
import AppBar from '@mui/material/AppBar'
import Box from '@mui/material/Box'
import CircularProgress from '@mui/material/CircularProgress'
import Drawer from '@mui/material/Drawer'
import IconButton from '@mui/material/IconButton'
import List from '@mui/material/List'
import ListItemButton from '@mui/material/ListItemButton'
import ListItemIcon from '@mui/material/ListItemIcon'
import ListItemText from '@mui/material/ListItemText'
import ListSubheader from '@mui/material/ListSubheader'
import Snackbar from '@mui/material/Snackbar'
import Tab from '@mui/material/Tab'
import Tabs from '@mui/material/Tabs'
import Toolbar from '@mui/material/Toolbar'
import Typography from '@mui/material/Typography'
import MenuIcon from '@mui/icons-material/Menu'
import MapIcon from '@mui/icons-material/Map'
import VideocamIcon from '@mui/icons-material/Videocam'
import ShowChartIcon from '@mui/icons-material/ShowChart'
import PlaceIcon from '@mui/icons-material/Place'
import CameraIcon from '@mui/icons-material/Camera'
import PrecisionManufacturingIcon from '@mui/icons-material/PrecisionManufacturing'
import SmartToyIcon from '@mui/icons-material/SmartToy'
import FaceIcon from '@mui/icons-material/Face'
import TuneIcon from '@mui/icons-material/Tune'
import Divider from '@mui/material/Divider'
import TabIcon from '@mui/icons-material/Tab'
import log from './logging'
import { batteryStatus, cutoffBanner } from './battery/batteryStatus'
import type { BatteryPayload, BatteryStatus } from './battery/batteryStatus'
import { BatteryChip } from './components/BatteryChip'
import { GpsChip } from './components/GpsChip'
import { GPS_BASE_STATUS_KEY, baseGpsStatus, roverGpsStatus } from './gps/gpsStatus'
import type { BaseGpsPayload, RoverGpsPayload } from './gps/gpsStatus'
import { StateBanner } from './components/StateBanner'
import { useRosBridge } from './hooks/useRosBridge'
import { subscribedTopics } from './hooks/subscription'
import { CalibratePage } from './calibrate/CalibratePage'
import { clearCalibrateHash, isCalibrateHash, openCalibrate } from './calibrate/calibrateRoute'
import { tabBarMode } from './layout/tabBarMode'
import { useLayout } from './layout/useLayout'
import { TOUCH_TARGET_PX } from './theme'
import { OverlayBar } from './overlays/OverlayBar'
import { usePhotoLayer } from './photos/usePhotoLayer'
import type { PhotoLayerData } from './photos/usePhotoLayer'
import { toPoiItems } from './poi/poiItems'
import type { PendingFocus, PoiItem } from './poi/poiItems'
import { parsePoiList } from './poi/style'
import { browserStorage, initialTabIndex, mapTabIndex, orderTabs, readStoredTabId, supportedTabs, writeStoredTabId } from './tabSelection'
import { AppConfig, TabConfig } from './types'
import { TILT_GUARD_STATE_KEY, tiltStatus } from './tilt/tiltStatus'
import type { TiltPayload, TiltStatus } from './tilt/tiltStatus'
import { feedAllGraphBuffers } from './utils/graphBuffers'

const ArmTab = lazy(() => import('./tabs/ArmTab'))
const CameraTab = lazy(() => import('./tabs/CameraTab'))
const SensorGraphTab = lazy(() => import('./tabs/SensorGraphTab'))
const PoiTab = lazy(() => import('./tabs/PoiTab'))
const HeadTab = lazy(() => import('./tabs/HeadTab'))
const RgbdCameraTab = lazy(() => import('./tabs/RgbdCameraTab'))
const MapNavTab = lazy(() => import('./tabs/MapNavTab'))
const AgentChatTab = lazy(() => import('./tabs/AgentChatTab'))

const APP_TITLE = 'Robot'
const POI_LIST_TOPIC = '/poi/list'
const POI_PHOTOS_TOPIC = '/poi/photos'
// How often the battery and GPS chips re-evaluate staleness (ms).
const BATTERY_TICK_MS = 1000
// How long a command-rejected toast stays visible (ms).
const ERROR_TOAST_MS = 8000

const TAB_ICONS: Record<string, ReactElement> = {
  map_nav: <MapIcon />,
  arm: <PrecisionManufacturingIcon />,
  camera: <VideocamIcon />,
  rgbd_camera: <CameraIcon />,
  head: <FaceIcon />,
  sensor_graph: <ShowChartIcon />,
  poi: <PlaceIcon />,
  agent_chat: <SmartToyIcon />,
}

function tabIcon(type: string): ReactElement {
  return TAB_ICONS[type] ?? <TabIcon />
}

function Centered({ children }: { children: ReactNode }) {
  return (
    <Box sx={{ height: '100%', display: 'flex', alignItems: 'center', justifyContent: 'center', gap: 2, p: 2 }}>
      {children}
    </Box>
  )
}

function renderTab(
  tab: TabConfig,
  topicData: Record<string, unknown>,
  publish: (t: string, mt: string, d: unknown) => void,
  battery: BatteryStatus,
  tilt: TiltStatus,
  poi: { items: PoiItem[]; onShowOnMap: (focus: PendingFocus) => void; photos: PhotoLayerData },
  pendingFocus: PendingFocus | null,
  onFocusConsumed: () => void,
) {
  const props = { tab, topicData, publish }
  switch (tab.type) {
    case 'arm': return <ArmTab {...props} />
    case 'camera': return <CameraTab {...props} />
    case 'sensor_graph': return <SensorGraphTab {...props} />
    case 'poi': return <PoiTab items={poi.items} onShowOnMap={poi.onShowOnMap} />
    case 'head': return <HeadTab {...props} />
    case 'rgbd_camera': return <RgbdCameraTab {...props} />
    case 'map_nav':
      return <MapNavTab {...props} tilt={tilt} photos={poi.photos} pendingFocus={pendingFocus} onFocusConsumed={onFocusConsumed} />
    case 'agent_chat': return <AgentChatTab {...props} battery={battery} />
    default:
      return (
        <Centered>
          <Typography color="text.secondary">Unknown tab type: {tab.type}</Typography>
        </Centered>
      )
  }
}

export default function App() {
  const mode = tabBarMode(useLayout())
  const [config, setConfig] = useState<AppConfig | null>(null)
  const [activeTab, setActiveTab] = useState(0)
  const [pendingFocus, setPendingFocus] = useState<PendingFocus | null>(null)
  const [drawerOpen, setDrawerOpen] = useState(false)
  const [calibrating, setCalibrating] = useState(() => isCalibrateHash(window.location.hash))
  const [errorToast, setErrorToast] = useState<string | null>(null)
  const [batteryRxAt, setBatteryRxAt] = useState<number | null>(null)
  const [roverRxAt, setRoverRxAt] = useState<number | null>(null)
  const [baseRxAt, setBaseRxAt] = useState<number | null>(null)
  const [nowMs, setNowMs] = useState(() => Date.now())

  // Map tabs first: the map is the primary view whatever order the config lists tabs in.
  // Tab types merged into the 3D map (or removed) are dropped so a stale config shows no dead tabs.
  const tabs = useMemo(() => (config ? orderTabs(supportedTabs(config.tabs)) : []), [config])

  // Calibrate is a hash route, never a tab: the hash alone decides whether it replaces the tab content.
  useEffect(() => {
    const sync = () => setCalibrating(isCalibrateHash(window.location.hash))
    window.addEventListener('hashchange', sync)
    return () => window.removeEventListener('hashchange', sync)
  }, [])

  useEffect(() => {
    fetch('/api/config')
      .then((r) => r.json())
      .then((data: AppConfig) => {
        log.info('[app] Config loaded -', data.tabs.length, 'tabs,', data.overlays.length, 'overlay items')
        setActiveTab(initialTabIndex(orderTabs(supportedTabs(data.tabs)), readStoredTabId(browserStorage())))
        setConfig(data)
      })
      .catch((e) => log.warn('[app] Failed to load config:', e))
  }, [])

  const activeTabConfig = tabs[Math.min(activeTab, tabs.length - 1)]
  const subscription = config ? subscribedTopics(config, activeTabConfig) : []
  const { topicData, connected, publish } = useRosBridge(subscription, (e) =>
    setErrorToast(e.message)
  )

  // POI data: the latched /poi/list plus the one photo store shared by the POI tab and the map's photo layer
  // (refetched when its /poi/photos revision moves).
  const poiTabCfg = config?.tabs.find((t) => t.type === 'poi')
  const photoTabCfg = poiTabCfg ?? config?.tabs.find((t) => t.type === 'map_nav')
  const poiList = useMemo(() => parsePoiList(topicData[poiTabCfg?.poi_list_topic ?? POI_LIST_TOPIC]), [topicData, poiTabCfg?.poi_list_topic])
  const photoSummary = topicData[photoTabCfg?.poi_photos_topic ?? POI_PHOTOS_TOPIC] as { revision?: unknown } | null | undefined
  const photoRevision = typeof photoSummary?.revision === 'number' ? photoSummary.revision : null
  const poiPhotos = usePhotoLayer(photoTabCfg !== undefined, 'all', photoRevision)
  const clearPendingFocus = useCallback(() => setPendingFocus(null), [])
  const poiItems = useMemo(() => toPoiItems(poiList?.pois ?? [], poiPhotos.photos), [poiList, poiPhotos.photos])

  const tilt = tiltStatus(topicData[TILT_GUARD_STATE_KEY] as TiltPayload | null | undefined)

  const batteryCfg = config?.battery ?? null
  const batteryData = batteryCfg ? (topicData[batteryCfg.topic] as BatteryPayload | null | undefined) : undefined
  // Receipt time (not the message stamp) drives staleness, so robot/browser clock offsets do not matter.
  useEffect(() => {
    setBatteryRxAt(batteryData ? Date.now() : null)
  }, [batteryData])
  const gpsCfg = config?.gps_status ?? null
  const roverGps = gpsCfg?.rover_topic ? (topicData[gpsCfg.rover_topic] as RoverGpsPayload | null | undefined) : undefined
  const baseGps = gpsCfg?.base_url ? (topicData[GPS_BASE_STATUS_KEY] as BaseGpsPayload | null | undefined) : undefined
  useEffect(() => {
    setRoverRxAt(roverGps ? Date.now() : null)
  }, [roverGps])
  useEffect(() => {
    setBaseRxAt(baseGps ? Date.now() : null)
  }, [baseGps])
  useEffect(() => {
    if (!batteryCfg && !gpsCfg) return
    const id = setInterval(() => setNowMs(Date.now()), BATTERY_TICK_MS)
    return () => clearInterval(id)
  }, [batteryCfg, gpsCfg])
  const battery = batteryStatus(batteryData, batteryCfg, nowMs, batteryRxAt)
  const banner = cutoffBanner(battery)
  const roverChip = roverGpsStatus(roverGps, gpsCfg, nowMs, roverRxAt)
  const baseChip = baseGpsStatus(baseGps, roverGps, gpsCfg, nowMs, baseRxAt, roverRxAt)

  // Feed all graph-type tabs' buffers on every WebSocket update, regardless of active tab.
  // This ensures opening any graph tab shows a pre-filled rolling window of data.
  useEffect(() => {
    if (!config) return
    feedAllGraphBuffers(config.tabs, topicData)
  }, [topicData])  // eslint-disable-line react-hooks/exhaustive-deps

  if (!config) {
    return (
      <Centered>
        <CircularProgress size={24} />
        <Typography color="text.secondary">Loading config...</Typography>
      </Centered>
    )
  }

  const selectTab = (i: number) => {
    const t = tabs[i]
    if (!t) return
    log.info('[app] Tab activated:', t.label)
    if (isCalibrateHash(window.location.hash)) clearCalibrateHash()
    setActiveTab(i)
    writeStoredTabId(browserStorage(), t.id)
    setDrawerOpen(false)
  }

  const selectCalibrate = () => {
    log.info('[app] Calibrate opened')
    openCalibrate()
    setDrawerOpen(false)
  }

  const tab = tabs[activeTab]

  // Show-on-map: remember what to focus, then switch to the map tab (it consumes the request).
  const showOnMap = (focus: PendingFocus) => {
    const mapIndex = mapTabIndex(tabs)
    if (mapIndex < 0) return
    setPendingFocus(focus)
    selectTab(mapIndex)
  }

  return (
    <>
      <AppBar position="static">
        <Toolbar variant="dense" disableGutters sx={{ px: { xs: 0.5, sm: 2 }, gap: 1, minHeight: TOUCH_TARGET_PX }}>
          {mode.menu && (
            <IconButton aria-label="Open menu" onClick={() => setDrawerOpen(true)} edge="start">
              <MenuIcon />
            </IconButton>
          )}
          {!mode.tabsInDrawer && (
            <Typography variant="subtitle1" component="h1" sx={{ fontWeight: 500, mr: 1, whiteSpace: 'nowrap' }}>
              {APP_TITLE}
            </Typography>
          )}
          <Tabs
            value={!calibrating && tabs.length > 0 ? Math.min(activeTab, tabs.length - 1) : false}
            onChange={(_e, i: number) => selectTab(i)}
            variant={mode.scrollable ? 'scrollable' : 'standard'}
            scrollButtons="auto"
            allowScrollButtonsMobile
            aria-label="Dashboard tabs"
            sx={{ flex: 1, minWidth: 0 }}
          >
            {tabs.map((t) => (
              <Tab
                key={t.id}
                label={t.label}
                sx={{ px: { xs: 1, sm: 2 }, minWidth: { xs: '4.5rem', sm: '5.625rem' }, minHeight: TOUCH_TARGET_PX }}
              />
            ))}
          </Tabs>
          {batteryCfg && <BatteryChip status={battery} />}
          {gpsCfg?.rover_topic && <GpsChip status={roverChip} role="robot" />}
          {gpsCfg?.base_url && <GpsChip status={baseChip} role="base" />}
        </Toolbar>
      </AppBar>
      {banner && (
        <StateBanner state="error">{banner}</StateBanner>
      )}
      {tilt.banner && <StateBanner state={tilt.banner.severity === 'error' ? 'error' : 'warn'}>{tilt.banner.text}</StateBanner>}

      <Drawer anchor="left" open={drawerOpen} onClose={() => setDrawerOpen(false)}>
        <List
          sx={{ width: 'clamp(16rem, 85vw, 20rem)' }}
          subheader={<ListSubheader sx={{ bgcolor: 'background.paper' }}>{APP_TITLE}</ListSubheader>}
        >
          {mode.tabsInDrawer && tabs.map((t, i) => (
            <ListItemButton key={t.id} selected={!calibrating && i === activeTab} onClick={() => selectTab(i)} sx={{ minHeight: TOUCH_TARGET_PX }}>
              <ListItemIcon>{tabIcon(t.type)}</ListItemIcon>
              <ListItemText primary={t.label} />
            </ListItemButton>
          ))}
          {mode.tabsInDrawer && <Divider />}
          <ListItemButton selected={calibrating} onClick={selectCalibrate} sx={{ minHeight: TOUCH_TARGET_PX }}>
            <ListItemIcon>
              <TuneIcon />
            </ListItemIcon>
            <ListItemText primary="Calibrate" />
          </ListItemButton>
        </List>
      </Drawer>

      <Box component="main" sx={{ flex: 1, minHeight: 0, overflow: 'hidden', position: 'relative' }}>
        <Suspense
          fallback={
            <Centered>
              <CircularProgress size={24} />
            </Centered>
          }
        >
          {calibrating && <CalibratePage onBack={() => setCalibrating(false)} />}
          {!calibrating && tab && renderTab(tab, topicData, publish, battery, tilt, { items: poiItems, onShowOnMap: showOnMap, photos: poiPhotos }, pendingFocus, clearPendingFocus)}
        </Suspense>
      </Box>

      <Snackbar
        open={errorToast !== null}
        autoHideDuration={ERROR_TOAST_MS}
        onClose={(_e, reason) => reason !== 'clickaway' && setErrorToast(null)}
        anchorOrigin={{ vertical: 'bottom', horizontal: 'center' }}
      >
        <Alert severity="error" variant="outlined" onClose={() => setErrorToast(null)} sx={{ width: '100%' }}>
          {errorToast}
        </Alert>
      </Snackbar>

      <OverlayBar
        overlays={config.overlays}
        topicData={topicData}
        jointStatesTopic={config.tabs.find((t) => t.type === 'map_nav')?.base_joint_states_topic}
        connected={connected}
      />
    </>
  )
}
