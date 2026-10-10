import { lazy, Suspense, useEffect, useMemo, useState } from 'react'
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
import useMediaQuery from '@mui/material/useMediaQuery'
import { useTheme } from '@mui/material/styles'
import MenuIcon from '@mui/icons-material/Menu'
import MapIcon from '@mui/icons-material/Map'
import VideocamIcon from '@mui/icons-material/Videocam'
import ShowChartIcon from '@mui/icons-material/ShowChart'
import ThreeDRotationIcon from '@mui/icons-material/ThreeDRotation'
import CameraIcon from '@mui/icons-material/Camera'
import SmartToyIcon from '@mui/icons-material/SmartToy'
import TabIcon from '@mui/icons-material/Tab'
import log from './logging'
import { batteryStatus, cutoffBanner } from './battery/batteryStatus'
import type { BatteryPayload, BatteryStatus } from './battery/batteryStatus'
import { BatteryChip } from './components/BatteryChip'
import { GpsChip } from './components/GpsChip'
import { GPS_BASE_STATUS_KEY, baseGpsStatus, roverGpsStatus } from './gps/gpsStatus'
import type { BaseGpsPayload, RoverGpsPayload } from './gps/gpsStatus'
import { useRosBridge } from './hooks/useRosBridge'
import { OverlayBar } from './overlays/OverlayBar'
import { browserStorage, initialTabIndex, orderTabs, readStoredTabId, supportedTabs, writeStoredTabId } from './tabSelection'
import { AppConfig, TabConfig } from './types'
import { feedAllGraphBuffers } from './utils/graphBuffers'

const CameraTab = lazy(() => import('./tabs/CameraTab'))
const SensorGraphTab = lazy(() => import('./tabs/SensorGraphTab'))
const ImuOrientationTab = lazy(() => import('./tabs/ImuOrientationTab'))
const RgbdCameraTab = lazy(() => import('./tabs/RgbdCameraTab'))
const MapNavTab = lazy(() => import('./tabs/MapNavTab'))
const AgentChatTab = lazy(() => import('./tabs/AgentChatTab'))

const APP_TITLE = 'Robot'
// How often the battery and GPS chips re-evaluate staleness (ms).
const BATTERY_TICK_MS = 1000
// How long a command-rejected toast stays visible (ms).
const ERROR_TOAST_MS = 8000

const TAB_ICONS: Record<string, ReactElement> = {
  map_nav: <MapIcon />,
  camera: <VideocamIcon />,
  rgbd_camera: <CameraIcon />,
  sensor_graph: <ShowChartIcon />,
  imu_orientation: <ThreeDRotationIcon />,
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
) {
  const props = { tab, topicData, publish }
  switch (tab.type) {
    case 'camera': return <CameraTab {...props} />
    case 'sensor_graph': return <SensorGraphTab {...props} />
    case 'imu_orientation': return <ImuOrientationTab {...props} />
    case 'rgbd_camera': return <RgbdCameraTab {...props} />
    case 'map_nav': return <MapNavTab {...props} />
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
  const theme = useTheme()
  const narrow = useMediaQuery(theme.breakpoints.down('sm'))
  const [config, setConfig] = useState<AppConfig | null>(null)
  const [activeTab, setActiveTab] = useState(0)
  const [drawerOpen, setDrawerOpen] = useState(false)
  const [errorToast, setErrorToast] = useState<string | null>(null)
  const [batteryRxAt, setBatteryRxAt] = useState<number | null>(null)
  const [roverRxAt, setRoverRxAt] = useState<number | null>(null)
  const [baseRxAt, setBaseRxAt] = useState<number | null>(null)
  const [nowMs, setNowMs] = useState(() => Date.now())

  // Map tabs first: the map is the primary view whatever order the config lists tabs in.
  // Tab types merged into the 3D map (or removed) are dropped so a stale config shows no dead tabs.
  const tabs = useMemo(() => (config ? orderTabs(supportedTabs(config.tabs)) : []), [config])

  useEffect(() => {
    fetch('/api/config')
      .then((r) => r.json())
      .then((data: AppConfig) => {
        log.info('[app] Config loaded —', data.tabs.length, 'tabs,', data.overlays.length, 'overlay items')
        setActiveTab(initialTabIndex(orderTabs(supportedTabs(data.tabs)), readStoredTabId(browserStorage())))
        setConfig(data)
      })
      .catch((e) => log.warn('[app] Failed to load config:', e))
  }, [])

  const allTopics = config
    ? [
        ...config.tabs.flatMap((t) => [
          t.topic,
          ...(t.topics?.map((ts) => ts.topic) ?? []),
          t.color_topic, t.depth_topic, t.camera_info_topic,
          t.map_topic, t.global_plan_topic, t.local_plan_topic, t.footprint_topic,
          t.local_costmap_topic, t.base_joint_states_topic, t.arm_joint_states_topic,
          ...(t.type === 'map_nav' ? [t.goal_topic] : []),
        ]),
        ...config.overlays.map((o) => o.topic),
      ].filter((t): t is string => Boolean(t))
    : []

  const { topicData, connected, publish } = useRosBridge(allTopics, (e) => setErrorToast(e.message))

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
        <Typography color="text.secondary">Loading config…</Typography>
      </Centered>
    )
  }

  const selectTab = (i: number) => {
    const t = tabs[i]
    if (!t) return
    log.info('[app] Tab activated:', t.label)
    setActiveTab(i)
    writeStoredTabId(browserStorage(), t.id)
    setDrawerOpen(false)
  }

  const tab = tabs[activeTab]

  return (
    <>
      <AppBar position="static">
        <Toolbar variant="dense" disableGutters sx={{ px: { xs: 0.5, sm: 1.5 }, gap: 1, minHeight: 48 }}>
          {narrow && (
            <IconButton aria-label="Open tab list" onClick={() => setDrawerOpen(true)} edge="start">
              <MenuIcon />
            </IconButton>
          )}
          {!narrow && (
            <Typography variant="subtitle1" component="h1" sx={{ fontWeight: 700, mr: 1, whiteSpace: 'nowrap' }}>
              {APP_TITLE}
            </Typography>
          )}
          <Tabs
            value={tabs.length > 0 ? Math.min(activeTab, tabs.length - 1) : false}
            onChange={(_e, i: number) => selectTab(i)}
            variant="scrollable"
            scrollButtons="auto"
            allowScrollButtonsMobile
            aria-label="Dashboard tabs"
            sx={{ flex: 1, minWidth: 0 }}
          >
            {tabs.map((t) => (
              <Tab
                key={t.id}
                label={t.label}
                icon={narrow ? undefined : tabIcon(t.type)}
                iconPosition="start"
                sx={{ px: { xs: 1.5, sm: 2 }, minWidth: { xs: 72, sm: 90 } }}
              />
            ))}
          </Tabs>
          {batteryCfg && <BatteryChip status={battery} />}
          {gpsCfg?.rover_topic && <GpsChip status={roverChip} role="robot" />}
          {gpsCfg?.base_url && <GpsChip status={baseChip} role="base" />}
        </Toolbar>
      </AppBar>
      {banner && (
        <Alert severity="error" variant="filled" square role="alert">
          {banner}
        </Alert>
      )}

      <Drawer anchor="left" open={drawerOpen} onClose={() => setDrawerOpen(false)}>
        <List
          sx={{ width: 260, maxWidth: '85vw' }}
          subheader={<ListSubheader sx={{ bgcolor: 'background.paper' }}>{APP_TITLE} tabs</ListSubheader>}
        >
          {tabs.map((t, i) => (
            <ListItemButton key={t.id} selected={i === activeTab} onClick={() => selectTab(i)} sx={{ minHeight: 48 }}>
              <ListItemIcon>{tabIcon(t.type)}</ListItemIcon>
              <ListItemText primary={t.label} />
            </ListItemButton>
          ))}
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
          {tab && renderTab(tab, topicData, publish, battery)}
        </Suspense>
      </Box>

      <Snackbar
        open={errorToast !== null}
        autoHideDuration={ERROR_TOAST_MS}
        onClose={(_e, reason) => reason !== 'clickaway' && setErrorToast(null)}
        anchorOrigin={{ vertical: 'bottom', horizontal: 'center' }}
      >
        <Alert severity="error" variant="filled" onClose={() => setErrorToast(null)} sx={{ width: '100%' }}>
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
