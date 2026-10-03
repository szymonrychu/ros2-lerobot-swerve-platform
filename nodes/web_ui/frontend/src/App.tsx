import { lazy, Suspense, useEffect, useMemo, useState } from 'react'
import type { ReactElement, ReactNode } from 'react'
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
import RadarIcon from '@mui/icons-material/Radar'
import PublicIcon from '@mui/icons-material/Public'
import ViewInArIcon from '@mui/icons-material/ViewInAr'
import PrecisionManufacturingIcon from '@mui/icons-material/PrecisionManufacturing'
import CameraIcon from '@mui/icons-material/Camera'
import TabIcon from '@mui/icons-material/Tab'
import log from './logging'
import { useRosBridge } from './hooks/useRosBridge'
import { OverlayBar } from './overlays/OverlayBar'
import { browserStorage, initialTabIndex, orderTabs, readStoredTabId, writeStoredTabId } from './tabSelection'
import { AppConfig, TabConfig } from './types'
import { feedAllGraphBuffers } from './utils/graphBuffers'

const CameraTab = lazy(() => import('./tabs/CameraTab'))
const SensorGraphTab = lazy(() => import('./tabs/SensorGraphTab'))
const EffectorGraphTab = lazy(() => import('./tabs/EffectorGraphTab'))
const ImuOrientationTab = lazy(() => import('./tabs/ImuOrientationTab'))
const NavLocalTab = lazy(() => import('./tabs/NavLocalTab'))
const NavGpsTab = lazy(() => import('./tabs/NavGpsTab'))
const Scene3DTab = lazy(() => import('./tabs/Scene3DTab'))
const RobotStatusTab = lazy(() => import('./tabs/RobotStatusTab'))
const RgbdCameraTab = lazy(() => import('./tabs/RgbdCameraTab'))
const MapNavTab = lazy(() => import('./tabs/MapNavTab'))

const APP_TITLE = 'Robot'

const TAB_ICONS: Record<string, ReactElement> = {
  map_nav: <MapIcon />,
  camera: <VideocamIcon />,
  rgbd_camera: <CameraIcon />,
  sensor_graph: <ShowChartIcon />,
  effector_graph: <ShowChartIcon />,
  imu_orientation: <ThreeDRotationIcon />,
  nav_local: <RadarIcon />,
  nav_gps: <PublicIcon />,
  scene3d: <ViewInArIcon />,
  robot_status: <PrecisionManufacturingIcon />,
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

function renderTab(tab: TabConfig, topicData: Record<string, unknown>, publish: (t: string, mt: string, d: unknown) => void) {
  const props = { tab, topicData, publish }
  switch (tab.type) {
    case 'camera': return <CameraTab {...props} />
    case 'sensor_graph': return <SensorGraphTab {...props} />
    case 'effector_graph': return <EffectorGraphTab {...props} />
    case 'imu_orientation': return <ImuOrientationTab {...props} />
    case 'nav_local': return <NavLocalTab {...props} />
    case 'nav_gps': return <NavGpsTab {...props} />
    case 'scene3d': return <Scene3DTab {...props} />
    case 'robot_status': return <RobotStatusTab {...props} />
    case 'rgbd_camera': return <RgbdCameraTab {...props} />
    case 'map_nav': return <MapNavTab {...props} />
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

  // Map tabs first: the map is the primary view whatever order the config lists tabs in.
  const tabs = useMemo(() => (config ? orderTabs(config.tabs) : []), [config])

  useEffect(() => {
    fetch('/api/config')
      .then((r) => r.json())
      .then((data: AppConfig) => {
        log.info('[app] Config loaded —', data.tabs.length, 'tabs,', data.overlays.length, 'overlay items')
        setActiveTab(initialTabIndex(orderTabs(data.tabs), readStoredTabId(browserStorage())))
        setConfig(data)
      })
      .catch((e) => log.warn('[app] Failed to load config:', e))
  }, [])

  const allTopics = config
    ? [
        ...config.tabs.flatMap((t) => [
          t.topic,
          ...(t.topics?.map((ts) => ts.topic) ?? []),
          t.scan_topic, t.costmap_topic, t.odom_topic, t.fix_topic, t.arm_joint_topic,
          t.color_topic, t.depth_topic, t.camera_info_topic,
          t.map_topic, t.global_plan_topic, t.local_plan_topic, t.footprint_topic,
          ...(t.type === 'map_nav' ? [t.goal_topic] : []),
        ]),
        ...config.overlays.map((o) => o.topic),
      ].filter((t): t is string => Boolean(t))
    : []

  const { topicData, connected, publish } = useRosBridge(allTopics)

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
        </Toolbar>
      </AppBar>

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
          {tab && renderTab(tab, topicData, publish)}
        </Suspense>
      </Box>

      <OverlayBar overlays={config.overlays} topicData={topicData} connected={connected} />
    </>
  )
}
