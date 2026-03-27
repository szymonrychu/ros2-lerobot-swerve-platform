import { lazy, Suspense, useEffect, useState } from 'react'
import log from './logging'
import { useRosBridge } from './hooks/useRosBridge'
import { OverlayBar } from './overlays/OverlayBar'
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
    default: return <div style={{ padding: 20, color: '#555' }}>Unknown tab type: {tab.type}</div>
  }
}

export default function App() {
  const [config, setConfig] = useState<AppConfig | null>(null)
  const [activeTab, setActiveTab] = useState(0)

  useEffect(() => {
    fetch('/api/config')
      .then((r) => r.json())
      .then((data: AppConfig) => {
        log.info('[app] Config loaded —', data.tabs.length, 'tabs,', data.overlays.length, 'overlay items')
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
    return <div style={{ padding: 40, color: '#555' }}>Loading config…</div>
  }

  const tab = config.tabs[activeTab]

  return (
    <>
      <nav className="tab-bar">
        {config.tabs.map((t, i) => (
          <button
            key={t.id}
            className={`tab-btn${i === activeTab ? ' active' : ''}`}
            onClick={() => {
              log.info('[app] Tab activated:', t.label)
              setActiveTab(i)
            }}
          >
            {t.label}
          </button>
        ))}
      </nav>

      <main className="tab-content">
        <Suspense fallback={<div style={{ padding: 20, color: '#555' }}>Loading…</div>}>
          {tab && renderTab(tab, topicData, publish)}
        </Suspense>
      </main>

      <OverlayBar overlays={config.overlays} topicData={topicData} connected={connected} />
    </>
  )
}
