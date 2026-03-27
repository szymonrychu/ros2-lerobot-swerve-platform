import { useEffect, useState, Suspense } from 'react'
import { Canvas } from '@react-three/fiber'
import { OrbitControls } from '@react-three/drei'
import { RobotModel } from '../components3d/RobotModel'
import { TabConfig } from '../types'
import log from '../logging'

interface MeshStatus {
  [path: string]: 'ok' | 'missing'
}

interface UrdfFileStatus {
  name: string
  path: string
  status: 'ok' | 'parse_error' | 'missing'
  link_count: number
  joint_count: number
  has_meshes: boolean
  mesh_status: MeshStatus
  error?: string
}

interface UrdfStatusResponse {
  files: UrdfFileStatus[]
}

interface Props {
  tab: TabConfig
  topicData: Record<string, unknown>
  publish: (topic: string, msgType: string, data: unknown) => void
}

export default function RobotStatusTab({ tab, topicData }: Props) {
  const [urdfStatus, setUrdfStatus] = useState<UrdfFileStatus[]>([])
  const [loading, setLoading] = useState(true)

  useEffect(() => {
    fetch('/api/urdf/status')
      .then((r) => r.json())
      .then((d: UrdfStatusResponse) => {
        setUrdfStatus(d.files)
        setLoading(false)
      })
      .catch((e) => {
        log.warn('[status] Failed to fetch URDF status:', e)
        setLoading(false)
      })
  }, [])

  const jointStates = tab.topic
    ? (topicData[tab.topic] as { name?: string[]; position?: number[] } | undefined)
    : undefined

  const armJointStates = tab.arm_joint_topic
    ? (topicData[tab.arm_joint_topic] as { name?: string[]; position?: number[] } | undefined)
    : undefined

  const statusColor = (s: string) => ({ ok: '#4a4', parse_error: '#c44', missing: '#664' }[s] ?? '#888')

  return (
    <div style={{ display: 'flex', height: '100%', gap: 0 }}>
      <div style={{ width: 380, padding: 20, overflowY: 'auto', borderRight: '1px solid #222', flexShrink: 0 }}>
        <div style={{ color: '#888', fontSize: 11, textTransform: 'uppercase', letterSpacing: '0.1em', marginBottom: 16 }}>
          URDF Status
        </div>
        {loading && <div style={{ color: '#555' }}>Loading…</div>}
        {urdfStatus.map((f) => (
          <div key={f.name} style={{ marginBottom: 16, padding: 12, border: '1px solid #222', background: '#0a0a0a' }}>
            <div style={{ display: 'flex', justifyContent: 'space-between', marginBottom: 8 }}>
              <span style={{ color: '#ccc', fontWeight: 'bold' }}>{f.name}</span>
              <span style={{ color: statusColor(f.status), fontSize: 11 }}>{f.status.toUpperCase()}</span>
            </div>
            {f.status === 'ok' && (
              <div style={{ color: '#666', fontSize: 12, lineHeight: 1.8 }}>
                <div>Links: {f.link_count} · Joints: {f.joint_count}</div>
                {f.has_meshes && (
                  <div>
                    Meshes:{' '}
                    {Object.entries(f.mesh_status).map(([path, stat]) => (
                      <span key={path} style={{ color: stat === 'ok' ? '#4a4' : '#c44', marginRight: 6, fontSize: 11 }}>
                        {path.split('/').pop()} {stat === 'ok' ? '✓' : '✗'}
                      </span>
                    ))}
                  </div>
                )}
              </div>
            )}
            {f.error && <div style={{ color: '#c44', fontSize: 11, marginTop: 4 }}>{f.error}</div>}
          </div>
        ))}

        {jointStates?.name && (
          <div style={{ marginTop: 16 }}>
            <div style={{ color: '#888', fontSize: 11, textTransform: 'uppercase', letterSpacing: '0.1em', marginBottom: 8 }}>
              Live Joint States
            </div>
            <table style={{ width: '100%', borderCollapse: 'collapse', fontSize: 12 }}>
              <thead>
                <tr>
                  <th style={{ color: '#555', textAlign: 'left', padding: '4px 8px' }}>Joint</th>
                  <th style={{ color: '#555', textAlign: 'right', padding: '4px 8px' }}>Position (rad)</th>
                </tr>
              </thead>
              <tbody>
                {jointStates.name.map((name, i) => (
                  <tr key={name} style={{ borderTop: '1px solid #1a1a1a' }}>
                    <td style={{ color: '#888', padding: '4px 8px' }}>{name}</td>
                    <td style={{ color: '#ccc', textAlign: 'right', padding: '4px 8px' }}>
                      {jointStates.position?.[i]?.toFixed(3) ?? '—'}
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        )}
      </div>

      <div style={{ flex: 1 }}>
        <Canvas camera={{ position: [1, 1, 1.5], fov: 50 }} style={{ background: '#050505' }}>
          <ambientLight intensity={0.7} />
          <directionalLight position={[3, 5, 3]} intensity={1} />
          <Suspense fallback={null}>
            <RobotModel urdfFile={tab.urdf_file ?? 'robot.urdf'} jointStates={jointStates} />
          </Suspense>
          {tab.arm_urdf_file && (
            <Suspense fallback={null}>
              <RobotModel urdfFile={tab.arm_urdf_file} jointStates={armJointStates} />
            </Suspense>
          )}
          <OrbitControls />
        </Canvas>
      </div>
    </div>
  )
}
