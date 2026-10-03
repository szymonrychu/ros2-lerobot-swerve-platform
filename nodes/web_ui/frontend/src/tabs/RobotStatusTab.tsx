import { useCallback, useEffect, useRef, useState, Suspense } from 'react'
import { Canvas } from '@react-three/fiber'
import { OrbitControls } from '@react-three/drei'
import type { OrbitControls as OrbitControlsImpl } from 'three-stdlib'
import { RobotModel } from '../components3d/RobotModel'
import { InteractiveArm } from '../components3d/InteractiveArm'
import { TabConfig } from '../types'
import Box from '@mui/material/Box'
import Card from '@mui/material/Card'
import CardContent from '@mui/material/CardContent'
import Checkbox from '@mui/material/Checkbox'
import Chip from '@mui/material/Chip'
import FormControlLabel from '@mui/material/FormControlLabel'
import Paper from '@mui/material/Paper'
import Stack from '@mui/material/Stack'
import Table from '@mui/material/Table'
import TableBody from '@mui/material/TableBody'
import TableCell from '@mui/material/TableCell'
import TableHead from '@mui/material/TableHead'
import TableRow from '@mui/material/TableRow'
import Typography from '@mui/material/Typography'
import log from '../logging'
import { CANVAS_BG, MONO_FONT } from '../theme'

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

const STATUS_COLOR: Record<UrdfFileStatus['status'], 'success' | 'error' | 'warning'> = {
  ok: 'success',
  parse_error: 'error',
  missing: 'warning',
}

interface Props {
  tab: TabConfig
  topicData: Record<string, unknown>
  publish: (topic: string, msgType: string, data: unknown) => void
}

export default function RobotStatusTab({ tab, topicData, publish }: Props) {
  const [urdfStatus, setUrdfStatus] = useState<UrdfFileStatus[]>([])
  const [loading, setLoading] = useState(true)
  const [armReady, setArmReady] = useState(false)
  const [hiddenUrdfs, setHiddenUrdfs] = useState<Set<string>>(new Set())
  const toggleUrdfVisibility = useCallback((name: string) => {
    setHiddenUrdfs((prev) => {
      const next = new Set(prev)
      if (next.has(name)) next.delete(name)
      else next.add(name)
      return next
    })
  }, [])
  const orbitRef = useRef<OrbitControlsImpl | null>(null)
  const onArmReady = useCallback((ready: boolean) => setArmReady(ready), [])

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

  return (
    <Stack direction={{ xs: 'column', md: 'row' }} sx={{ height: '100%', minHeight: 0 }}>
      <Paper
        square
        elevation={0}
        sx={{
          width: { xs: '100%', md: 380 },
          maxHeight: { xs: '45%', md: 'none' },
          flexShrink: 0,
          overflowY: 'auto',
          p: { xs: 1.5, sm: 2.5 },
          borderRight: { md: 1 },
          borderBottom: { xs: 1, md: 0 },
          borderColor: 'divider',
        }}
      >
        <Typography variant="overline" color="text.secondary" component="h2" sx={{ display: 'block', mb: 1 }}>
          URDF Status
        </Typography>
        {loading && <Typography color="text.secondary">Loading…</Typography>}
        <Stack spacing={1.5}>
          {urdfStatus.map((f) => (
            <Card key={f.name} variant="outlined">
              <CardContent sx={{ p: 1.5, '&:last-child': { pb: 1.5 } }}>
                <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', gap: 1 }}>
                  <FormControlLabel
                    sx={{ minWidth: 0, mr: 0 }}
                    control={
                      <Checkbox
                        color="success"
                        checked={!hiddenUrdfs.has(f.name)}
                        onChange={() => toggleUrdfVisibility(f.name)}
                      />
                    }
                    label={
                      <Typography sx={{ fontWeight: 700, overflowWrap: 'anywhere' }}>{f.name}</Typography>
                    }
                  />
                  <Chip
                    size="small"
                    variant="outlined"
                    color={STATUS_COLOR[f.status] ?? 'default'}
                    label={f.status.toUpperCase()}
                  />
                </Box>
                {f.status === 'ok' && (
                  <Box sx={{ color: 'text.secondary', fontSize: 13, lineHeight: 1.8 }}>
                    <div>Links: {f.link_count} · Joints: {f.joint_count}</div>
                    {f.has_meshes && (
                      <div>
                        Meshes:{' '}
                        {Object.entries(f.mesh_status).map(([path, stat]) => (
                          <Box
                            component="span"
                            key={path}
                            sx={{ color: stat === 'ok' ? 'success.main' : 'error.main', mr: 0.75, fontSize: 12 }}
                          >
                            {path.split('/').pop()} {stat === 'ok' ? '✓' : '✗'}
                          </Box>
                        ))}
                      </div>
                    )}
                  </Box>
                )}
                {f.error && (
                  <Typography variant="caption" color="error" sx={{ display: 'block', mt: 0.5 }}>
                    {f.error}
                  </Typography>
                )}
              </CardContent>
            </Card>
          ))}
        </Stack>

        {jointStates?.name && (
          <Box sx={{ mt: 2 }}>
            <Typography variant="overline" color="text.secondary" component="h2" sx={{ display: 'block' }}>
              Live Joint States
            </Typography>
            <Table size="small" aria-label="Live joint states">
              <TableHead>
                <TableRow>
                  <TableCell>Joint</TableCell>
                  <TableCell align="right">Position (rad)</TableCell>
                </TableRow>
              </TableHead>
              <TableBody>
                {jointStates.name.map((name, i) => (
                  <TableRow key={name}>
                    <TableCell sx={{ color: 'text.secondary', overflowWrap: 'anywhere' }}>{name}</TableCell>
                    <TableCell align="right" sx={{ fontFamily: MONO_FONT }}>
                      {jointStates.position?.[i]?.toFixed(3) ?? '—'}
                    </TableCell>
                  </TableRow>
                ))}
              </TableBody>
            </Table>
          </Box>
        )}

        {tab.arm_command_topic && (
          <Typography variant="body2" color={armReady ? 'success.main' : 'warning.main'} sx={{ mt: 2 }}>
            {armReady ? 'Drag arm joints to command position' : 'Waiting for servo positions…'}
          </Typography>
        )}
      </Paper>

      <Box sx={{ flex: 1, minHeight: 0, minWidth: 0, bgcolor: CANVAS_BG }}>
        <Canvas camera={{ position: [1, 1, 1.5], fov: 50 }} style={{ background: CANVAS_BG }}>
          <ambientLight intensity={0.7} />
          <directionalLight position={[3, 5, 3]} intensity={1} />
          {!hiddenUrdfs.has(tab.urdf_file ?? 'robot.urdf') && (
            <Suspense fallback={null}>
              <RobotModel urdfFile={tab.urdf_file ?? 'robot.urdf'} jointStates={jointStates} />
            </Suspense>
          )}
          {tab.arm_urdf_file && !hiddenUrdfs.has(tab.arm_urdf_file) && (
            tab.arm_command_topic ? (
              <Suspense fallback={null}>
                <InteractiveArm
                  urdfFile={tab.arm_urdf_file}
                  liveJointStates={armJointStates}
                  position={tab.arm_offset ?? [0.25, 0, 0]}
                  commandTopic={tab.arm_command_topic}
                  publish={publish}
                  orbitRef={orbitRef}
                  onReady={onArmReady}
                />
              </Suspense>
            ) : (
              <Suspense fallback={null}>
                <RobotModel
                  urdfFile={tab.arm_urdf_file}
                  jointStates={armJointStates}
                  position={tab.arm_offset ?? [0.25, 0, 0]}
                />
              </Suspense>
            )
          )}
          <OrbitControls ref={orbitRef} />
        </Canvas>
      </Box>
    </Stack>
  )
}
