import Box from '@mui/material/Box'
import Checkbox from '@mui/material/Checkbox'
import Divider from '@mui/material/Divider'
import FormControlLabel from '@mui/material/FormControlLabel'
import ToggleButton from '@mui/material/ToggleButton'
import ToggleButtonGroup from '@mui/material/ToggleButtonGroup'
import Tooltip from '@mui/material/Tooltip'
import Typography from '@mui/material/Typography'
import InfoOutlinedIcon from '@mui/icons-material/InfoOutlined'
import { ToolButton } from '../components/ToolButton'
import { OverlayPanel } from '../layout/OverlayPanel'
import { LAYER_LABELS, type LayerKey, type LayerState } from '../map3d/layers'
import { LAYER_SWATCH, MAP_LEGEND } from '../map3d/palette'
import type { TileSourceInfo } from '../map3d/tileSources'
import { LAYER_MAX_PHOTOS, PHOTO_AGE_LABELS, PHOTO_AGES, type PhotoAge } from '../photos/geometry'
import { PoiLegend } from '../poi/PoiLegend'
import { TOKENS } from '../theme'
import type { Map3dStatusPayload } from '../types'
import { Map3dStatus, map3dSummary } from './Map3dStatus'

const GPS_ANCHOR_TOPIC = '/web_ui/gps_anchor'
const SWATCH_REM = '0.875rem'
const TOGGLE_SX = { py: 0, px: 1, textTransform: 'none' } as const

const LAYER_GROUPS: { title: string; keys: LayerKey[] }[] = [
  { title: 'Maps', keys: ['slamMap', 'localCostmap', 'gpsMap', 'rtkBase', 'obstacles3d', 'map3dCloud', 'headRgbd', 'lidarWalls'] },
  { title: 'Navigation', keys: ['globalPlan', 'localPlan', 'goal', 'footprint'] },
  { title: 'Robot model', keys: ['robotBase', 'robotWheels', 'robotArm', 'head'] },
  { title: 'Points of interest', keys: ['pois', 'photos'] },
]

export interface LayersPanelProps {
  open: boolean
  onClose: () => void
  layers: LayerState
  onToggle: (key: LayerKey) => void
  /** False until the GPS anchor exists; disables the GPS map. */
  gpsAvailable: boolean
  /** False without an anchor and a fresh base station position; disables the RTK base. */
  rtkAvailable: boolean
  /** Whether the tab names a local costmap topic. */
  costmapConfigured: boolean
  /** Whether the tab names both the obstacle cloud and the scan points topic (absent = yes). */
  obstaclesConfigured?: boolean
  /** Whether the tab names rgbd_cloud_topic, the head RGB-D cloud (absent = yes). */
  rgbdCloudConfigured?: boolean
  tileSources: readonly TileSourceInfo[]
  tileSource: TileSourceInfo | null
  onTileSource: (id: string) => void
  /** Warning about the current tile source (e.g. offline), or null. */
  tileCaption: string | null
  /** One-line summary of the GPS anchor, or null without one. */
  anchorSummary: string | null
  photoAge: PhotoAge
  onPhotoAge: (age: PhotoAge) => void
  photoStatus: { shown: number; total: number; error: string | null }
  topView: boolean
  goalMode: boolean
  /** The tab has an arm command topic. */
  hasArm: boolean
  armReady: boolean
  /** Latest /map3d/status JSON; undefined/null before one arrived. */
  map3dStatus?: Map3dStatusPayload | null
}

/**
 * Reason a layer cannot be switched on, shown as the row tooltip.
 *
 * @param key - layer key
 * @param p - panel props
 * @returns the reason, or null when the layer is usable
 */
export function disabledReason(
  key: LayerKey,
  p: Pick<LayersPanelProps, 'gpsAvailable' | 'rtkAvailable' | 'costmapConfigured' | 'obstaclesConfigured' | 'rgbdCloudConfigured'>,
): string | null {
  if (key === 'gpsMap' && !p.gpsAvailable) return `Needs a GPS fix and compass heading (or drive ~5 m to fit): waiting for ${GPS_ANCHOR_TOPIC}.`
  if (key === 'rtkBase' && !p.rtkAvailable) return 'Needs the GPS anchor and a fresh base station position.'
  if (key === 'localCostmap' && !p.costmapConfigured) return 'No costmap topic configured.'
  if (key === 'obstacles3d' && p.obstaclesConfigured === false) return 'No obstacle cloud or scan points topic configured.'
  if (key === 'headRgbd' && p.rgbdCloudConfigured === false) return 'No head RGB-D cloud topic configured.'
  return null
}

function Swatch({ id, color }: { id: string; color: string }) {
  return (
    <Box
      component="span"
      data-testid={`layer-swatch-${id}`}
      data-color={color}
      sx={{ width: SWATCH_REM, height: SWATCH_REM, bgcolor: color, border: 1, borderColor: 'divider', flexShrink: 0, display: 'inline-block' }}
    />
  )
}

/**
 * Gesture and arm help: shown behind the header info button instead of as paragraphs.
 *
 * @param p - topView, goalMode, hasArm and armReady
 * @returns the help lines
 */
function Help({ topView, goalMode, hasArm, armReady }: Pick<LayersPanelProps, 'topView' | 'goalMode' | 'hasArm' | 'armReady'>) {
  const gesture = topView
    ? goalMode
      ? 'Press to place the goal, drag to set heading. Two fingers / right button pan, wheel or pinch zooms.'
      : 'Top view: drag to pan, wheel or pinch to zoom.'
    : 'Orbit: drag to rotate, right button / two fingers pan, wheel or pinch zooms.'
  return (
    <Box>
      <Box>{gesture}</Box>
      {hasArm && <Box>{armReady ? 'Drag arm rings/handles to move the arm' : 'Waiting for arm servo positions...'}</Box>}
    </Box>
  )
}

/**
 * The map layers as a short list of toggles, with the map and marker legends.
 *
 * @param p - see LayersPanelProps
 * @returns the panel, docked in the sidebar on tablet and desktop
 */
export function LayersPanel(p: LayersPanelProps) {
  const { layers, tileSources, tileSource } = p
  const help = <Help topView={p.topView} goalMode={p.goalMode} hasArm={p.hasArm} armReady={p.armReady} />
  return (
    <OverlayPanel
      title="Layers"
      open={p.open}
      anchor="right"
      onClose={p.onClose}
      actions={
        <ToolButton label="Map help" tooltip={help} icon={<InfoOutlinedIcon fontSize="small" />} onClick={() => undefined} />
      }
    >
      {LAYER_GROUPS.map((g) => (
        <Box key={g.title} sx={{ mb: 1 }}>
          <Typography variant="overline" color="text.secondary" sx={{ lineHeight: 2 }}>
            {g.title}
          </Typography>
          {g.keys.map((k) => {
            const reason = disabledReason(k, p)
            const swatch = LAYER_SWATCH[k]
            return (
              <Box key={k}>
                <Tooltip title={reason ?? ''} disableHoverListener={reason === null}>
                  <Box data-testid={`layer-row-${k}`}>
                    <FormControlLabel
                      sx={{ display: 'flex', mr: 0, minHeight: '2.5rem' }}
                      disabled={reason !== null}
                      control={
                        <Checkbox
                          size="small"
                          color="default"
                          checked={layers[k] && reason === null}
                          onChange={() => p.onToggle(k)}
                          sx={{ '&.Mui-checked': { color: TOKENS.accent } }}
                        />
                      }
                      label={
                        <Box component="span" sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                          {swatch && <Swatch id={k} color={swatch} />}
                          <Typography variant="body2" component="span">
                            {LAYER_LABELS[k]}
                          </Typography>
                        </Box>
                      }
                    />
                    {k === 'map3dCloud' && (
                      <Typography variant="caption" color="text.secondary" sx={{ display: 'block', pl: 4, mt: -1 }}>
                        {map3dSummary(p.map3dStatus)}
                      </Typography>
                    )}
                  </Box>
                </Tooltip>
                {k === 'map3dCloud' && layers.map3dCloud && p.map3dStatus && (
                  <Box sx={{ pl: 4, mb: 0.5 }}>
                    <Map3dStatus status={p.map3dStatus} />
                  </Box>
                )}
                {k === 'gpsMap' && tileSources.length > 1 && tileSource && (
                  <ToggleButtonGroup
                    exclusive
                    size="small"
                    value={tileSource.id}
                    onChange={(_, id: string | null) => id !== null && p.onTileSource(id)}
                    aria-label="Map tile source"
                    sx={{ mb: 0.5 }}
                  >
                    {tileSources.map((src) => (
                      <ToggleButton key={src.id} value={src.id} sx={TOGGLE_SX}>
                        {src.label}
                      </ToggleButton>
                    ))}
                  </ToggleButtonGroup>
                )}
                {k === 'gpsMap' && layers.gpsMap && p.tileCaption && (
                  <Typography variant="caption" color="warning.main" sx={{ display: 'block' }}>
                    {p.tileCaption}
                  </Typography>
                )}
                {k === 'gpsMap' && p.anchorSummary && (
                  <Typography variant="caption" color="text.secondary" sx={{ display: 'block' }}>
                    Anchor: {p.anchorSummary}
                  </Typography>
                )}
                {k === 'photos' && layers.photos && (
                  <Box sx={{ mb: 0.5 }}>
                    <ToggleButtonGroup
                      exclusive
                      size="small"
                      value={p.photoAge}
                      onChange={(_, v: PhotoAge | null) => v !== null && p.onPhotoAge(v)}
                      aria-label="Photo age filter"
                    >
                      {PHOTO_AGES.map((a) => (
                        <ToggleButton key={a} value={a} sx={TOGGLE_SX}>
                          {PHOTO_AGE_LABELS[a]}
                        </ToggleButton>
                      ))}
                    </ToggleButtonGroup>
                    <Typography variant="caption" color={p.photoStatus.error ? 'error' : 'text.secondary'} sx={{ display: 'block' }}>
                      {p.photoStatus.error
                        ? `Could not load photos: ${p.photoStatus.error}`
                        : `showing ${p.photoStatus.shown} of ${p.photoStatus.total}${p.photoStatus.total > LAYER_MAX_PHOTOS ? ` (newest ${LAYER_MAX_PHOTOS} max)` : ''}`}
                    </Typography>
                  </Box>
                )}
              </Box>
            )
          })}
          {g.title === 'Points of interest' && <PoiLegend />}
        </Box>
      ))}
      <Divider sx={{ my: 1 }} />
      <Typography variant="overline" color="text.secondary" sx={{ lineHeight: 2 }}>
        Legend
      </Typography>
      {MAP_LEGEND.map(([label, color]) => (
        <Box key={label} sx={{ display: 'flex', alignItems: 'center', gap: 1, lineHeight: '1.375rem' }}>
          <Swatch id={label} color={color} />
          <Typography variant="body2">{label}</Typography>
        </Box>
      ))}
    </OverlayPanel>
  )
}
