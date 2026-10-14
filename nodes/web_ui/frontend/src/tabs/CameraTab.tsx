import Box from '@mui/material/Box'
import Typography from '@mui/material/Typography'
import { CameraStream } from '../components/CameraStream'
import { useLayout } from '../layout/useLayout'
import { TabConfig } from '../types'

interface Props {
  tab: TabConfig
  topicData: Record<string, unknown>
  publish: (topic: string, msgType: string, data: unknown) => void
}

const CELL_SX = { flex: '1 1 0', minWidth: 0, minHeight: 0, overflow: 'hidden', display: 'flex', flexDirection: 'column' } as const

/**
 * Camera tab: the one stream of `tab.topic`, or, when the tab lists `topics`, all of them in one grid (side by side on
 * tablet and desktop, stacked on a phone), each under its `label`.
 *
 * @param tab - tab config (topic, or topics with optional labels)
 * @param topicData - latest payload per topic
 */
export default function CameraTab({ tab, topicData }: Props) {
  const layout = useLayout()
  const streams = tab.topics ?? []
  if (streams.length === 0) {
    return <CameraStream topic={tab.topic} data={tab.topic ? topicData[tab.topic] : undefined} />
  }
  const direction = layout === 'phone' ? 'column' : 'row'
  const cellMinHeight = layout === 'phone' ? 'clamp(10dvh, 28dvh, 40dvh)' : 0
  return (
    <Box
      data-testid="camera-grid"
      data-direction={direction}
      sx={{ width: '100%', height: '100%', display: 'flex', flexDirection: direction, gap: 1, overflowY: layout === 'phone' ? 'auto' : 'hidden' }}
    >
      {streams.map((stream) => (
        <Box key={stream.topic} sx={{ ...CELL_SX, minHeight: cellMinHeight }}>
          {stream.label && (
            <Typography variant="body2" color="text.secondary" sx={{ flex: '0 0 auto', px: 1 }}>
              {stream.label}
            </Typography>
          )}
          <Box sx={{ flex: '1 1 0', minHeight: 0 }}>
            <CameraStream topic={stream.topic} data={topicData[stream.topic]} alt={stream.label ?? stream.topic} />
          </Box>
        </Box>
      ))}
    </Box>
  )
}
