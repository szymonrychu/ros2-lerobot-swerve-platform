import { useEffect, useState } from 'react'
import Box from '@mui/material/Box'
import { CANVAS_BG } from '../theme'
import { TabConfig } from '../types'
import { WaitingMessage } from './WaitingMessage'

interface Props {
  tab: TabConfig
  topicData: Record<string, unknown>
  publish: (topic: string, msgType: string, data: unknown) => void
}

interface ImageData {
  jpeg_b64: string | null
  error?: string
}

export default function CameraTab({ tab, topicData }: Props) {
  const [src, setSrc] = useState<string | null>(null)

  useEffect(() => {
    if (!tab.topic) return
    const data = topicData[tab.topic] as ImageData | undefined
    if (data?.jpeg_b64) {
      setSrc(`data:image/jpeg;base64,${data.jpeg_b64}`)
    }
  }, [topicData, tab.topic])

  if (!src) {
    return <WaitingMessage>Waiting for {tab.topic}…</WaitingMessage>
  }

  return (
    <Box sx={{ width: '100%', height: '100%', display: 'flex', alignItems: 'center', justifyContent: 'center', bgcolor: CANVAS_BG }}>
      <img
        src={src}
        alt="camera feed"
        style={{ maxWidth: '100%', maxHeight: '100%', objectFit: 'contain' }}
      />
    </Box>
  )
}
