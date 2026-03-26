import { useEffect, useState } from 'react'
import { TabConfig } from '../types'

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
    return (
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'center', height: '100%', color: '#444' }}>
        Waiting for {tab.topic}…
      </div>
    )
  }

  return (
    <div style={{ width: '100%', height: '100%', display: 'flex', alignItems: 'center', justifyContent: 'center', background: '#000' }}>
      <img
        src={src}
        alt="camera feed"
        style={{ maxWidth: '100%', maxHeight: '100%', objectFit: 'contain' }}
      />
    </div>
  )
}
