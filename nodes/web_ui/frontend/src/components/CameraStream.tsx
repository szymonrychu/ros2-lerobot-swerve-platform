import { useEffect, useState } from 'react'
import Box from '@mui/material/Box'
import { CANVAS_BG } from '../theme'
import { WaitingMessage } from '../tabs/WaitingMessage'

interface Props {
  /** ROS image topic name (shown in the waiting message). */
  topic: string | undefined
  /** Latest topic payload for that topic ({ jpeg_b64 } once a frame arrived). */
  data: unknown
  /** Alt text of the rendered image. */
  alt?: string
}

interface ImageData {
  jpeg_b64: string | null
  error?: string
}

/**
 * Renders the latest JPEG frame of a camera topic, keeping the last frame when a
 * later message carries none. Shows the waiting message until a real frame arrives.
 *
 * @param topic - image topic name
 * @param data - topicData entry of that topic
 * @param alt - image alt text
 */
export function CameraStream({ topic, data, alt = 'camera feed' }: Props) {
  const [src, setSrc] = useState<string | null>(null)
  const jpeg = (data as ImageData | undefined)?.jpeg_b64

  useEffect(() => {
    if (jpeg) setSrc(`data:image/jpeg;base64,${jpeg}`)
  }, [jpeg])

  if (!src) return <WaitingMessage>Waiting for {topic}…</WaitingMessage>

  return (
    <Box sx={{ width: '100%', height: '100%', display: 'flex', alignItems: 'center', justifyContent: 'center', bgcolor: CANVAS_BG }}>
      <img src={src} alt={alt} style={{ maxWidth: '100%', maxHeight: '100%', objectFit: 'contain' }} />
    </Box>
  )
}
