import Alert from '@mui/material/Alert'
import type { TiltStatus } from '../tilt/tiltStatus'

/** Full-width banner under the AppBar: amber at tilt warning, red at tilt error, nothing otherwise. */
export function TiltBanner({ status }: { status: TiltStatus }) {
  if (!status.banner) return null
  return (
    <Alert severity={status.banner.severity} variant="filled" square role="alert">
      {status.banner.text}
    </Alert>
  )
}
