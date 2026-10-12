import Alert from '@mui/material/Alert'
import type { ReactNode } from 'react'

export type BannerState = 'warn' | 'error'

const SEVERITY = { warn: 'warning', error: 'error' } as const

/**
 * The one full-width strip announcing a problem: standard outlined Alert with a 3 px left border in the state colour.
 * @param state - 'warn' (amber) or 'error' (red)
 * @param children - the message
 * @returns the banner element
 */
export function StateBanner({ state, children }: { state: BannerState; children: ReactNode }) {
  const severity = SEVERITY[state]
  return (
    <Alert
      severity={severity}
      variant="outlined"
      square
      role="alert"
      sx={{ width: '100%', borderLeft: (t) => `3px solid ${t.palette[severity].main}` }}
    >
      {children}
    </Alert>
  )
}
