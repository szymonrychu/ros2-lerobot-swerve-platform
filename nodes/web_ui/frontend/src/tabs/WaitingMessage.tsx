import type { ReactNode } from 'react'
import Box from '@mui/material/Box'
import Typography from '@mui/material/Typography'

/**
 * Centered, themed placeholder shown while a tab has no real data yet (nothing is drawn instead).
 *
 * @param children - message text
 */
export function WaitingMessage({ children }: { children: ReactNode }) {
  return (
    <Box sx={{ width: '100%', height: '100%', display: 'flex', alignItems: 'center', justifyContent: 'center', p: 2 }}>
      <Typography variant="body2" color="text.secondary" align="center" sx={{ overflowWrap: 'anywhere' }}>
        {children}
      </Typography>
    </Box>
  )
}
