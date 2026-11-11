import type { ReactNode } from 'react'
import Alert from '@mui/material/Alert'
import Box from '@mui/material/Box'
import Chip from '@mui/material/Chip'
import Typography from '@mui/material/Typography'

/** One step of a calibration flow: one action (children) and what the user should see after it. */
export interface FlowStep {
  id: string
  title: string
  /** What the user should see once the step worked. */
  see: string
  done: boolean
  /** An optional step never locks the steps after it. */
  optional?: boolean
  /** Never locked, whatever the steps before it say (e.g. Home while the arm holds the board). */
  alwaysOpen?: boolean
  /** Live status line of the step. */
  status?: ReactNode
  /** Last error of the step. */
  error?: string | null
  children: ReactNode
}

/**
 * Number of steps open to the user: every step up to and including the first required one that is not done.
 *
 * @param steps - the flow's steps in order
 * @returns count of unlocked steps from the start
 */
export function unlockedCount(steps: FlowStep[]): number {
  const blocker = steps.findIndex((s) => !s.done && !s.optional)
  return blocker === -1 ? steps.length : blocker + 1
}

/** Label and colour of a step's chip. */
function chipOf(s: FlowStep, unlocked: boolean): { label: string; color: 'success' | 'primary' | 'default' } {
  if (s.done) return { label: 'Done', color: 'success' }
  if (!unlocked) return { label: 'Locked', color: 'default' }
  return s.optional ? { label: 'Optional', color: 'default' } : { label: 'Current', color: 'primary' }
}

/**
 * Numbered steps of a calibration flow. Steps unlock in order; a locked step shows only its title.
 *
 * @param label - accessible name of the step list
 * @param steps - the steps in order
 */
export function FlowStepper({ label, steps }: { label: string; steps: FlowStep[] }) {
  const open = unlockedCount(steps)
  const blocker = steps.findIndex((s) => !s.done && !s.optional)
  return (
    <Box component="ol" aria-label={label} sx={{ listStyle: 'none', pl: 0, m: 0 }}>
      {steps.map((s, i) => {
        const unlocked = i < open || s.alwaysOpen === true
        const chip = chipOf(s, unlocked)
        return (
          <Box component="li" key={s.id} sx={{ border: 1, borderColor: 'divider', borderRadius: 1, p: 2, mb: 1 }}>
            <Box sx={{ display: 'flex', alignItems: 'center', gap: 1, flexWrap: 'wrap' }}>
              <Typography variant="subtitle1" component="h3">{`${i + 1}. ${s.title}`}</Typography>
              <Chip size="small" label={chip.label} color={chip.color} variant={chip.color === 'default' ? 'outlined' : 'filled'} />
            </Box>
            {unlocked ? (
              <>
                <Typography variant="body2" color="text.secondary">{`What you should see: ${s.see}`}</Typography>
                {s.error && (
                  <Alert severity="error" sx={{ mt: 1 }}>
                    {s.error}
                  </Alert>
                )}
                {s.status && <Box sx={{ mt: 1 }}>{s.status}</Box>}
                <Box sx={{ mt: 1 }}>{s.children}</Box>
              </>
            ) : (
              <Typography variant="body2" color="text.secondary">{`Finish step ${blocker + 1} first.`}</Typography>
            )}
          </Box>
        )
      })}
    </Box>
  )
}
