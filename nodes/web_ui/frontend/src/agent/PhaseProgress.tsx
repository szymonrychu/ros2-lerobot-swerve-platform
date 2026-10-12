/** Attempt / iteration / retry-estimate of a plan phase as one plain text line joined by " - " (empty for old plans that report none of them). */
import Box from '@mui/material/Box'
import { TOKENS } from '../theme'

/**
 * @param attemptLabel - e.g. "attempt 2/3", or null
 * @param iterationLabel - e.g. "iteration 4", or null
 * @param pRetrySuccess - retry success estimate (0..1), or null
 * @param lead - optional plain text placed first, e.g. "raised"
 * @returns the text line, or nothing when no part is known
 */
export function PhaseProgress({
  attemptLabel,
  iterationLabel,
  pRetrySuccess,
  lead,
}: {
  attemptLabel: string | null
  iterationLabel: string | null
  pRetrySuccess: number | null
  lead?: string
}) {
  const parts: { id: string; text: string }[] = []
  if (lead) parts.push({ id: 'phase-lead', text: lead })
  if (attemptLabel !== null) parts.push({ id: 'phase-attempt', text: attemptLabel })
  if (iterationLabel !== null) parts.push({ id: 'phase-iteration', text: iterationLabel })
  if (pRetrySuccess !== null) parts.push({ id: 'phase-p-retry', text: `p ${pRetrySuccess.toFixed(2)}` })
  if (parts.length === 0) return null
  return (
    <Box component="span" data-testid="phase-progress" sx={{ fontSize: TOKENS.fontSizes.xs, color: 'text.secondary', whiteSpace: 'nowrap' }}>
      {parts.map((p, i) => (
        <span key={p.id}>
          {i > 0 && ' - '}
          <span data-testid={p.id}>{p.text}</span>
        </span>
      ))}
    </Box>
  )
}
