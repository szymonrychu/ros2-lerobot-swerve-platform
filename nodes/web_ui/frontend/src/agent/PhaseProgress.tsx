/** Attempt / iteration / retry-estimate chips of a plan phase (empty for old plans that report none of them). */
import Chip from '@mui/material/Chip'

export function PhaseProgress({
  attemptLabel,
  iterationLabel,
  pRetrySuccess,
}: {
  attemptLabel: string | null
  iterationLabel: string | null
  pRetrySuccess: number | null
}) {
  if (attemptLabel === null && iterationLabel === null && pRetrySuccess === null) return null
  return (
    <>
      {attemptLabel !== null && <Chip size="small" variant="outlined" label={attemptLabel} data-testid="phase-attempt" />}
      {iterationLabel !== null && <Chip size="small" variant="outlined" label={iterationLabel} data-testid="phase-iteration" />}
      {pRetrySuccess !== null && <Chip size="small" variant="outlined" label={`p ${pRetrySuccess.toFixed(2)}`} data-testid="phase-p-retry" />}
    </>
  )
}
