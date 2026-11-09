import Alert from '@mui/material/Alert'
import AlertTitle from '@mui/material/AlertTitle'
import Box from '@mui/material/Box'
import Typography from '@mui/material/Typography'
import type { HeadArmEvent, HeadArmSequence } from './headArmApi'

/** State poll period of the head-arm sequence while it runs: 1 Hz. */
export const ARM_POLL_MS = 1000
/** Height of the events log before it scrolls. */
const LOG_MAX_HEIGHT_PX = 200
const LEVEL_ERROR = 'error'
const LEVEL_COLOUR: Record<string, string> = { error: 'error.main', warning: 'warning.main' }

/**
 * Local clock time of an event stamp, or the stamp itself when it does not parse.
 *
 * @param time - ISO 8601 event time
 * @returns the local time text
 */
export function eventTime(time: string): string {
  const date = new Date(time)
  return Number.isNaN(date.getTime()) ? time : date.toLocaleTimeString()
}

/**
 * The head-arm sequence activity: current status, the last error in a banner and the recent events (newest first,
 * scrollable). The node logs every event too (journal of head_rgbd_calibration).
 *
 * @param sequence - the sequence state of GET /head-arm/state
 * @param withStatus - show the status line; false where the caller's progress line already carries the status
 */
export function SequenceActivity({ sequence, withStatus = true }: { sequence: HeadArmSequence; withStatus?: boolean }) {
  const newestError: HeadArmEvent | undefined = sequence.events.find((e) => e.level === LEVEL_ERROR)
  return (
    <Box sx={{ mt: 1 }}>
      {withStatus && <Typography variant="body2">{`Status (${sequence.phase}): ${sequence.status}`}</Typography>}
      {sequence.error && (
        <Alert severity="error" sx={{ mt: 1 }}>
          <AlertTitle>Last head-arm error</AlertTitle>
          {sequence.error}
          {newestError && (
            <Typography variant="caption" component="div">
              {`at ${eventTime(newestError.time)}, phase ${newestError.phase}`}
            </Typography>
          )}
        </Alert>
      )}
      {sequence.events.length === 0 ? (
        <Typography variant="body2" color="text.secondary" sx={{ mt: 1 }}>
          No head-arm events yet
        </Typography>
      ) : (
        <Box
          component="ul"
          aria-label="Recent head-arm events"
          sx={{
            mt: 1,
            mb: 0,
            pl: 0,
            listStyle: 'none',
            maxHeight: LOG_MAX_HEIGHT_PX,
            overflowY: 'auto',
            border: 1,
            borderColor: 'divider',
            borderRadius: 1,
            p: 1,
          }}
        >
          {sequence.events.map((e, i) => (
            <Typography
              key={`${e.time}-${i}`}
              component="li"
              variant="body2"
              sx={{ color: LEVEL_COLOUR[e.level] ?? 'text.primary', fontFamily: 'monospace', wordBreak: 'break-word' }}
            >
              {`${eventTime(e.time)} ${e.level.toUpperCase()} [${e.phase}] ${e.message}`}
            </Typography>
          ))}
        </Box>
      )}
    </Box>
  )
}

/**
 * One line per sequencer skip reason with its count (for example "inside D435 min-Z: 3", "outside colour FOV: 2").
 *
 * @param skipped - skip reason -> count of the sequence
 */
export function SkipReasons({ skipped }: { skipped: Record<string, number> }) {
  return (
    <>
      {Object.entries(skipped).map(([reason, n]) => (
        <Typography key={reason} variant="body2" color="text.secondary">
          {`${reason}: ${n}`}
        </Typography>
      ))}
    </>
  )
}
