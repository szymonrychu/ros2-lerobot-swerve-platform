import { useState } from 'react'
import Alert from '@mui/material/Alert'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import { ConfirmDialog } from '../layout/ConfirmDialog'
import { TOUCH_TARGET_PX } from '../theme'
import { type BoardRecord, type HeadArmSequence, clearBoardRecord } from './headArmApi'

const SECONDS_PER_MINUTE = 60
const SECONDS_PER_HOUR = 3600
const SECONDS_PER_DAY = 86400
/** Ages below twice a unit are shown in the next smaller unit (90 s, 90 min, 48 h). */
const UNIT_SWITCH = 1.5
const HOURS_TO_DAYS = 48

export const CLEAR_BUTTON = 'The board is already off the gripper: clear the record'
export const CLEAR_TITLE = 'Clear the board record?'
export const CLEAR_BODY =
  "Only when the calibration board is already off the gripper. Nothing moves: the node removes its record of the board on the gripper, so Home arm is no longer refused for it and the Arm to camera flow's Home no longer carries the board. If the board is still on the gripper, cancel and use the flow's Home with the board instead."
export const HOME_REFUSED_TEXT = 'Home arm is refused while the record exists.'

/**
 * Short text of an age in seconds: "45 s", "12 min", "5 h", "3 d".
 *
 * @param seconds - age in seconds (>= 0)
 * @returns the age with its unit
 */
export function formatAge(seconds: number): string {
  if (seconds < UNIT_SWITCH * SECONDS_PER_MINUTE) return `${Math.round(seconds)} s`
  if (seconds < UNIT_SWITCH * SECONDS_PER_HOUR) return `${Math.round(seconds / SECONDS_PER_MINUTE)} min`
  if (seconds < HOURS_TO_DAYS * SECONDS_PER_HOUR) return `${Math.round(seconds / SECONDS_PER_HOUR)} h`
  return `${Math.round(seconds / SECONDS_PER_DAY)} d`
}

/**
 * One line naming a board record: the session, the attach time and age, or why it is unreadable.
 *
 * @param record - the node's board record
 * @returns the description
 */
export function boardRecordText(record: BoardRecord): string {
  if (record.clipped_at === null) return `The node has a board record it cannot read (${record.problem ?? 'unreadable'}).`
  const session = record.session_id === null ? 'without a capture session' : `in session ${record.session_id}`
  const age = record.age_s === null ? '' : `, ${formatAge(record.age_s)} ago`
  return `The node records the calibration board on the gripper: attached ${session} at ${record.clipped_at}${age}.`
}

interface Props {
  /** The head-arm sequence's board record (null: none, nothing is shown). */
  record: BoardRecord | null
  /** A head-arm sequence or Home runs: the clear is off (the node refuses it then). */
  running: boolean
  /** What to do when the board is still on (where the Home with the board is). */
  homeHint: string
  /** Called with the sequence the node answers after a clear. */
  onCleared?: (sequence: HeadArmSequence) => void
}

/**
 * The board record banner of the Calibrate page (Arm to camera flow and General section): shown whenever the node has
 * a board record, whatever its age, with "The board is already off the gripper: clear the record" behind a confirm.
 * The clear moves nothing; the flow's Home with the board stays the way when the board is still on.
 */
export function BoardRecordNotice({ record, running, homeHint, onCleared }: Props) {
  const [open, setOpen] = useState(false)
  const [busy, setBusy] = useState(false)
  const [error, setError] = useState<string | null>(null)
  if (record === null) return null

  const clear = async () => {
    setOpen(false)
    setBusy(true)
    const r = await clearBoardRecord()
    setBusy(false)
    if (r.ok) {
      setError(null)
      onCleared?.(r.value)
    } else setError(r.message)
  }

  return (
    <Box aria-label="Board record" component="section" sx={{ mt: 1 }}>
      <Alert severity="warning">
        {`${boardRecordText(record)} ${HOME_REFUSED_TEXT} Board still on the gripper: ${homeHint} Board already off: clear the record.`}
      </Alert>
      <Button variant="outlined" sx={{ minHeight: TOUCH_TARGET_PX, mt: 1 }} disabled={busy || running} onClick={() => setOpen(true)}>
        {CLEAR_BUTTON}
      </Button>
      {error && (
        <Alert severity="error" sx={{ mt: 1 }} onClose={() => setError(null)}>
          {error}
        </Alert>
      )}
      <ConfirmDialog
        open={open}
        title={CLEAR_TITLE}
        body={CLEAR_BODY}
        confirmLabel="Clear the record"
        onCancel={() => setOpen(false)}
        onConfirm={() => void clear()}
      />
    </Box>
  )
}
