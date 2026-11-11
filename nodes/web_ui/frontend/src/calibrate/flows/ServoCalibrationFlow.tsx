import { useEffect, useState } from 'react'
import Accordion from '@mui/material/Accordion'
import AccordionDetails from '@mui/material/AccordionDetails'
import AccordionSummary from '@mui/material/AccordionSummary'
import Alert from '@mui/material/Alert'
import Box from '@mui/material/Box'
import Typography from '@mui/material/Typography'
import ExpandMoreIcon from '@mui/icons-material/ExpandMore'
import { TOUCH_TARGET_PX } from '../../theme'
import { ServoCalibrationCard } from '../ServoCalibrationCard'
import { SERVO_POLL_MS, type ServoAction, type ServoActionResult, servoAction, useServoGroups } from '../servoApi'
import { ServoHomeFlow } from './ServoHomeFlow'

/** Poll period while a range capture runs, so the live min/max bars follow the joints. */
export const CAPTURE_POLL_MS = 250

/**
 * Flow 1, servo calibration: one card per group offering the servo calibration (Torque off, Record home, Record servo
 * ranges, Store), then the older neutral, zero restore and Set home panels in an "Advanced / repair" accordion.
 * Polls every 250 ms while any capture runs, else at the normal servo rate.
 */
export function ServoCalibrationFlow() {
  const [fast, setFast] = useState(false)
  // One poll loop: the hook restarts at the fast period while any capture runs and back at the normal one after.
  const servo = useServoGroups(fast ? CAPTURE_POLL_MS : SERVO_POLL_MS)
  const capturing = servo.groups.some((g) => g.state?.servo_calibration?.capture?.running === true)
  useEffect(() => setFast(capturing), [capturing])
  const cards = servo.groups.filter((g) => g.servo_calibration)

  /**
   * Runs a servo action of one group and folds a transport or refusal failure into an `ok: false` result.
   *
   * @param groupId - servo group id
   * @param action - the servo action
   * @param planRunId - plan run_id for store actions
   * @returns the action result
   */
  const run = async (groupId: string, action: ServoAction, planRunId?: string): Promise<ServoActionResult> => {
    const r = await servoAction(groupId, action, planRunId)
    return r.ok ? r.value : { ok: false, message: r.message }
  }

  return (
    <Box>
      {servo.error !== null && <Alert severity="error" sx={{ mb: 1 }}>{servo.error}</Alert>}
      {cards.length === 0 && !servo.loading && <Typography variant="body2">No servo group offers the servo calibration.</Typography>}
      {cards.map((g) => (
        <Box key={g.id} component="section" sx={{ mb: 2 }}>
          <Typography variant="subtitle1" component="h3" sx={{ mb: 1 }}>
            {g.label}
          </Typography>
          <ServoCalibrationCard group={g} onAction={(action, planRunId) => run(g.id, action, planRunId)} />
        </Box>
      ))}
      <Accordion disableGutters>
        <AccordionSummary expandIcon={<ExpandMoreIcon />} sx={{ minHeight: TOUCH_TARGET_PX }}>
          <Typography>Advanced / repair</Typography>
        </AccordionSummary>
        <AccordionDetails>
          <ServoHomeFlow />
        </AccordionDetails>
      </Accordion>
    </Box>
  )
}
