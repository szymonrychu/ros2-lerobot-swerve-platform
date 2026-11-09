import { useCallback, useEffect, useState } from 'react'
import Alert from '@mui/material/Alert'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import Typography from '@mui/material/Typography'
import { ConfirmDialog } from '../../layout/ConfirmDialog'
import { TOUCH_TARGET_PX } from '../../theme'
import { FlowStepper } from '../FlowStepper'
import { GateDecision, GateReport, QualityGate } from '../QualityGate'
import { IMU_LEVEL_MAX, ImuStatus, fetchImuStatus, imuLevelColor, postImuReset, postImuSave, saveEnabled } from '../imuApi'

/** Status poll period: 1 Hz. */
export const IMU_POLL_MS = 1000
const RESET_TEXT = 'The IMU forgets its calibration and starts from zero.'
const ACCEPT_BODY = 'Saves the current offsets in the IMU node; it restores them at its next start.'
const LEVELS = ['sys', 'gyro', 'accel', 'mag'] as const
const GUIDANCE: Record<string, string> = {
  gyro: 'gyro: keep the robot still for a few seconds.',
  accel: 'accel: hold a few static poses, lifting the robot and setting it down on different sides.',
  mag: 'mag: rotate the robot or drive figure-eights, away from motors and metal.',
}
/** Gate warning of a level below 3 (mag below 3 refuses instead). */
const LOW_LEVEL_HINT: Record<string, string> = {
  sys: 'keep moving: sys follows the other three',
  gyro: 'keep the robot still a few seconds',
  accel: 'hold a few more static poses',
}

type StatusState = { kind: 'loading' } | { kind: 'ok'; status: ImuStatus } | { kind: 'error'; message: string }
type Feedback = { ok: boolean; message: string } | null

/**
 * Gate of the IMU flow: the offsets save only with mag at 3; other levels below 3 warn.
 *
 * @param status - the live IMU calibration status, null before the first sample
 * @returns the gate report
 */
export function imuGate(status: ImuStatus | null): GateReport {
  if (status === null) {
    return { plausible: false, refusals: ['No IMU calibration status yet'], warnings: [], score: [], params: [], accuracy: [], comparison: [], notes: [] }
  }
  const level = (k: (typeof LEVELS)[number]) => `${status[k]} / ${IMU_LEVEL_MAX}`
  return {
    plausible: saveEnabled(status),
    refusals: saveEnabled(status) ? [] : [`mag ${level('mag')}: the offsets save only with mag at 3`],
    warnings: (['sys', 'gyro', 'accel'] as const).filter((k) => status[k] < IMU_LEVEL_MAX).map((k) => `${k} ${level(k)}: ${LOW_LEVEL_HINT[k]}`),
    score: LEVELS.map((k) => ({ label: k, value: level(k) })),
    params: [],
    accuracy: [],
    comparison: [],
    notes: [],
  }
}

/** Flow 2, IMU calibration: live status, the guided moves until mag is 3, then the gate whose Accept saves the offsets. */
export function ImuFlow() {
  const [active, setActive] = useState(false)
  const [state, setState] = useState<StatusState>({ kind: 'loading' })
  const [feedback, setFeedback] = useState<Feedback>(null)
  const [confirmReset, setConfirmReset] = useState(false)
  const [decision, setDecision] = useState<GateDecision>(null)
  const [busy, setBusy] = useState(false)

  useEffect(() => {
    if (!active) return
    let live = true
    const poll = async () => {
      if (document.visibilityState === 'hidden') return
      const r = await fetchImuStatus()
      if (!live) return
      setState(r.ok ? { kind: 'ok', status: r.value } : { kind: 'error', message: r.message })
    }
    void poll()
    const id = setInterval(() => void poll(), IMU_POLL_MS)
    return () => {
      live = false
      clearInterval(id)
    }
  }, [active])

  const act = useCallback(async (call: typeof postImuSave) => {
    setBusy(true)
    const r = await call()
    setBusy(false)
    const ok = r.ok && r.value.ok
    setFeedback({ ok, message: r.ok ? r.value.message : r.message })
    return ok
  }, [])

  const accept = async () => {
    if (await act(postImuSave)) setDecision('accepted')
  }

  const status = state.kind === 'ok' ? state.status : null
  const btn = { minHeight: TOUCH_TARGET_PX }
  return (
    <Box>
      <Box sx={{ display: 'flex', justifyContent: 'flex-end', mb: 1 }}>
        <Button variant="outlined" color="error" sx={btn} onClick={() => setConfirmReset(true)}>
          Reset IMU calibration
        </Button>
      </Box>
      {feedback && (
        <Alert severity={feedback.ok ? 'success' : 'warning'} sx={{ mb: 1 }} onClose={() => setFeedback(null)}>
          {feedback.message}
        </Alert>
      )}
      <FlowStepper
        label="IMU calibration steps"
        steps={[
          {
            id: 'start',
            title: 'Start live status',
            done: active,
            see: 'The four levels sys, gyro, accel and mag from 0 to 3 and whether saved offsets were restored.',
            children: (
              <Button variant="contained" sx={btn} disabled={active} onClick={() => setActive(true)}>
                Calibrate IMU
              </Button>
            ),
          },
          {
            id: 'move',
            title: 'Move the robot',
            done: saveEnabled(status),
            see: 'mag climbs to 3 (green); gyro and accel follow the moves below.',
            error: state.kind === 'error' ? state.message : null,
            children: (
              <>
                {status && (
                  <Box>
                    {LEVELS.map((k) => (
                      <Typography key={k} sx={{ color: imuLevelColor(status[k]) }}>
                        {`${k} ${status[k]} / ${IMU_LEVEL_MAX}`}
                      </Typography>
                    ))}
                    <Typography>{`Restored from saved offsets: ${status.restored ? 'yes' : 'no'}`}</Typography>
                  </Box>
                )}
                <Box sx={{ mt: 1 }}>
                  {Object.values(GUIDANCE).map((g) => (
                    <Typography key={g} variant="body2" color="text.secondary">
                      {g}
                    </Typography>
                  ))}
                </Box>
              </>
            ),
          },
          {
            id: 'gate',
            title: 'Check and save',
            done: decision !== null,
            see: 'Plausible with mag at 3; Accept saves the offsets and shows the node answer.',
            children: (
              <QualityGate
                report={imuGate(status)}
                acceptBody={ACCEPT_BODY}
                decision={decision}
                busy={busy}
                onAccept={() => void accept()}
                onDecline={() => setDecision('declined')}
              />
            ),
          },
        ]}
      />
      <ConfirmDialog
        open={confirmReset}
        title={RESET_TEXT}
        confirmLabel="Reset"
        onConfirm={() => {
          setConfirmReset(false)
          setDecision(null)
          void act(postImuReset)
        }}
        onCancel={() => setConfirmReset(false)}
      />
    </Box>
  )
}
