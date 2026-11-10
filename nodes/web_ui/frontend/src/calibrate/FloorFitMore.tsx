/**
 * More fits of the Camera to ground flow: the depth floor fit (head floor sweep, compute, gate and apply). It was the
 * interim camera to ground calibration (decision 325); the ground board fit is now primary, so it sits behind More
 * fits.
 */
import { useState } from 'react'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import LinearProgress from '@mui/material/LinearProgress'
import Typography from '@mui/material/Typography'
import { TOUCH_TARGET_PX } from '../theme'
import { FlowStepper } from './FlowStepper'
import { AppliedView, FloorResultDetails, RestartView, useFloorFit } from './HeadFloorFit'
import { GateDecision, QualityGate, floorGate } from './QualityGate'

const SWEEP_TEXT =
  'Robot on a flat floor, not on blocks; about 2 m in front and 1.5 m to each side clear; only the head moves. The sweep fits the head mount z, roll, pitch, tilt zero and tilt roll from the floor plane from the D435 depth.'
const ACCEPT_BODY =
  'Applies the fitted head chain: the head camera, obstacle and 3D map nodes restart with it. The arm-to-camera fit builds on it.'

/**
 * The depth floor fit: 1 head floor sweep, 2 compute, 3 the gate whose Accept applies the head chain.
 *
 * @param onApplied - called after an apply (the head chain in effect changed)
 */
export function DepthFloorFit({ onApplied }: { onApplied?: () => void }) {
  const floor = useFloorFit()
  const [decision, setDecision] = useState<GateDecision>(null)
  const { sweep, result, running, busy } = floor
  const target = { minHeight: TOUCH_TARGET_PX }

  const start = async () => {
    setDecision(null)
    await floor.start()
  }
  const compute = async () => {
    setDecision(null)
    await floor.compute()
  }
  const accept = async () => {
    if (await floor.apply()) {
      setDecision('accepted')
      onApplied?.()
    }
  }

  return (
    <Box>
      {floor.applied ? (
        <AppliedView applied={floor.applied} />
      ) : (
        <Typography variant="body2" color="text.secondary">
          No head chain applied yet.
        </Typography>
      )}
      <FlowStepper
        label="Depth floor fit steps"
        steps={[
          {
            id: 'sweep',
            title: 'Head floor sweep',
            done: sweep?.session_id != null && !running,
            see: 'The head pans and tilts through the poses; the pose count climbs with observations kept and few skipped.',
            error: floor.error ?? sweep?.error ?? null,
            status: sweep?.session_id ? (
              <Box>
                {running && <LinearProgress variant="determinate" value={sweep.total ? (100 * sweep.pose) / sweep.total : 0} />}
                <Typography variant="body2">
                  {`Pose ${sweep.pose} of ${sweep.total}, ${sweep.observations} observations, ${sweep.skipped} skipped - ${sweep.status}`}
                </Typography>
              </Box>
            ) : undefined,
            children: (
              <>
                <Typography variant="body2" color="text.secondary">
                  {SWEEP_TEXT}
                </Typography>
                <Box sx={{ display: 'flex', gap: 1, mt: 1, flexWrap: 'wrap' }}>
                  <Button variant="contained" sx={target} disabled={busy || running} onClick={() => void start()}>
                    Start floor sweep
                  </Button>
                  <Button variant="outlined" sx={target} disabled={!running} onClick={() => void floor.stop()}>
                    Stop floor sweep
                  </Button>
                </Box>
              </>
            ),
          },
          {
            id: 'compute',
            title: 'Compute',
            done: result !== null,
            see: 'The fit result with its parameters and residuals appears in the next step.',
            children: (
              <Button variant="contained" sx={target} disabled={busy || running || !sweep?.session_id} onClick={() => void compute()}>
                Compute floor fit
              </Button>
            ),
          },
          {
            id: 'gate',
            title: 'Check and apply',
            done: decision !== null,
            see: 'Plausible with small residuals; Accept applies the head chain and shows the unit restart state.',
            children: (
              <>
                {result && (
                  <>
                    <QualityGate
                      report={floorGate(result)}
                      acceptBody={ACCEPT_BODY}
                      decision={decision}
                      busy={busy}
                      onAccept={() => void accept()}
                      onDecline={() => setDecision('declined')}
                    />
                    <FloorResultDetails result={result} />
                  </>
                )}
                {floor.restart && <RestartView restart={floor.restart} />}
              </>
            ),
          },
        ]}
      />
    </Box>
  )
}

/**
 * More fits of the Camera to ground flow, closed until opened: the depth floor fit.
 *
 * @param disabled - a ground board session or verification moves the head: the depth floor fit stays closed
 * @param onApplied - called after an apply of the depth floor fit
 */
export function FloorFitMore({ disabled = false, onApplied }: { disabled?: boolean; onApplied?: () => void }) {
  const [open, setOpen] = useState(false)
  return (
    <Box sx={{ mt: 1 }}>
      <Button variant="outlined" sx={{ minHeight: TOUCH_TARGET_PX }} disabled={disabled && !open} onClick={() => setOpen(!open)}>
        More fits
      </Button>
      {open && (
        <Box component="section" aria-label="More fits" sx={{ mt: 1 }}>
          <Typography variant="subtitle1" component="h3">
            Depth floor fit
          </Typography>
          <Typography variant="body2" color="text.secondary">
            The head chain from the D435 depth floor plane alone (the interim camera to ground calibration). The ground
            board fit above is the primary one; use this one only when the board fit cannot run.
          </Typography>
          <DepthFloorFit onApplied={onApplied} />
        </Box>
      )}
    </Box>
  )
}
