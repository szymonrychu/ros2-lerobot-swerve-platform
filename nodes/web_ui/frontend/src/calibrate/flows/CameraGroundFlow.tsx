import { useState } from 'react'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import LinearProgress from '@mui/material/LinearProgress'
import Typography from '@mui/material/Typography'
import { TOUCH_TARGET_PX } from '../../theme'
import { CsiIntrinsics } from '../CsiIntrinsics'
import { FlowStepper } from '../FlowStepper'
import { AppliedView, FloorResultDetails, RestartView, useFloorFit } from '../HeadFloorFit'
import { GateDecision, QualityGate, floorGate } from '../QualityGate'
import { RgbdCalibration } from '../RgbdCalibration'

const SWEEP_TEXT =
  'Robot on a flat floor, not on blocks; about 2 m in front and 1.5 m to each side clear; only the head moves. The sweep fits the head mount z, roll, pitch, tilt zero and tilt roll from the floor plane from the D435 depth.'
const ACCEPT_BODY =
  'Applies the fitted head chain: the head camera, obstacle and 3D map nodes restart with it. The arm-to-camera fit builds on it.'
const VERIFY_TEXT =
  'Coming: the ChArUco ground check (the big board flat on the ground) verifies the applied chain and replaces the depth floor fit. Until then, check the 3D view: the floor points lie flat at height 0 in every head pose.'

/**
 * Flow 3, camera to ground: the optional D435 intrinsics check, the front/back CSI cameras (always visible), the head
 * floor sweep, its compute, the gate whose Accept applies the head chain, and the ground verification (coming).
 */
export function CameraGroundFlow() {
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
    if (await floor.apply()) setDecision('accepted')
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
      label="Camera to ground calibration steps"
      steps={[
        {
          id: 'd435',
          title: 'D435 intrinsics check',
          optional: true,
          done: false,
          see: 'Every coverage cell has corners; Compute compares with the D435 factory values and Accept check records a passed check.',
          children: <RgbdCalibration heading={false} />,
        },
        {
          id: 'csi',
          title: 'Front/back cameras',
          optional: true,
          done: false,
          see: 'For each of the front and back cameras: views kept over the whole image, a small RMS after Compute, Apply restarts the camera node.',
          children: <CsiIntrinsics heading={false} />,
        },
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
                  Start sweep
                </Button>
                <Button variant="outlined" sx={target} disabled={!running} onClick={() => void floor.stop()}>
                  Stop
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
              Compute
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
        {
          id: 'verify',
          title: 'Verify on the ground (coming)',
          optional: true,
          done: false,
          see: 'The ground check result (coming).',
          children: <Typography variant="body2">{VERIFY_TEXT}</Typography>,
        },
      ]}
      />
    </Box>
  )
}
