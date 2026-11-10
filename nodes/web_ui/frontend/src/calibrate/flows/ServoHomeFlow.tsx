import { useCallback, useState } from 'react'
import Button from '@mui/material/Button'
import Typography from '@mui/material/Typography'
import { TOUCH_TARGET_PX } from '../../theme'
import { FlowStepper } from '../FlowStepper'
import { GateDecision, GateReport, QualityGate } from '../QualityGate'
import { ServoNeutral, torqueLevel } from '../ServoNeutral'
import { type ServoAction, type ServoGroup, useServoGroups } from '../servoApi'

const SAVE_ACTIONS: ServoAction[] = ['save', 'save_confirmed']
const ZERO_APPLY_ACTIONS: ServoAction[] = ['zero_restore_apply', 'zero_restore_apply_confirmed']
const LEVEL_TEXT: Record<string, string> = { on: 'Torque on', off: 'Torque off', partly: 'Partly off', unknown: 'Unknown' }
const ACCEPT_BODY = 'Records that the servo home check passed on this page. Nothing is written to the servos.'

/**
 * Gate of the servo home flow: every group fresh with torque on and no stale calibration.
 *
 * @param groups - the servo groups (GET /api/calibration/servos)
 * @returns the gate report
 */
export function servoGate(groups: ServoGroup[]): GateReport {
  const refusals = groups.flatMap((g) => {
    const level = torqueLevel(g)
    const out: string[] = []
    if (level === 'unknown') out.push(`${g.label}: no fresh torque state`)
    else if (level !== 'on') out.push(`${g.label}: torque is ${level === 'off' ? 'off' : 'partly off'}`)
    if (g.calibration_stale && g.calibration_stale.length > 0) out.push(`${g.label}: calibration stale (${g.calibration_stale.join(', ')})`)
    else if (g.state?.zero_restore?.stale_joint_offsets) out.push(`${g.label}: arm zero stale (joint_offsets)`)
    return out
  })
  if (groups.length === 0) refusals.push('No servo group state yet')
  return {
    plausible: refusals.length === 0,
    refusals,
    warnings: [],
    score: groups.map((g) => ({ label: g.label, value: LEVEL_TEXT[torqueLevel(g)] })),
    params: [],
    accuracy: [],
    comparison: [],
    notes: [],
  }
}

/**
 * Flow 1, servo home calibration: record and save the neutral (large-shift guard), the Arm zero restore, Set home (the
 * stale marker's joint_offsets and home shown), Move to neutral pose, then the check gate. The neutral, zero
 * and Set home steps are optional: they are needed after a servo swap or a re-zero (a zero restore clears joint_offsets
 * from the stale marker but leaves home stale until Set home).
 */
export function ServoHomeFlow() {
  const servo = useServoGroups()
  const [saved, setSaved] = useState(false)
  const [zeroApplied, setZeroApplied] = useState(false)
  const [moved, setMoved] = useState(false)
  const [homeSet, setHomeSet] = useState(false)
  const [decision, setDecision] = useState<GateDecision>(null)

  const onResult = useCallback((_group: string, action: ServoAction, ok: boolean) => {
    if (!ok) return
    if (SAVE_ACTIONS.includes(action)) setSaved(true)
    if (ZERO_APPLY_ACTIONS.includes(action)) setZeroApplied(true)
    if (action === 'move_to_neutral') setMoved(true)
    if (action === 'set_home') setHomeSet(true)
  }, [])

  const restart = () => {
    setSaved(false)
    setZeroApplied(false)
    setMoved(false)
    setHomeSet(false)
    setDecision(null)
  }

  const canMove = servo.groups.some((g) => g.move_to_neutral)
  const hasZero = servo.groups.some((g) => g.zero_restore)
  const hasHome = servo.groups.some((g) => g.set_home)
  return (
    <FlowStepper
      label="Servo home calibration steps"
      steps={[
        {
          id: 'neutral',
          title: 'Record and save neutral',
          optional: true,
          done: saved,
          see: 'Torque off, the joints held in the neutral pose, Record neutral shows small shifts; after Re-enable torque + save the chip says Torque on. Only after a servo swap or a horn moved.',
          children: <ServoNeutral parts={['torque']} servo={servo} heading={false} onResult={onResult} />,
        },
        {
          id: 'zero',
          title: 'Arm zero restore',
          optional: true,
          done: zeroApplied,
          see: 'The dry run plan per joint; after Apply every joint is applied and the stale flag is gone. Only when the arm zero is stale or lost.',
          children: hasZero ? (
            <ServoNeutral parts={['zero']} servo={servo} heading={false} onResult={onResult} />
          ) : (
            <Typography variant="body2">No servo group offers the zero restore.</Typography>
          ),
        },
        {
          id: 'home',
          title: 'Set home',
          optional: true,
          done: homeSet,
          see: 'Pose the arm first (by hand with torque off, or Move to neutral pose in step 4), then Set home records that pose as home; nothing moves. The home chip of the stale marker turns ok. Needed after a zero restore: it clears joint_offsets but leaves home stale.',
          children: hasHome ? (
            <ServoNeutral parts={['home']} servo={servo} heading={false} onResult={onResult} />
          ) : (
            <Typography variant="body2">No servo group offers Set home.</Typography>
          ),
        },
        {
          id: 'move',
          title: 'Move to neutral pose',
          optional: !canMove,
          done: moved,
          see: 'Every arm joint at 0: the arm in the neutral pose of its model, matching the joint marks.',
          children: canMove ? (
            <ServoNeutral parts={['move']} servo={servo} heading={false} onResult={onResult} />
          ) : (
            <Typography variant="body2">No servo group offers Move to neutral pose.</Typography>
          ),
        },
        {
          id: 'gate',
          title: 'Check and accept',
          done: decision !== null,
          see: 'Every group with torque on and nothing stale: Plausible. Accept when the arm stood straight in the neutral pose.',
          children: (
            <>
              <QualityGate
                report={servoGate(servo.groups)}
                acceptBody={ACCEPT_BODY}
                decision={decision}
                onAccept={() => setDecision('accepted')}
                onDecline={() => setDecision('declined')}
              />
              {decision !== null && (
                <Button variant="outlined" sx={{ minHeight: TOUCH_TARGET_PX, mt: 1 }} onClick={restart}>
                  Start over
                </Button>
              )}
            </>
          ),
        },
      ]}
    />
  )
}
