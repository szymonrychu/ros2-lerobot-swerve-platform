/**
 * Grasp panel of the map tab (presentational): where the grasp result is shown. The object position comes from a map
 * click after choosing a strategy in the toolbar Grasp dropdown; the panel holds the collapsed object size
 * and advanced settings, Plan again / Execute / Release, the execute confirmation and the feasibility /
 * outcome rows. State lives in useGrasp.
 */
import { useState } from 'react'
import Alert from '@mui/material/Alert'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import Collapse from '@mui/material/Collapse'
import Stack from '@mui/material/Stack'
import Tooltip from '@mui/material/Tooltip'
import TextField from '@mui/material/TextField'
import ToggleButton from '@mui/material/ToggleButton'
import ToggleButtonGroup from '@mui/material/ToggleButtonGroup'
import Typography from '@mui/material/Typography'
import ExpandLessIcon from '@mui/icons-material/ExpandLess'
import ExpandMoreIcon from '@mui/icons-material/ExpandMore'
import { ADVANCED_PARAMS, describeHold, describeOutcome, GraspAnswer, GRIP_PROFILES, isGripProfile, summarizePlan } from './grasp'
import type { Severity } from './grasp'
import { StatusItem } from '../components/StatusItem'
import type { StatusState } from '../components/StatusItem'
import type { GraspPanelState } from './useGrasp'
import { OverlayPanel } from '../layout/OverlayPanel'
import { useLayout } from '../layout/useLayout'
import { MONO_FONT } from '../theme'

interface Props {
  state: GraspPanelState
  onClose: () => void
}

function Field({
  label,
  value,
  onChange,
  placeholder,
  unit,
  disabled,
}: {
  label: string
  value: string
  onChange: (v: string) => void
  placeholder?: string
  unit?: string
  disabled?: boolean
}) {
  return (
    <TextField
      size="small"
      label={unit ? `${label} (${unit})` : label}
      value={value}
      placeholder={placeholder}
      disabled={disabled}
      onChange={(e) => onChange(e.target.value)}
      inputProps={{ inputMode: 'decimal', autoComplete: 'off', style: { fontFamily: MONO_FONT } }}
      InputLabelProps={{ shrink: true }}
      sx={{ flex: '1 1 5.6rem', minWidth: '5.6rem' }}
    />
  )
}

/** One result line: wraps long text instead of running off the panel. */
const ROW_SX = { '& > *': { whiteSpace: 'normal', flexShrink: 1 } } as const

function Row({ label, value, state }: { label: string; value: string; state?: StatusState }) {
  return (
    <Box sx={ROW_SX}>
      <StatusItem label={label} value={value} state={state} />
    </Box>
  )
}

const OUTCOME_STATE: Record<Severity, StatusState> = { success: 'ok', warning: 'warn', error: 'error' }

function Result({ answer, title }: { answer: GraspAnswer; title: string }) {
  if (!answer.ok) return <Row label={title} value={answer.error ?? 'failed'} state="error" />
  if (!answer.outcome) return null
  const { text, severity } = describeOutcome(answer.outcome)
  const plan = answer.plan
  const hold = describeHold(answer)
  const reasons = [...answer.reasons, ...(plan?.reasons ?? []).filter((r) => !answer.reasons.includes(r))]
  return (
    <Stack spacing={0.5} data-testid="grasp-result">
      <Row label="Result" value={text} state={OUTCOME_STATE[severity]} />
      {plan && plan.strategy && (
        <Row label="Strategy" value={`${plan.strategy}${plan.approachPitchDeg !== undefined ? `, pitch ${Math.round(plan.approachPitchDeg)} deg` : ''}`} />
      )}
      {reasons.map((r) => (
        <Row key={r} label="Reason" value={r} />
      ))}
      {plan && plan.attempts.length > 1 &&
        plan.attempts.map((a, i) => (
          <Row key={i} label="Attempt" value={`${a.strategy ?? '?'}: ${a.feasible ? 'feasible' : 'infeasible'}${a.reasons.length ? ` (${a.reasons.join('; ')})` : ''}`} />
        ))}
      {answer.steps.map((s, i) => (
        <Row key={i} label="Step" value={`${s.label}: ${s.status}${s.message ? ` - ${s.message}` : ''}`} />
      ))}
      {hold && <Row label="Hold" value={hold} state={answer.slipping || answer.crushRisk ? 'warn' : undefined} />}
      {(answer.gripperPositionRad !== undefined || answer.gripperEffort !== undefined) && (
        <Row label="Gripper" value={`${answer.gripperPositionRad?.toFixed(2) ?? '?'} rad, load ${answer.gripperEffort?.toFixed(0) ?? '?'}`} />
      )}
    </Stack>
  )
}

export function GraspPanel({ state, onClose }: Props) {
  const [objectOpen, setObjectOpen] = useState(false)
  const [advanced, setAdvanced] = useState(false)
  const { form, setForm } = state
  const executing = state.executing
  const planAnswer = state.plan?.outcome ?? (state.lastAnswer?.action === 'plan' ? state.lastAnswer : null)
  const run = state.stream?.state === 'done' ? state.stream.outcome : null
  const actionBusy = executing || state.planning
  // A phone bottom sheet would cover the map while the user taps the object, so it shrinks to the pick banner.
  const compactForPick = useLayout() === 'phone' && state.pickMode
  const feasiblePlan = state.planFresh && state.plan?.outcome.plan?.feasible === true

  return (
    <OverlayPanel title="Grasp" open anchor="bottom" onClose={onClose}>
      <Stack spacing={1} aria-label="Grasp panel" sx={{ maxHeight: { xs: '62dvh', sm: '70dvh' } }}>
        <Stack spacing={1} sx={{ display: compactForPick ? 'none' : 'flex' }}>
        <Tooltip title="Width is across the jaws, depth along the approach, gap below is the clear height under the object (Scoop needs room under it). Saved in this browser.">
          <Button
            color="inherit"
            onClick={() => setObjectOpen((o) => !o)}
            endIcon={objectOpen ? <ExpandLessIcon /> : <ExpandMoreIcon />}
            aria-expanded={objectOpen}
            sx={{ justifyContent: 'space-between' }}
          >
            Object
          </Button>
        </Tooltip>
        <Collapse in={objectOpen}>
          <Stack spacing={1}>
            <Stack direction="row" useFlexGap sx={{ gap: 1, flexWrap: 'wrap' }}>
              <Field label="Width" unit="m" value={form.width} onChange={(width) => setForm({ width })} />
              <Field label="Depth" unit="m" value={form.depth} onChange={(depth) => setForm({ depth })} />
              <Field label="Height" unit="m" value={form.height} onChange={(height) => setForm({ height })} />
              {form.strategy === 'scoop' && (
                <Field label="Gap below" unit="m" value={form.gapBelow} onChange={(gapBelow) => setForm({ gapBelow })} />
              )}
              <Field label="Yaw" unit="deg" placeholder="across" value={form.yaw} onChange={(yaw) => setForm({ yaw })} />
            </Stack>
          </Stack>
        </Collapse>

        <Stack spacing={0.5}>
          <StatusItem label="Grip strength" value={GRIP_PROFILES.find((g) => g.value === form.gripProfile)?.label ?? ''} />
          <ToggleButtonGroup
            exclusive
            fullWidth
            aria-label="Grip strength"
            value={form.gripProfile}
            onChange={(_e, value: unknown) => {
              if (isGripProfile(value)) setForm({ gripProfile: value })
            }}
          >
            {GRIP_PROFILES.map((g) => (
              <ToggleButton key={g.value} value={g.value} title={g.hint}>
                {g.label}
              </ToggleButton>
            ))}
          </ToggleButtonGroup>
        </Stack>

        <Button
          color="inherit"
          onClick={() => setAdvanced((a) => !a)}
          endIcon={advanced ? <ExpandLessIcon /> : <ExpandMoreIcon />}
          aria-expanded={advanced}
          sx={{ justifyContent: 'space-between' }}
        >
          Advanced
        </Button>
        <Collapse in={advanced}>
          <Stack spacing={1}>
            <Stack direction="row" useFlexGap sx={{ gap: 1, flexWrap: 'wrap' }}>
              {form.strategy === 'angled' && (
                <Field label="Pitch" unit="deg" placeholder="45" value={form.pitchDeg} onChange={(pitchDeg) => setForm({ pitchDeg })} />
              )}
              <Field label="Surface z" unit="m" placeholder="0" value={form.surfaceZ} onChange={(surfaceZ) => setForm({ surfaceZ })} />
              <Field label="Tilt roll" unit="deg" placeholder="IMU" value={form.tiltRoll} onChange={(tiltRoll) => setForm({ tiltRoll })} />
              <Field label="Tilt pitch" unit="deg" placeholder="IMU" value={form.tiltPitch} onChange={(tiltPitch) => setForm({ tiltPitch })} />
            </Stack>
            <Stack direction="row" useFlexGap sx={{ gap: 1, flexWrap: 'wrap' }}>
              {ADVANCED_PARAMS.map((p) => (
                <Field
                  key={p.key}
                  label={p.label}
                  unit={p.unit}
                  placeholder={String(p.default)}
                  value={form.params[p.key] ?? ''}
                  onChange={(v) => state.setParam(p.key, v)}
                />
              ))}
            </Stack>
          </Stack>
        </Collapse>

        {state.showErrors && state.errors.length > 0 && (
          <Alert severity="warning">
            {state.errors.map((e) => (
              <Typography key={e} variant="body2">
                {e}
              </Typography>
            ))}
          </Alert>
        )}

        <Stack direction="row" useFlexGap sx={{ gap: 1, flexWrap: 'wrap' }}>
          {state.hasTarget && (
            <Button variant="outlined" disabled={actionBusy} onClick={state.requestPlan} sx={{ flex: '1 1 5.6rem' }}>
              Plan again
            </Button>
          )}
          {feasiblePlan && (
            <Button
              variant="contained"
              color="warning"
              disabled={!state.canExecute}
              onClick={state.requestExecute}
              sx={{ flex: '1 1 5.6rem' }}
            >
              Execute
            </Button>
          )}
          <Button
            variant={state.releaseArmed ? 'contained' : 'outlined'}
            color={state.releaseArmed ? 'warning' : 'primary'}
            disabled={actionBusy}
            onClick={state.release}
            sx={{ flex: '1 1 5.6rem' }}
          >
            {state.releaseArmed ? 'Confirm release' : 'Release'}
          </Button>
        </Stack>
        {state.plan && !state.planFresh && (
          <Typography variant="caption" color="warning.main" data-testid="grasp-caption">
            Settings changed since the last plan: plan again to get Execute.
          </Typography>
        )}

        {state.confirming && state.plan?.outcome.plan && (
          <Alert severity="warning" icon={false} sx={{ '& .MuiAlert-message': { width: '100%' } }}>
            <Typography variant="body2" sx={{ fontWeight: 500 }}>
              The arm will move. Confirm execute?
            </Typography>
            {summarizePlan(state.plan.outcome.plan).map((l) => (
              <Typography key={l} variant="body2">
                {l}
              </Typography>
            ))}
            <Stack direction="row" spacing={1} sx={{ mt: 1 }}>
              <Button variant="contained" color="warning" onClick={state.confirmExecute} sx={{ flex: 1 }}>
                Confirm execute
              </Button>
              <Button variant="outlined" color="inherit" onClick={state.cancelConfirm} sx={{ flex: 1 }}>
                Cancel
              </Button>
            </Stack>
          </Alert>
        )}

        {(state.planning || state.stream?.state === 'running') && (
          <Row label="Status" value={state.planning ? 'Planning' : `Running ${state.stream?.action ?? 'grasp'}`} />
        )}
        {run && <Result answer={run} title={state.stream?.action ?? 'Grasp'} />}
        {planAnswer && !state.planning && <Result answer={planAnswer} title="Plan" />}
        {!planAnswer && state.lastAnswer && !state.lastAnswer.ok && <Result answer={state.lastAnswer} title="Grasp" />}
        </Stack>
      </Stack>
    </OverlayPanel>
  )
}
