/**
 * Grasp panel of the map tab (presentational): object and strategy inputs, advanced parameters, Plan / Execute /
 * Release / Stop, the execute confirmation and the feasibility / outcome readout. State lives in useGrasp.
 */
import { useState } from 'react'
import Alert from '@mui/material/Alert'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import Collapse from '@mui/material/Collapse'
import IconButton from '@mui/material/IconButton'
import MenuItem from '@mui/material/MenuItem'
import Paper from '@mui/material/Paper'
import Stack from '@mui/material/Stack'
import TextField from '@mui/material/TextField'
import ToggleButton from '@mui/material/ToggleButton'
import ToggleButtonGroup from '@mui/material/ToggleButtonGroup'
import Typography from '@mui/material/Typography'
import CloseIcon from '@mui/icons-material/Close'
import ExpandLessIcon from '@mui/icons-material/ExpandLess'
import ExpandMoreIcon from '@mui/icons-material/ExpandMore'
import StopCircleIcon from '@mui/icons-material/StopCircle'
import AdsClickIcon from '@mui/icons-material/AdsClick'
import { ADVANCED_PARAMS, describeOutcome, GraspAnswer, GraspFrame, GraspStrategy, STRATEGIES, summarizePlan } from './grasp'
import type { GraspPanelState } from './useGrasp'
import { MONO_FONT } from '../theme'

interface Props {
  state: GraspPanelState
  /** Pick-on-map needs the top view. */
  topView: boolean
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
      sx={{ flex: '1 1 90px', minWidth: 90 }}
    />
  )
}

function Result({ answer, title }: { answer: GraspAnswer; title: string }) {
  if (!answer.ok) return <Alert severity="error">{`${title}: ${answer.error ?? 'failed'}`}</Alert>
  if (!answer.outcome) return null
  const { text, severity } = describeOutcome(answer.outcome)
  const plan = answer.plan
  return (
    <Alert severity={severity} sx={{ '& .MuiAlert-message': { width: '100%' } }}>
      <Typography variant="body2" sx={{ fontWeight: 700 }}>
        {text}
      </Typography>
      {plan && plan.strategy && (
        <Typography variant="body2">
          Strategy: {plan.strategy}
          {plan.approachPitchDeg !== undefined ? `, pitch ${Math.round(plan.approachPitchDeg)} deg` : ''}
        </Typography>
      )}
      {[...answer.reasons, ...(plan?.reasons ?? []).filter((r) => !answer.reasons.includes(r))].map((r) => (
        <Typography key={r} variant="body2">
          {`- ${r}`}
        </Typography>
      ))}
      {plan && plan.attempts.length > 1 && (
        <Box sx={{ mt: 0.5 }}>
          <Typography variant="caption" color="text.secondary">
            Auto attempts
          </Typography>
          {plan.attempts.map((a, i) => (
            <Typography key={i} variant="caption" sx={{ display: 'block' }}>
              {`${a.strategy ?? '?'}: ${a.feasible ? 'feasible' : 'infeasible'}${a.reasons.length ? ` (${a.reasons.join('; ')})` : ''}`}
            </Typography>
          ))}
        </Box>
      )}
      {answer.steps.length > 0 && (
        <Box sx={{ mt: 0.5 }}>
          {answer.steps.map((s, i) => (
            <Typography key={i} variant="caption" sx={{ display: 'block', fontFamily: MONO_FONT }}>
              {`${s.label}: ${s.status}${s.message ? ` - ${s.message}` : ''}`}
            </Typography>
          ))}
        </Box>
      )}
      {(answer.gripperPositionRad !== undefined || answer.gripperEffort !== undefined) && (
        <Typography variant="caption" sx={{ display: 'block', fontFamily: MONO_FONT }}>
          {`gripper ${answer.gripperPositionRad?.toFixed(2) ?? '?'} rad, load ${answer.gripperEffort?.toFixed(0) ?? '?'}`}
        </Typography>
      )}
    </Alert>
  )
}

export function GraspPanel({ state, topView, onClose }: Props) {
  const [advanced, setAdvanced] = useState(false)
  const { form, setForm } = state
  const executing = state.executing
  const planAnswer = state.plan?.outcome ?? (state.lastAnswer?.action === 'plan' ? state.lastAnswer : null)
  const run = state.stream?.state === 'done' ? state.stream.outcome : null
  const actionBusy = executing || state.planning

  return (
    <Paper
      variant="outlined"
      aria-label="Grasp panel"
      sx={{ maxHeight: { xs: '62vh', sm: '70vh' }, overflowY: 'auto', bgcolor: 'rgba(22, 27, 34, 0.94)' }}
    >
      <Stack spacing={1.25} sx={{ p: 1.5 }}>
        <Stack direction="row" sx={{ alignItems: 'center', justifyContent: 'space-between' }}>
          <Typography variant="overline" sx={{ lineHeight: 2 }}>
            Grasp
          </Typography>
          <IconButton size="small" aria-label="Close grasp panel" onClick={onClose}>
            <CloseIcon fontSize="small" />
          </IconButton>
        </Stack>

        {executing && (
          <Button
            fullWidth
            variant="contained"
            color="error"
            size="large"
            startIcon={<StopCircleIcon />}
            onClick={state.stop}
            sx={{ fontWeight: 800, letterSpacing: '0.05em', py: 1.5 }}
          >
            STOP GRASP
          </Button>
        )}

        <Stack direction="row" spacing={1} sx={{ alignItems: 'center', flexWrap: 'wrap', gap: 1 }}>
          <ToggleButtonGroup
            size="small"
            exclusive
            value={form.frame}
            onChange={(_e, v: GraspFrame | null) => v && state.setFrame(v)}
            aria-label="Object frame"
          >
            <ToggleButton value="base_link" aria-label="Frame base_link">
              base_link
            </ToggleButton>
            <ToggleButton value="arm" aria-label="Frame arm">
              arm
            </ToggleButton>
          </ToggleButtonGroup>
          <Button
            size="small"
            variant={state.pickMode ? 'contained' : 'outlined'}
            startIcon={<AdsClickIcon />}
            disabled={!topView}
            aria-pressed={state.pickMode}
            onClick={() => state.setPickMode(!state.pickMode)}
            title={topView ? 'Click the map ground to fill x and y (base_link)' : 'Switch to Top view to pick on the map'}
          >
            {state.pickMode ? 'Click map' : 'Pick on map'}
          </Button>
        </Stack>

        <Stack direction="row" useFlexGap sx={{ gap: 1, flexWrap: 'wrap' }}>
          <Field label="x" unit="m" value={form.x} onChange={(x) => setForm({ x })} />
          <Field label="y" unit="m" value={form.y} onChange={(y) => setForm({ y })} />
          <Field label="Support z" unit="m" value={form.supportZ} onChange={(supportZ) => setForm({ supportZ })} />
        </Stack>
        <Stack direction="row" useFlexGap sx={{ gap: 1, flexWrap: 'wrap' }}>
          <Field label="Width" unit="m" value={form.width} onChange={(width) => setForm({ width })} />
          <Field label="Depth" unit="m" value={form.depth} onChange={(depth) => setForm({ depth })} />
          <Field label="Height" unit="m" value={form.height} onChange={(height) => setForm({ height })} />
          <Field label="Yaw" unit="deg" placeholder="across" value={form.yaw} onChange={(yaw) => setForm({ yaw })} />
        </Stack>
        <Typography variant="caption" color="text.secondary">
          Support z is the height of the object bottom (the surface it stands on) in the chosen frame. Width is across the
          jaws, depth along the approach.
        </Typography>

        <Stack direction="row" useFlexGap sx={{ gap: 1, flexWrap: 'wrap' }}>
          <TextField
            select
            size="small"
            label="Strategy"
            value={form.strategy}
            onChange={(e) => setForm({ strategy: e.target.value as GraspStrategy })}
            sx={{ flex: '2 1 160px', minWidth: 160 }}
          >
            {STRATEGIES.map((s) => (
              <MenuItem key={s.value} value={s.value}>
                {s.label}
              </MenuItem>
            ))}
          </TextField>
          {form.strategy === 'angled' && (
            <Field label="Pitch" unit="deg" placeholder="45" value={form.pitchDeg} onChange={(pitchDeg) => setForm({ pitchDeg })} />
          )}
        </Stack>

        <Button
          size="small"
          color="inherit"
          onClick={() => setAdvanced((a) => !a)}
          endIcon={advanced ? <ExpandLessIcon /> : <ExpandMoreIcon />}
          aria-expanded={advanced}
          sx={{ justifyContent: 'space-between' }}
        >
          Advanced
        </Button>
        <Collapse in={advanced} unmountOnExit>
          <Stack spacing={1}>
            <Typography variant="caption" color="text.secondary">
              Blank = the server default (shown in grey).
            </Typography>
            <Stack direction="row" useFlexGap sx={{ gap: 1, flexWrap: 'wrap' }}>
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
          <Button variant="outlined" disabled={actionBusy} onClick={state.requestPlan} sx={{ flex: '1 1 90px' }}>
            {state.planning ? 'Planning...' : 'Plan'}
          </Button>
          <Button
            variant="contained"
            color="warning"
            disabled={!state.canExecute}
            onClick={state.requestExecute}
            title="Needs a feasible plan for exactly these inputs"
            sx={{ flex: '1 1 90px' }}
          >
            Execute
          </Button>
          <Button
            variant={state.releaseArmed ? 'contained' : 'outlined'}
            color={state.releaseArmed ? 'warning' : 'primary'}
            disabled={actionBusy}
            onClick={state.release}
            title="Open the gripper and lift. Click twice to confirm."
            sx={{ flex: '1 1 90px' }}
          >
            {state.releaseArmed ? 'Confirm release' : 'Release'}
          </Button>
        </Stack>
        {state.plan && !state.planFresh && (
          <Typography variant="caption" color="warning.main">
            Inputs changed since the last plan: plan again to enable Execute.
          </Typography>
        )}

        {state.confirming && state.plan?.outcome.plan && (
          <Alert severity="warning" icon={false} sx={{ '& .MuiAlert-message': { width: '100%' } }}>
            <Typography variant="body2" sx={{ fontWeight: 700 }}>
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

        {!executing && (
          <Button
            fullWidth
            variant="outlined"
            color="error"
            startIcon={<StopCircleIcon />}
            onClick={state.stop}
            title="Stop and hold the arm (also aborts a running grasp)"
            sx={{ fontWeight: 700 }}
          >
            Stop
          </Button>
        )}

        {state.stream?.state === 'running' && (
          <Alert severity="info">{`Running ${state.stream.action ?? 'grasp'}... use Stop to abort.`}</Alert>
        )}
        {state.planning && <Alert severity="info">Planning...</Alert>}
        {run && <Result answer={run} title={state.stream?.action ?? 'Grasp'} />}
        {planAnswer && !state.planning && <Result answer={planAnswer} title="Plan" />}
        {!planAnswer && state.lastAnswer && !state.lastAnswer.ok && <Result answer={state.lastAnswer} title="Grasp" />}
      </Stack>
    </Paper>
  )
}
