import Box from '@mui/material/Box'
import Typography from '@mui/material/Typography'
import type { CaptureJoint } from './servoApi'

const MAX_STEPS = 4095
const PERCENT = 100

/**
 * Position of a step value on the 0..4095 bar, clamped: the captured extremes are unwrapped and can fall outside it.
 *
 * @param steps - servo steps (may be outside 0..4095)
 * @returns percent from the bar start, in 0..100
 */
export function barPercent(steps: number): number {
  return (Math.min(Math.max(steps, 0), MAX_STEPS) / MAX_STEPS) * PERCENT
}

/**
 * Text of an optional number: the value with the unit, or "-" before the first sample (never a fake zero).
 *
 * @param value - the number or null
 * @param unit - suffix appended to a present value
 * @returns the text
 */
export function valueText(value: number | null, unit: string): string {
  return value === null ? '-' : `${Math.round(value * 10) / 10}${unit}`
}

/**
 * One bar per joint over 0..4095 steps: the captured min..max span, the present marker and the range in degrees.
 * Extremes outside 0..4095 (an unwrapped capture near the wrap) are clamped for drawing; the text keeps true values.
 *
 * @param joints - joint name -> capture reading (null fields before the first sample)
 */
export function RangeBars({ joints }: { joints: Record<string, CaptureJoint> }) {
  return (
    <Box component="ul" aria-label="Captured ranges" sx={{ listStyle: 'none', p: 0, m: 0 }}>
      {Object.entries(joints).map(([name, j]) => {
        const span = j.min_steps !== null && j.max_steps !== null
        const left = span ? barPercent(j.min_steps as number) : 0
        const right = span ? barPercent(j.max_steps as number) : 0
        return (
          <Box component="li" key={name} sx={{ mb: 1 }}>
            <Typography variant="body2">{name}</Typography>
            <Box
              data-testid={`bar-${name}`}
              sx={{ position: 'relative', height: 14, bgcolor: 'action.hover', borderRadius: 0.5, my: 0.5 }}
            >
              {span && (
                <Box
                  data-testid={`span-${name}`}
                  style={{ left: `${left}%`, width: `${right - left}%` }}
                  sx={{ position: 'absolute', top: 0, bottom: 0, bgcolor: 'primary.light', borderRadius: 0.5 }}
                />
              )}
              {j.present_steps !== null && (
                <Box
                  data-testid={`present-${name}`}
                  style={{ left: `${barPercent(j.present_steps)}%` }}
                  sx={{ position: 'absolute', top: -2, bottom: -2, width: 3, bgcolor: 'error.main' }}
                />
              )}
            </Box>
            <Typography variant="caption" color="text.secondary">
              {`min ${valueText(j.min_steps, '')} max ${valueText(j.max_steps, '')} present ${valueText(j.present_steps, '')} range ${valueText(j.range_deg, ' deg')}`}
            </Typography>
          </Box>
        )
      })}
    </Box>
  )
}
