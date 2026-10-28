import Box from '@mui/material/Box'
import Typography from '@mui/material/Typography'
import { Coverage, emptyCells } from './rgbdApi'

/** Edge of one coverage cell in px. */
const CELL_PX = 24
/** Corner count at which a cell is shaded at full strength. */
const FULL_SHADE_COUNT = 4

interface GridProps {
  label: string
  coverage: Coverage
}

function Grid({ label, coverage }: GridProps) {
  return (
    <Box sx={{ mr: 2, mb: 1 }}>
      <Typography variant="subtitle2">{label}</Typography>
      <Box
        sx={{ display: 'grid', gridTemplateColumns: `repeat(${coverage.cols}, ${CELL_PX}px)`, gap: '2px' }}
        aria-label={`${label} coverage`}
      >
        {coverage.counts.flatMap((row, r) =>
          row.map((n, c) => (
            <Box
              key={`${r}-${c}`}
              data-testid={n === 0 ? 'coverage-cell-empty' : 'coverage-cell'}
              title={`${n}`}
              sx={{
                width: CELL_PX,
                height: CELL_PX,
                bgcolor: n === 0 ? 'transparent' : 'success.main',
                opacity: n === 0 ? 1 : Math.min(1, 0.25 + (0.75 * n) / FULL_SHADE_COUNT),
                border: 1,
                borderColor: n === 0 ? 'error.main' : 'divider',
                borderStyle: n === 0 ? 'dashed' : 'solid',
              }}
            />
          )),
        )}
      </Box>
      <Typography variant="body2" color="text.secondary">
        {`${emptyCells(coverage)} of ${coverage.counts.reduce((s, row) => s + row.length, 0)} image areas without corners`}
      </Typography>
    </Box>
  )
}

/** Colour image-area coverage of the stored stills. */
export function CoverageGrid({ rgb }: { rgb: Coverage }) {
  return (
    <Box sx={{ display: 'flex', flexWrap: 'wrap', my: 1 }}>
      <Grid label="RGB" coverage={rgb} />
    </Box>
  )
}
